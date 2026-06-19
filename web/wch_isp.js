// WebUSB driver for the WCH ISP (USB bootloader) protocol, targeting the CH572. Ported from minichlink's pgm-wch-isp.c
//
// Wire format: each command is [cmd, lenLo, lenHi, ...payload]. The device replies with [cmd, lenLo, lenHi, ...payload]; status byte is reply[4].
//
// Flash sequence:
//   1. IDENTIFY (0xa1) -> chip_type (u16 LE in reply[4..6])
//   2. READ CONFIG (0xa7) -> 30-byte reply; UID at [22..30]
//   3. derive 8-byte XOR key from UID + chip_type, then SEND KEY (0xa3); device replies with sum(key) for verification
//   4. ERASE (0xa4) sized to ceil(blob/1024), min 8 KiB
//   5. WRITE (0xa5) in 56-byte chunks, payload XORed with the key
//   6. END (0xa2 01 00 01) reboots into app

export const WCH_ISP_VID = 0x1a86;
export const WCH_ISP_PID = 0x55e0;

const CMD_IDENTIFY  = 0xa1;
const CMD_END       = 0xa2;
const CMD_KEY       = 0xa3;
const CMD_ERASE     = 0xa4;
const CMD_WRITE     = 0xa5;
const CMD_READ_CONF = 0xa7;

const IDENTIFY_PAYLOAD = new Uint8Array([
    0x52, 0x11,
    ...new TextEncoder().encode("MCU ISP & WCH.CN"),
]);

const READ_CONFIG_PAYLOAD = new Uint8Array([0x1f, 0x00]);

const KEY_LEN  = 8;
const UID_LEN  = 8;
const CHUNK    = 56;

export class WchIsp {
    constructor(device) {
        this.device = device;
        this.epIn = null;
        this.epOut = null;
        this.chipType = 0;
        this.uid = new Uint8Array(UID_LEN);
        this.xorKey = new Uint8Array(KEY_LEN);
        this.bootloaderVersion = "";
    }

    static async request() {
        const device = await navigator.usb.requestDevice({
            filters: [{ vendorId: WCH_ISP_VID, productId: WCH_ISP_PID }],
        });
        const isp = new WchIsp(device);
        await isp._open();
        return isp;
    }

    async _open() {
        await this.device.open();
        if (this.device.configuration === null) {
            await this.device.selectConfiguration(1);
        }
        const iface = this.device.configuration.interfaces[0];
        await this.device.claimInterface(iface.interfaceNumber);
        for (const ep of iface.alternate.endpoints) {
            if (ep.type !== "bulk") continue;
            if (ep.direction === "in")  this.epIn  = ep.endpointNumber;
            if (ep.direction === "out") this.epOut = ep.endpointNumber;
        }
        if (this.epIn === null || this.epOut === null) {
            throw new Error("could not find bulk IN/OUT endpoints on the ISP interface");
        }
    }

    async close() {
        try { await this.device.close(); } catch {}
    }

    async _cmd(cmd, payload = new Uint8Array(0), readMax = 256) {
        const pkt = new Uint8Array(3 + payload.length);
        pkt[0] = cmd;
        pkt[1] = payload.length & 0xff;
        pkt[2] = (payload.length >> 8) & 0xff;
        pkt.set(payload, 3);
        const out = await this.device.transferOut(this.epOut, pkt);
        if (out.status !== "ok") throw new Error(`transferOut: ${out.status}`);
        const inn = await this.device.transferIn(this.epIn, readMax);
        if (inn.status !== "ok") throw new Error(`transferIn: ${inn.status}`);
        return new Uint8Array(inn.data.buffer, inn.data.byteOffset, inn.data.byteLength);
    }

    async identify() {
        const r = await this._cmd(CMD_IDENTIFY, IDENTIFY_PAYLOAD);
        if (r.length < 6) throw new Error(`IDENTIFY: short reply (${r.length} B)`);
        this.chipType = r[4] | (r[5] << 8);
        return this.chipType;
    }

    async readConfig() {
        const r = await this._cmd(CMD_READ_CONF, READ_CONFIG_PAYLOAD);
        if (r.length < 30) throw new Error(`READ_CONFIG: short reply (${r.length} B)`);
        this.uid = r.slice(22, 22 + UID_LEN);
        this.bootloaderVersion = `${r[19]}.${r[20]}${r[21]}`;
        return r;
    }

    // Derive the XOR key from UID + chip_type and hand it to the device.
    async sendKey() {
        let sum = 0;
        for (let i = 0; i < UID_LEN; i++) sum = (sum + this.uid[i]) & 0xff;
        const key = new Uint8Array(KEY_LEN);
        for (let i = 0; i < KEY_LEN - 1; i++) key[i] = sum;
        key[KEY_LEN - 1] = (sum + this.chipType) & 0xff;
        this.xorKey = key;

        let expected = 0;
        for (let i = 0; i < KEY_LEN; i++) expected = (expected + key[i]) & 0xff;

        // 0x1e bytes of zero payload (the device only cares that we sent a key
        // command; the actual key was implied by the UID it already knows).
        const r = await this._cmd(CMD_KEY, new Uint8Array(0x1e));
        if (r[4] !== expected) {
            throw new Error(`SEND_KEY: device sum 0x${r[4].toString(16)} != expected 0x${expected.toString(16)}`);
        }
    }

    async erase(byteLen) {
        let sectors = Math.ceil(byteLen / 1024);
        if (sectors < 8) sectors = 8;
        const payload = new Uint8Array(4);
        payload[0] = sectors & 0xff;
        payload[1] = (sectors >> 8) & 0xff;
        const r = await this._cmd(CMD_ERASE, payload);
        if (r[4] !== 0) throw new Error(`ERASE: status 0x${r[4].toString(16)}`);
    }

    async writeBin(bin, baseAddr = 0, onProgress = () => {}) {
        const writeSize = (bin.length % 256) ? (Math.floor(bin.length / 256) + 1) * 256 : bin.length;
        const iters = Math.floor(writeSize / CHUNK) + 1;

        // 5-byte sub-header + 56-byte XORed data
        const payload = new Uint8Array(5 + CHUNK);

        for (let i = 0; i < iters; i++) {
            const offset = baseAddr + i * CHUNK;
            payload[0] = offset & 0xff;
            payload[1] = (offset >> 8)  & 0xff;
            payload[2] = (offset >> 16) & 0xff;
            payload[3] = (offset >> 24) & 0xff;
            payload[4] = (bin.length - i * CHUNK) & 0xff;

            for (let j = 0; j < CHUNK; j++) {
                const idx = i * CHUNK + j;
                const src = idx < bin.length ? bin[idx] : 0xff;
                payload[5 + j] = src ^ this.xorKey[j % KEY_LEN];
            }

            const r = await this._cmd(CMD_WRITE, payload);
            const st = r[4];
            // 0x00 ok; 0xfe / 0xf5 = past-end chunk, also fine
            if (st !== 0x00 && st !== 0xfe && st !== 0xf5) {
                throw new Error(`WRITE @0x${offset.toString(16)}: status 0x${st.toString(16)}`);
            }
            onProgress(Math.min(1, (i + 1) * CHUNK / bin.length));
        }
    }

    async reboot() {
        await this._cmd(CMD_END, new Uint8Array([0x01]));
    }

    async flash(bin, onProgress = () => {}) {
        await this.identify();
        await this.readConfig();
        await this.sendKey();
        await this.erase(bin.length);
        await this.writeBin(bin, 0, onProgress);
        await this.reboot();
    }
}

export function hex(n, width = 2) {
    return n.toString(16).padStart(width, "0");
}

export function formatUid(uid) {
    return Array.from(uid, b => hex(b)).join("-");
}

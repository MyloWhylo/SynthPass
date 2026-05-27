/*
 * Composite USB device:
 *   - CDC ACM  (EP1/2/3) : debug serial (printf out, 'b' command in)
 *   - MSC BOT  (EP5/6)   : mass storage for collected messages
 *
 * The MSC side currently exposes a FAT16 RAM disk with a placeholder README;
 * collected peer messages will populate it in a later step. Built on ch32fun's
 * iSLER (2.4GHz BLE-advertising radio) + fsusb. The MSC machinery is adapted
 * from ch32fun's examples_usb/USBFS/usbfs_msc.
 */
#include "ch32fun.h"
#include "ch5xxhw.h"
#include "iSLER.h"
#include <stdio.h>
#include <string.h>
#include "fsusb.h"

#include "synthpass.h"

#define LED PA11

// ---- USB endpoints (must match the descriptors in usb_config.h) ----
#define EP_CDC_IRQ 1   // interrupt IN  - CDC notifications (unused)
#define EP_CDC_OUT 2   // bulk OUT      - CDC debug input
#define EP_CDC_IN  3   // bulk IN       - CDC debug output (printf)
#define EP_MSC_OUT 6   // bulk OUT      - MSC data from host
#define EP_MSC_IN  5   // bulk IN       - MSC data to host


// MSD: FAT16 RAM disk (placeholder; will hold collected messages)
#define MSC_RAM_DISK_SIZE   (4 * 1024)
#define MSC_BLOCK_SIZE      512
#define MSC_TOTAL_SECTORS   0x4000      // reported geometry (8 MB)
#define FILE_CLUSTER        2

uint8_t msc_ram_disk[MSC_RAM_DISK_SIZE] __attribute__((aligned(4)));

// SECTOR 0: FAT16 Boot Sector (BPB) for an 8MB drive
const uint8_t BootSector[512] = {
	0xEB, 0x3C, 0x90,                               // Jump Instruction
	's','y','n','t','h','p','s','s',                // OEM Name (8)
	0x00, 0x02,                                     // Bytes per sector (512)
	0x08,                                           // Sectors per cluster (4KB clusters)
	0x01, 0x00,                                     // Reserved sectors (1)
	0x02,                                           // Number of FATs (2)
	0x00, 0x02,                                     // Root dir entries (512)
	0x00, 0x40,                                     // Total sectors small (16384 = 8MB)
	0xF8,                                           // Media descriptor
	0x10, 0x00,                                     // Sectors per FAT (16)
	0x3F, 0x00,                                     // Sectors per track
	0xFF, 0x00,                                     // Heads
	0x00, 0x00, 0x00, 0x00,                         // Hidden sectors
	0x00, 0x00, 0x00, 0x00,                         // Large sectors
	0x80,                                           // Drive number
	0x00,                                           // Reserved
	0x29,                                           // Ext boot signature
	0x12, 0x34, 0x56, 0x78,                         // Serial Number
	'S','Y','N','T','H','P','A','S','S',' ',' ',    // Volume Label (11)
	'F', 'A', 'T', '1', '6', ' ', ' ', ' ',         // FS Type
	[510] = 0x55,
	[511] = 0xAA
};

// Placeholder file contents (will become the collected-message store later).
const uint8_t README_TXT[] =
	"SynthPass message storage.\r\n"
	"Collected peer messages will appear here.\r\n";

static volatile int file_changed;
static volatile uint16_t active_file_cluster = FILE_CLUSTER;
static volatile uint32_t active_file_size = sizeof(README_TXT) - 1;

// Root directory entry for "README.TXT"
uint8_t RootDirEntry[32] = {
	'R', 'E', 'A', 'D', 'M', 'E', ' ', ' ', 'T', 'X', 'T',      // 0x00: Name (11)
	0x20,                                                       // 0x0B: Attributes (Archive)
	0x00,                                                       // 0x0C: Reserved (NT)
	0x00,                                                       // 0x0D: CrtTimeTenth
	0x00, 0x00,                                                 // 0x0E: CrtTime
	0x00, 0x00,                                                 // 0x10: CrtDate
	0x00, 0x00,                                                 // 0x12: Last Access Date
	0x00, 0x00,                                                 // 0x14: High Cluster
	0x21, 0x00,                                                 // 0x16: WrtTime
	0x21, 0x00,                                                 // 0x18: WrtDate
	(FILE_CLUSTER & 0xFF),
	(FILE_CLUSTER >> 8),                                        // 0x1A: Low Cluster (2)
	((sizeof(README_TXT) - 1) & 0xFF),
	((sizeof(README_TXT) - 1) >> 8),
	((sizeof(README_TXT) - 1) >> 16),
	((sizeof(README_TXT) - 1) >> 24),                           // 0x1C: Size (Little Endian)
};

// Disk layout constants (based on BPB above)
#define START_FAT1      1
#define START_FAT2      17
#define START_ROOT      33  // 1 + 16 + 16
#define START_DATA      65  // 33 + 32 sectors for root dir

// MSC Bulk-Only Transport state machine
typedef enum {
	MSC_IDLE,       // Waiting for CBW
	MSC_DATA_OUT,   // Receiving data from PC (Write)
	MSC_DATA_IN,    // Sending data to PC (Read/Inquiry)
	MSC_SEND_CSW    // Sending Status Wrapper
} msc_state_t;

volatile msc_state_t msc_state = MSC_IDLE;

// Command Block Wrapper (31 bytes)
struct CBW {
	uint32_t Signature;          // "USBC" (0x43425355)
	uint32_t Tag;
	uint32_t DataTransferLength;
	uint8_t  Flags;
	uint8_t  LUN;
	uint8_t  CBLength;
	uint8_t  CB[16];             // SCSI Command Block
} __attribute__((packed));

// Command Status Wrapper (13 bytes)
struct CSW {
	uint32_t Signature;          // "USBS" (0x53425355)
	uint32_t Tag;
	uint32_t DataResidue;
	uint8_t  Status;             // 0=Pass, 1=Fail, 2=Phase Error
} __attribute__((packed));

volatile struct CBW cbw;
volatile struct CSW csw;

volatile uint32_t msc_current_offset = 0;
volatile uint32_t msc_bytes_remaining = 0;


// ============================================================================
// Radio / protocol globals
// ============================================================================
__attribute__((aligned(4))) SynthPass_Frame_T tx_frame = {};
uint32_t synthpass_uid;
uint32_t last_broadcast_tick;
uint32_t broadcast_random_ticks;

// TODO replace with two timers (last_prox, last_booped)
Synthpass_BroadcastPeriod_T period = BROADCAST_PERIOD_NORMAL;

// Last byte received on the CDC debug endpoint (0 = none), processed in main().
volatile char cdc_input;


void blink(int n) {
	for(int i = n-1; i >= 0; i--) {
		funDigitalWrite( LED, FUN_LOW ); // Turn on LED
		Delay_Ms(33);
		funDigitalWrite( LED, FUN_HIGH ); // Turn off LED
		if(i) Delay_Ms(33);
	}
}


// ============================================================================
// CDC debug serial I/O (printf backend)
// ============================================================================
int _write(int fd, const char *buf, int size) {
	(void)fd;
	if(USBFS_SendEndpointNEW(EP_CDC_IN, (uint8_t*)buf, size, /*copy*/1) == -1) { // -1 == busy
		Delay_Ms(1); // wait and try once more
		USBFS_SendEndpointNEW(EP_CDC_IN, (uint8_t*)buf, size, /*copy*/1);
	}
	return size;
}

int putchar(int c) {
	uint8_t single = c;
	if(USBFS_SendEndpointNEW(EP_CDC_IN, &single, 1, /*copy*/1) == -1) { // -1 == busy
		Delay_Ms(1);
		USBFS_SendEndpointNEW(EP_CDC_IN, &single, 1, /*copy*/1);
	}
	return 1;
}


// ============================================================================
// Radio / SynthPass protocol
// ============================================================================
void synthpass_rx() {
	iSLERRX(ACCESS_ADDRESS, SYNTHPASS_CHANNEL, SYNTHPASS_PHY_MODE);
}

uint8_t synthpass_tx(SynthPass_MessageType_T type, uint8_t* data, uint8_t data_length) {
	uint32_t len = data_length + sizeof(SynthPass_Header_T) + SYNTHPASS_MAC_SIZE;
	if(len > 255) {
		return 1;
	}
	tx_frame.length = len;

	tx_frame.msg.hdr.ad_len = data_length + sizeof(SynthPass_Header_T) - 1; // length of message+header, minus the ad_len byte itself

	tx_frame.msg.hdr.msg_type = type;
	memcpy(tx_frame.msg.data, data, data_length);

	uint32_t isler_frame_length = len + 2;

	iSLERTX(ACCESS_ADDRESS, (uint8_t*)&tx_frame, isler_frame_length, SYNTHPASS_CHANNEL, SYNTHPASS_PHY_MODE);

	return 0;
}

uint8_t synthpass_broadcast() {

	uint8_t status = synthpass_tx(SYNTHPASS_BROADCAST, 0, 0);
	if(status == 0) {
		printf("beep!\n");
	} else {
		printf("sad beep :(\n");
	}
	last_broadcast_tick = funSysTick32();
	return status;
}

void printf_uid(uint32_t uid) {
	for(int i = 0; i < 4; ++i) {
		printf(":%02x", (unsigned)((uid >> (8 * i)) & 0xFF));
	}
}

uint8_t synthpass_init() {
	// Get our UID, based on the chip's built-in ID
	uint64_t chip_uid = *(uint64_t*)(0x3F018);
	synthpass_uid = (chip_uid >> 32) ^ (chip_uid & 0xFFFFFFFF);

	tx_frame.pdu = 0x02;
	tx_frame.length = SYNTHPASS_MAC_SIZE;
	strncpy((char*) tx_frame.mac, SYNTHPASS_MAC, SYNTHPASS_MAC_SIZE);

	tx_frame.msg.hdr.ad_type = 0xFF; // "Manufacturer specific data"

	tx_frame.msg.hdr.ref_rssi = SYNTHPASS_REF_RSSI;
	tx_frame.msg.hdr.sender_uid = synthpass_uid;

	printf("Synthpass init! UID");
	printf_uid(synthpass_uid);
	printf("\n");

	// set last broadcast time to now
	last_broadcast_tick = funSysTick32();
	broadcast_random_ticks = 0;

	// first broadcast
	synthpass_broadcast();
	// start listening for frames
	synthpass_rx();

	return 0;
}

uint8_t validate_synthpass_frame(volatile SynthPass_Frame_T *frame) {
	return (
		frame->pdu == SYNTHPASS_PDU
		&& (strncmp((const char*)(frame->mac), SYNTHPASS_MAC, SYNTHPASS_MAC_SIZE) == 0)
	) ? 1 : 0;
}

void incoming_frame_handler() {
	// The chip stores the incoming frame in LLE_BUF, defined in extralibs/iSLER.h
	volatile SynthPass_Frame_T *frame = (volatile SynthPass_Frame_T*)LLE_BUF;

	// check if the RX'd frame is a synthpass frame (PDU and MAC match)
	if(!validate_synthpass_frame(frame)) {
		return;
	}

	int rssi = iSLERRSSI();
	int corrected_rssi = rssi - frame->msg.hdr.ref_rssi - SYNTHPASS_REF_RXRSSI;

	SynthPass_MessageType_T type = frame->msg.hdr.msg_type;

	switch(type) {
		case SYNTHPASS_BROADCAST:
			{
				// received broadcast data from another SynthPass, reply with PROX frame
				printf("BROADCAST peer uid");
				printf_uid(frame->msg.hdr.sender_uid);
				printf("\n");

				SynthPass_Prox_T msg = {
					.peer_uid=frame->msg.hdr.sender_uid,
					.rx_rssi=corrected_rssi
				};

				// Wait for the peer to transition back into RX after its broadcast
				// before replying, otherwise the PROX arrives while it can't receive.
				Delay_Ms(10);

				uint8_t status = synthpass_tx(SYNTHPASS_PROX, (uint8_t*)&msg, sizeof(msg));
				(void)status;

				// switch to faster message rate
				if(period == SYNTHPASS_BROADCAST_PERIOD) period = SYNTHPASS_PROX_PERIOD;
				synthpass_rx();
			}
			break;
		case SYNTHPASS_PROX:
			{
				// Received a response
				SynthPass_Prox_T *rxData = (SynthPass_Prox_T *) frame->msg.data;
				if(rxData->peer_uid == synthpass_uid) {
					printf("PROX peer uid");
					printf_uid(frame->msg.hdr.sender_uid);
					printf(" rx_rssi=%d\n", rxData->rx_rssi);
				} else {
					printf("(not for me) PROX\n");
				}
			}
			break;
		default:
			printf("Unrecognized type %d\n", type);
			break;
	}
}


// ============================================================================
// MSD: SCSI / Bulk-Only Transport
// ============================================================================

// Decide and send the next IN packet (read / inquiry / etc.), or the CSW.
void MSC_PrepareDataIn(void) {
	uint8_t msc_response_buffer[64]  __attribute__((aligned(4))) = {0}; // Scratch buffer
	uint32_t len_to_send;
	uint8_t *src_ptr = NULL;
	int is_short_transfer = 0;

	if (msc_state == MSC_SEND_CSW) return;

	// Done sending data -> send CSW.
	if (msc_bytes_remaining == 0) {
		msc_state = MSC_SEND_CSW;
		while(USBFS_SendEndpointNEW(EP_MSC_IN, (uint8_t*)&csw, sizeof(csw), 1) == -1); // -1 == busy
		return;
	}

	switch(cbw.CB[0]) {
	// --- STREAMING DATA COMMANDS (from RAM) ---
	case 0x28: { // READ 10
		uint32_t lba_start = (cbw.CB[2] << 24) | (cbw.CB[3] << 16) | (cbw.CB[4] << 8) | cbw.CB[5];

		// Offset relative to current READ_10 command start
		uint32_t lba_offset = (cbw.DataTransferLength - msc_bytes_remaining) / 512;
		uint32_t current_lba = lba_start + lba_offset;

		len_to_send = (msc_bytes_remaining > 64) ? 64 : msc_bytes_remaining;

		// --- 1. Boot Sector ---
		if (current_lba == 0) {
			uint32_t byte_offset_in_sector = (cbw.DataTransferLength - msc_bytes_remaining) % 512;
			memcpy(msc_response_buffer, &BootSector[byte_offset_in_sector], len_to_send);
		}
		// --- 2. FAT Tables (Sectors 1..32) ---
		else if (current_lba >= START_FAT1 && current_lba < START_ROOT) {
			uint32_t byte_offset_in_sector = (cbw.DataTransferLength - msc_bytes_remaining) % 512;
			// Entry 0: F8 FF, Entry 1: FF FF, Entry 2: FF FF (EOF for the file)
			const uint8_t fat_head[] = { 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
			if ((current_lba == START_FAT1 || current_lba == START_FAT2) && byte_offset_in_sector < 6) {
				for(int i=0; i<len_to_send; i++) {
					if (byte_offset_in_sector + i < 6) {
						msc_response_buffer[i] = fat_head[byte_offset_in_sector + i];
					}
				}
			}
		}
		// --- 3. Root Directory (Sectors 33..64) ---
		else if (current_lba >= START_ROOT && current_lba < START_DATA) {
			uint32_t byte_offset_in_sector = (cbw.DataTransferLength - msc_bytes_remaining) % 512;
			if (current_lba == START_ROOT) {
				active_file_cluster = FILE_CLUSTER;
				*(uint32_t*)(RootDirEntry + 28) = active_file_size;
			}
			if (current_lba == START_ROOT) {
				for(int i = 0; i < len_to_send; i++) {
					int pos = byte_offset_in_sector + i;
					if (pos < 32) {
						msc_response_buffer[i] = RootDirEntry[pos];
					}
				}
			}
		}
		// --- 4. Data Area (Cluster 2 starts at START_DATA) ---
		else if (current_lba >= START_DATA) {
			uint32_t ram_sector_idx = current_lba - START_DATA;
			uint32_t ram_offset = (ram_sector_idx * 512) + ((cbw.DataTransferLength - msc_bytes_remaining) % 512);
			if (ram_offset + len_to_send <= MSC_RAM_DISK_SIZE) {
				memcpy(msc_response_buffer, &msc_ram_disk[ram_offset], len_to_send);
			}
		}

		src_ptr = msc_response_buffer;
		break;
	}

	// --- ONE-SHOT COMMANDS (generated headers) ---
	case 0x03: // REQUEST SENSE
		msc_response_buffer[0] = 0x70; // Response Code (Current, Fixed)
		msc_response_buffer[2] = 0x00; // Sense Key (No Sense)
		msc_response_buffer[7] = 0x0A; // Additional Sense Length (10)
		len_to_send = 18;
		src_ptr = msc_response_buffer;
		is_short_transfer = 1;
		break;

	case 0x12: { // INQUIRY
		msc_response_buffer[0] = 0x00; // Direct Access Device
		msc_response_buffer[1] = 0x80; // Removable
		msc_response_buffer[2] = 0x02; // Version
		msc_response_buffer[3] = 0x02; // Format
		msc_response_buffer[4] = 32;   // Additional Length
		memcpy(&msc_response_buffer[8],  "SynthPass", 8);
		memcpy(&msc_response_buffer[16], "Messages        ", 16);
		memcpy(&msc_response_buffer[32], "1.00", 4);

		uint32_t actual_len = 37;
		len_to_send = (msc_bytes_remaining < actual_len) ? msc_bytes_remaining : actual_len;
		src_ptr = msc_response_buffer;
		is_short_transfer = 1;
		break;
	}

	case 0x25: // READ CAPACITY 10
		*(uint32_t*)&msc_response_buffer[0] = __builtin_bswap32(MSC_TOTAL_SECTORS - 1);
		*(uint32_t*)&msc_response_buffer[4] = __builtin_bswap32(MSC_BLOCK_SIZE);
		len_to_send = 8;
		src_ptr = msc_response_buffer;
		is_short_transfer = 1;
		break;

	case 0x1A: // MODE SENSE 6
		memset(msc_response_buffer, 0, 4);
		msc_response_buffer[0] = 3;
		len_to_send = 4;
		src_ptr = msc_response_buffer;
		is_short_transfer = 1;
		break;

	case 0x5A: // MODE SENSE 10
		memset(msc_response_buffer, 0, 8);
		msc_response_buffer[1] = 6;
		len_to_send = 8;
		src_ptr = msc_response_buffer;
		is_short_transfer = 1;
		break;

	default:
		len_to_send = 0;
		break;
	}

	if (len_to_send > 0 && src_ptr != NULL) {
		while(USBFS_SendEndpointNEW(EP_MSC_IN, src_ptr, len_to_send, 1) == -1); // -1 == busy
		if (is_short_transfer) {
			csw.DataResidue = msc_bytes_remaining - len_to_send;
			msc_bytes_remaining = 0;
		}
		else {
			msc_bytes_remaining -= len_to_send;
			msc_current_offset += len_to_send;
		}
	}
	else {
		msc_state = MSC_SEND_CSW;
		while(USBFS_SendEndpointNEW(EP_MSC_IN, (uint8_t*)&csw, sizeof(csw), 1) == -1); // -1 == busy
	}
}

// Called by the USB IRQ when an IN packet has been taken by the host.
int HandleInRequest(struct _USBState *ctx, int endp, uint8_t *data, int len) {
	(void)ctx; (void)data; (void)len;
	if ((endp == EP_MSC_IN) && (msc_state == MSC_SEND_CSW)) {
		msc_state = MSC_IDLE; // Host grabbed the CSW.
	}
	return 0;
}

// Called by the USB IRQ when an OUT packet has been received from the host.
void HandleDataOut(struct _USBState *ctx, int endp, uint8_t *data, int len) {
	if (endp == 0) {
		ctx->USBFS_SetupReqLen = 0;
	}
	else if (endp == EP_CDC_OUT) {
		// CDC debug input; processed in main() (incl. the 'b' bootloader command).
		if (len > 0) cdc_input = data[0];
	}
	else if (endp == EP_MSC_OUT) {

		// --- 1. IDLE State: Expecting CBW ---
		if (msc_state == MSC_IDLE) {
			if (len != 31) return; // Invalid CBW
			memcpy((void*)&cbw, data, 31);
			if (cbw.Signature != 0x43425355) return; // Invalid Sig

			csw.Signature = 0x53425355;
			csw.Tag = cbw.Tag;
			csw.DataResidue = 0;
			csw.Status = 0;

			uint32_t lba = (cbw.CB[2] << 24) | (cbw.CB[3] << 16) | (cbw.CB[4] << 8) | cbw.CB[5];
			uint32_t blocks = (cbw.CB[7] << 8) | cbw.CB[8];

			switch (cbw.CB[0]) {
			// -- DATA IN COMMANDS --
			case 0x03: // REQUEST SENSE
			case 0x12: // INQUIRY
			case 0x25: // READ CAPACITY
			case 0x1A: // MODE SENSE 6
			case 0x5A: // MODE SENSE 10
				msc_state = MSC_DATA_IN;
				msc_current_offset = 0;
				msc_bytes_remaining = cbw.DataTransferLength;
				MSC_PrepareDataIn();
				break;

			case 0x28: // READ 10
				msc_state = MSC_DATA_IN;
				msc_current_offset = lba * MSC_BLOCK_SIZE;
				msc_bytes_remaining = blocks * MSC_BLOCK_SIZE;
				MSC_PrepareDataIn();
				break;

			// -- DATA OUT COMMANDS --
			case 0x2A: // WRITE 10
				msc_state = MSC_DATA_OUT;
				msc_current_offset = lba * MSC_BLOCK_SIZE;
				msc_bytes_remaining = blocks * MSC_BLOCK_SIZE;
				break;

			// -- NO DATA COMMANDS --
			case 0x00: // TEST UNIT READY
			case 0x1E: // PREVENT_ALLOW_REMOVAL
			default:
				if(cbw.DataTransferLength > 0 && (cbw.Flags & 0x80)) {
					csw.Status = 1;
				}
				msc_state = MSC_SEND_CSW;
				while(USBFS_SendEndpointNEW(EP_MSC_IN, (uint8_t*)&csw, sizeof(csw), 1) == -1); // -1 == busy
				break;
			}
		}

		// --- 2. DATA OUT State: Receiving Data from PC ---
		else if (msc_state == MSC_DATA_OUT) {
			uint32_t write_len = (len < msc_bytes_remaining) ? len : msc_bytes_remaining;
			uint32_t current_lba = msc_current_offset / 512;

			// --- CASE A: Root Directory Update (snoop for filename) ---
			if (current_lba >= START_ROOT && current_lba < START_DATA) {
				for (int i = 0; i < write_len; i += 32) {
					if (memcmp(data + i, "README  TXT", 11) == 0) {
						uint16_t new_cluster = data[i + 26] | (data[i + 27] << 8);
						if (new_cluster != 0) {
							active_file_cluster = new_cluster;
							uint32_t new_size = *(uint32_t*)&data[i + 0x1C];
							active_file_size = (new_size > MSC_RAM_DISK_SIZE) ? MSC_RAM_DISK_SIZE : new_size;
							file_changed = 1;
						}
					}
				}
			}
			// --- CASE B: Data Area Write (filter by cluster) ---
			else if (current_lba >= START_DATA) {
				uint32_t target_cluster = 2 + (current_lba - START_DATA) / 8;
				if (target_cluster >= active_file_cluster) {
					uint32_t sector_offset = (current_lba - START_DATA) % 8;
					uint32_t byte_offset = (sector_offset * 512) + (msc_current_offset % 512);
					if (byte_offset + write_len <= MSC_RAM_DISK_SIZE) {
						memcpy(&msc_ram_disk[byte_offset], data, write_len);
					}
				}
			}

			msc_current_offset += write_len;
			msc_bytes_remaining -= write_len;

			if (msc_bytes_remaining == 0) {
				msc_state = MSC_SEND_CSW;
				while(USBFS_SendEndpointNEW(EP_MSC_IN, (uint8_t*)&csw, sizeof(csw), 1) == -1); // -1 == busy
			}
		}
	}
}

// Handle class-specific SETUP requests: CDC line coding + MSC GET_MAX_LUN.
int HandleSetupCustom(struct _USBState *ctx, int setup_code) {
	int ret = -1;
	if ( ctx->USBFS_SetupReqType & USB_REQ_TYP_CLASS ) {
		switch ( setup_code ) {
		case CDC_SET_LINE_CODING:
		case CDC_SET_LINE_CTLSTE:
		case CDC_SEND_BREAK:
			ret = ( ctx->USBFS_SetupReqLen ) ? ctx->USBFS_SetupReqLen : -1;
			break;
		case CDC_GET_LINE_CODING:
			ret = ctx->USBFS_SetupReqLen;
			break;
		case 0xFE: // MSC_GET_MAX_LUN
			ctx->pCtrlPayloadPtr = CTRL0BUFF;
			ctx->pCtrlPayloadPtr[0] = 0; // single LUN (0)
			ret = 1;
			break;
		default:
			ret = 0;
			break;
		}
	}
	else {
		ret = 0; // Go to STALL
	}
	return ret;
}


int main()
{
	SystemInit();

	funGpioInitAll();
	funPinMode( LED, GPIO_CFGLR_OUT_10Mhz_PP );

	// Seed the RAM disk with the placeholder file.
	memcpy(msc_ram_disk, README_TXT, sizeof(README_TXT));

	USBFSSetup();

	synthpass_init();

	iSLERInit(LL_TX_POWER_0_DBM);

	blink(1);

	while(1) {
		// CDC debug input: a lone 'b' drops into the USB bootloader (see
		// flash_all.sh); anything else is echoed back to the host.
		if(cdc_input) {
			char c = cdc_input;
			cdc_input = 0;
			if(c == 'b') {
				blink(5);
				USBFSReset();
				jump_isprom(); // enters the USB ISP bootloader; does not return
			} else {
				_write(0, &c, 1);
			}
		}

		// MSD: when the host has taken the previous IN packet, send the next.
		if(msc_state == MSC_DATA_IN) {
			MSC_PrepareDataIn();
		}

		// Radio: handle a received frame.
		if(rx_ready) {
			incoming_frame_handler();
			synthpass_rx();
		}

		// Radio: periodic broadcast.
		uint32_t broadcast_period_ticks = broadcast_random_ticks;
		if(period == BROADCAST_PERIOD_NORMAL) { broadcast_period_ticks += SYNTHPASS_BROADCAST_PERIOD * DELAY_MS_TIME; }
		else if(period == BROADCAST_PERIOD_PROX) { broadcast_period_ticks += SYNTHPASS_PROX_PERIOD * DELAY_MS_TIME; }
		else /* BROADCAST_PERIOD_BOOP */ { broadcast_period_ticks += SYNTHPASS_BOOP_PERIOD * DELAY_MS_TIME; }

		if(funSysTick32() - last_broadcast_tick > broadcast_period_ticks) {
			synthpass_broadcast();
			synthpass_rx();
			broadcast_random_ticks = 0; // todo randomize
		}

	}
}

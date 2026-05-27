# PlatformIO upload helper for the SynthPass CH572 (USB-bootloader / ISP flow).
#
# This project flashes via `upload_protocol = custom` (minichlink -C isp; see
# platformio.ini) because the CH572 runs in USB-bootloader mode with the SWIO
# debug interface disabled. Run as a post-script so the platform's "upload"
# target already exists when we attach to it.
#
# It does two things:
#   1. Makes `upload` depend on the .bin, so the image minichlink flashes is
#      always freshly built (the "custom" protocol only builds the .elf, which
#      would otherwise leave a stale/missing .bin).
#   2. Adds a pre-upload hook that auto-enters the bootloader: if SynthPass is
#      running (USB CDC 1209:d035) it sends the single-byte 'b' command (which
#      calls jump_isprom() in firmware) and waits for the WCH ISP device
#      (1a86:55e0) to enumerate -- so a bare `pio run -t upload` performs the
#      whole  b -> bootloader -> flash -> app  cycle with no button press.

import glob
import os
import time

Import("env")  # noqa: F821 (injected by SCons/PlatformIO)

SYNTHPASS_VID = 0x1209
SYNTHPASS_PID = 0xD035
ISP_VID = "1a86"
ISP_PID = "55e0"


def _isp_present():
    """True if the WCH USB ISP bootloader (1a86:55e0) is enumerated (Linux sysfs)."""
    for vid_path in glob.glob("/sys/bus/usb/devices/*/idVendor"):
        try:
            with open(vid_path) as f:
                if f.read().strip().lower() != ISP_VID:
                    continue
            with open(os.path.dirname(vid_path) + "/idProduct") as f:
                if f.read().strip().lower() == ISP_PID:
                    return True
        except OSError:
            continue
    return False


def _find_synthpass_port():
    """Return the /dev path of the running SynthPass CDC port, or None."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return None
    for p in list_ports.comports():
        if p.vid == SYNTHPASS_VID and p.pid == SYNTHPASS_PID:
            return p.device
    return None


def _configure_raw_115200(fd):
    """Match `stty raw -echo 115200`: raw mode plus a real baud, so the cdc-acm
    driver sends SET_LINE_CODING / asserts DTR and the device actually accepts
    data. (Setting raw alone leaves the baud unset and the byte is never sent.)"""
    import termios
    iflag, oflag, cflag, lflag, _ispeed, _ospeed, cc = termios.tcgetattr(fd)
    iflag &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP |
               termios.INLCR | termios.IGNCR | termios.ICRNL | termios.IXON)
    oflag &= ~termios.OPOST
    lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG |
               termios.IEXTEN)
    cflag &= ~(termios.CSIZE | termios.PARENB)
    cflag |= termios.CS8
    termios.tcsetattr(fd, termios.TCSANOW,
                      [iflag, oflag, cflag, lflag, termios.B115200, termios.B115200, cc])


def before_upload(source, target, env):  # noqa: ARG001 (SCons action signature)
    if _isp_present():
        print("[isp_upload] board already in bootloader; skipping 'b' trigger")
        return

    port = _find_synthpass_port()
    if not port:
        print("[isp_upload] no running SynthPass (1209:d035) and no ISP device found; "
              "assuming the board is / will be put in bootloader manually")
        return

    print("[isp_upload] sending 'b' to %s to enter the bootloader..." % port)
    fd = None
    try:
        # Hold the port open across the re-enumeration: a bare open/close toggles
        # DTR and the byte is frequently dropped before the firmware reads it.
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
        try:
            _configure_raw_115200(fd)
        except Exception:
            pass
        os.write(fd, b"b")
        try:
            import termios
            termios.tcdrain(fd)  # make sure the byte is flushed to the device
        except Exception:
            pass
        for _ in range(40):  # wait up to ~6s for the ISP device to appear
            time.sleep(0.15)
            if _isp_present():
                time.sleep(0.4)  # let the ISP device settle before minichlink
                print("[isp_upload] ISP bootloader is up")
                return
        print("[isp_upload] WARNING: ISP device (1a86:55e0) did not appear after 'b'; "
              "minichlink may fail")
    except Exception as e:
        print("[isp_upload] WARNING: failed to trigger bootloader: %s" % e)
    finally:
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass


# (1) always rebuild the .bin before uploading
env.Depends("upload", env.subst("$BUILD_DIR/${PROGNAME}.bin"))  # noqa: F821

# (2) auto-enter the bootloader right before the flash command runs
env.AddPreAction("upload", before_upload)  # noqa: F821

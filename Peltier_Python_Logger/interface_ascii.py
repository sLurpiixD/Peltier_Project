import serial
import time

# ==========================================
# CONFIGURATION
# ==========================================
PSU_PORT  = "COM3"    # Set to your PSU port
BAUD_RATE = 9600
FORMAT    = "{:07.3f}"  # 7 wide → strip dot → 6-digit data field

# Global serial object
ser = None

# ── Core primitives ─────────────────────────────────────────

def psu_write(cmd):
    if ser and ser.is_open:
        ser.write(cmd.encode())

def psu_read_decode():
    if ser and ser.is_open:
        # read_until(b">") is correct — PSU terminates every reply frame with ">"
        return ser.read_until(b">").decode(errors="replace")
    return ""

def exchange(cmd):
    psu_write(cmd)
    return psu_read_decode()

def build(fc, value=0):
    """
    Formats a value into the PSU's 6-digit data field.
    e.g. build("01", 4.5) → "<01004500000>"
    FORMAT = "{:07.3f}" → "004.500" → remove "." → "004500"
    """
    digits = FORMAT.format(value).replace(".", "")
    return f"<{fc}{digits}000>"

# ── Control functions (used by data_gatherer.py) ────────────

def psu_Connect():
    """Opens the serial port and establishes a session with the PSU."""
    global ser
    if ser is None or not ser.is_open:
        ser = serial.Serial(port=PSU_PORT, baudrate=BAUD_RATE, timeout=1)
        ser.flush()
    return exchange("<09100000000>")   # reply: <19OK0000000>

def psu_Disconnect():
    """Closes the PSU session and the serial port."""
    global ser
    if ser and ser.is_open:
        reply = exchange("<09200000000>")   # reply: <19OK0000000>
        ser.close()
        return reply
    return None

def psu_SetVoltage(v):
    return exchange(build("01", v))    # reply: <11OK0000000>

def psu_SetCurrent(a):
    """
    Sets the current LIMIT — not a forced output current.
    CC mode activates automatically when the load draws up to this limit.
    """
    return exchange(build("03", a))    # reply: <13OK0000000>

def psu_OutputON():
    # Note: reply is an echo of whatever the PSU last sent, not a confirmation.
    return exchange("<07000000000>")

def psu_OutputOFF():
    # Note: reply is an echo of whatever the PSU last sent, not a confirmation.
    return exchange("<08000000000>")

# ── Measurement / read functions ─────────────────────────────

def psu_get_voltage():
    """Returns actual terminal voltage (0.000 V when output is OFF)."""
    reply = exchange("<02000000000>")
    try:    return float(reply[3:9]) * 0.001
    except: return None

def psu_get_current():
    """Returns actual terminal current (0.000 A when output is OFF)."""
    reply = exchange("<04000000000>")
    try:    return float(reply[3:9]) * 0.001
    except: return None

def psu_get_mode():
    """
    Returns the PSU's operating mode: "CV", "CC", or "Unknown".

    This uses command 04 (the same frame as psu_get_current()) because
    on this PSU's ASCII protocol the operating mode is encoded in the
    second character of the current-read reply:
        reply[1] == '1'  →  Constant Voltage (CV)
        reply[1] == 'C'  →  Constant Current (CC)
    There is no separate mode-query command in this protocol.
    """
    reply = exchange("<04000000000>")
    if len(reply) > 1:
        if reply[1] == '1': return "CV"
        if reply[1] == 'C': return "CC"
    return "Unknown"

def psu_get_firmwareVersion():
    return exchange("<06000000000>")


# ── Standalone test sequence ──────────────────────────────────
# Runs ONLY when this file is executed directly.
# NOT executed when imported by data_gatherer.py.

if __name__ == "__main__":
    print("PSU Programming Interface Test")
    print("Transport : USB Virtual COM (CP210x UART Bridge)")
    print("Protocol  : Proprietary ASCII Command Protocol")

    time.sleep(1)

    psu_Connect()
    print("PSU Connected")
    time.sleep(0.2)

    print(psu_get_firmwareVersion())
    print('cannot get additional device info')
    print('cannot switch Buzzer')

    psu_SetCurrent(0.1)    # Set current limit first
    psu_SetVoltage(20.0)   # Then set voltage

    print('cannot read current and voltage set dials')

    psu_OutputON()
    print('PSU output ON')
    time.sleep(1)

    for i in range(5):
        v = psu_get_voltage()
        a = psu_get_current()
        m = psu_get_mode()
        v_str = f"{v:.3f}V" if v is not None else "Err"
        a_str = f"{a:.3f}A" if a is not None else "Err"
        print(f"[{i + 1}] V={v_str}  A={a_str}  Mode={m}")
        time.sleep(0.5)

    psu_OutputOFF()
    print('PSU output OFF')

    time.sleep(0.2)
    psu_Disconnect()
    print("PSU Disconnected")
"""Circuit of the OT2-Pipette tool board (COM-free, imported by build_pcb.py).

Like Opentrons' pipette-main board, plus the XIAO ESP32-S3 that emulates the identity EEPROM:
the robot's 2x5 ribbon header, pass-through to the plunger motor and the home switch, a 3-channel
BSS138 level shifter (SCL, SDA, and MIN so the XIAO can watch the home switch), the ADDRESS
divider, a Schottky from the ribbon's +5 V to the XIAO, and an optional 24C02 (option A).

Ribbon pinout (Opentrons pipette-main.sch JP1, motor board JP4/JP7):
  1 SCL  2 SDA  3 ADDRESS (GND left / 5 V right)  4 MIN  5 GND  6 +5V  7 M2A  8 M2B  9 M1A  10 M1B
Motor plug as on Opentrons pipettes: 1 = 1A, 2 = 1B, 3 = 2B, 4 = 2A.
XIAO ESP32-S3 pins (Seeed): 1 D0/GPIO1 ... 7 D6/GPIO43 on one side, 8 D7/GPIO44 ... 11 D10/GPIO9,
12 3V3, 13 GND, 14 5V on the other (numbered counter-clockwise from D0, USB at the top).
"""

# ref: (footprint "Lib:Name", value, LCSC, {pad: net}, populate, note)
# LCSC numbers checked in JLCPCB's parts search on 2026-11-19 (Basic: resistors, capacitors, B5819W;
# Preferred Extended: BSS138 C7420339, SRV05-4 C7420376, M24C02 C7562; the THT connectors are Extended).
# U1 (XIAO) is not assembled by JLCPCB: in the tool it is soldered on its own pin headers (no room for
# a socket inside the GEN2 outline); on a bench board two 1x7 sockets C2832270 (8.5 mm) fit the same pads.
PARTS = {
    "J1": ("Connector_IDC:IDC-Header_2x05_P2.54mm_Horizontal", "2x5 box header, right angle", "C9140",
           {"1": "SCL5", "2": "SDA5", "3": "ADDR", "4": "MIN", "5": "GND", "6": "+5V",
            "7": "M2A", "8": "M2B", "9": "M1A", "10": "M1B"}, True, "robot ribbon, on the back side"),
    "J2": ("Connector_JST:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical", "JST XH 4P motor", "C144395",
           {"1": "M1A", "2": "M1B", "3": "M2B", "4": "M2A"}, True, "YPF actuator"),
    "J3": ("Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical", "JST XH 2P switch", "C158012",
           {"1": "MIN", "2": "GND"}, True, "D2F-L: COM -> 1, NC -> 2"),
    "J4": ("Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical", "AUX", "",
           {"1": "+3V3", "2": "AUX1", "3": "AUX2", "4": "GND"}, False, "spare XIAO pins (e.g. ejector switch)"),
    "U1": ("XIAO", "XIAO ESP32-S3 on its pin headers", "",
           {"1": "A_SENSE", "2": "AUX1", "3": "AUX2", "4": "MIN3", "5": "SDA3", "6": "SCL3", "7": None,
            "8": None, "9": None, "10": None, "11": None, "12": "+3V3", "13": "GND", "14": "+5VX"}, True,
           "hand-soldered on its supplied pin headers; two 1x7 sockets C2832270 only on a bench board"),
    "Q1": ("Package_TO_SOT_SMD:SOT-23", "BSS138", "C7420339", {"1": "+3V3", "2": "SCL3", "3": "SCL5"}, True, "SCL shifter"),
    "Q2": ("Package_TO_SOT_SMD:SOT-23", "BSS138", "C7420339", {"1": "+3V3", "2": "SDA3", "3": "SDA5"}, True, "SDA shifter"),
    "Q3": ("Package_TO_SOT_SMD:SOT-23", "BSS138", "C7420339", {"1": "+3V3", "2": "MIN3", "3": "MIN"}, True, "home switch to the XIAO"),
    "R1": ("Resistor_SMD:R_0603_1608Metric", "10k", "C25804", {"1": "SCL3", "2": "+3V3"}, True, "LV pull-up"),
    "R2": ("Resistor_SMD:R_0603_1608Metric", "10k", "C25804", {"1": "SDA3", "2": "+3V3"}, True, "LV pull-up"),
    "R3": ("Resistor_SMD:R_0603_1608Metric", "10k", "C25804", {"1": "MIN3", "2": "+3V3"}, True, "LV pull-up"),
    "R4": ("Resistor_SMD:R_0603_1608Metric", "10k", "C25804", {"1": "SCL5", "2": "+5V"}, False, "HV pull-up (robot has 10k)"),
    "R5": ("Resistor_SMD:R_0603_1608Metric", "10k", "C25804", {"1": "SDA5", "2": "+5V"}, False, "HV pull-up (robot has 10k)"),
    "R6": ("Resistor_SMD:R_0603_1608Metric", "10k", "C25804", {"1": "ADDR", "2": "A_SENSE"}, True, "ADDRESS divider top"),
    "R7": ("Resistor_SMD:R_0603_1608Metric", "15k", "C22809", {"1": "A_SENSE", "2": "GND"}, True, "ADDRESS divider bottom (5 V -> 3.0 V)"),
    "C1": ("Capacitor_SMD:C_0603_1608Metric", "1nF", "C1588", {"1": "MIN", "2": "GND"}, True, "switch filter (as Opentrons)"),
    "C2": ("Capacitor_SMD:C_0805_2012Metric", "10uF", "C15850", {"1": "+5VX", "2": "GND"}, True, "XIAO supply"),
    "C3": ("Capacitor_SMD:C_0603_1608Metric", "100nF", "C14663", {"1": "+3V3", "2": "GND"}, True, "3V3 decoupling"),
    "C4": ("Capacitor_SMD:C_0603_1608Metric", "100nF", "C14663", {"1": "+5V", "2": "GND"}, False, "24C02 decoupling (option A)"),
    "D1": ("Diode_SMD:D_SOD-123", "B5819W", "C8598", {"1": "+5VX", "2": "+5V"}, True, "ribbon 5 V -> XIAO, no USB back-feed"),
    "U3": ("Package_TO_SOT_SMD:SOT-23-6", "SRV05-4", "C7420376",
           {"1": "SCL5", "2": "GND", "3": "SDA5", "4": "MIN", "5": "+5V", "6": "ADDR"}, True, "ESD on the ribbon signals"),
    "U2": ("Package_SO:SOIC-8_3.9x4.9mm_P1.27mm", "24C02", "C7562",
           {"1": "ADDR", "2": "GND", "3": "GND", "4": "GND", "5": "SDA5", "6": "SCL5", "7": "GND", "8": "+5V"}, False,
           "option A: EEPROM instead of the XIAO (never both)"),
}
CLASSES = {   # net: (track width, via diameter, via drill), mm
    "default": (0.25, 0.6, 0.3),
    "+5V": (0.4, 0.7, 0.35), "+5VX": (0.4, 0.7, 0.35), "+3V3": (0.3, 0.6, 0.3), "GND": (0.4, 0.7, 0.35),
    "M1A": (0.5, 0.8, 0.4), "M1B": (0.5, 0.8, 0.4), "M2A": (0.5, 0.8, 0.4), "M2B": (0.5, 0.8, 0.4),
}
ROUTE_FIRST = ["M1A", "M1B", "M2A", "M2B", "+5V", "+5VX"]

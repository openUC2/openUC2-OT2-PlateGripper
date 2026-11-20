/*
  OT-2 pipette identity EEPROM emulated by a Seeed XIAO ESP32-S3 (no 24C02 needed).

  The OT-2 motor controller (SmoothiewareOT, src/modules/utils/instrument/Instrument.cpp) reads
  the pipette EEPROM over the ribbon's I2C at 10 kHz, address 0x50 (left mount, ribbon pin 3 at
  GND) or 0x51 (right mount, pin 3 at 5 V). Every access is two separate transactions with a
  STOP and 10 ms between them: write 1 byte (memory pointer), then read 8 bytes. Layout:
    0x00  8 bytes   signature "opentron"
    0x30  32 bytes  ID     (ASCII, zero padded) - key for calibration and settings
    0x60  32 bytes  MODEL  (ASCII, zero padded) - "p20_single_v2.2" = P20 Single-Channel GEN2
    0xA0  32 bytes  DATA
  The robot writes only on M370 / M372 / M374 (9-byte write: pointer + 8 data bytes).

  The ESP32-S3 I2C slave (arduino-esp32 3.x) holds SCL low after an address match with the read
  bit until onRequest() has filled the TX FIFO, and drops unread bytes at the STOP, so the
  callbacks below behave like a 24C02. The image lives in NVS (Preferences), so writes from
  the robot survive a power cycle like on a real EEPROM.

  Wiring (details in PipetteTool/electronics/README.md, "Option B"). The robot's I2C is 5 V with
  10 k pull-ups: the ESP32-S3 pins are NOT 5 V tolerant, so SDA/SCL go through a BSS138 level
  shifter (2 channels: HV = 5 V ribbon side, LV = 3V3).
    ribbon 6 (+5 V)   -> XIAO 5V, shifter HV        ribbon 5 (GND) -> XIAO GND, shifter GND
    XIAO 3V3          -> shifter LV
    ribbon 1 (SCL)    -> shifter HV1, LV1 -> D5 (GPIO6)
    ribbon 2 (SDA)    -> shifter HV2, LV2 -> D4 (GPIO5)
    ribbon 3 (ADDR)   -> 10 k -> D0 (GPIO1), 15 k from D0 to GND (5 V -> 3.0 V)
  Ribbon pins 4 (MIN, home switch) and 7-10 (motor) pass by; on the tool board (electronics/pcb)
  a third BSS138 channel copies MIN to D3 (GPIO4), so the XIAO can log the home switch:
  every edge is printed with a millisecond time stamp ("switch HOME" = open = what the robot
  reads as homed), bounces within 5 ms are counted. Together with the "read 0xNN" lines of the
  robot's EEPROM scans this shows what happened around a failed homing.

  Serial console (USB, 115200, line ending LF): show | id <text> | model <text> | wp on|off |
  log on|off (print every EEPROM read, default off) | reset. Give every tool its own ID: the robot stores the plunger settings and calibration
  under it. The default ID is derived from the chip's MAC.

  Board: "XIAO_ESP32S3" in arduino-esp32 >= 3.0 (Tools > USB CDC On Boot: Enabled).
*/
#include <Preferences.h>
#include <Wire.h>

static const int PIN_SDA = D4;                 // GPIO5
static const int PIN_SCL = D5;                 // GPIO6
static const int PIN_ADDR = D0;                // GPIO1: ribbon pin 3 through the divider
static const int PIN_LED = LED_BUILTIN;        // GPIO21, active low: blinks on every read
static const int PIN_MIN = D3;                 // GPIO4: home switch via the shifter, HIGH = open = homed

static const uint8_t ADDR_LEFT = 0x50, ADDR_RIGHT = 0x51;
static const uint8_t LOC_SIG = 0x00, LOC_ID = 0x30, LOC_MODEL = 0x60, LOC_DATA = 0xA0, FIELD = 32;
static const char SIG[8] = {'o', 'p', 'e', 'n', 't', 'r', 'o', 'n'};
static const char *DEFAULT_MODEL = "p20_single_v2.2";

static uint8_t mem[256];
static volatile uint8_t ptr = 0;
static volatile bool dirty = false;
static volatile uint32_t lastWrite = 0, nReads = 0, nWrites = 0;
static bool writeProtect = false;
static uint8_t i2cAddr = ADDR_LEFT;
static volatile uint8_t lastPtr = 0;
static bool logReads = false;
Preferences prefs;

// home switch edges (ISR -> loop)
static volatile uint32_t edgeT[32];
static volatile uint8_t edgeV[32];
static volatile uint8_t edgeHead = 0;
static uint8_t edgeTail = 0;
static uint32_t nEdges = 0, nBounces = 0, lastEdgeMs = 0;

void IRAM_ATTR onMinEdge() {
  uint8_t h = edgeHead;
  edgeT[h & 31] = millis();
  edgeV[h & 31] = digitalRead(PIN_MIN);
  edgeHead = h + 1;
}

// ---------------------------------------------------------------- I2C slave (runs in the I2C task)
void onReceive(int n) {
  if (n <= 0 || !Wire.available()) return;
  ptr = Wire.read();                           // first byte: memory pointer
  if (--n <= 0) return;                        // pointer only: a read follows
  while (n-- > 0 && Wire.available()) {        // data: linear write from the pointer
    uint8_t b = Wire.read();
    if (!writeProtect) mem[ptr] = b;
    ptr++;
  }
  if (!writeProtect) dirty = true;
  lastWrite = millis();
  nWrites++;
}

void onRequest() {
  uint8_t buf[8];                              // the robot always reads 8 bytes
  for (uint8_t i = 0; i < 8; i++) buf[i] = mem[(uint8_t)(ptr + i)];
  lastPtr = ptr;
  Wire.write(buf, 8);
  ptr += 8;                                    // like the EEPROM's internal address counter
  nReads++;
}

// ---------------------------------------------------------------- image
void setField(uint8_t loc, const char *s) {
  memset(mem + loc, 0, FIELD);
  strncpy((char *)mem + loc, s, FIELD);
}

void defaults() {
  memset(mem, 0, sizeof mem);
  memcpy(mem + LOC_SIG, SIG, 8);
  char id[FIELD + 1];
  snprintf(id, sizeof id, "P20SV22%010lu", (unsigned long)(ESP.getEfuseMac() & 0xFFFFFFFFUL));
  setField(LOC_ID, id);
  setField(LOC_MODEL, DEFAULT_MODEL);
}

void save() {
  uint8_t copy[256];
  dirty = false;
  memcpy(copy, mem, sizeof copy);                // a write landing meanwhile sets dirty again
  prefs.putBytes("img", copy, sizeof copy);
  prefs.putBool("wp", writeProtect);
}

void load() {
  size_t n = prefs.getBytes("img", mem, sizeof mem);
  writeProtect = prefs.getBool("wp", false);
  if (n != sizeof mem || memcmp(mem + LOC_SIG, SIG, 8) != 0) {
    defaults();
    save();
  }
}

String field(uint8_t loc) {
  char s[FIELD + 1] = {0};
  memcpy(s, mem + loc, FIELD);
  return String(s);
}

void show() {
  Serial.printf("I2C address 0x%02X (%s mount), signature %s, write protect %s\n", i2cAddr,
                i2cAddr == ADDR_LEFT ? "left" : "right", memcmp(mem, SIG, 8) == 0 ? "ok" : "MISSING",
                writeProtect ? "on" : "off");
  Serial.printf("id    %s\nmodel %s\nreads %lu, writes %lu\n", field(LOC_ID).c_str(), field(LOC_MODEL).c_str(),
                (unsigned long)nReads, (unsigned long)nWrites);
  Serial.printf("home switch %s, %lu edges, %lu bounces (< 5 ms)\n", digitalRead(PIN_MIN) ? "HOME (open)" : "rest (closed)",
                (unsigned long)nEdges, (unsigned long)nBounces);
}

// ---------------------------------------------------------------- console
void command(String line) {
  line.trim();
  int sp = line.indexOf(' ');
  String cmd = sp < 0 ? line : line.substring(0, sp);
  String arg = sp < 0 ? "" : line.substring(sp + 1);
  arg.trim();
  if (cmd == "id" && arg.length() && arg.length() <= FIELD) {
    setField(LOC_ID, arg.c_str());
    save();
  } else if (cmd == "model" && arg.length() && arg.length() <= FIELD) {
    setField(LOC_MODEL, arg.c_str());
    save();
  } else if (cmd == "wp") {
    writeProtect = arg == "on";
    save();
  } else if (cmd == "log") {
    logReads = arg == "on";
  } else if (cmd == "reset") {
    defaults();
    save();
  } else if (cmd != "show" && cmd.length()) {
    Serial.println("commands: show | id <text> | model <text> | wp on|off | log on|off | reset");
    return;
  }
  show();
}

void setup() {
  setCpuFrequencyMhz(80);                      // radio stays off: ~30 mA from the ribbon's +5 V
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  pinMode(PIN_ADDR, INPUT_PULLDOWN);           // unconnected reads as the left mount
  pinMode(PIN_MIN, INPUT_PULLUP);              // R3 on the board; an open input reads HIGH
  attachInterrupt(digitalPinToInterrupt(PIN_MIN), onMinEdge, CHANGE);
  Serial.begin(115200);
  prefs.begin("ot2eeprom", false);
  load();
  i2cAddr = digitalRead(PIN_ADDR) ? ADDR_RIGHT : ADDR_LEFT;
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.begin(i2cAddr, PIN_SDA, PIN_SCL, 100000);   // slave; the robot clocks at 10 kHz
  delay(200);
  show();
}

void loop() {
  static uint32_t seenReads = 0, ledOff = 0;
  static String line;
  while (edgeTail != edgeHead) {               // home switch edges, oldest first
    uint32_t t = edgeT[edgeTail & 31];
    uint8_t v = edgeV[edgeTail & 31];
    edgeTail++;
    nEdges++;
    bool bounce = nEdges > 1 && t - lastEdgeMs < 5;
    if (bounce) nBounces++;
    Serial.printf("%10lu ms  switch %s%s\n", (unsigned long)t, v ? "HOME (open)" : "rest (closed)",
                  bounce ? "  [bounce]" : "");
    lastEdgeMs = t;
  }
  if (nReads != seenReads) {                   // blink on bus activity
    if (logReads) Serial.printf("%10lu ms  read 0x%02X\n", (unsigned long)millis(), lastPtr);
    seenReads = nReads;
    digitalWrite(PIN_LED, LOW);
    ledOff = millis() + 30;
  }
  if (ledOff && millis() > ledOff) {
    digitalWrite(PIN_LED, HIGH);
    ledOff = 0;
  }
  if (dirty && millis() - lastWrite > 100) save();   // robot wrote a block: persist when quiet
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (line.length()) command(line);
      line = "";
    } else if (line.length() < 80) {
      line += c;
    }
  }
}

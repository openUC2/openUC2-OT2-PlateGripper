/*
  OT-2 pipette identity writer for a 24C02 (or AT24MAC402) EEPROM.

  The OT-2 recognises an instrument in a mount only if the EEPROM on the
  pipette's 2x5 ribbon header answers on I2C with this layout
  (SmoothiewareOT, src/modules/utils/instrument/Instrument.h):
    0x00  8 bytes   signature "opentron"
    0x30  32 bytes  ID     (ASCII, zero padded) - key for calibration and settings
    0x60  32 bytes  MODEL  (ASCII, zero padded) - e.g. "p20_single_v2.2"
    0xA0  32 bytes  DATA   (unused by the robot software)
  The robot reads it at 10 kHz: write the pointer (STOP), 10 ms, read 8 bytes (STOP). This sketch
  reads the same way, so it also tests the ESP32-S3 emulator (../ot2_eeprom_emulator_esp32s3,
  wired directly at 3.3 V: SDA-SDA, SCL-SCL, GND, 4.7 k pull-ups; its D0 low = 0x50).
  Left mount = 0x50 (A0 low), right mount = 0x51 (A0 high, pin 3 of the ribbon).

  Wiring for programming (off the robot), Seeed XIAO RP2040 / SAMD21 / any Arduino:
    XIAO 3V3 -> EEPROM VCC (ribbon pin 6)      XIAO GND -> GND (pin 5) and A0 (pin 3)
    XIAO SDA -> SDA (pin 2)                    XIAO SCL -> SCL (pin 1)
    4.7 k pull-ups SDA/SCL to 3V3; EEPROM WP tied to GND on the tool board.
  Open the serial monitor at 115200: it prints the current content; send 'w' to write,
  'r' to read again.
*/
#include <Wire.h>

static const uint8_t EEPROM_ADDR = 0x50;          // A0 = GND while programming
static const char *SIGNATURE = "opentron";
static const char *PIPETTE_ID = "P20SV222026103001"; // unique per tool; the OT-2 stores settings under it
static const char *PIPETTE_MODEL = "p20_single_v2.2"; // P20 Single-Channel GEN2 (alt: "p10_single_v1.5" = GEN1 P10)

static const uint8_t LOC_SIGNATURE = 0x00;
static const uint8_t LOC_ID = 0x30;
static const uint8_t LOC_MODEL = 0x60;
static const uint8_t LOC_DATA = 0xA0;
static const uint8_t FIELD_LEN = 32;
static const uint8_t PAGE = 8;                    // 24C02 page size

bool writeBlock(uint8_t loc, const uint8_t *buf, uint8_t len) {
  for (uint8_t off = 0; off < len; off += PAGE) {
    Wire.beginTransmission(EEPROM_ADDR);
    Wire.write(loc + off);
    for (uint8_t i = 0; i < PAGE && off + i < len; i++) Wire.write(buf[off + i]);
    if (Wire.endTransmission() != 0) return false;
    delay(10);                                    // write cycle (5 ms max for 24C02)
  }
  return true;
}

bool readBlock(uint8_t loc, uint8_t *buf, uint8_t len) {
  for (uint8_t off = 0; off < len; off += PAGE) {
    Wire.beginTransmission(EEPROM_ADDR);
    Wire.write(loc + off);
    if (Wire.endTransmission() != 0) return false;    // STOP, then read: like the robot
    delay(10);
    uint8_t n = Wire.requestFrom((int)EEPROM_ADDR, (int)PAGE);
    for (uint8_t i = 0; i < n && off + i < len; i++) buf[off + i] = Wire.read();
  }
  return true;
}

void field(const char *s, uint8_t *out) {
  memset(out, 0, FIELD_LEN);
  strncpy((char *)out, s, FIELD_LEN);
}

void dump() {
  uint8_t sig[8], id[FIELD_LEN + 1] = {0}, model[FIELD_LEN + 1] = {0};
  if (!readBlock(LOC_SIGNATURE, sig, 8) || !readBlock(LOC_ID, id, FIELD_LEN) ||
      !readBlock(LOC_MODEL, model, FIELD_LEN)) {
    Serial.println("no EEPROM answering at 0x50 - check wiring, A0 to GND, pull-ups");
    return;
  }
  bool ok = memcmp(sig, SIGNATURE, 8) == 0;
  Serial.print("signature: "); Serial.println(ok ? "ok (opentron)" : "missing");
  Serial.print("id:        "); Serial.println((char *)id);
  Serial.print("model:     "); Serial.println((char *)model);
}

void writeAll() {
  uint8_t buf[FIELD_LEN];
  bool ok = writeBlock(LOC_SIGNATURE, (const uint8_t *)SIGNATURE, 8);
  field(PIPETTE_ID, buf);    ok &= writeBlock(LOC_ID, buf, FIELD_LEN);
  field(PIPETTE_MODEL, buf); ok &= writeBlock(LOC_MODEL, buf, FIELD_LEN);
  memset(buf, 0, FIELD_LEN); ok &= writeBlock(LOC_DATA, buf, FIELD_LEN);
  Serial.println(ok ? "written" : "write failed (WP high?)");
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  Wire.begin();
  Wire.setClock(100000);
  dump();
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 'w') { writeAll(); dump(); }
  if (c == 'r') dump();
}

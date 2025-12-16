#include <Wire.h>

#define BQ_ADDR 0x6B   // BQ25713 I2C address

bool writeReg16(uint8_t lsbReg, uint16_t value) {
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(lsbReg);
  Wire.write(value & 0xFF);            // LSB
  Wire.write((value >> 8) & 0xFF);     // MSB
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    Serial.print("writeReg16 err reg 0x");
    Serial.print(lsbReg, HEX);
    Serial.print(": ");
    Serial.println(err);
    return false;
  }
  delay(3);
  return true;
}

uint16_t readReg16(uint8_t lsbReg) {
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(lsbReg);
  uint8_t err = Wire.endTransmission(false); // repeated START
  if (err != 0) {
    Serial.print("readReg16 setptr err reg 0x");
    Serial.print(lsbReg, HEX);
    Serial.print(": ");
    Serial.println(err);
    return 0xFFFF;
  }
  delay(2);

  uint8_t n = Wire.requestFrom((uint8_t)BQ_ADDR, (uint8_t)2);
  if (n < 2) {
    Serial.print("readReg16 short read reg 0x");
    Serial.print(lsbReg, HEX);
    Serial.print(" got ");
    Serial.println(n);
    return 0xFFFF;
  }

  uint16_t v = Wire.read();            // LSB
  v |= ((uint16_t)Wire.read()) << 8;   // MSB
  return v;
}

void bq_init() {
  Serial.println("=== BQ25713 INIT ===");

  // 0) Disable watchdog / safety timer so config doesn't time out
  writeReg16(0x2E, 0x0000);

  // 1) Make sure CHRG_INHIBIT = 0
  uint16_t chargeOpt0 = readReg16(0x00);
  Serial.print("ChargeOption0 before: 0x"); Serial.println(chargeOpt0, HEX);
  if (chargeOpt0 != 0xFFFF) {
    chargeOpt0 &= ~0x0001;   // clear bit0
    writeReg16(0x00, chargeOpt0);
  }
  uint16_t chargeOpt0_after = readReg16(0x00);
  Serial.print("ChargeOption0 after:  0x"); Serial.println(chargeOpt0_after, HEX);

  // 2) VBAT target = 7.20 V (code = V/0.008 = 900, stored at bits [13:3])
  uint16_t vbat_code = 900;
  uint16_t vbat_raw  = vbat_code << 3;
  writeReg16(0x04, vbat_raw);

  // 3) VSYS_MIN = 6.40 V (same shifted encoding we validated)
  uint16_t vsys_code = 800;           // 6.4 / 0.008
  uint16_t vsys_raw  = vsys_code << 3;
  writeReg16(0x0C, vsys_raw);

  // 4) ICHG ≈ 1.47 A (code = I/0.064 = 23, stored at bits [12:6])
  uint16_t ichg_code = 23;
  uint16_t ichg_raw  = ichg_code << 6;
  writeReg16(0x02, ichg_raw);

  // Read back once
  uint16_t vbat_read = readReg16(0x04);
  uint16_t vsys_read = readReg16(0x0C);
  uint16_t ichg_read = readReg16(0x02);

  Serial.print("VBAT RAW: 0x"); Serial.println(vbat_read, HEX);
  Serial.print("VSYS RAW: 0x"); Serial.println(vsys_read, HEX);
  Serial.print("ICHG RAW: 0x"); Serial.println(ichg_read, HEX);

  if (vbat_read != 0xFFFF) {
    Serial.print("VBAT cfg (V): ");
    Serial.println((vbat_read >> 3) * 0.008f, 3);
  }
  if (vsys_read != 0xFFFF) {
    Serial.print("VSYS_MIN cfg (V): ");
    Serial.println((vsys_read >> 3) * 0.008f, 3);
  }
  if (ichg_read != 0xFFFF) {
    Serial.print("ICHG cfg (A): ");
    Serial.println((ichg_read >> 6) * 0.064f, 3);
  }

  Serial.println("=== INIT DONE ===");
}

unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin();
  Wire.setClock(100000);

  bq_init();
}

void loop() {
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();

    // Start ADC conversion on VSYS/VBAT
    uint16_t adc_opt = 0;
    uint8_t adc_lsb = 0;
    uint8_t adc_msb = 0;

    adc_lsb |= (1 << 1); // EN_ADC_VSYS
    adc_lsb |= (1 << 0); // EN_ADC_VBAT

    adc_msb |= (1 << 6); // ADC_START

    adc_opt = ((uint16_t)adc_msb << 8) | adc_lsb;
    writeReg16(0x3A, adc_opt);

    delay(15); // allow conversion

    uint16_t adc_vsys_vbat = readReg16(0x2C);
    uint16_t status0 = readReg16(0x20);
    uint16_t status1 = readReg16(0x21);

    uint8_t vsys_code = (adc_vsys_vbat >> 8) & 0xFF;
    uint8_t vbat_code = adc_vsys_vbat & 0xFF;

    float vsys_adc = 2.88 + vsys_code * 0.064;
    float vbat_adc = 2.88 + vbat_code * 0.064;

    Serial.println("--- TICK ---");
    Serial.print("ADCVSYSVBAT RAW: 0x"); Serial.println(adc_vsys_vbat, HEX);
    Serial.print("VSYS_ADC (V): "); Serial.println(vsys_adc, 3);
    Serial.print("VBAT_ADC (V): "); Serial.println(vbat_adc, 3);
    Serial.print("STATUS0: 0x"); Serial.println(status0, HEX);
    Serial.print("STATUS1: 0x"); Serial.println(status1, HEX);
  }
}

#include <M5Unified.h>
#include <Wire.h>

// SparkFun ATECC library
#include <SparkFun_ATECCX08a_Arduino_Library.h>

static const uint8_t I2C_SDA  = 32;
static const uint8_t I2C_SCL  = 33;
static const uint8_t ECC_ADDR = 0x35; // ATECC608B-TNGTLS default

ATECCX08A atecc;

void printHexByte(uint8_t b) {
  if (b < 0x10) Serial.print('0');
  Serial.print(b, HEX);
}

void drawCenteredLine(int16_t y, const String &text, int textSize = 2, uint16_t color = 0xFFFF) {
  auto &lcd = M5.Display;
  lcd.setTextSize(textSize);
  lcd.setTextColor(color);
  int16_t w = lcd.textWidth(text);
  int16_t x = (lcd.width() - w) / 2;
  lcd.setCursor(x, y);
  lcd.print(text);
}

void showError(const String &msg) {
  Serial.println(msg);
  auto &lcd = M5.Display;
  lcd.clear();
  drawCenteredLine(20, "SECURE ELEMENT", 2, 0xF800);
  drawCenteredLine(50, "TEST FAILED", 2, 0xF800);
  lcd.setTextSize(1);
  lcd.setTextColor(0xFFFF);
  lcd.setCursor(10, 80);
  lcd.print(msg);
}

bool scanFor(uint8_t target) {
  bool found = false;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.printf("I2C device found at 0x%02X\n", addr);
      if (addr == target) found = true;
    }
  }
  return found;
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);

  Serial.begin(115200);
  delay(400);

  auto &lcd = M5.Display;
  lcd.setRotation(1);
  lcd.clear();
  drawCenteredLine(18, "Secure Element", 2);
  drawCenteredLine(42, "Self Test", 2);
  delay(600);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  Serial.println("\n=== Secure Element Test ===");
  lcd.clear();
  drawCenteredLine(20, "Scanning I2C...", 2);

  if (!scanFor(ECC_ADDR)) {
    showError("ATECC608B-TNGTLS not found at 0x35");
    return;
  }

  Serial.println("ATECC608 detected at 0x35.");
  lcd.clear();
  drawCenteredLine(20, "I2C OK", 2, 0x07E0);
  drawCenteredLine(45, "0x35 detected", 2);
  delay(600);

  lcd.clear();
  drawCenteredLine(20, "Init ATECC608...", 2);
  if (!atecc.begin(ECC_ADDR, Wire, Serial)) {
    showError("SparkFun begin() failed.");
    return;
  }

  Serial.println("ATECC608 init OK.");
  lcd.clear();
  drawCenteredLine(20, "ATECC608", 2);
  drawCenteredLine(45, "Init OK", 2, 0x07E0);
  delay(500);

  if (!atecc.readConfigZone(false)) {
    showError("Cannot read config zone.");
    return;
  }

  Serial.print("Revision: ");
  for (int i = 0; i < 4; i++) {
    printHexByte(atecc.revisionNumber[i]);
    Serial.print(' ');
  }
  Serial.println();

  Serial.print("Serial  : ");
  for (int i = 0; i < 9; i++) {
    printHexByte(atecc.serialNumber[i]);
    Serial.print(i < 8 ? ':' : '\n');
  }

  Serial.print("Config Lock : ");
  Serial.println(atecc.configLockStatus ? "Locked" : "Not locked");
  Serial.print("Data/OTP    : ");
  Serial.println(atecc.dataOTPLockStatus ? "Locked" : "Not locked");
  Serial.print("Slot0 lock  : ");
  Serial.println(atecc.slot0LockStatus ? "Unlocked (bit=1)" : "Locked (bit=0)");

  // Professor’s fix:
  // SparkFun example uses:
  //   if (atecc.configLockStatus && atecc.dataOTPLockStatus && atecc.slot0LockStatus)
  // But in the TNG datasheet, SlotLocked bit = 0 when locked.
  // So use '!' on slot0LockStatus:
  if (atecc.configLockStatus && atecc.dataOTPLockStatus && !atecc.slot0LockStatus) {
    Serial.println("\nConfig and Data/OTP are locked and Slot0 is locked: generating public key...");
    if (!atecc.generatePublicKey()) {
      showError("Failure to generate device public key");
      return;
    }
  } else {
    Serial.println("\nDevice not fully personalized/locked, skipping public key generation.");
  }

  lcd.clear();
  drawCenteredLine(10, "SECURE ELEMENT", 2, 0x07E0);
  drawCenteredLine(34, "TEST PASSED", 2, 0x07E0);
  lcd.setTextSize(1);
  lcd.setTextColor(0xFFFF);
  lcd.setCursor(10, 60);
  lcd.print("Serial: ");
  for (int i = 0; i < 9; i++) {
    if (atecc.serialNumber[i] < 0x10) lcd.print('0');
    lcd.print(atecc.serialNumber[i], HEX);
    if (i < 8) lcd.print(':');
  }

  lcd.setCursor(10, 78);
  lcd.print("Cfg: ");
  lcd.print(atecc.configLockStatus ? "Locked" : "Not locked");
  lcd.setCursor(10, 92);
  lcd.print("Data/OTP: ");
  lcd.print(atecc.dataOTPLockStatus ? "Locked" : "Not locked");
}

void loop() {
  M5.update();
}

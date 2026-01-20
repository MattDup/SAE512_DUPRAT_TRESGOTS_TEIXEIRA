// LoRa_ATECC_ProvisionKey.ino
#include <M5Unified.h>
#include <Wire.h>
#include <SparkFun_ATECCX08a_Arduino_Library.h>

static const uint8_t I2C_SDA  = 32;
static const uint8_t I2C_SCL  = 33;
static const uint8_t ECC_ADDR = 0x35;

ATECCX08A atecc;

// 32-byte shared LoRa key (example). Use the SAME bytes on both good M5s.
const uint8_t LORA_KEY[32] = {
  0x7A, 0xF4, 0x39, 0x81, 0xC2, 0x55, 0x9E, 0x01,
  0x24, 0xB8, 0x93, 0x6D, 0x11, 0xA7, 0xCB, 0xEF,
  0x52, 0x3C, 0x68, 0x9D, 0xFA, 0x0B, 0x7E, 0x19,
  0x40, 0x22, 0xDD, 0x56, 0x8A, 0xBC, 0x33, 0x77
};

// ---- Slot 8 helpers ----
static uint16_t slot8AddressForBlock(uint8_t block)
{
  // Slot 8, 32-byte block addressing. 
  return ((uint16_t)8 << 3) | ((uint16_t)block << 8);
}

bool writeDataSlot8Block(ATECCX08A &atecc, uint8_t block, const uint8_t *data32)
{
  if (block >= 13) return false;
  uint16_t addr = slot8AddressForBlock(block);
  if (!atecc.write(ZONE_DATA, addr, (uint8_t *)data32, 32))
  {
    Serial.print("write block "); Serial.print(block); Serial.println(" failed");
    return false;
  }
  return true;
}

void printHex(const uint8_t *buf, size_t len)
{
  for (size_t i = 0; i < len; i++) {
    if (i && (i % 16 == 0)) Serial.println();
    if (buf[i] < 0x10) Serial.print('0');
    Serial.print(buf[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(1);
  M5.Display.clear();
  M5.Display.setCursor(10, 20);
  M5.Display.println("Provision LoRa key (slot 8)");

  Serial.begin(115200);
  delay(500);
  Serial.println("\n--- ATECC Provision LoRa key (slot 8) ---");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  if (!atecc.begin(ECC_ADDR, Wire, Serial)) {
    Serial.println("ERROR: atecc.begin() failed");
    M5.Display.setCursor(10, 50);
    M5.Display.println("ATECC begin failed");
    return;
  }

  if (!atecc.readConfigZone(false)) {
    Serial.println("ERROR: read onfigZone() failed");
    M5.Display.setCursor(10, 50);
    M5.Display.println("readConfig failed");
    return;
  }

  Serial.print("ConfigLock=");   Serial.println(atecc.configLockStatus);
  Serial.print("DataOTPLock=");  Serial.println(atecc.dataOTPLockStatus);
  Serial.print("Slot0LockBit="); Serial.println(atecc.slot0LockStatus);

  Serial.println("Writing 32-byte LoRa key into slot 8 block 0...");
  Serial.print("Key = ");
  printHex(LORA_KEY, 32);

  if (!writeDataSlot8Block(atecc, 0, LORA_KEY)) {
    Serial.println("ERROR writing key to slot 8 block 0");
    M5.Display.setCursor(10, 50);
    M5.Display.println("Write failed");
    return;
  }

  Serial.println("Provisioning complete. Slot 8 block 0 now holds LoRa key.");
  M5.Display.setCursor(10, 50);
  M5.Display.println("Done. Power-cycle device.");
}

void loop() {
  M5.update();
}

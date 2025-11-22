#include <M5Unified.h>
#include <Wire.h>
#include <SparkFun_ATECCX08a_Arduino_Library.h>

static const uint8_t I2C_SDA  = 32;
static const uint8_t I2C_SCL  = 33;
static const uint8_t ECC_ADDR = 0x35;

ATECCX08A atecc;

// Include the subscriber key blob with length prefix
// Make sure these symbol names match what you saw in m5sub_key_der.h
#include "m5sub_key_der.h"

// If your header defines:
//
//   unsigned char m5sub_display_withlen_bin[];
//   unsigned int  m5sub_display_withlen_bin_len;
//
// keep these typedefs as-is. If the names are slightly different,
// change them here.
const uint8_t *sub_withlen_bin = m5sub_display_withlen_bin;
const size_t sub_withlen_bin_len = m5sub_display_withlen_bin_len;

// Helpers for slot 8
static uint16_t slot8AddressForBlock(uint8_t block)
{
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

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(1);
  M5.Display.clear();
  M5.Display.setCursor(10, 20);
  M5.Display.println("Provision slot 8 (SUB)");

  Serial.begin(115200);
  delay(500);
  Serial.println("\n--- ATECC Provision (Subscriber, v2) ---");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  if (!atecc.begin(ECC_ADDR, Wire, Serial)) {
    Serial.println("ERROR: atecc.begin() failed");
    return;
  }

  if (!atecc.readConfigZone(false)) {
    Serial.println("ERROR: readConfigZone() failed");
    return;
  }

  Serial.print("ConfigLock="); Serial.println(atecc.configLockStatus);
  Serial.print("DataOTPLock="); Serial.println(atecc.dataOTPLockStatus);
  Serial.print("Slot0Lock="); Serial.println(atecc.slot0LockStatus);

  Serial.print("Key blob length (with prefix) = ");
  Serial.println(sub_withlen_bin_len);

  size_t offset = 0;
  uint8_t block = 0;
  while (offset < sub_withlen_bin_len && block < 13) {
    uint8_t buf32[32];
    memset(buf32, 0, sizeof(buf32));

    size_t remaining = sub_withlen_bin_len - offset;
    size_t toCopy = (remaining > 32) ? 32 : remaining;

    memcpy(buf32, &sub_withlen_bin[offset], toCopy);

    Serial.print("Writing slot8 block ");
    Serial.print(block);
    Serial.print(", bytes ");
    Serial.println(toCopy);

    if (!writeDataSlot8Block(atecc, block, buf32)) {
      Serial.println("ERROR writing block, aborting");
      return;
    }

    offset += toCopy;
    block++;
  }

  Serial.println("Provisioning complete.");
  M5.Display.setCursor(10, 50);
  M5.Display.println("Done. Power-cycle.");
}

void loop() {
  M5.update();
}

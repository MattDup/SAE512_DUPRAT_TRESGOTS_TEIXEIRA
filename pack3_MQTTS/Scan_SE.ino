// Découverte et test du secure element ATECC608B-TNGTLS sur M5
// - Scan du bus I2C pour trouver l'adresse 0x35
// - Initialisation de la librairie SparkFun
// - Lecture de la zone de configuration (revision, serial, locks)
// - Essai de génération de la clé publique si le device est "locké" comme attendu

#include <M5Unified.h>
#include <Wire.h>

// SparkFun ATECC library
#include <SparkFun_ATECCX08a_Arduino_Library.h>

// Brochage I2C utilisé pour le secure element
static const uint8_t I2C_SDA  = 32;
static const uint8_t I2C_SCL  = 33;
// Adresse I2C de l'ATECC608B-TNGTLS (valeur usine)
static const uint8_t ECC_ADDR = 0x35; // ATECC608B-TNGTLS default

// Instance de l'objet ATECC de la librairie SparkFun
ATECCX08A atecc;

// Affiche un octet en hexadécimal sur le port série, sur 2 digits
void printHexByte(uint8_t b) {
  if (b < 0x10) Serial.print('0');
  Serial.print(b, HEX);
}

// Affichage centré d'une ligne de texte sur l'écran M5
void drawCenteredLine(int16_t y, const String &text, int textSize = 2, uint16_t color = 0xFFFF) {
  auto &lcd = M5.Display;
  lcd.setTextSize(textSize);
  lcd.setTextColor(color);
  int16_t w = lcd.textWidth(text);
  int16_t x = (lcd.width() - w) / 2;
  lcd.setCursor(x, y);
  lcd.print(text);
}

// Affiche un message d'erreur à la fois sur le port série et sur l'écran
void showError(const String &msg) {
  Serial.println(msg);
  auto &lcd = M5.Display;
  lcd.clear();
  // Titre rouge "SECURE ELEMENT" et "TEST FAILED"
  drawCenteredLine(20, "SECURE ELEMENT", 2, 0xF800);
  drawCenteredLine(50, "TEST FAILED", 2, 0xF800);
  lcd.setTextSize(1);
  lcd.setTextColor(0xFFFF);
  lcd.setCursor(10, 80);
  lcd.print(msg);   // détail de l'erreur
}

// Scan I2C complet (1..126) et retour true si l'adresse "target" est trouvée
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
  // Écran d'introduction du test
  drawCenteredLine(18, "Secure Element", 2);
  drawCenteredLine(42, "Self Test", 2);
  delay(600);

  // Initialisation du bus I2C pour l'ATECC608B
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  Serial.println("\n=== Secure Element Test ===");
  lcd.clear();
  drawCenteredLine(20, "Scanning I2C...", 2);

  // Scan I2C pour trouver l'ATECC608B à l'adresse 0x35
  if (!scanFor(ECC_ADDR)) {
    // Si l'adresse 0x35 n'est pas vue, on considère que l'ATECC est absent
    showError("ATECC608B-TNGTLS not found at 0x35");
    return;
  }

  Serial.println("ATECC608 detected at 0x35.");
  lcd.clear();
  drawCenteredLine(20, "I2C OK", 2, 0x07E0);
  drawCenteredLine(45, "0x35 detected", 2);
  delay(600);

  // Initialisation de la librairie SparkFun pour l'ATECC
  lcd.clear();
  drawCenteredLine(20, "Init ATECC608...", 2);
  if (!atecc.begin(ECC_ADDR, Wire, Serial)) {
    // Problème d'initialisation (commande Wake ou config)
    showError("SparkFun begin() failed.");
    return;
  }

  Serial.println("ATECC608 init OK.");
  lcd.clear();
  drawCenteredLine(20, "ATECC608", 2);
  drawCenteredLine(45, "Init OK", 2, 0x07E0);
  delay(500);

  // Lecture de la zone de configuration (remplit les champs revisionNumber, serialNumber, lock statuses, etc.)
  if (!atecc.readConfigZone(false)) {
    showError("Cannot read config zone.");
    return;
  }

  // Affichage de la révision
  Serial.print("Revision: ");
  for (int i = 0; i < 4; i++) {
    printHexByte(atecc.revisionNumber[i]);
    Serial.print(' ');
  }
  Serial.println();

  // Affichage du numéro de série complet (9 octets)
  Serial.print("Serial  : ");
  for (int i = 0; i < 9; i++) {
    printHexByte(atecc.serialNumber[i]);
    Serial.print(i < 8 ? ':' : '\n');
  }

  // État des locks (config, data/OTP, slot0)
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
  // Ici, on considère que le device est "pleinement provisionné" si :
  // - configLockStatus == true
  // - dataOTPLockStatus == true
  // - slot0LockStatus == false (bit 0 => slot verrouillé)
  if (atecc.configLockStatus && atecc.dataOTPLockStatus && !atecc.slot0LockStatus) {
    Serial.println("\nConfig and Data/OTP are locked and Slot0 is locked: generating public key...");
    // On demande à l'ATECC de générer la clé publique correspondant à la clé privée slot 0
    if (!atecc.generatePublicKey()) {
      showError("Failure to generate device public key");
      return;
    }
  } else {
    Serial.println("\nDevice not fully personalized/locked, skipping public key generation.");
  }

  // Si tout est OK, on affiche un écran de test réussi avec quelques infos de config
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

  // Affichage des locks sur l'écran (config et Data/OTP)
  lcd.setCursor(10, 78);
  lcd.print("Cfg: ");
  lcd.print(atecc.configLockStatus ? "Locked" : "Not locked");
  lcd.setCursor(10, 92);
  lcd.print("Data/OTP: ");
  lcd.print(atecc.dataOTPLockStatus ? "Locked" : "Not locked");
}

void loop() {
  // Boucle vide, on laisse M5Unified gérer les events si nécessaire
  M5.update();
}

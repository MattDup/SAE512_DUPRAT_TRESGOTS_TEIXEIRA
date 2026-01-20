// ====== ETAPE 1 : ID UNIQUE ATECC608B-TNGTLS ======
#include <Wire.h>
#include <SparkFun_ATECCX08a_Arduino_Library.h>

#define SDA_PIN   32     // M5StickC Plus 2 / Core2 Port A
#define SCL_PIN   33
#define ECC_ADDR  0x35   // Adresse 7 bits par défaut du TNGTLS

ATECCX08A atecc;

void printHex(uint8_t b){ if(b<0x10) Serial.print('0'); Serial.print(b, HEX); }

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // 100 kHz pour être tolérant

  Serial.println("\n=== ETAPE 1 : Lecture ID unique (serial) ===");

  if (!atecc.begin(ECC_ADDR, Wire, Serial)) {
    Serial.println("ATECC non détecté (adresse 0x35). Vérifie Port A et l'alim.");
    while (1) delay(1000);
  }
  Serial.println("Secure Element réveillé.");

  // Lecture de la zone config pour remplir revisionNumber[] et serialNumber[]
  if (!atecc.readConfigZone(false)) {
    Serial.println("Echec readConfigZone().");
    while (1) delay(1000);
  }

  Serial.print("Revision: ");
  for (int i=0;i<4;i++) printHex(atecc.revisionNumber[i]);
  Serial.println();

  Serial.print("Serial  : ");
  for (int i=0;i<9;i++)  printHex(atecc.serialNumber[i]);
  Serial.println();

  Serial.println("\nCritère de réussite : vous voyez 9 octets hex pour 'Serial'.");
}

void loop() {}

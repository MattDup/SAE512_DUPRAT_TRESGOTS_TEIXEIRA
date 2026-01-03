// Subscriber sécurisé avec ATECC608B :
// - La clé privée TLS du subscriber est stockée dans le slot 8 (même format que le publisher).
// - Au démarrage, lecture slot 8 -> DER -> Base64 -> PEM.
// - Utilisation de cette clé PEM dans WiFiClientSecure pour l'authentification mutuelle TLS.

#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Wire.h>

// ATECC608B (SparkFun)
#include <SparkFun_ATECCX08a_Arduino_Library.h>

// Brochage I2C de l'ATECC côté subscriber
static const uint8_t I2C_SDA  = 32;
static const uint8_t I2C_SCL  = 33;
static const uint8_t ECC_ADDR = 0x35;
ATECCX08A atecc;

// -------- WIFI ----------
const char* WIFI_SSID = "Livebox-D580";
const char* WIFI_PASS = "jyxvet-0qyrPi";

// -------- MQTT ----------
const char* MQTT_HOST = "192.168.1.12";
const uint16_t MQTT_PORT = 8883;
const char* TOPIC = "m5go/env";

// -------- CA CERT (same CA as publisher) ----------
// CA commune au broker, au publisher et au subscriber
static const char ca_cert[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIBwDCCAWYCCQDCLzlU+p80FTAKBggqhkjOPQQDAjBoMQswCQYDVQQGEwJGUjES
MBAGA1UECAwJT2NjaXRhbmllMREwDwYDVQQHDAhUb3Vsb3VzZTEMMAoGA1UECgwD
TGFiMQ0wCwYDVQQLDARNUVRUMRUwEwYDVQQDDAxMb2NhbE1RVFQtQ0EwHhcNMjUx
MTE4MDc1ODQ0WhcNMzUxMTE2MDc1ODQ0WjBoMQswCQYDVQQGEwJGUjESMBAGA1UE
CAwJT2NjaXRhbmllMREwDwYDVQQHDAhUb3Vsb3VzZTEMMAoGA1UECgwDTGFiMQ0w
CwYDVQQLDARNUVRUMRUwEwYDVQQDDAxMb2NhbE1RVFQtQ0EwWTATBgcqhkjOPQIB
BggqhkjOPQMBBwNCAATPDXo6/5pAbxyrHaQpEHR33AR1vWDcJk+H37T/oMOgMpRL
+ipsBETdbz46Dz2v8/da6NKXzeaGnVFyZIeeqJPjMAoGCCqGSM49BAMCA0gAMEUC
IGPrNzv/LGSCuUrBLCHPUTrDWnm1T7BtV8I2J4lruIuGAiEAknzvAWYCkJyYuq0S
yHP8+qRKegc2kbXrHI5Fw00bM40=
-----END CERTIFICATE-----
)EOF";

// -------- CLIENT CERT (subscriber: m5sub-display) ----------
// Certificat X.509 du subscriber (m5sub-display)
static const char client_cert[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIBxDCCAWoCCQCU9fxQbpMYETAKBggqhkjOPQQDAjBoMQswCQYDVQQGEwJGUjES
MBAGA1UECAwJT2NjaXRhbmllMREwDwYDVQQHDAhUb3Vsb3VzZTEMMAoGA1UECgwD
TGFiMQ0wCwYDVQQLDARNUVRUMRUwEwYDVQQDDAxMb2NhbE1RVFQtQ0EwHhcNMjUx
MTE4MDc1OTI2WhcNMjYxMTE4MDc1OTI2WjBsMQswCQYDVQQGEwJGUjESMBAGA1UE
CAwJT2NjaXRhbmllMREwDwYDVQQHDAhUb3Vsb3VzZTEMMAoGA1UECgwDTGFiMRAw
DgYDVQQLDAdEZXZpY2VzMRYwFAYDVQQDDA1tNXN1Yi1kaXNwbGF5MFkwEwYHKoZI
zj0CAQYIKoZIzj0DAQcDQgAEIVXU/cVVe9o47aoqEAXK1lsUeJPzBN4d2LT15j1a
EUpUB/7FhUJduZpGUYSTZgpx9eyR5HtSdn6GXKCtI4EzjTAKBggqhkjOPQQDAgNI
ADBFAiAYnrRyQFHy2E7k7O+zt7YGvssxEH2g+c1dNhaiTe58awIhAOzbfpHlcbnH
pH8/wnXlCS/Scki7fXG1vK6yimqflcky
-----END CERTIFICATE-----
)EOF";

// ------------ MQTT CLIENT ------------
// Client TLS + MQTT
WiFiClientSecure tlsClient;
PubSubClient mqtt(tlsClient);

// ------------ UI COLORS ------------
static const uint16_t COLOR_BG      = 0x0000; // black
static const uint16_t COLOR_CARD    = 0x0005; // deep blue
static const uint16_t COLOR_ACCENT  = 0x07FF; // cyan
static const uint16_t COLOR_TEXT    = 0xFFFF; // white
static const uint16_t COLOR_OK      = 0x07E0; // green
static const uint16_t COLOR_WARN    = 0xF800; // red

// ------------ Base64 helper ------------
// Table Base64 utilisée pour l'encodage DER -> PEM
static const char b64_table[] =
  "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

// Encodage Base64 générique
String base64Encode(const uint8_t *data, size_t len) {
  String out;
  size_t i = 0;

  while (i < len) {
    size_t remain = len - i;

    uint32_t octet_a = data[i++];
    uint32_t octet_b = (remain > 1) ? data[i++] : 0;
    uint32_t octet_c = (remain > 2) ? data[i++] : 0;

    uint32_t triple = (octet_a << 16) | (octet_b << 8) | octet_c;

    out += b64_table[(triple >> 18) & 0x3F];
    out += b64_table[(triple >> 12) & 0x3F];
    out += (remain > 1) ? b64_table[(triple >> 6) & 0x3F] : '=';
    out += (remain > 2) ? b64_table[triple & 0x3F]       : '=';
  }

  return out;
}

// ---------- ATECC608B Slot 8 helpers ----------
// Calcul de l'adresse ATECC pour slot 8 / bloc "block"
static uint16_t slot8AddressForBlock(uint8_t block)
{
  return ((uint16_t)8 << 3) | ((uint16_t)block << 8);
}

// Lecture d'un bloc de 32 octets du slot 8
bool readDataSlot8Block(ATECCX08A &atecc, uint8_t block, uint8_t *out32)
{
  if (block >= 13) return false;

  uint16_t addr = slot8AddressForBlock(block);

  if (!atecc.read(ZONE_DATA, addr, 32, false)) {
    Serial.println("readDataSlot8Block: read() failed");
    return false;
  }

  // atecc.inputBuffer: [0]=count, [1..32]=data, [33..34]=CRC
  for (int i = 0; i < 32; i++) {
    out32[i] = atecc.inputBuffer[1 + i];
  }

  return true;
}

// Reconstruire la clé privée PEM à partir du blob [len][DER] en slot 8
bool loadPrivateKeyPEMFromATECC(ATECCX08A &atecc, String &outPem) {
  uint8_t slotData[416];

  // Lecture des 13 blocs du slot 8
  for (uint8_t b = 0; b < 13; b++) {
    if (!readDataSlot8Block(atecc, b, slotData + b * 32)) {
      Serial.printf("Error reading slot 8 block %u\n", b);
      return false;
    }
  }

  // Lecture de la longueur DER sur 2 octets
  uint16_t derLen = (slotData[0] << 8) | slotData[1];

  if (derLen == 0 || derLen > 414) {
    Serial.printf("Invalid DER length in slot 8: %u\n", derLen);
    return false;
  }

  const uint8_t *der = &slotData[2];

  if (der[0] != 0x30) {
    Serial.println("Warning: DER does not start with 0x30 (SEQUENCE)");
  }

  // Encodage Base64 puis ajout headers PEM
  String b64 = base64Encode(der, derLen);

  outPem = "-----BEGIN PRIVATE KEY-----\n";
  for (size_t i = 0; i < b64.length(); i += 64) {
    outPem += b64.substring(i, i + 64) + "\n";
  }
  outPem += "-----END PRIVATE KEY-----\n";

  return true;
}

// ------------ UI HELPERS ------------
// En-tête de l'écran du subscriber
void uiDrawHeader() {
  auto& d = M5.Display;
  d.fillScreen(COLOR_BG);
  d.fillRoundRect(6, 4, d.width() - 12, 40, 8, COLOR_CARD);
  d.setTextColor(COLOR_ACCENT, COLOR_CARD);
  d.setTextSize(1.5);
  d.setCursor(14, 20);
  d.print("M5Stack Secure ENV Subscriber");
}

// Parties statiques (Wi-Fi, MQTTS, texte topic)
void uiDrawStatic() {
  auto& d = M5.Display;

  d.fillRoundRect(6, 50, (d.width() / 2) - 9, 50, 8, COLOR_CARD);
  d.setTextColor(COLOR_TEXT, COLOR_CARD);
  d.setTextSize(1.5);
  d.setCursor(12, 60);
  d.print("Wi-Fi");

  int rx = (d.width() / 2) + 3;
  d.fillRoundRect(rx, 50, (d.width() / 2) - 9, 50, 8, COLOR_CARD);
  d.setCursor(rx + 6, 60);
  d.print("MQTTS");

  d.fillRoundRect(6, 106, d.width() - 12, 126, 8, COLOR_CARD);
  d.setCursor(12, 120);
  d.setTextColor(COLOR_ACCENT, COLOR_CARD);
  d.print("Data received from topic m5go/env");
}

// Affichage statut Wi-Fi
void uiSetWiFiStatus(const String& line, bool ok) {
  auto& d = M5.Display;
  int x = 12;
  int y = 80;
  int w = (d.width() / 2) - 24;
  d.fillRect(x, y, w, 10, COLOR_CARD);
  d.setCursor(x, y);
  d.setTextColor(ok ? COLOR_OK : COLOR_WARN, COLOR_CARD);
  d.print(line);
}

// Affichage statut MQTT
void uiSetMQTTStatus(const String& line, bool ok) {
  auto& d = M5.Display;
  int cardX = (d.width() / 2) + 3;
  int x = cardX + 6;
  int y = 80;
  int w = (d.width() / 2) - 24;
  d.fillRect(x, y, w, 10, COLOR_CARD);
  d.setCursor(x, y);
  d.setTextColor(ok ? COLOR_OK : COLOR_WARN, COLOR_CARD);
  d.print(line);
}

// Affichage des valeurs T/H/P
void uiSetSensorValues(float t, float h, float p) {
  auto& d = M5.Display;
  d.setTextColor(COLOR_TEXT, COLOR_CARD);
  d.setTextSize(2);

  d.fillRect(12, 150, d.width() - 24, 20, COLOR_CARD);
  d.setCursor(12, 150);
  d.print("T:");
  if (!isnan(t)) { d.print(t, 1); d.print("C"); }
  else d.print("--.-C");
  d.print("  H:");
  if (!isnan(h)) { d.print(h, 1); d.print("%"); }
  else d.print("--.-%");

  d.fillRect(12, 180, d.width() - 24, 20, COLOR_CARD);
  d.setCursor(12, 180);
  d.print("P:");
  if (!isnan(p)) { d.print(p, 1); d.print("hPa"); }
  else d.print("---.-hPa");
}

// Indication d'une dernière mise à jour
void uiSetLastUpdate() {
  auto& d = M5.Display;
  d.setTextSize(1.5);
  d.setTextColor(COLOR_ACCENT, COLOR_CARD);
  d.fillRect(12, 210, d.width() - 24, 12, COLOR_CARD);
  d.setCursor(12, 210);
  d.print("Last update received");
}

// ------------ WIFI / TLS ------------
// Connexion Wi-Fi
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uiSetWiFiStatus("Connecting...", false);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
  }
  uiSetWiFiStatus(WiFi.localIP().toString(), true);
}

// Clé privée PEM issue de l'ATECC
String g_clientKeyPem;

// Configuration TLS (CA, cert client, private key PEM)
void setupTLS() {
  tlsClient.setCACert(ca_cert);
  tlsClient.setCertificate(client_cert);
  tlsClient.setPrivateKey(g_clientKeyPem.c_str());
}

// ------------ MQTT CALLBACK ------------
// Callback appelée pour chaque message reçu sur les topics abonnés
void handleMessage(char* topic, byte* payload, unsigned int length) {
  static char buf[256];
  unsigned int len = (length < sizeof(buf)-1) ? length : sizeof(buf)-1;
  memcpy(buf, payload, len);
  buf[len] = '\0';

  String msg = String(buf);

  float t = NAN, h = NAN, p = NAN;
  // Parsing manuel du JSON par recherche de sous-chaînes
  int ti = msg.indexOf("\"temp_c\":");
  int hi = msg.indexOf("\"hum_pct\":");
  int pi = msg.indexOf("\"press_hpa\":");
  if (ti >= 0) t = msg.substring(ti + 9).toFloat();
  if (hi >= 0) h = msg.substring(hi + 10).toFloat();
  if (pi >= 0) p = msg.substring(pi + 12).toFloat();

  // Mise à jour visuelle
  uiSetSensorValues(t, h, p);
  uiSetLastUpdate();

  // Log série pour debug
  Serial.print("Msg [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(msg);
}

// ------------ MQTT ------------
// Connexion sécurisé au broker et abonnement au topic m5go/env
void connectMQTT() {
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(handleMessage);

  while (!mqtt.connected()) {
    String cid = "m5sub-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    uiSetMQTTStatus("Connecting...", false);

    if (mqtt.connect(cid.c_str())) {
      uiSetMQTTStatus("Connected", true);
      mqtt.subscribe(TOPIC);
    } else {
      uiSetMQTTStatus("Err:" + String(mqtt.state()), false);
      delay(1500);
    }
  }
}

// ------------ SETUP / LOOP ------------
// Setup du subscriber avec ATECC
void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(1);
  uiDrawHeader();
  uiDrawStatic();

  Serial.begin(115200);

  // I2C vers l'ATECC
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  Serial.println("Bringing up ATECC608B (subscriber)...");
  if (!atecc.begin(ECC_ADDR, Wire, Serial)) {
    Serial.println("ERROR: atecc.begin failed (TLS key will not load)");
  } else {
    if (!atecc.readConfigZone(false)) {
      Serial.println("ERROR: readConfigZone failed");
    } else {
      Serial.printf("ConfigLock=%d, DataOTPLock=%d, Slot0Lock=%d\n",
                    atecc.configLockStatus,
                    atecc.dataOTPLockStatus,
                    atecc.slot0LockStatus);
    }

    // Lecture de la clé privée spécifiquement provisionnée pour le subscriber
    if (!loadPrivateKeyPEMFromATECC(atecc, g_clientKeyPem)) {
      Serial.println("ERROR: could not load private key from ATECC");
    } else {
      Serial.println("Private key PEM built from ATECC slot 8.");
#ifdef DEBUG_PRINT_PEM
      // Affichage de la clé en PEM sur le port série (pour debug)
      Serial.println(g_clientKeyPem);
#endif
    }

    Serial.print("Key length from ATECC: ");
    Serial.println(g_clientKeyPem.length());
  }

  // Connexion Wi-Fi
  connectWiFi();

  // Si aucune clé n'a été lue, on ne tente pas la connexion TLS/MQTT
  if (g_clientKeyPem.length() == 0) {
    Serial.println("No key from ATECC; skipping TLS/MQTT connection.");
    return;
  }

  // Configuration TLS avec la clé lue, puis connexion MQTT
  setupTLS();
  connectMQTT();
}

void loop() {
  M5.update();

  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
    if (g_clientKeyPem.length() > 0) setupTLS();
  }

  if (g_clientKeyPem.length() > 0 && !mqtt.connected()) {
    connectMQTT();
  }

  mqtt.loop();
}

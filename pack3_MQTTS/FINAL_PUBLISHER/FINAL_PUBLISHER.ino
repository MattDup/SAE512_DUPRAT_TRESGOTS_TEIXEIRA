#define DEBUG_PRINT_PEM
#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Wire.h>

#include <Adafruit_SHT31.h>
#include <Adafruit_BMP280.h>

// ATECC608B (SparkFun)
#include <SparkFun_ATECCX08a_Arduino_Library.h>

static const uint8_t I2C_SDA  = 32;
static const uint8_t I2C_SCL  = 33;
static const uint8_t ECC_ADDR = 0x35;
ATECCX08A atecc;

// -------- WIFI ----------
const char* WIFI_SSID = "Livebox-D580";
const char* WIFI_PASS = "jyxvet-0qyrPi";

// -------- MQTT ----------
const char* MQTT_HOST = "192.168.1.12";  // Mac IP
const uint16_t MQTT_PORT = 8883;
const char* TOPIC = "m5go/env";

// -------- CA CERT ----------
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

// -------- CLIENT CERT (m5go-pub) ----------
static const char client_cert[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIBwDCCAWUCCQCU9fxQbpMYEDAKBggqhkjOPQQDAjBoMQswCQYDVQQGEwJGUjES
MBAGA1UECAwJT2NjaXRhbmllMREwDwYDVQQHDAhUb3Vsb3VzZTEMMAoGA1UECgwD
TGFiMQ0wCwYDVQQLDARNUVRUMRUwEwYDVQQDDAxMb2NhbE1RVFQtQ0EwHhcNMjUx
MTE4MDc1OTE1WhcNMjYxMTE4MDc1OTE1WjBnMQswCQYDVQQGEwJGUjESMBAGA1UE
CAwJT2NjaXRhbmllMREwDwYDVQQHDAhUb3Vsb3VzZTEMMAoGA1UECgwDTGFiMRAw
DgYDVQQLDAdEZXZpY2VzMREwDwYDVQQDDAhtNWdvLXB1YjBZMBMGByqGSM49AgEG
CCqGSM49AwEHA0IABBRXlow/1+4RDh5FSlzbvvQbeo32ugDDXY+RffX1kAv95DsB
6fLhhSc8Fg2rr3sySemPbX7Bt5XIiyPGw78oxw8wCgYIKoZIzj0EAwIDSQAwRgIh
AJJGed0q5YxEyu6rxq7RYa8q11ETrm2EkD4GHxqrMUPkAiEAyrm2JuIKgBxvBywl
NlQDpBw01/c7AyxWYXQ0iARLybA=
-----END CERTIFICATE-----
)EOF";

// ------------ SENSORS ------------
Adafruit_SHT31  sht31 = Adafruit_SHT31();
Adafruit_BMP280 bmp280;

// ------------ MQTT CLIENT ------------
WiFiClientSecure tlsClient;
PubSubClient mqtt(tlsClient);

unsigned long lastPub = 0;

// ------------ UI COLORS ------------
static const uint16_t COLOR_BG      = 0x0000;
static const uint16_t COLOR_CARD    = 0x0005;
static const uint16_t COLOR_ACCENT  = 0x07FF;
static const uint16_t COLOR_TEXT    = 0xFFFF;
static const uint16_t COLOR_OK      = 0x07E0;
static const uint16_t COLOR_WARN    = 0xF800;

// ========== Base64 & ATECC helpers (from section 7) ==========
static const char b64_table[] =
  "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

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


// ---------- ATECC608B Slot 8 helpers (SparkFun-only) ----------

// Compute ATECC data zone address for slot 8, 32-byte block read/write (block 0..12)
static uint16_t slot8AddressForBlock(uint8_t block)
{
  // Addr[6:3] = slot number (8), Addr[8+] = block index, Addr[2:0] = word offset (0 for 32-byte access)
  // See Microchip ATECC608x datasheet, "Data Zone Addressing"
  return ((uint16_t)8 << 3) | ((uint16_t)block << 8);
}

// Write 32 bytes into Slot 8, given block index
bool writeDataSlot8Block(ATECCX08A &atecc, uint8_t block, const uint8_t *data32)
{
  if (block >= 13) return false;   // slot 8 = 13 blocks of 32 bytes (416 bytes total)

  uint16_t addr = slot8AddressForBlock(block);

  // SparkFun high-level wrapper:
  // write(zone, address, data, length_of_data)
  if (!atecc.write(ZONE_DATA, addr, (uint8_t *)data32, 32))
  {
    Serial.println("writeDataSlot8Block: write() failed");
    return false;
  }

  return true;
}

// Read 32 bytes from Slot 8, given block index
bool readDataSlot8Block(ATECCX08A &atecc, uint8_t block, uint8_t *out32)
{
  if (block >= 13) return false;

  uint16_t addr = slot8AddressForBlock(block);

  // SparkFun high-level read:
  // read(zone, address, length, debug)
  if (!atecc.read(ZONE_DATA, addr, 32, false))
  {
    Serial.println("readDataSlot8Block: read() failed");
    return false;
  }

  // Response format in atecc.inputBuffer:
  // [0] = COUNT = 32 (data) + 3 = 35
  // [1..32] = data
  // [33..34] = CRC
  for (int i = 0; i < 32; i++)
  {
    out32[i] = atecc.inputBuffer[1 + i];
  }

  return true;
}


bool loadPrivateKeyPEMFromATECC(ATECCX08A &atecc, String &outPem) {
  // Slot 8 size: 13 blocks * 32 bytes = 416 bytes
  uint8_t slotData[416];

  // Read all blocks 0..12
  for (uint8_t b = 0; b < 13; b++) {
    if (!readDataSlot8Block(atecc, b, slotData + b * 32)) {
      Serial.printf("Error reading slot 8 block %u\n", b);
      return false;
    }
  }

  // Debug: dump first bytes
  Serial.println("Slot 8 first 32 bytes:");
  for (int i = 0; i < 32; i++) {
    if (i && (i % 16 == 0)) Serial.println();
    Serial.printf("%02X ", slotData[i]);
  }
  Serial.println();

  uint16_t derLen = (slotData[0] << 8) | slotData[1];
  Serial.print("DER length from slot 8: ");
  Serial.println(derLen);

  // 416 bytes total, 2 bytes used for length => max DER size = 414
  if (derLen == 0 || derLen > 414) {
    Serial.printf("Invalid DER length in slot 8: %u\n", derLen);
    return false;
  }

  const uint8_t *der = &slotData[2];

  // Optional quick sanity check: DER should start with 0x30 (SEQUENCE)
  if (der[0] != 0x30) {
    Serial.println("Warning: DER does not start with 0x30 (SEQUENCE)");
  }

  String b64 = base64Encode(der, derLen);

  outPem = "-----BEGIN PRIVATE KEY-----\n";
  for (size_t i = 0; i < b64.length(); i += 64) {
    outPem += b64.substring(i, i + 64) + "\n";
  }
  outPem += "-----END PRIVATE KEY-----\n";

  return true;
}


// ------------ UI HELPERS (same as your original) ------------
void uiDrawHeader() {
  auto& d = M5.Display;
  d.fillScreen(COLOR_BG);
  d.fillRoundRect(6, 4, d.width() - 12, 40, 8, COLOR_CARD);
  d.setTextColor(COLOR_ACCENT, COLOR_CARD);
  d.setTextSize(1.5);
  d.setCursor(14, 20);
  d.print("M5Go Secure ENV Publisher");
}

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
  d.print("Live data sent to topic m5go/env");
}

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

void uiSetPublishInfo() {
  auto& d = M5.Display;
  d.setTextSize(1.5);
  d.setTextColor(COLOR_ACCENT, COLOR_CARD);
  d.fillRect(12, 210, d.width() - 24, 12, COLOR_CARD);
  d.setCursor(12, 210);
  d.print("Publishing every 5s");
}

// ------------ WIFI / MQTT / TLS ------------
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uiSetWiFiStatus("Connecting...", false);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
  }
  uiSetWiFiStatus(WiFi.localIP().toString(), true);
}

String g_clientKeyPem; // dynamic key from ATECC

void setupTLS() {
  tlsClient.setCACert(ca_cert);
  tlsClient.setCertificate(client_cert);
  tlsClient.setPrivateKey(g_clientKeyPem.c_str());
}

void connectMQTT() {
  mqtt.setServer(MQTT_HOST, MQTT_PORT);

  while (!mqtt.connected()) {
    String cid = "m5go-pub-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    uiSetMQTTStatus("Connecting...", false);

    if (mqtt.connect(cid.c_str())) {
      uiSetMQTTStatus("Connected", true);
    } else {
      uiSetMQTTStatus("Err:" + String(mqtt.state()), false);
      delay(1500);
    }
  }
}

// ------------ SENSORS ------------
void setupSensors() {
  Wire.begin(); // sensors on default I2C; ECC uses same pins but we already began earlier
  sht31.begin(0x44);
  bmp280.begin(0x76);
}

// ------------ SETUP / LOOP ------------
void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(1);
  uiDrawHeader();
  uiDrawStatic();

  Serial.begin(115200);

  // I2C shared with ECC
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  Serial.println("Bringing up ATECC608B...");
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

    if (!loadPrivateKeyPEMFromATECC(atecc, g_clientKeyPem)) {
      Serial.println("ERROR: could not load private key from ATECC");
    } else {
      Serial.println("Private key PEM built from ATECC slot 8.");
#ifdef DEBUG_PRINT_PEM
      Serial.println(g_clientKeyPem);
#endif
    }
  Serial.print("Key length from ATECC: ");
  Serial.println(g_clientKeyPem.length());
  Serial.println(g_clientKeyPem);
  }
  
  if (!loadPrivateKeyPEMFromATECC(atecc, g_clientKeyPem)) {
  Serial.println("ERROR: could not load private key from ATECC");
} else {
  Serial.println("Private key PEM built from ATECC slot 8.");
#ifdef DEBUG_PRINT_PEM
  Serial.println(g_clientKeyPem);
#endif
}

Serial.print("Key length from ATECC: ");
Serial.println(g_clientKeyPem.length());
Serial.println(g_clientKeyPem);


  setupSensors();
  connectWiFi();
  if (g_clientKeyPem.length() == 0) {
  Serial.println("No key from ATECC; skipping TLS/MQTT connection.");
  return; // or while(1) to stop here
}
  setupTLS();
  connectMQTT();
}

void loop() {
  M5.update();

  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
    setupTLS();
  }

  if (!mqtt.connected()) {
    connectMQTT();
  }

  mqtt.loop();

  unsigned long now = millis();
  if (now - lastPub >= 5000) {
    lastPub = now;

    float t = sht31.readTemperature();
    float h = sht31.readHumidity();
    float p = NAN;
    float pPa = bmp280.readPressure();
    if (!isnan(pPa)) p = pPa / 100.0f;

    uiSetSensorValues(t, h, p);
    uiSetPublishInfo();

    char payload[160];
    snprintf(payload, sizeof(payload),
             "{\"temp_c\":%.2f,\"hum_pct\":%.2f,\"press_hpa\":%.2f}",
             t, h, p);

    mqtt.publish(TOPIC, payload, false);
  }
}

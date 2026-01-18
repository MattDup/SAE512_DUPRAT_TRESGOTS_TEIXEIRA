/******************************************************
 *  M5Stack LoRa – UI complète + ACK FIABLE + AES/ATECC
 *  Nouvelle version corrigée (TS supprimé, parser stable)
 *  PARTIE 1/3 — Includes / UI / Variables / Setup
 *******************************************************/

#include <M5Unified.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <SparkFun_ATECCX08a_Arduino_Library.h>
#include "mbedtls/aes.h"

// ----------------------------------------------------
//                      CONFIG
// ----------------------------------------------------

#ifndef ATECCX08A_ZONE_DATA
#define ATECCX08A_ZONE_DATA 0x02
#endif

#define LORA_CS   5
#define LORA_RST  13
#define LORA_IRQ  34
#define LORA_FREQ 868E6

#define COLOR_SECURE   COLOR_OK     // vert
#define COLOR_UNSECURE COLOR_WARN   // rouge

// Modifier sur chaque M5 : "001" ou "002"
String NODE_ID = "002";

static const uint8_t I2C_SDA  = 32;
static const uint8_t I2C_SCL  = 33;
static const uint8_t ECC_ADDR = 0x35;

ATECCX08A atecc;
uint8_t g_loraKey[32];
bool g_haveKey = false;
bool secureMode = true;

// ----------------------------------------------------
//          VARIABLES LOra / ACK / MESSAGE
// ----------------------------------------------------
String lastTX = "-";
String lastRX = "Waiting...";
String lastPlaintext = "";
int    lastRSSI = 0;
String lastPacketTX = "";
String lastPacketRX = "";
String lastEncryptedTX = "";
String lastEncryptedRX = "";


// Stop-and-wait ACK
static int      g_nextMsgId        = 1;
static bool     g_waitingAck       = false;
static int      g_waitAckId        = -1;
static String   g_waitAckDst       = "";
static unsigned long g_ackDeadline = 0;
static uint8_t  g_retryCount       = 0;
static String   g_waitAckPacket    = "";

static const uint8_t  LORA_MAX_RETRIES   = 3;
static const unsigned long LORA_ACK_TIMEOUT = 3000; // 3 sec

// duplicate detection
uint16_t g_lastSeenFrom001 = 0;
uint16_t g_lastSeenFrom002 = 0;

// Max plaintext size (for AES)
static const size_t LORA_MAX_PLAINTEXT = 80;

// ----------------------------------------------------
//                      UI CONSTANTS
// ----------------------------------------------------
static const uint16_t COLOR_BG      = 0x0000;
static const uint16_t COLOR_CARD    = 0x0005;
static const uint16_t COLOR_ACCENT  = 0x07FF;
static const uint16_t COLOR_TEXT    = 0xFFFF;
static const uint16_t COLOR_OK      = 0x07E0;
static const uint16_t COLOR_WARN    = 0xF800;

static const int UI_MARGIN       = 6;
static const int UI_CARD_RADIUS  = 8;
static const int UI_CARD_SPACING = 6;
static const int UI_HEADER_H     = 32;
static const int UI_BOTTOM_BAR_H = 14;

// throttle : limiter les refresh UI
unsigned long nextUI = 0;

// ----------------------------------------------------
//                  UI LAYOUT
// ----------------------------------------------------
static void uiGetLayout(int &yStatus, int &hStatus,
                        int &yTraffic, int &hTraffic,
                        int &yDecrypt, int &hDecrypt)
{
  auto &d = M5.Display;
  int screenH = d.height();

  hStatus  = 48;
  hTraffic = 72;
  yStatus  = 4 + UI_HEADER_H + UI_CARD_SPACING;
  yTraffic = yStatus + hStatus + UI_CARD_SPACING;
  yDecrypt = yTraffic + hTraffic + UI_CARD_SPACING;
  hDecrypt = screenH - yDecrypt - UI_BOTTOM_BAR_H - 4;

  if (hDecrypt < 32) hDecrypt = 32;
}

// ----------------------------------------------------
//                HEADER (titre + NodeID)
// ----------------------------------------------------
void uiDrawHeader() {
  auto& d = M5.Display;
  d.fillScreen(COLOR_BG);

  int w = d.width() - 2 * UI_MARGIN;
  d.fillRoundRect(UI_MARGIN, 4, w, UI_HEADER_H, UI_CARD_RADIUS, COLOR_CARD);

  d.setTextColor(COLOR_ACCENT, COLOR_CARD);
  d.setTextSize(2);
  d.setCursor(UI_MARGIN + 8, 8);
  d.print("LoRa Secure Node");

  d.setTextSize(1);
  d.setCursor(UI_MARGIN + 8, 24);
  d.printf("Node ID: %s", NODE_ID.c_str());
}

// ----------------------------------------------------
//              STATIC CARDS (fond des 3 cartes)
// ----------------------------------------------------
void uiDrawStaticCards() {
  auto& d = M5.Display;
  int w = d.width() - 2 * UI_MARGIN;

  int yStatus, hStatus, yTraffic, hTraffic, yDecrypt, hDecrypt;
  uiGetLayout(yStatus, hStatus, yTraffic, hTraffic, yDecrypt, hDecrypt);

  d.fillRoundRect(UI_MARGIN, yStatus,  w, hStatus,  UI_CARD_RADIUS, COLOR_CARD);
  d.fillRoundRect(UI_MARGIN, yTraffic, w, hTraffic, UI_CARD_RADIUS, COLOR_CARD);
  d.fillRoundRect(UI_MARGIN, yDecrypt, w, hDecrypt, UI_CARD_RADIUS, COLOR_CARD);

  d.setTextColor(COLOR_TEXT, COLOR_CARD);
  d.setTextSize(1);

  d.setCursor(UI_MARGIN + 6, yStatus + 4);
  d.print("Node / Security");

  d.setCursor(UI_MARGIN + 6, yTraffic + 4);
  d.print("Last TX / RX");

  d.setCursor(UI_MARGIN + 6, yDecrypt + 4);
  d.print("Decryption / RSSI");
}

// ----------------------------------------------------
//             UI UPDATE: STATUS CARD
// ----------------------------------------------------
void uiUpdateStatusCard() {
  auto& d = M5.Display;
  int screenW = d.width();

  int yStatus, hStatus, yTraffic, hTraffic, yDecrypt, hDecrypt;
  uiGetLayout(yStatus, hStatus, yTraffic, hTraffic, yDecrypt, hDecrypt);

  int xCard = UI_MARGIN;
  int wCard = screenW - 2 * UI_MARGIN;
  int contentY = yStatus + 18;
  int contentH = hStatus - 22;
  
  // nettoyage complet de la zone texte
  d.fillRect(xCard + 2, contentY, wCard - 4, contentH, COLOR_CARD);

  d.setTextSize(1);
  d.setCursor(xCard + 8, contentY);
  d.setTextColor(COLOR_TEXT, COLOR_CARD);
  d.printf("Node: %s", NODE_ID.c_str());

  d.setCursor(xCard + 8, contentY + 12);
  if (g_haveKey) {
    d.setTextColor(COLOR_OK, COLOR_CARD);
    d.print("Key: OK  ");
  } else {
    d.setTextColor(COLOR_WARN, COLOR_CARD);
    d.print("Key: MISSING  ");
  }

  d.setTextColor(COLOR_TEXT, COLOR_CARD);
  d.print("Secure: ");

  if (!g_haveKey) {
    d.setTextColor(COLOR_WARN, COLOR_CARD);
    d.print("OFF (no key)");
  } else if (secureMode) {
    d.setTextColor(COLOR_OK, COLOR_CARD);
    d.print("ON");
  } else {
    d.setTextColor(COLOR_WARN, COLOR_CARD);
    d.print("OFF");
  }
}

// ----------------------------------------------------
//              UI UPDATE: TX/RX CARD
// ----------------------------------------------------
void uiUpdateTrafficCard()
{
    auto& d = M5.Display;
    int screenW = d.width();

    int yStatus, hStatus, yTraffic, hTraffic, yDecrypt, hDecrypt;
    uiGetLayout(yStatus, hStatus, yTraffic, hTraffic, yDecrypt, hDecrypt);

    int xCard = UI_MARGIN;
    int wCard = screenW - 2 * UI_MARGIN;
    int contentY = yTraffic + 18;

    // Effacer zone
    d.fillRect(xCard + 2, contentY, wCard - 4, hTraffic - 22, COLOR_CARD);

    d.setTextSize(1);
    d.setCursor(xCard + 8, contentY);

    // ---- TX ----
    d.setCursor(xCard + 8, contentY);
    d.setTextColor( secureMode ? COLOR_SECURE : COLOR_UNSECURE , COLOR_CARD );
    d.print("TX: ");

    d.setTextColor(COLOR_TEXT, COLOR_CARD);
    d.println(lastPacketTX);

    // ---- ENC TX ----
    d.setCursor(xCard + 8, contentY + 14);
    d.setTextColor( secureMode ? COLOR_SECURE : COLOR_UNSECURE , COLOR_CARD );
    d.print("ENC: ");

    d.setTextColor(COLOR_TEXT, COLOR_CARD);
    String shortEncTX = lastEncryptedTX;
    if (shortEncTX.length() > 32)
        shortEncTX = shortEncTX.substring(0, 32) + "...";
    d.println(shortEncTX);

    // ---- RX ----
    d.setCursor(xCard + 8, contentY + 32);
    d.setTextColor( secureMode ? COLOR_SECURE : COLOR_UNSECURE , COLOR_CARD );
    d.print("RX: ");

    d.setTextColor(COLOR_TEXT, COLOR_CARD);
    d.println(lastPacketRX);

    // ---- ENC RX ----
    d.setCursor(xCard + 8, contentY + 46);
    d.setTextColor( secureMode ? COLOR_SECURE : COLOR_UNSECURE , COLOR_CARD );
    d.print("ENC: ");

    d.setTextColor(COLOR_TEXT, COLOR_CARD);
    String shortEncRX = lastEncryptedRX;
    if (shortEncRX.length() > 32)
        shortEncRX = shortEncRX.substring(0, 32) + "...";
    d.println(shortEncRX);
}


// ----------------------------------------------------
//            UI UPDATE: DECRYPT + RSSI CARD
// ----------------------------------------------------
void uiUpdateDecryptCard() {
  auto& d = M5.Display;

  int yStatus, hStatus, yTraffic, hTraffic, yDecrypt, hDecrypt;
  uiGetLayout(yStatus, hStatus, yTraffic, hTraffic, yDecrypt, hDecrypt);

  int xCard = UI_MARGIN;
  int wCard = d.width() - 2 * UI_MARGIN;

  int contentY = yDecrypt + 18;
  int contentH = hDecrypt - 22;

  d.fillRect(xCard + 2, contentY, wCard - 4, contentH, COLOR_CARD);

  d.setTextSize(1);
  int textX = xCard + 8;

  d.setCursor(textX, contentY);
  d.setTextColor(COLOR_ACCENT, COLOR_CARD);
  d.printf("RSSI: %d", lastRSSI);

  // ---------------------------------------------
  //  TRONQUER lastPlaintext (max 24 caractères)
  // ---------------------------------------------
  String shortPlain = lastPlaintext;
  if (shortPlain.length() > 24) {
      shortPlain = shortPlain.substring(0, 24) + "...";
  }

  d.setCursor(textX, contentY + 12);
  d.setTextColor(COLOR_TEXT, COLOR_CARD);
  d.print("Dec: " + shortPlain);

  // bottom bar (A/B/C)
  int bottomY = d.height() - UI_BOTTOM_BAR_H;
  d.fillRect(0, bottomY, d.width(), UI_BOTTOM_BAR_H, COLOR_BG);
  d.setCursor(8, bottomY + 2);
  d.print("A:Send   B:-   C:Secure");
}

// ----------------------------------------------------
//             UI REFRESH (throttled)
// ----------------------------------------------------
void uiRefresh() {
  if (millis() < nextUI) return;
  nextUI = millis() + 120; // max 8 fps

  uiUpdateStatusCard();
  uiUpdateTrafficCard();
  uiUpdateDecryptCard();
}

void selfTestCrypto()
{
  if (!g_haveKey) {
    Serial.println("SelfTest: NO KEY, abort");
    return;
  }

  String plain = "HELLO";
  String ivB64, ctB64;

  Serial.println("SelfTest: starting with plain = " + plain);

  if (!loraEncrypt(plain, g_loraKey, ivB64, ctB64)) {
    Serial.println("SelfTest: Encrypt FAILED");
    return;
  }

  Serial.println("SelfTest: IV = " + ivB64);
  Serial.println("SelfTest: CT = " + ctB64);

  String decrypted;
  if (!loraDecrypt(ivB64, ctB64, g_loraKey, decrypted)) {
    Serial.println("SelfTest: Decrypt FAILED");
    return;
  }

  Serial.println("SelfTest: decrypted = " + decrypted);

  if (decrypted == plain) {
    Serial.println("SelfTest:  OK (HELLO -> HELLO)");
  } else {
    Serial.println("SelfTest:  MISMATCH");
  }
  Serial.println("------------------------------");
}


// ----------------------------------------------------
//                       SETUP
// ----------------------------------------------------
void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(1);

  Serial.begin(115200);
  randomSeed((uint32_t)esp_random());
  delay(300);

  // -------- AJOUT : désactivation IRQ LoRa pendant secure-element --------
  pinMode(LORA_IRQ, INPUT);
  gpio_intr_disable(GPIO_NUM_34); // désactive l’IRQ proprement


  Serial.println();
  Serial.println("==== BOOT ====");

  // -------------------------------------------------
  //   1) ATECC FIRST – avant UI, avant LoRa
  // -------------------------------------------------
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  g_haveKey = false;

  if (atecc.begin(ECC_ADDR, Wire, Serial)) {
      if (atecc.readConfigZone(false)) {
          if (readDataSlot8Block(atecc, 0, g_loraKey)) {
              g_haveKey = true;
          }
      }
  }

  // -------- AJOUT : double lecture pour fiabiliser I2C --------
  if (g_haveKey) {
      uint8_t tmpKey[32];
      if (readDataSlot8Block(atecc, 0, tmpKey)) {
          memcpy(g_loraKey, tmpKey, 32);
      }
  }

  Serial.print("Node ");
  Serial.print(NODE_ID);
  Serial.print(" — Slot 8 Key = ");
  if (g_haveKey) {
      for (int i = 0; i < 32; i++) Serial.printf("%02X", g_loraKey[i]);
  } else {
      Serial.print("NO KEY");
  }
  Serial.println();
  Serial.println("--------------------------------------");

  // -------------------------------------------------
  //   2) Self-test crypto — maintenant ça marchera
  // -------------------------------------------------
  selfTestCrypto();
  Serial.println("--------------------------------------");

  // -------------------------------------------------
  //   3) UI / LoRa seulement après le test
  // -------------------------------------------------
  uiDrawHeader();
  uiDrawStaticCards();
  uiRefresh();

  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed!");
  }

  // -------- AJOUT : réactivation IRQ LoRa --------
  attachInterrupt(LORA_IRQ, NULL, RISING);

  if (!g_haveKey) secureMode = false;

  lastPlaintext = g_haveKey ? "ATECC key OK" : "NO KEY";
  uiRefresh();
}



// ======================================================================
//                     PARTIE 2/3 — Crypto + Parsing + ACK
// ======================================================================

// ----------------------------------------------------
//               BASE64 ENCODE / DECODE
// ----------------------------------------------------
static const char b64_table[] =
  "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static String base64Encode(const uint8_t *data, size_t len) {
  String out;
  out.reserve(((len + 2) / 3) * 4);

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


static int8_t b64DecodeChar(char c)
{
  if (c >= 'A' && c <= 'Z') return c - 'A';
  if (c >= 'a' && c <= 'z') return c - 'a' + 26;
  if (c >= '0' && c <= '9') return c - '0' + 52;
  if (c == '+') return 62;
  if (c == '/') return 63;
  if (c == '=') return -2;
  return -1;
}

static bool base64Decode(const String &in, uint8_t *out,
                         size_t &outLen, size_t maxOutLen)
{
  outLen = 0;
  int val = 0, valb = -8;

  for (int i = 0; i < in.length(); i++) {
    int8_t c = b64DecodeChar(in[i]);
    if (c == -1) continue;
    if (c == -2) break;

    val = (val << 6) + c;
    valb += 6;

    if (valb >= 0) {
      if (outLen >= maxOutLen) return false;
      out[outLen++] = (uint8_t)((val >> valb) & 0xFF);
      valb -= 8;
    }
  }
  return true;
}

// ----------------------------------------------------
//                     AES-CTR
// ----------------------------------------------------
static bool aesCtrCrypt(const uint8_t *key, size_t keyBits,
                        const uint8_t iv[16],
                        const uint8_t *input, size_t len,
                        uint8_t *output)
{
  mbedtls_aes_context ctx;
  mbedtls_aes_init(&ctx);

  if (mbedtls_aes_setkey_enc(&ctx, key, keyBits) != 0) {
    mbedtls_aes_free(&ctx);
    return false;
  }

  size_t nc_off = 0;
  unsigned char nonce_counter[16];
  unsigned char stream_block[16];
  memcpy(nonce_counter, iv, 16);
  memset(stream_block, 0, 16);

  int res = mbedtls_aes_crypt_ctr(&ctx, len, &nc_off,
                                  nonce_counter, stream_block,
                                  input, output);

  mbedtls_aes_free(&ctx);
  return (res == 0);
}

// ----------------- Encrypt + Base64 -----------------
static bool loraEncrypt(const String &plain,
                        const uint8_t key[32],
                        String &outIVb64,
                        String &outCTb64)
{
  size_t len = plain.length();
  if (len == 0) return true;
  if (len > LORA_MAX_PLAINTEXT) len = LORA_MAX_PLAINTEXT;

  uint8_t bufIn[LORA_MAX_PLAINTEXT];
  for (size_t i = 0; i < len; i++) bufIn[i] = plain[i];

  uint8_t iv[16];
  for (int i = 0; i < 16; i++) iv[i] = random(0, 256);

  uint8_t cipher[LORA_MAX_PLAINTEXT];

  if (!aesCtrCrypt(key, 256, iv, bufIn, len, cipher))
    return false;

  outIVb64 = base64Encode(iv, 16);
  outCTb64 = base64Encode(cipher, len);
  return true;
}

// ----------------- Decrypt -----------------
static bool loraDecrypt(const String &ivB64,
                        const String &ctB64,
                        const uint8_t key[32],
                        String &outPlain)
{
  uint8_t iv[16]; size_t ivLen = 0;
  uint8_t cipher[LORA_MAX_PLAINTEXT]; size_t cipherLen = 0;

  if (!base64Decode(ivB64, iv, ivLen, sizeof(iv))) return false;
  if (ivLen != 16) return false;
  if (!base64Decode(ctB64, cipher, cipherLen, sizeof(cipher))) return false;

  if (cipherLen == 0) { outPlain = ""; return true; }

  uint8_t plain[LORA_MAX_PLAINTEXT];

  if (!aesCtrCrypt(key, 256, iv, cipher, cipherLen, plain))
    return false;

  outPlain = "";
  for (size_t i = 0; i < cipherLen; i++)
    outPlain += (char)plain[i];

  return true;
}

// ----------------------------------------------------
//                  ATECC Slot 8 Key Load
// ----------------------------------------------------
bool readDataSlot8Block(ATECCX08A &atecc, uint8_t block, uint8_t *out32)
{
  if (block >= 13) return false;

  uint16_t addr = ((uint16_t)8 << 3) | ((uint16_t)block << 8);

  if (!atecc.read(ATECCX08A_ZONE_DATA, addr, 32, false)) {
    Serial.println("ATECC read error");
    return false;
  }

  for (int i = 0; i < 32; i++)
    out32[i] = atecc.inputBuffer[1 + i];

  return true;
}

// ======================================================================
//                    PARSER FIXE (ordre strict)
// ======================================================================
//
// Format A imposé :
//   SRC=xxx;DST=xxx;ID=n;TYPE=xxx;MSG=xxxxx
//
// -> Parsing **100 % fiable**
// -> Pas de TS
// -> Pas de champs optionnels
// ======================================================================

bool parsePacket(const String &p,
                 String &src, String &dst,
                 int &id, String &type, String &msg)
{
  // champs à rechercher
  int pSRC  = p.indexOf("SRC=");
  int pDST  = p.indexOf(";DST=");
  int pID   = p.indexOf(";ID=");
  int pTYPE = p.indexOf(";TYPE=");
  int pMSG  = p.indexOf(";MSG=");

  if (pSRC < 0 || pDST < 0 || pID < 0 || pTYPE < 0 || pMSG < 0)
    return false;

  // extraction
  src  = p.substring(pSRC + 4,  pDST);
  dst  = p.substring(pDST + 5,  pID);
  id   = p.substring(pID  + 4,  pTYPE).toInt();
  type = p.substring(pTYPE + 6, pMSG);
  msg  = p.substring(pMSG  + 5);

  return true;
}

// ======================================================================
//                DUPLICATE DETECTION (ID par source)
// ======================================================================
uint16_t getLastSeen(const String &src)
{
  if (src == "001") return g_lastSeenFrom001;
  if (src == "002") return g_lastSeenFrom002;
  return 0;
}

void setLastSeen(const String &src, uint16_t id)
{
  if (src == "001") g_lastSeenFrom001 = id;
  if (src == "002") g_lastSeenFrom002 = id;
}

// ======================================================================
//                SEND ACK (format A, immédiat)
// ======================================================================
void sendAck(const String &dst, int msgId, const String &originalPlain)
{
    // On construit le texte clair de l’acquittement
    String ackPlain = "ACK(" + originalPlain + ")";

    String content = ackPlain;
    lastEncryptedTX = "";

    // ---- Chiffrement si nécessaire ----
    if (secureMode && g_haveKey)
    {
        String ivB64, ctB64;
        if (!loraEncrypt(ackPlain, g_loraKey, ivB64, ctB64)) {
            Serial.println("ACK Encrypt ERR");
            return;
        }

        content = "SEC1:IV=" + ivB64 + ":CT=" + ctB64;
        lastEncryptedTX = content;
    }

    // ---- Construction du paquet LoRa ----
    String packet =
        "SRC=" + NODE_ID +
        ";DST=" + dst +
        ";ID=" + String(msgId) +
        ";TYPE=ACK;MSG=" + content;

    LoRa.beginPacket();
    LoRa.print(packet);
    LoRa.endPacket();

    Serial.println("TX ACK: " + packet);

    // ---- Affichage TX clair ----
    lastPacketTX =
        "SRC=" + NODE_ID +
        ";DST=" + dst +
        ";ID=" + String(msgId) +
        ";TYPE=ACK;MSG=" + ackPlain;

    lastTX = "ACK (" + ackPlain + ")";
}



// ======================================================================
//                SEND MESSAGE (encrypt si needed)
// ======================================================================
void sendMessage(const String &dst, const String &plain)
{
    if (g_waitingAck) {
        lastPlaintext = "ERR: ACK pending";
        return;
    }

    if (secureMode && !g_haveKey) {
        lastPlaintext = "Secure ON but no key";
        return;
    }

    String content = plain;
    lastEncryptedTX = "";    // reset par défaut

    // ---------------------
    //   CHIFFREMENT
    // ---------------------
    if (secureMode && g_haveKey)
    {
        String ivB64, ctB64;
        if (!loraEncrypt(plain, g_loraKey, ivB64, ctB64)) {
            lastPlaintext = "Encrypt error";
            return;
        }

        // Le message envoyé réellement
        content = "SEC1:IV=" + ivB64 + ":CT=" + ctB64;

        // Pour ENC TX (affichage)
        lastEncryptedTX = content;
    }

    // Générer ID
    int msgId = g_nextMsgId++;
    if (g_nextMsgId > 30000) g_nextMsgId = 1;

    // ---------------------
    //     PACKET RADIO
    // ---------------------
    String packet =
        "SRC=" + NODE_ID +
        ";DST=" + dst +
        ";ID=" + String(msgId) +
        ";TYPE=MSG;MSG=" + content;

    // Envoi LoRa
    LoRa.beginPacket();
    LoRa.print(packet);
    LoRa.endPacket();

    // ---------------------
    //     AFFICHAGE
    // ---------------------

    // TX clair POUR L’UI
    lastPacketTX =
        "SRC=" + NODE_ID +
        ";DST=" + dst +
        ";ID="  + String(msgId) +
        ";TYPE=MSG;MSG=" + plain;

    lastTX = "MSG (" + plain + ")";
    lastPlaintext = "Sent: " + plain;

    Serial.println("TX MSG: " + packet);

    // ---------------------
    //     ACK WAIT
    // ---------------------
    g_waitingAck    = true;
    g_waitAckId     = msgId;
    g_waitAckDst    = dst;
    g_retryCount    = 0;
    g_waitAckPacket = packet;
    g_ackDeadline   = millis() + LORA_ACK_TIMEOUT;
}


// ======================================================================
//                 PARTIE 3/3 — RX + ACK TIMEOUT + LOOP
// ======================================================================

// -------------------------------------------------------------
//                        HANDLE RECEIVE
// -------------------------------------------------------------
void handleReceivePacket(const String &incoming)
{
    lastRSSI = LoRa.packetRssi();
    Serial.println("RX: " + incoming);

    // -------------------------------
    //         PARSING
    // -------------------------------
    String src, dst, type, msgField;
    int msgId;
    String plain = "";

    if (!parsePacket(incoming, src, dst, msgId, type, msgField)) {
        lastPlaintext = "Parse error";
        lastPacketRX = "Parse ERR";
        lastEncryptedRX = "";
        return;
    }

    Serial.printf("Parsed → SRC=%s DST=%s ID=%d TYPE=%s MSG=%s\n",
                  src.c_str(), dst.c_str(), msgId, type.c_str(), msgField.c_str());


    // ============================================================
    //                      ACK RECEIVED
    // ============================================================
    if (type == "ACK")
    {
        // Enregistrer la version chiffrée dans ENC RX
        if (msgField.startsWith("SEC1:IV="))
            lastEncryptedRX = msgField;
        else
            lastEncryptedRX = "";

        String plainAck = "";

        // Déchiffrement si nécessaire
        if (msgField.startsWith("SEC1:IV=") && g_haveKey)
        {
            int ivPos = msgField.indexOf("IV=") + 3;
            int ctPos = msgField.indexOf(":CT=");
            String ivB64 = msgField.substring(ivPos, ctPos);
            String ctB64 = msgField.substring(ctPos + 4);

            if (!loraDecrypt(ivB64, ctB64, g_loraKey, plainAck))
                plainAck = "ACK(DecryptERR)";
        }
        else
        {
            // ACK non chiffré
            plainAck = msgField;
        }

        // ---- Affichage RX clair ----
        lastPacketRX =
            "SRC=" + src +
            ";DST=" + dst +
            ";ID=" + String(msgId) +
            ";TYPE=ACK;MSG=" + plainAck;

        // Message de statut
        if (g_waitingAck && dst == NODE_ID && msgId == g_waitAckId)
        {
            g_waitingAck = false;
            g_retryCount = 0;
            lastPlaintext = "ACK recu (ID=" + String(msgId) + ")";
        }
        else
        {
            lastPlaintext = "ACK inattendu (ID=" + String(msgId) + ")";
        }

        return;
    }



    // ============================================================
    //                      NOT FOR ME
    // ============================================================
    if (dst != NODE_ID) {
        lastPlaintext = "Not for me";
        lastPacketRX = "Ignored";
        lastEncryptedRX = "";
        return;
    }


    // ============================================================
    //                     DUPLICATE CHECK
    // ============================================================
    uint16_t last = getLastSeen(src);
    if (last == (uint16_t)msgId)
    {
        Serial.println("Duplicate detected → resend ACK");

        String plainACK;

        if (msgField.startsWith("SEC1:IV=") && g_haveKey)
        {
            int ivPos = msgField.indexOf("IV=") + 3;
            int ctPos = msgField.indexOf(":CT=");
            String ivB64 = msgField.substring(ivPos, ctPos);
            String ctB64 = msgField.substring(ctPos + 4);
            loraDecrypt(ivB64, ctB64, g_loraKey, plainACK);
        }
        else plainACK = msgField;

        sendAck(src, msgId, plainACK);

        lastPlaintext = "Duplicate";
        lastPacketRX = "Duplicate";
        return;
    }
    setLastSeen(src, msgId);


    // ============================================================
    //                 MESSAGE NORMAL (TYPE=MSG)
    // ============================================================
    if (msgField.startsWith("SEC1:IV="))
    {
        // Partie chiffrée pour ENC RX
        lastEncryptedRX = msgField;

        if (!g_haveKey) {
            lastPlaintext = "Encrypted but NO KEY";
            lastPacketRX = "MSG (NO KEY)";
        }
        else
        {
            int ivPos = msgField.indexOf("IV=") + 3;
            int ctPos = msgField.indexOf(":CT=");

            String ivB64 = msgField.substring(ivPos, ctPos);
            String ctB64 = msgField.substring(ctPos + 4);

            if (loraDecrypt(ivB64, ctB64, g_loraKey, plain)) {
                lastPlaintext = plain;
            } else {
                plain = "Decrypt ERR";
                lastPlaintext = "Decrypt ERR";
            }

            // RX CLAIR → affichage
            lastPacketRX =
                "SRC=" + src +
                ";DST=" + dst +
                ";ID=" + String(msgId) +
                ";TYPE=MSG;MSG=" + plain;
        }
    }
    else
    {
        // Pas de chiffrement
        lastEncryptedRX = "";
        plain = msgField;
        lastPlaintext = msgField;

        lastPacketRX =
            "SRC=" + src +
            ";DST=" + dst +
            ";ID=" + String(msgId) +
            ";TYPE=MSG;MSG=" + msgField;
    }


    // ============================================================
    //                        SEND ACK
    // ============================================================
    sendAck(src, msgId, plain);
}


// -------------------------------------------------------------
//                    ACK TIMEOUT + RETRIES
// -------------------------------------------------------------
void handleAckTimeout()
{
  if (!g_waitingAck) return;

  unsigned long now = millis();
  if (now < g_ackDeadline) return;

  // timeout reached
  if (g_retryCount >= LORA_MAX_RETRIES) {
    lastPlaintext = "ACK TIMEOUT → abandon (ID=" + String(g_waitAckId) + ")";
    Serial.println(lastPlaintext);

    g_waitingAck = false;
    g_waitAckId = -1;
    g_waitAckPacket = "";
    return;
  }

  // retry
  g_retryCount++;
  lastPlaintext = "Retry " + String(g_retryCount) +
                  " (ID=" + String(g_waitAckId) + ")";
  Serial.println(lastPlaintext);

  if (g_waitAckPacket.length()) {
    Serial.println("Re-TX: " + g_waitAckPacket);

    LoRa.beginPacket();
    LoRa.print(g_waitAckPacket);
    LoRa.endPacket();

    lastTX = g_waitAckPacket;
  }

  g_ackDeadline = now + LORA_ACK_TIMEOUT;
}


// -------------------------------------------------------------
//                 BUTTON A : HELLO / OK / PING
// -------------------------------------------------------------
int msgIndex = 0;
const char* messages[] = {"HELLO", "OK", "PING"};

void handleButtonA() {
  String target = (NODE_ID == "001" ? "002" : "001");
  String msg = messages[msgIndex];
  msgIndex = (msgIndex + 1) % 3;
  sendMessage(target, msg);
}


// -------------------------------------------------------------
//                        MAIN LOOP
// -------------------------------------------------------------
void loop()
{
  M5.update();

  // ----------------------
  //   RX LOOPS
  // ----------------------
  int packetSize;
  while ((packetSize = LoRa.parsePacket())) {
    String incoming;
    while (LoRa.available())
      incoming += (char)LoRa.read();

    handleReceivePacket(incoming);
  }

  // ----------------------
  //   ACK TIMEOUT
  // ----------------------
  handleAckTimeout();

  // ----------------------
  //   UI REFRESH
  // ----------------------
  uiRefresh();

  // ----------------------
  //   BUTTONS
  // ----------------------
  if (M5.BtnA.wasPressed()) handleButtonA();

  if (M5.BtnC.wasPressed()) {
    if (!g_haveKey) {
      secureMode = false;
      lastPlaintext = "No key → Secure OFF";
    } else {
      secureMode = !secureMode;
      lastPlaintext = secureMode ? "Secure ON" : "Secure OFF";
    }
    Serial.println(lastPlaintext);
  }

  delay(5);
}









void handleReceivePacket(const String &incoming)
{
    lastRSSI = LoRa.packetRssi();

    // 1) Parsing du paquet
    String src, dst, type, msgField;
    int msgId;
    String plain = "";

    if (!parsePacket(incoming, src, dst, msgId, type, msgField)) {
        // parse error ...
        return;
    }

    // 2) Cas où on reçoit un ACK
    if (type == "ACK") {
        // ... (déchiffrement éventuel)
        if (g_waitingAck && dst == NODE_ID && msgId == g_waitAckId) {
            g_waitingAck = false;       // ACK attendu → OK
            g_retryCount = 0;
            // lastPlaintext = "ACK recu (ID=...)";
        }
        return;
    }

    // 3) Paquet pas pour ce nœud → ignoré
    if (dst != NODE_ID) {
        // lastPlaintext = "Not for me";
        return;
    }

    // 4) Détection de doublon (même SRC + même ID)
    uint16_t last = getLastSeen(src);
    if (last == (uint16_t)msgId) {
        // ... (reconstruire plain si chiffré)
        sendAck(src, msgId, plain);     // on renvoie juste l’ACK
        // lastPlaintext = "Duplicate";
        return;
    }
    setLastSeen(src, msgId);

    // 5) Message normal (TYPE=MSG)
    if (msgField.startsWith("SEC1:IV=")) {
        // ... déchiffrement avec loraDecrypt(...)
        // plain = message déchiffré ou "Decrypt ERR"
    } else {
        plain = msgField;               // message en clair
    }

    // 6) Envoi d’un ACK pour ce message
    sendAck(src, msgId, plain);
}



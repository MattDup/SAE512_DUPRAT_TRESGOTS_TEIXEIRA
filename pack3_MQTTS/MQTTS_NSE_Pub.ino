// Publisher sécurisé : M5Go envoie les mesures ENV II via MQTT sur un broker TLS
// La clé privée et le certificat client sont stockés en clair dans le firmware (PROGMEM).

#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Wire.h>

#include <Adafruit_SHT31.h>
#include <Adafruit_BMP280.h>

// -------- WIFI ----------
// Réseau Wi-Fi local auquel M5Go va se connecter
const char* WIFI_SSID = "Livebox-D580";
const char* WIFI_PASS = "jyxvet-0qyrPi";

// -------- MQTT ----------
// Broker Mosquitto configuré avec TLS sur le port 8883
const char* MQTT_HOST = "192.168.1.12";  // Mac IP
const uint16_t MQTT_PORT = 8883;
const char* TOPIC = "m5go/env";

// -------- CA CERT ----------
// Certificat de l'autorité de certification (CA) qui a signé le certificat du broker.
// Permet au client de vérifier qu'il parle bien au bon broker TLS.
static const char ca_cert[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFsTCCA5mgAwIBAgIUCQ3b6jzGLXOCrozP4jU46+/w71wwDQYJKoZIhvcNAQEL
BQAwaDELMAkGA1UEBhMCRlIxEjAQBgNVBAgMCU9jY2l0YW5pZTERMA8GA1UEBwwI
VG91bG91c2UxDDAKBgNVBAoMA0xhYjENMAsGA1UECwwETVFUVDEVMBMGA1UEAwwM
TG9jYWxNUVRULUNBMB4XDTI1MTExMTE3MzM1NVoXDTM1MTEwOTE3MzM1NVowaDEL
MAkGA1UEBhMCRlIxEjAQBgNVBAgMCU9jY2l0YW5pZTERMA8GA1UEBwwIVG91bG91
c2UxDDAKBgNVBAoMA0xhYjENMAsGA1UECwwETVFUVDEVMBMGA1UEAwwMTG9jYWxN
UVRULUNBMIICIjANBgkqhkiG9w0BAQEFAAOCAg8AMIICCgKCAgEAmrAkrWR8biEZ
e+wY+wev8QW6f487FE/rWEqXnpFVB4IuLnv0Djvgjlwbrk1pQ6Wfbp0jVExSaIoF
7NvxuvmNLLJAGW2mzwYM6ZXl716/kTDyA7ucmgYcVgBY4YIk5QVq2aikb4srI6dr
oUkgxkfstbIQQo3OE9BrI96ubEnrRNPUeFQEkp0ZO47zEY4nD6U6OPwfDf9eOFC4
TLV1dUM8i5V4ReB1dhrkyh2ljdf/WruFD4BFfGcIbMpTlQRiXAjJXLoI9PejEC/k
r+I1lml0GJQaQB/fW4ZlTjag1FVMAeSCK3uVAaS4lXAB7Rz1Juxc4vVjxbkWtqEX
YvubUn75W4hcQWbE1NUqlzlbSRAfJnigns6XH1U9jMNqhVRkCLbAKHqoO1nhuQWo
zXXwQJQohfspbAqIzuNoEzT0QzWwB+BNb1QDfwYMZ79A4lyIvOq2NpDecL3u8J4N
CSD6d/dafkWdiM0Le+FMoMPeVMzs6vKttZsNu9+eoTyq2rjlA8hVN50hYEgskZU+
6yUdfN25lPA8ZIMlP/XltXUtJO6GkG4oGb8m7nBV5OHrt09TOJAhMAyxx3DUrwgJ
UEK65eeIgNjfFOi2Ul6LLzUs0PY99F736YuDfWSxQ+8RMlY58OaPwSd4eHcudlSb
vx0K+2N7GtSy2IYsJvtWrwNU9bIAoI0CAwEAAaNTMFEwHQYDVR0OBBYEFG72k3vn
Sxr5a0eGnVObzaAhOJdZMB8GA1UdIwQYMBaAFG72k3vnSxr5a0eGnVObzaAhOJdZ
MA8GA1UdEwEB/wQFMAMBAf8wDQYJKoZIhvcNAQELBQADggIBABuWRAzwRX0TMlnc
e/YY3OxgAzphDhWEeWujaL5ybqk3ttOkYomG+YGbMJxbY4z8CMJiKOXH6/J7ZEpJ
sDWZXd48Fd6WEgpkluB3HUb3XfCu5CPIbL0m4hg0kr2pCeffqP005iC8ONMFMtGA
xanV3R2teA7we9NCqSzk5MyG3uxeeOobmZ3Moeg9RG8gEQ1tEhB8OWNPqkq4OERn
3Gs0NN2gPP0Ep8a4El3yr+lP9sy2uUTBLZm0jDDlTX+bXubEzg2W3W1UOmVI1xtu
78VE3PoqWj8GO1PVl2kOw45yov//XA0tTEvUyguiCWhO4RaXJUGCC2roiYGZ6voP
9XNLFQkXi6y61KBsr/h7wmRYFYRkwBSKTQ7mKxdMV6R4dwFiIuHaXxyzLzwJKegZ
uY3SMGXTuSorndmXDg6U9zQDiOFOGWfQnH6UaP5VjpblANu4uzn41gjFiJN9Kt3q
KrOnC58qsI7g5PbfaKypsc15wOTlZhZl8U/UKnmuBT7T6X8gzdQecSR2fc1T35fe
+a5EtKyHnhpfc893SGmd+o3FD/U/IQUBZK/Ailo/pRO+muBRpbQcIkvLX5PC8B/j
r1uTV+/tXOHybbkYrT0p1i5Coc5+nRF1/utwPTm6jzAHlLJXe9Q/8lurQ1qjPvaG
KGuyhMbNWN2zYFWrpRiyuWjElDuF
-----END CERTIFICATE-----
)EOF";

// -------- CLIENT CERT (m5go-pub) ----------
// Certificat X.509 du client (M5Go publisher), signé par la CA ci-dessus.
static const char client_cert[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIEnzCCAoegAwIBAgIUZ0trSk8RXBw+4/8NrTPscJPOiGEwDQYJKoZIhvcNAQEL
BQAwaDELMAkGA1UEBhMCRlIxEjAQBgNVBAgMCU9jY2l0YW5pZTERMA8GA1UEBwwI
VG91bG91c2UxDDAKBgNVBAoMA0xhYjENMAsGA1UECwwETVFUVDEVMBMGA1UEAwwM
TG9jYWxNUVRULUNBMB4XDTI1MTExMTE3NDUxNVoXDTI2MTExMTE3NDUxNVowZzEL
MAkGA1UEBhMCRlIxEjAQBgNVBAgMCU9jY2l0YW5pZTERMA8GA1UEBwwIVG91bG91
c2UxDDAKBgNVBAoMA0xhYjEQMA4GA1UECwwHRGV2aWNlczERMA8GA1UEAwwIbTVn
by1wdWIwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQDwesIQB5i/AEO9
pC8wWEH40RZT53wXkn81w9DYtZrSU/Y1Aih2MlgoG97AdlAhnQXMrCkpZ48J46OA
h0B+hZlAsQwZIl9Rleirt1gr4M19XXANTQDs/DwCuQ6tUI0k8ySVW+b0MAPbtxbO
sheYUho/XU7tNCZVOjg+OX1PxM4N9uKp6K77cpJHQkhE2hqVIhYgLOejj1D+V5OT
BkNZ2qVM+7M0GtS8EWQyDYVuKgBdrlaAXVaBUVMs0L8GicI/cEB0qzebhZhv0ANP
U0mILQQ0i7H3HdZIxhJye0yeff82akjfY7Cc/TRRVyi4IPjNXtmLHUWnQghrFywh
7vCnPyObAgMBAAGjQjBAMB0GA1UdDgQWBBRylU3ZhLbvd8w4veGLeb8giqTm8jAf
BgNVHSMEGDAWgBRu9pN750sa+WtHhp1Tm82gITiXWTANBgkqhkiG9w0BAQsFAAOC
AgEAU93GH3175NH4D5Ibzfq6kc8/ZWGzTOXAOMhXN3Sefg1oEuJR5f53u0+35ab0
Lg3qM2JroIAFulAKQBbSvQ70+sk46KtljI3VBZSQrmzTqh88R2hXifpPDGn58K9N
c/mYd8/BMJcZx9SIlcll+OlBGq3JzajWWl8bg8LVgDDovfr4CGcKuSizm4WC8soG
zCjXjQv8Vyikhw6UzUhgBU9iXV9MMs2k1iieXQDLoYZq+t/mtMjnRmb0SrNV0/XC
8ZjwRiQmkJv1ADefwJjPzagBy/YvoMg/e1IV9KNrS9zUgw1/z8f466FeqlKuUczW
43ho2X1oXPB4YS1LKEyI5ipOaKew8i6lle/r6GFrEI85S/ic+foPZhNlLAU9rsN2
84RqEyl1V87YkxO6/L/C2ebenpizJLPH00GwKtlkSBIhfHyfCVOT1koNYHG5sG/0
CzMaY5elcFJQREMUtT03LnFI1v/nb9ulvNY16eODi9YklLWmuiCrxMCCcHVQPuVY
hRAbsWhYD10LFPVCMQHTnwpITAkfheMThMY+HOkObfj5H7O5q2e3WFRP6b6EIOwJ
AypR3wVRJ0f31ntqpYjpmAAmqlCmkhxTMz1IdteGOeTVMK59GSMbqYObVqQGCEuw
4YXW3Gup0KiWk7ZsOijN/SHjm/nP8SojOqzK/sB27EAafm4=
-----END CERTIFICATE-----
)EOF";

// -------- CLIENT KEY (m5go-pub) ----------
// Clé privée correspondant au certificat client ci-dessus (stockée en clair dans le firmware).
static const char client_key[] PROGMEM = R"EOF(
-----BEGIN PRIVATE KEY-----
MIIEvgIBADANBgkqhkiG9w0BAQEFAASCBKgwggSkAgEAAoIBAQDwesIQB5i/AEO9
pC8wWEH40RZT53wXkn81w9DYtZrSU/Y1Aih2MlgoG97AdlAhnQXMrCkpZ48J46OA
h0B+hZlAsQwZIl9Rleirt1gr4M19XXANTQDs/DwCuQ6tUI0k8ySVW+b0MAPbtxbO
sheYUho/XU7tNCZVOjg+OX1PxM4N9uKp6K77cpJHQkhE2hqVIhYgLOejj1D+V5OT
BkNZ2qVM+7M0GtS8EWQyDYVuKgBdrlaAXVaBUVMs0L8GicI/cEB0qzebhZhv0ANP
U0mILQQ0i7H3HdZIxhJye0yeff82akjfY7Cc/TRRVyi4IPjNXtmLHUWnQghrFywh
7vCnPyObAgMBAAECggEAa6BVSD1OJHWm27ImmHl3lTdmych5ZkdffZ2U09h7YRTI
xTKhDz067UCD8hlBhbm0BcUraud5QhDKdVSTDc0XKLfUVU9n36i7CFc3M/QZo7j0
1E8ZUfcVmJZgNjst4FKdlecat0DiCSypHXrhSn+8VY2aLFlBqrUyxM6QAepv2hk9
b2QtAUV24t55nj6d7QpgIO+gsiJ2Xzx54S2YY5Pf/mxTslIE0t+ocFumvegPY7SG
FJkOqlV1LtuF47geBN9K5hWUcPMg1h5gijFQ39btrPHP4Pg7V4qFCjkrOpBWR/S5
8dpf+LZiOYJa49cvC5wajAM02RDyg7BsxaHKnhmEuQKBgQD+qCP0BJhbZ7eevU3y
v/0KDJNyQIdzMuUfngACNojiKzRfixPl0X5M8jL2ws4c8VrkLiL7Y+AiII1cV2FV
OEsnRrHQ7mL8baXWeCI4jW8wTX7ces2mnxlYLpx1H9BxxLUHiRK04DlL1kmeNwfv
ap7qULMvt4eBVcGLEPv59EF1lwKBgQDxv3lnjlA9w/DPYpdaIoY0lszlSkkagBdU
/A4WU2ZCXZslEJHGyTrC1Rerbo/IQ4YCce959MuBC7LPFVzC1Sg5l4slEoFsVYqI
7uFwxH19SYy582WKccF+vhzQrAGNRF29osVCRG3/0fg7imju983+bmi3dC1kMS1f
K2hIZxLqnQKBgCPXdnf5zZfP2UA2VKo961dmvbnu6yGDoEv66PVmx41Nl2l7IanO
+n/J9vJUKL5aGfjTpYjMXddvzXWZttFPwwQcJxrI8pWkuRqeffKHtYaO4bQWdKtm
6SJILS0u9R+OGAyfdkqO5IGP/3yNMki4MPW5tf6ZTjEd6Mex9EUR48SxAoGBAK2s
KyHUQZ096QB0Cdu9NcEOHUEUbxRUtW5ebhhn7ez7pnuoPbIb2tUhlZGZKj5rFBkp
lSt+S7z3lIvlAvENhYpqbpJBiy0y/wWE5/zFjIm3jxv/2hDtzF6rYbQf/jVoyhd0
mlYTJxtD0xujQeN7r0d8Nkqlcf6qvbfDTXXGZvaZAoGBAMxmQoUUY0ibwviE8X7H
5PArmuSDoKY1JPBLzdmi7JyUJqCKrPDScqFsK8sNyLiO+8/O/XQf0XCzyFOGPhgy
H/zKp9PQRcXvSWIYAXTAdBR87EQcm2QxyJe0IDYE+hLK0NYIVM4YhVEDyFxJiq72
bsw9XMOYaaccbZWZg8rhxZGm
-----END PRIVATE KEY-----
)EOF";

// ------------ SENSORS ------------
// Objets pour les capteurs ENV II
Adafruit_SHT31  sht31 = Adafruit_SHT31();
Adafruit_BMP280 bmp280;

// ------------ MQTT CLIENT ------------
// Client TLS (WiFiClientSecure) encapsulé dans PubSubClient pour MQTT
WiFiClientSecure tlsClient;
PubSubClient mqtt(tlsClient);

unsigned long lastPub = 0;   // horodatage de la dernière publication

// ------------ UI COLORS (RGB565) ------------
// Palette de couleurs pour l'affichage (fond, cartes, texte, etc.)
static const uint16_t COLOR_BG      = 0x0000; // black
static const uint16_t COLOR_CARD    = 0x0005; // deep blue
static const uint16_t COLOR_ACCENT  = 0x07FF; // cyan
static const uint16_t COLOR_TEXT    = 0xFFFF; // white
static const uint16_t COLOR_OK      = 0x07E0; // green
static const uint16_t COLOR_WARN    = 0xF800; // red

// ------------ UI HELPERS ------------
// Dessine l'en-tête de l'écran
void uiDrawHeader() {
  auto& d = M5.Display;
  d.fillScreen(COLOR_BG);
  d.fillRoundRect(6, 4, d.width() - 12, 40, 8, COLOR_CARD);
  d.setTextColor(COLOR_ACCENT, COLOR_CARD);
  d.setTextSize(1.5);
  d.setCursor(14, 20);
  d.print("M5Go Secure ENV Publisher");
}

// Dessine les cartes statiques (Wi-Fi, MQTTS, info topic)
void uiDrawStatic() {
  auto& d = M5.Display;

  // WiFi card
  d.fillRoundRect(6, 50, (d.width() / 2) - 9, 50, 8, COLOR_CARD);
  d.setTextColor(COLOR_TEXT, COLOR_CARD);
  d.setTextSize(1.5);
  d.setCursor(12, 60);
  d.print("Wi-Fi");

  // MQTT card
  int rx = (d.width() / 2) + 3;
  d.fillRoundRect(rx, 50, (d.width() / 2) - 9, 50, 8, COLOR_CARD);
  d.setCursor(rx + 6, 60);
  d.print("MQTTS");

  // Data card
  d.fillRoundRect(6, 106, d.width() - 12, 126, 8, COLOR_CARD);
  d.setCursor(12, 120);
  d.setTextColor(COLOR_ACCENT, COLOR_CARD);
  d.print("Live data sent to topic m5go/env");
}

// Met à jour l'état Wi-Fi (texte + couleur OK/WARN)
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

// Met à jour l'état MQTT (texte + couleur OK/WARN)
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

// Affiche les valeurs de température, humidité et pression sur l'écran
void uiSetSensorValues(float t, float h, float p) {
  auto& d = M5.Display;
  d.setTextColor(COLOR_TEXT, COLOR_CARD);
  d.setTextSize(2);

  // Ligne 1: T / H
  d.fillRect(12, 150, d.width() - 24, 20, COLOR_CARD);
  d.setCursor(12, 150);
  d.print("T:");
  if (!isnan(t)) { d.print(t, 1); d.print("C"); }
  else d.print("--.-C");
  d.print("  H:");
  if (!isnan(h)) { d.print(h, 1); d.print("%"); }
  else d.print("--.-%");

  // Ligne 2: P
  d.fillRect(12, 180, d.width() - 24, 20, COLOR_CARD);
  d.setCursor(12, 180);
  d.print("P:");
  if (!isnan(p)) { d.print(p, 1); d.print("hPa"); }
  else d.print("---.-hPa");
}

// Affiche une info sur la fréquence de publication
void uiSetPublishInfo() {
  auto& d = M5.Display;
  d.setTextSize(1.5);
  d.setTextColor(COLOR_ACCENT, COLOR_CARD);
  d.fillRect(12, 210, d.width() - 24, 12, COLOR_CARD);
  d.setCursor(12, 210);
  d.print("Publishing every 5s");
}

// ------------ WIFI / MQTT / TLS ------------
// Connexion au Wi-Fi (bloquante tant que non connectée)
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uiSetWiFiStatus("Connecting...", false);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
  }
  uiSetWiFiStatus(WiFi.localIP().toString(), true);
}

// Configuration du contexte TLS avec la CA, le certificat client et la clé privée
void setupTLS() {
  tlsClient.setCACert(ca_cert);          // CA du broker
  tlsClient.setCertificate(client_cert); // certificat client m5go-pub
  tlsClient.setPrivateKey(client_key);   // clé privée du client
}

// Connexion au broker MQTT sur TLS (port 8883)
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
// Initialisation des capteurs ENV II sur I2C
void setupSensors() {
  Wire.begin();
  sht31.begin(0x44);
  bmp280.begin(0x76);
}

// ------------ SETUP / LOOP ------------
// Initialisation du M5, des capteurs, du Wi-Fi et de la pile TLS/MQTT
void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(1);
  uiDrawHeader();
  uiDrawStatic();

  Serial.begin(115200);

  setupSensors();
  connectWiFi();
  setupTLS();
  connectMQTT();
}

// Boucle principale : maintien des connexions et publication toutes les 5s
void loop() {
  M5.update();

  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
    setupTLS();  // on re-applique la config TLS si reconnection Wi-Fi
  }

  if (!mqtt.connected()) {
    connectMQTT();
  }

  mqtt.loop();   // gestion interne du client MQTT

  unsigned long now = millis();
  if (now - lastPub >= 5000) {
    lastPub = now;

    // Lecture capteurs
    float t = sht31.readTemperature();
    float h = sht31.readHumidity();
    float p = NAN;
    float pPa = bmp280.readPressure();
    if (!isnan(pPa)) p = pPa / 100.0f;

    // Mise à jour de l'affichage local
    uiSetSensorValues(t, h, p);
    uiSetPublishInfo();

    // Création du payload JSON
    char payload[160];
    snprintf(payload, sizeof(payload),
             "{\"temp_c\":%.2f,\"hum_pct\":%.2f,\"press_hpa\":%.2f}",
             t, h, p);

    // Publication MQTT (QoS 0, non-retained)
    mqtt.publish(TOPIC, payload, false);
  }
}

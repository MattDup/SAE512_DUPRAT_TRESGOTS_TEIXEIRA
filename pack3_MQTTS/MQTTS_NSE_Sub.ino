// Subscriber sécurisé : M5Stack Core s'abonne à m5go/env via MQTT/TLS
// et affiche les valeurs reçues en temps réel.

#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// -------- WIFI ----------
const char* WIFI_SSID = "Livebox-D580";
const char* WIFI_PASS = "jyxvet-0qyrPi";

// -------- MQTT ----------
const char* MQTT_HOST = "192.168.1.12";
const uint16_t MQTT_PORT = 8883;
const char* TOPIC = "m5go/env";

// -------- CA CERT ----------
// CA qui a signé le certificat du broker
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

// -------- CLIENT CERT (m5sub-display) ----------
// Certificat client du subscriber (identité m5sub-display)
static const char client_cert[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIEpDCCAoygAwIBAgIUZ0trSk8RXBw+4/8NrTPscJPOiGIwDQYJKoZIhvcNAQEL
BQAwaDELMAkGA1UEBhMCRlIxEjAQBgNVBAgMCU9jY2l0YW5pZTERMA8GA1UEBwwI
VG91bG91c2UxDDAKBgNVBAoMA0xhYjENMAsGA1UECwwETVFUVDEVMBMGA1UEAwwM
TG9jYWxNUVRULUNBMB4XDTI1MTExMTE3NDUyOFoXDTI2MTExMTE3NDUyOFowbDEL
MAkGA1UEBhMCRlIxEjAQBgNVBAgMCU9jY2l0YW5pZTERMA8GA1UEBwwIVG91bG91
c2UxDDAKBgNVBAoMA0xhYjEQMA4GA1UECwwHRGV2aWNlczEWMBQGA1UEAwwNbTVz
dWItZGlzcGxheTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAKRjJ/EO
LNc4bQ1e2G2OgoPxFw320BFVYGkGLzxR+rRxXPJl2AvUhFN2QYq9s/JgPJeBDBtV
zDjGSND6t5oPVd6F0/EYdp5ncvbu/onYoYphFHwKMBka6eTn2g3IDRG5X/ldbRGO
fsIMpW1QN+6TTB3vx39+2JStpxWlR/df6oJZmxuprrXlMJWEnXNg/A8DF0FqOmxU
COBCudIsMuNR4v+1idnSdVGOv9ud3UUJBOe9uGzeGU7eCTz6RuC/qSJzr0FEdFM4
f1FXZ/+MV44MArGZibUJO0J1IIu1lOY3L+IZS1gjOBlvj6SYJaUpoxpvzBxcZgm4
snqRjBH/4DNZrykCAwEAAaNCMEAwHQYDVR0OBBYEFIDubbgpQD3vIWkiVG/em5i2
H0YoMB8GA1UdIwQYMBaAFG72k3vnSxr5a0eGnVObzaAhOJdZMA0GCSqGSIb3DQEB
CwUAA4ICAQAql9iyTrkVrtjQTyY3A2VMowMoNR9bd+YBNrR98StlddHMdCBmHF+t
gznt5vU0cG+K0avqEu8W/VAfYIP8jEvZtYMAeNRDHZfHu8ZAR8NTqeb4WFCl8xbG
3ThnBBpXf6ZYDpg3ksszHKNg+lh/zmmtrdPA8oxytqYf4/uLCSSrcXFlFIFZHhUx
Zqd4h/UMcklcmg9myH+fLssjTRGHunF85+GPjZrrLLL9bXnrtSVu7SPJe5mwhM+q
wvzD+MS24I/yo2HARO7my8Xi5XKukhLwwaAA4othkiQ5JD4bLz6VHt2ZXzAqPioV
PGrjVkAg5ydDuOQNlnz5S9l+iqEN5dmAS5oj5M1mCPzvws0Fa+pTZzVmeJWDIOEq
aUbCfygdBBt3nrL53pi/nBZWEzcFOwA63U2cRjHM6IGkr4ardNMpuY0nEsON6jmo
xUML1YzDGJVuwT4PXsN/JH7eFm8Lqkt4btb9hLi4ZpstCFPfCq7EAqWRnPDwsLJM
h/nJr5nIYVK7HJo53nFIpyXS5U9Xui5fL4G10Fd4GcYW64qwfD5VNdyd0qbBGoZH
hIwQytkRWVXvf0r+fevkZYymKicra33i0Cj3VJ6ecy8HeRXm+ueAc7X8NatfFJAL
4RwV97MjLKSnrMLa7wdA+kcAzcyu+7E5AVlB1LGowbk+xVkww9gAxg==
-----END CERTIFICATE-----
)EOF";

// -------- CLIENT KEY (m5sub-display) ----------
// Clé privée correspondant au certificat client du subscriber
static const char client_key[] PROGMEM = R"EOF(
-----BEGIN PRIVATE KEY-----
MIIEvgIBADANBgkqhkiG9w0BAQEFAASCBKgwggSkAgEAAoIBAQCkYyfxDizXOG0N
XthtjoKD8RcN9tARVWBpBi88Ufq0cVzyZdgL1IRTdkGKvbPyYDyXgQwbVcw4xkjQ
+reaD1XehdPxGHaeZ3L27v6J2KGKYRR8CjAZGunk59oNyA0RuV/5XW0Rjn7CDKVt
UDfuk0wd78d/ftiUracVpUf3X+qCWZsbqa615TCVhJ1zYPwPAxdBajpsVAjgQrnS
LDLjUeL/tYnZ0nVRjr/bnd1FCQTnvbhs3hlO3gk8+kbgv6kic69BRHRTOH9RV2f/
jFeODAKxmYm1CTtCdSCLtZTmNy/iGUtYIzgZb4+kmCWlKaMab8wcXGYJuLJ6kYwR
/+AzWa8pAgMBAAECggEAThvfSqbVn53sE0JRzBy+3zqbHcN8sf/DQ6PwpdS+Lyb+
Zx8FPPnJtm6mTeo9qpkXsVVcD1EBLKjqyIGY49cNkiI8xn49Pp8g/TwqLg2FFF90
gEvG9ubW5DFWZt37d8SBWdeOj50JleK0Z/CJ4w6SSFc6j9YRyvOIRLy4KU6l+zEd
hBArFDCO4TtZjw758EWUpUmInhBFSDimGPt0JlivjvO/0gHUaBsQvwYI7s3zJt3N
zrfyGLYZ8PvEAq+7+h6pt8ugTMHX64z1K5h9NAIs0QFMSGHD87tzTavYvOSgVNsY
6kODA7lantkLSY5QkE9dwibozTRG91gLkl+HQmPRkQKBgQDSQJpRdF/OhTrokQL2
MkeUoRMFVhZ9Gat5z4SaVHvARey+YxVuOMwR6R5hJuRMMfPkGP/w6FA5dHjzWeXD
xr+MnjKfQ2tXLoLsRtLVDKYTXXaaN9tQkY/+lo9ou7PpzXPDgmMpuKISUtt/lg4w
Dt1wUT6pG34CDoFlLBFFS96GQwKBgQDIJ8v+FmFqd5cf94HTOjmUOntUuW9nNHjx
7lHb0sR2HzQaw9ZUD8mcZy33ZFwEEq1Ytppxh1ZhdzxDIWgpemU3M7bXlzL1lQ+a
M0TMfWQ32BTiMKzHjxAFX1Z1faG2y0E4eawV0O7pvEAI2nJ5qohHdXLRHsoo2HTA
k7J67OocIwKBgQCOGtnyaeyDQisxmylcS1l0DHY1qhzjCuGKIAco5CcMKna7q26F
o9/RzzUYRtgQ48ZLCdaa9fmbC6zgFYelDgTTPu1KFaBMOYSFu8yt8LGi7w3FaDFU
QD4JvatKB2uvf4xZvRvWzBLGvbfbgQkv1Cw4yMDIPWuqajFstx8pLgFFzwKBgCXA
Y9hrzjnvjoCIBWOawstzcFmdlCaKHhm7kpL8oPOKlSBLObynMaafS2sy8awO/cUS
w/SPyzoc7C/ZODVCkZ6k0WK+cO0jDUtPSjWrnOBvkBjNh3koQaRRxBPq+zpoAcgu
IsgGnVlWmVlSIm9SO9wGif5paUXk9bhw4yQOVWWzAoGBAJ7+lEtDDcV1lEFW98Oi
Z91xG4JfaXj7uRCEq7cSAgjF1KqKoqzi3rfNGyqLD0C19mpb9nb4HmIsDzf4V4tH
G4h3jb+0pIgSW1gcPov5SiJw5cMqqRhf1f/6lH34S9t+BgjFTSXyAVz17e3zzMOP
AjIBJJ7vNCi5FWU+a4w0gtm5
-----END PRIVATE KEY-----
)EOF";

// ------------ MQTT CLIENT ------------
// Client TLS pour MQTT
WiFiClientSecure tlsClient;
PubSubClient mqtt(tlsClient);

// ------------ UI COLORS ------------
// Palette couleur identique au publisher, adaptée à l'UI du subscriber
static const uint16_t COLOR_BG      = 0x0000; // black
static const uint16_t COLOR_CARD    = 0x0005; // deep blue
static const uint16_t COLOR_ACCENT  = 0x07FF; // cyan
static const uint16_t COLOR_TEXT    = 0xFFFF; // white
static const uint16_t COLOR_OK      = 0x07E0; // green
static const uint16_t COLOR_WARN    = 0xF800; // red

// ------------ UI HELPERS ------------
// En-tête de l'écran
void uiDrawHeader() {
  auto& d = M5.Display;
  d.fillScreen(COLOR_BG);
  d.fillRoundRect(6, 4, d.width() - 12, 40, 8, COLOR_CARD);
  d.setTextColor(COLOR_ACCENT, COLOR_CARD);
  d.setTextSize(1.5);
  d.setCursor(14, 20);
  d.print("M5Stack Secure ENV Subscriber");
}

// Parties statiques : cartes Wi-Fi, MQTTS, texte explicatif
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

// Affichage des valeurs T/H/P sur l'écran
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

// Indique qu'un nouveau message a été reçu
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

// Configuration TLS (CA, cert client, clé privée)
void setupTLS() {
  tlsClient.setCACert(ca_cert);
  tlsClient.setCertificate(client_cert);
  tlsClient.setPrivateKey(client_key);
}

// ------------ MQTT CALLBACK ------------
// Callback appelée à chaque message MQTT reçu sur les topics abonnés
void handleMessage(char* topic, byte* payload, unsigned int length) {
  static char buf[256];
  unsigned int len = (length < sizeof(buf)-1) ? length : sizeof(buf)-1;
  memcpy(buf, payload, len);
  buf[len] = '\0';

  String msg = String(buf);

  float t = NAN, h = NAN, p = NAN;
  // Parsing très simple du JSON par recherche de sous-chaînes
  int ti = msg.indexOf("\"temp_c\":");
  int hi = msg.indexOf("\"hum_pct\":");
  int pi = msg.indexOf("\"press_hpa\":");
  if (ti >= 0) t = msg.substring(ti + 9).toFloat();
  if (hi >= 0) h = msg.substring(hi + 10).toFloat();
  if (pi >= 0) p = msg.substring(pi + 12).toFloat();

  // Mise à jour de l'affichage
  uiSetSensorValues(t, h, p);
  uiSetLastUpdate();

  // Debug série
  Serial.print("Msg [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(msg);
}

// ------------ MQTT ------------
// Connexion au broker, abonnement, et boucle de reconnexion
void connectMQTT() {
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(handleMessage);

  while (!mqtt.connected()) {
    String cid = "m5sub-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    uiSetMQTTStatus("Connecting...", false);

    if (mqtt.connect(cid.c_str())) {
      uiSetMQTTStatus("Connected", true);
      mqtt.subscribe(TOPIC);   // abonnement au topic m5go/env
    } else {
      uiSetMQTTStatus("Err:" + String(mqtt.state()), false);
      delay(1500);
    }
  }
}

// ------------ SETUP / LOOP ------------
// Initialisation générale
void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(1);
  uiDrawHeader();
  uiDrawStatic();

  Serial.begin(115200);

  connectWiFi();
  setupTLS();
  connectMQTT();
}

// Boucle principale : maintien du Wi-Fi, de MQTT et traitement des messages
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
}

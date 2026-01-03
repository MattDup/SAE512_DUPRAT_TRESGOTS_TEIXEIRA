// Mesures ENV II (SHT31 + BMP280) sur M5Go et publication en clair sur un broker MQTT (sans TLS)

#include <M5Unified.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

#include <Adafruit_SHT31.h>
#include <Adafruit_BMP280.h>

// Identifiants Wi-Fi (réseau non chiffré au niveau application, seulement WPA2 au niveau Wi-Fi)
const char* WIFI_SSID = "Arii";
const char* WIFI_PASS = "safeplace";

// Paramètres du broker MQTT (sans TLS)
const char* MQTT_HOST = "10.169.78.170";
const uint16_t MQTT_PORT = 1883;
const char* TOPIC = "m5go/env";

// Objets pour les capteurs ENV II
Adafruit_SHT31  sht31 = Adafruit_SHT31();
Adafruit_BMP280 bmp280;

// Client TCP sans TLS -> tout le trafic MQTT passe en clair
WiFiClient wifiClient;       // pas de TLS -> trafic en clair
PubSubClient mqtt(wifiClient);

unsigned long lastPub = 0;   // timestamp de la dernière publication (ms depuis boot)

// Fonction utilitaire : affiche une ligne de texte sur l'écran à la ligne "line"
void drawLine(const String& s, int line) {
  M5.Display.setCursor(0, 24 * line);
  M5.Display.fillRect(0, 24 * line, M5.Display.width(), 24, BLACK);
  M5.Display.print(s);
}

// Connexion au Wi-Fi (bloquante tant que non connectée)
void connectWiFi() {
  WiFi.mode(WIFI_STA);                     // mode station (client)
  WiFi.begin(WIFI_SSID, WIFI_PASS);        // démarrage de la connexion
  drawLine("WiFi: connecting...", 0);
  while (WiFi.status() != WL_CONNECTED) {  // boucle jusqu'à connexion
    delay(300);
  }
  // Une fois connecté, on affiche l'adresse IP locale
  drawLine("WiFi: " + WiFi.localIP().toString(), 0);
}

// Connexion au broker MQTT (sans TLS)
void connectMQTT() {
  mqtt.setServer(MQTT_HOST, MQTT_PORT);    // configuration de l'adresse du broker
  while (!mqtt.connected()) {              // boucle jusqu'à connexion MQTT
    // clientId unique basé sur l'adresse MAC de l'ESP32
    String cid = "m5go-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    drawLine("MQTT: connecting...", 1);
    if (mqtt.connect(cid.c_str())) {
      drawLine("MQTT: connected", 1);
    } else {
      // Affiche le code d'erreur MQTT (state)
      drawLine("MQTT rc=" + String(mqtt.state()), 1);
      delay(1500);
    }
  }
}

// Initialisation des capteurs I2C ENV II
void setupSensors() {
  Wire.begin();                           // SDA=21, SCL=22 (bus I2C par défaut du M5)
  if (!sht31.begin(0x44)) {              // initialisation SHT31 à l'adresse 0x44
    drawLine("SHT31 not found", 2);
  }
  if (!bmp280.begin(0x76)) {             // initialisation BMP280 à l'adresse 0x76
    drawLine("BMP280 not found", 3);
  }
}

// Fonction setup : initialisation générale (écran, série, capteurs, Wi-Fi, MQTT)
void setup() {
  auto cfg = M5.config();                 // configuration par défaut M5Unified
  M5.begin(cfg);
  M5.Display.setRotation(1);              // orientation paysage
  M5.Display.fillScreen(BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(WHITE, BLACK);
  drawLine("M5Go ENV -> MQTT", 0);        // en-tête sur l'écran

  Serial.begin(115200);                   // debug série
  setupSensors();                         // init capteurs ENV II
  connectWiFi();                          // connexion Wi-Fi
  connectMQTT();                          // connexion au broker MQTT
}

// Boucle principale : assure la connexion et publie toutes les 5 secondes
void loop() {
  M5.update();                            // gestion des boutons / événements M5

  // Si on perd le Wi-Fi ou MQTT, on tente de se reconnecter
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (!mqtt.connected()) connectMQTT();
  mqtt.loop();                            // gestion interne du client MQTT (keep-alive, etc.)

  unsigned long now = millis();
  if (now - lastPub >= 5000) {  // toutes les 5 s
    lastPub = now;

    // Lecture capteur SHT30/SHT31
    float t = sht31.readTemperature();     // température en °C (NAN si échec)
    float h = sht31.readHumidity();        // humidité relative en %

    // Lecture capteur BMP280
    float p = NAN;
    float pPa = bmp280.readPressure();     // pression en Pascal
    if (!isnan(pPa)) p = pPa / 100.0f;     // conversion Pa -> hPa

    // Construction du payload JSON (format texte)
    char payload[160];
    snprintf(payload, sizeof(payload),
             "{\"temp_c\":%.2f,\"hum_pct\":%.2f,\"press_hpa\":%.2f}",
             t, h, p);

    // Affichage local sur l'écran
    drawLine("T: " + String(t,1) + " C  H: " + String(h,1) + " %", 2);
    drawLine("P: " + String(p,1) + " hPa", 3);
    drawLine(String("PUB ")+TOPIC, 4);

    // Publication MQTT sur le topic (QoS 0, message non retenu)
    mqtt.publish(TOPIC, payload, false);
  }
}

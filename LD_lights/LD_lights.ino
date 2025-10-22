#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <time.h>

const char* WIFI_SSID  = "YOUR_WIFI";
const char* WIFI_PASS  = "YOUR_WIFI_PASSWORD";
const char* AIO_USERNAME = "YOUR_AIO_USERNAME";
const char* AIO_KEY      = "YOUR_AIO_KEY";
const char* DEVICE_ID    = "A";  //  "B" for the other esp-32

const char* MQTT_HOST = "io.adafruit.com";
const uint16_t MQTT_PORT = 1883;
// You can keep the plain feed or switch to /json. Either works.
String FEED_PATH = String(AIO_USERNAME) + "/feeds/longlamp.shared";

const int PIN_GATE   = 25;
const int PIN_BUTTON = 26;

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET   -1
#define OLED_I2C_ADDR 0x3C
#define SDA_PIN 32
#define SCL_PIN 33
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Timezone
const char* TZ_PST = "PST8PDT,M3.2.0/2,M11.1.0/2";
const char* NTP1 = "pool.ntp.org";
const char* NTP2 = "time.nist.gov";

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// ===== Reconnect backoff (NEW) =====
unsigned long nextMqttTryMs = 0;
unsigned long backoffMs = 1000;                 // start at 1s
const unsigned long BACKOFF_MAX_MS = 30000;     // cap at 30s

// State
bool myOn = false;
uint8_t myBrightness = 255;

// Button debounce + edge
bool btnReading = HIGH, btnLastReading = HIGH, btnStable = HIGH;
unsigned long lastDebounceMs = 0;
const unsigned long DEBOUNCE_MS = 30;
unsigned long lastToggleMs  = 0;
const unsigned long MIN_TOGGLE_GAP = 250;

// Publish rate limit with queue
unsigned long lastPublishMs = 0;
// soften bursts a bit (was 600)
const unsigned long MIN_PUB_GAP_MS = 1200;
bool publishPending = false;
bool pendingOn = false;

// Timers
unsigned long lastWiFiChk = 0;

// Heart bitmap
const uint8_t HEART_BMP[] PROGMEM = {
  0b00000000,0b00000000,
  0b00011000,0b01100000,
  0b00111100,0b11110000,
  0b01111111,0b11111000,
  0b01111111,0b11111000,
  0b00111111,0b11110000,
  0b00011111,0b11100000,
  0b00001111,0b11000000,
  0b00000111,0b10000000
};

void driveLamp(bool on){ digitalWrite(PIN_GATE, on?HIGH:LOW); }

String mkState(bool on){
  return String("{\"on\":")+(on?"true":"false")+
         ",\"brightness\":"+String(myBrightness)+
         ",\"from\":\""+DEVICE_ID+"\"}";
}

void doPublish(bool on){
  String p = mkState(on);
  bool ok = mqtt.publish(FEED_PATH.c_str(), p.c_str(), true); // retained
  if (ok) {
    lastPublishMs = millis();
    publishPending = false;
  }
  Serial.print("PUB "); Serial.println(ok ? p : String("FAIL ")+p);
}

void requestPublish(bool on){
  unsigned long now = millis();
  if (now - lastPublishMs >= MIN_PUB_GAP_MS) {
    if (mqtt.connected()) doPublish(on);
    else { publishPending = true; pendingOn = on; }
    return;
  }
  publishPending = true;
  pendingOn = on;
}

String fmtNow(){
  struct tm tinfo;
  if(!getLocalTime(&tinfo)) return String("--:--");
  int hour=tinfo.tm_hour%12; if(hour==0) hour=12;
  char buf[16];
  snprintf(buf,sizeof(buf),"%d:%02d %s",hour,tinfo.tm_min,tinfo.tm_hour<12?"AM":"PM");
  return String(buf);
}

void drawCentered(const String& txt,int y,int size){
  display.setTextSize(size);
  display.setTextColor(SSD1306_WHITE);
  int16_t x1,y1; uint16_t w,h;
  display.getTextBounds(txt,0,0,&x1,&y1,&w,&h);
  int x=max(0,(SCREEN_WIDTH-(int)w)/2);
  display.setCursor(x,y);
  display.print(txt);
}

void showLastSeen(){
  display.clearDisplay();
  drawCentered("Last Seen:", SCREEN_HEIGHT==32?0:8, SCREEN_HEIGHT==32?1:2);
  String t=fmtNow();
  drawCentered(t, SCREEN_HEIGHT==32?16:34, SCREEN_HEIGHT==32?1:2);
  int hx=SCREEN_WIDTH-16, hy=SCREEN_HEIGHT==32?16:34;
  display.drawBitmap(hx,hy,HEART_BMP,11,9,SSD1306_WHITE);
  display.display();
}

// MQTT
void onMsg(const String& msg){
  Serial.print("RX  "); Serial.println(msg);
  if(msg.indexOf(String("\"from\":\"")+DEVICE_ID+"\"")!=-1){ Serial.println("self→ignore"); return; }
  bool on=(msg.indexOf("\"on\":true")!=-1);
  myOn=on; driveLamp(myOn);
  showLastSeen();
}

void mqttCallback(char* topic, byte* payload, unsigned int len){
  String m; m.reserve(len);
  for(unsigned i=0;i<len;i++) m+=(char)payload[i];

  String t(topic);
  if (t.endsWith("/throttle")) {  // see account-level rate/cooldown notices
    Serial.print("THROTTLE "); Serial.println(m);
    return;
  }
  onMsg(m);
}

// ======= Wi-Fi helpers =======
bool ensureWiFi(){
  if(WiFi.status()==WL_CONNECTED) return true;
  Serial.print("WiFi→ "); Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA); WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID,WIFI_PASS);
  unsigned long t=millis();
  while(WiFi.status()!=WL_CONNECTED && millis()-t<15000){ delay(250); Serial.print("."); }
  Serial.println(WiFi.status()==WL_CONNECTED?" OK":" FAIL");
  if(WiFi.status()==WL_CONNECTED) configTzTime(TZ_PST,NTP1,NTP2);
  return WiFi.status()==WL_CONNECTED;
}

// ======= MQTT with exponential backoff (NEW) =======
bool ensureMqtt(){
  if(mqtt.connected()) return true;
  if(millis() < nextMqttTryMs) return false; // wait until allowed to try again

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setKeepAlive(60);
  mqtt.setSocketTimeout(10);
  mqtt.setBufferSize(256);

  // Unique client ID: device letter + last 3 bytes of MAC
  uint64_t mac = ESP.getEfuseMac();
  char macfrag[7];
  snprintf(macfrag, sizeof(macfrag), "%02X%02X%02X", (uint8_t)(mac>>16), (uint8_t)(mac>>8), (uint8_t)mac);
  String cid = String("esp32-longlamp-") + DEVICE_ID + "-" + macfrag;

  Serial.print("MQTT connect "); Serial.print(cid); Serial.print(" ... ");

  bool ok = mqtt.connect(cid.c_str(), AIO_USERNAME, AIO_KEY);
  if (ok) {
    Serial.println("OK");
    backoffMs = 1000; // reset backoff on success
    mqtt.subscribe(FEED_PATH.c_str());
    mqtt.subscribe((String(AIO_USERNAME) + "/throttle").c_str());
    requestPublish(myOn);   // single retained state after reconnect
    return true;
  } else {
    int rc = mqtt.state();
    Serial.print("FAIL rc="); Serial.println(rc);
    backoffMs = min(backoffMs * 2, BACKOFF_MAX_MS);                  // exponential
    nextMqttTryMs = millis() + backoffMs + (uint32_t)random(0, 500); // jitter
    return false;
  }
}

// Setup/loop
void setup(){
  Serial.begin(115200);

  // Stagger boot so A & B don't sync-storm
  randomSeed(esp_random());
  delay(random(0, 2000));

  pinMode(PIN_GATE,OUTPUT); digitalWrite(PIN_GATE,LOW);
  pinMode(PIN_BUTTON,INPUT_PULLUP);

  Wire.begin(SDA_PIN,SCL_PIN);
  Wire.setClock(100000);
  delay(200);
  if(!display.begin(SSD1306_SWITCHCAPVCC,OLED_I2C_ADDR)){
    Serial.println("SSD1306 init failed"); while(true){ delay(1000); }
  }
  display.clearDisplay();
  drawCentered("LongLamp Ready",12,2);
  drawCentered("Waiting...",36,2);
  display.display();

  ensureWiFi();
}

void loop(){
  if(millis()-lastWiFiChk>3000){ lastWiFiChk=millis(); if(WiFi.status()!=WL_CONNECTED) ensureWiFi(); }
  if(WiFi.status()==WL_CONNECTED) ensureMqtt();
  if(mqtt.connected()) mqtt.loop();

  if(publishPending && (millis()-lastPublishMs >= MIN_PUB_GAP_MS) && mqtt.connected()){
    doPublish(pendingOn);
  }

  // Debounced, edge-triggered button (act on RELEASE)
  bool reading = digitalRead(PIN_BUTTON);
  if (reading != btnLastReading) lastDebounceMs = millis();
  btnLastReading = reading;

  if (millis() - lastDebounceMs > DEBOUNCE_MS) {
    if (btnStable != reading) {
      btnStable = reading;
      if (btnStable == HIGH) {
        if (millis() - lastToggleMs > MIN_TOGGLE_GAP) {
          lastToggleMs = millis();
          myOn = !myOn;
          driveLamp(myOn);
          if (mqtt.connected()) requestPublish(myOn);
          else { publishPending = true; pendingOn = myOn; }
        }
      }
    }
  }
}

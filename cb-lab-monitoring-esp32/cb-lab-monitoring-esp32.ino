// ═══════════════════════════════════════════════
//  ESP32 — WiFi + UART + AWS IoT + ILI9341 
//  UI: Carbelim Modern Grid Dashboard
// ═══════════════════════════════════════════════

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <time.h> 

// ── WiFi ─────────────────────────────────────────
#define WIFI_SSID     "CARBELIM_EX"
#define WIFI_PASSWORD "Rahul@25$"

// ── AWS Configuration ────────────────────────────
const char* AWS_ENDPOINT = "a3gv8ditd263ec-ats.iot.us-east-1.amazonaws.com";
const char* THING_NAME   = "BTTE1250007";
const char* MQTT_TOPIC   = "devices/BTTE125/0007/telemetry";

// AWS Publish Timer (10 Seconds)
const unsigned long AWS_INTERVAL = 10000; 

// AWS Root CA 1
static const char AWS_ROOT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";

// Device Certificate
static const char DEVICE_CERT[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDWTCCAkGgAwIBAgIUXDFIhB6F6/o279M9Ny46wsuXBuowDQYJKoZIhvcNAQEL
BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g
SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTI2MDMyMDE0MDIy
MloXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0
ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAJZLxe5svX3SkXfbbl1g
GZRvnRRHtHg447FUCnjjOKgnMxV2G0nRoq7I448w7IYsMA9HYMuqYhOO5eg0Zp38
UVIQcbTn8NJETcvC1mocxGEVLcQQPnjnLJtcy1AJATknR4g4C4/P/70xsbiG90ux
dnS8c99DZomL7X0JimZ4c6KSZEqrMPlLV5PD1Rz47pVhZMZ50psPpxPAsWNS2fWz
WS47D+eCH1btySNY78WuihKbNg14cYNAZXX+X5OnmbkhyDVjt/027QxO2YiTIWUJ
auMOyHvJ2IYeGdkjJR1d1ddGW4EAFTN80jr2jgZ1RO8cl+xKl+4EGgWmdSG3hUEr
cQcCAwEAAaNgMF4wHwYDVR0jBBgwFoAUVc/8nIdXERBKpeNN/Y+5ekGJp18wHQYD
VR0OBBYEFPcNTB3UMr0FLYPTAexb7TuQL7eMMAwGA1UdEwEB/wQCMAAwDgYDVR0P
AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQBXCdnfZk8VegYc01RvSSfrIC/k
cUBNmpgkkJvau798lXVXt4LEL5rWq3Gy5ssmMBeSiwZ58SyMb69xu4HosWv8AUkm
NswyygO/Jzf7oPFaxXsVeazoGJCYTZX1kb+cgpprzKZKuQ6J7se8vps8Tqo7ltfs
KLTWGtj72mTKAeVwBuKhekvY7HIYfAtLFyzNmMUxoXCdTvLFJ9FLQC5onW42lX3h
sEKhEVyR+RW8gF5FHcQRnHH7ddsUOkqwF5xS0Csryk9A7xhCldVgtEeRVu2wNM5Z
Sb8V+eKx6Ezn+rC6N0RD1gH/TR2Z47dEcdAnvyo4bmnnQR4il2gItsIoNqtY
-----END CERTIFICATE-----
)EOF";

// Device Private Key
static const char PRIVATE_KEY[] PROGMEM = R"EOF(
-----BEGIN RSA PRIVATE KEY-----
MIIEowIBAAKCAQEAlkvF7my9fdKRd9tuXWAZlG+dFEe0eDjjsVQKeOM4qCczFXYb
SdGirsjjjzDshiwwD0dgy6piE47l6DRmnfxRUhBxtOfw0kRNy8LWahzEYRUtxBA+
eOcsm1zLUAkBOSdHiDgLj8//vTGxuIb3S7F2dLxz30NmiYvtfQmKZnhzopJkSqsw
+UtXk8PVHPjulWFkxnnSmw+nE8CxY1LZ9bNZLjsP54IfVu3JI1jvxa6KEps2DXhx
g0Bldf5fk6eZuSHINWO3/TbtDE7ZiJMhZQlq4w7Ie8nYhh4Z2SMlHV3V10ZbgQAV
M3zSOvaOBnVE7xyX7EqX7gQaBaZ1IbeFQStxBwIDAQABAoIBAFkiORGMldjQD4uI
rGxeOO1qQDstx17hRk/9anSFNS2sicQ5ljdyR0vnQeXC+xMbk6kMCODgfGfjAIB0
y0L4x/WmYPkL8SuSbJziQS0PQHvfBpRXmeU0HKVLJBykKoCNxgOIUXsJvT6kASVR
jcnrRjg3J3UKUP5T6RWVmenpl5Ka3IrZr+SMhcHMl/tIcNpH2Km6AqF9SQPG6PRY
zQTbYwx5zdN+eyyZI/mmKqP1CjnK8xVdiOqcpzxuv1CYNsCIYQXmrinMSTnH6mxX
25ISijMIfaG51LwbENbTNA6gYNEWn1YmpcYkIpF53NdAwS99fuK5Rs1c2l19hDs2
kjmyyXkCgYEAxZpXhACXt6DDjcJU0QnbAUbJ/OfY7XUFuzujQgGIf8DdIE9/rsGt
lJhV+38978XM9EFzBlk9HAkCEOrWvAmQLlpkKLGJGhnaW6qKzJmSfo7oYXDdJ1bM
eVfU8hPkyFmvNfYtDM2TPFh7/bfA5nT61gFAxypshG4W+Eb+ADxxJPsCgYEAwrZs
rGI8RjSFnKuvm6qYg+hRC4/Ksg06He+phmIRcGEGUccZ+BuOLFgYsc1/3NNPC4B4
HXzf6VBjBsIchFc6mGyhEYdS75QU8hQ2NrvyJI+fRKcdhZoZ/0lW0mt4s2BxJV2u
HZhVTChBBXmeYwUrdP+vp7Zcq1heqdDpeTasbmUCgYEAjNZChey+Nby5ZDIbrZmu
pCxGDqFYdXYwjZAl/geFuDhH4p2GzZFxHDWvIr8/78M1Hun/B4lXCsJI3LHkNsVC
JCd8t5xBsX63qRnWL5lNVjKY4cpFdaJhviKhvZ/8Mefp/zXkSfnDdQFD7kOnkxnR
JfJirxFHouOsL2y54IUzxz0CgYAIwXFDkytIsOXtNJ8x7crr649xaKbhuMyLOhKy
c0h7eDFzjfNx/7M19UQvAqGkSXA9gSplkQ70i+PYEvR7UvzUi9X2VtJ46XzDofsK
HQt3b2nnCWi/cD2JCEA6OBD2Z/jOiFbjB82WHh1GZMCljT72BOVeMkoafl/LXRSS
mc3JLQKBgAIt1C3rQKAHes0kDvu1eSMR8npUdd6xTXk+vAz4bhoMO/g5/cKT8Evy
3JjbWZkoByOcxLVdb1Jvjhc9iyc4IP153JVfRHg4+aRM3l4kfWLLtZdjT48eqmHJ
4gZ0v9Jjtk9KUtVvDOziZkErHA2KT72NjZR+cGQeJDsUHeXoTfui
-----END RSA PRIVATE KEY-----
)EOF";

WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

// ── ILI9341 Display Pins ─────────────────────────
#define TFT_CS   5
#define TFT_RST  4
#define TFT_DC   2
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// ── UART from STM32 ──────────────────────────────
HardwareSerial stm32Serial(2);
#define STM32_RX  16
#define STM32_TX  17

// ── State Variables ──────────────────────────────
String inputBuffer = "";
String awsPayload = "";        
unsigned long lastAwsPublish = 0; 
bool hasFreshData = false;

// ── Custom Colors (RGB565) ───────────────────────
#define BG_COLOR       tft.color565(10, 15, 20)    
#define HEADER_COLOR   tft.color565(0, 150, 80)    
#define LABEL_COLOR    tft.color565(180, 180, 180) 
#define DIVIDER_COLOR  tft.color565(50, 60, 70)    

#define C_TURB  tft.color565(0, 255, 255)   
#define C_PH    tft.color565(255, 100, 255) 
#define C_TDS   tft.color565(100, 150, 255) 
#define C_PM    tft.color565(255, 200, 0)   
#define C_CO2   tft.color565(200, 255, 100) 
#define C_VOC   tft.color565(255, 150, 50)  
#define C_TEMP  tft.color565(255, 80, 80)   
#define C_HUM   tft.color565(0, 200, 255)   
#define C_CH2O  tft.color565(150, 255, 150) 
#define C_CO    tft.color565(220, 220, 220) 
#define C_O3    tft.color565(180, 100, 255) 
#define C_NO2   tft.color565(255, 180, 100) 

#define COL1 10
#define COL2 115
#define COL3 220

#define ROW1_LBL 35  
#define ROW1_VAL 48  
#define ROW2_LBL 85  
#define ROW2_VAL 98
#define ROW3_LBL 135 
#define ROW3_VAL 148
#define ROW4_LBL 185 
#define ROW4_VAL 198

// ═══════════════════════════════════════════════
//  TIME SYNC
// ═══════════════════════════════════════════════
void syncTime() {
  configTime(19800, 0, "pool.ntp.org", "time.nist.gov"); // IST Offset
  Serial.print("[NTP] Waiting for time sync");
  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println("\n[NTP] Time Synced!");
}

// ═══════════════════════════════════════════════
//  DISPLAY UI SETUP
// ═══════════════════════════════════════════════
void initDisplay() {
  tft.begin();
  tft.setRotation(1); 
  tft.fillScreen(BG_COLOR);

  tft.fillRect(0, 0, 320, 25, HEADER_COLOR);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2); 
  tft.setCursor(22, 5); 
  tft.print("Carbelim Lab Monitoring");

  tft.drawFastHLine(0, 75, 320, DIVIDER_COLOR);  
  tft.drawFastHLine(0, 125, 320, DIVIDER_COLOR); 
  tft.drawFastHLine(0, 175, 320, DIVIDER_COLOR); 
  tft.drawFastVLine(105, 30, 210, DIVIDER_COLOR); 
  tft.drawFastVLine(210, 30, 210, DIVIDER_COLOR);

  tft.setTextSize(1);
  tft.setTextColor(LABEL_COLOR);

  tft.setCursor(COL1, ROW1_LBL); tft.print("Turbidity (NTU)");
  tft.setCursor(COL2, ROW1_LBL); tft.print("pH Level");
  tft.setCursor(COL3, ROW1_LBL); tft.print("TDS (ppm)");

  tft.setCursor(COL1, ROW2_LBL); tft.print("PM2.5 (ug/m3)");
  tft.setCursor(COL2, ROW2_LBL); tft.print("CO2 (ppm)");
  tft.setCursor(COL3, ROW2_LBL); tft.print("VOC Grade");

  tft.setCursor(COL1, ROW3_LBL); tft.print("Temp (C)");
  tft.setCursor(COL2, ROW3_LBL); tft.print("Humidity (%)");
  tft.setCursor(COL3, ROW3_LBL); tft.print("CH2O (mg/m3)");

  tft.setCursor(COL1, ROW4_LBL); tft.print("CO (ppm)");
  tft.setCursor(COL2, ROW4_LBL); tft.print("O3 (ppm)");
  tft.setCursor(COL3, ROW4_LBL); tft.print("NO2 (ppm)");
}

// ═══════════════════════════════════════════════
//  UPDATE DISPLAY 
// ═══════════════════════════════════════════════
void updateDisplay(StaticJsonDocument<1024>& doc) {
  tft.setTextSize(2); 
  char buf[10];
  
  #define PRINT_VAL(x, y, color, format, value) \
    tft.setTextColor(color, BG_COLOR); \
    tft.setCursor(x, y); \
    sprintf(buf, format, value); \
    tft.print(buf);

  PRINT_VAL(COL1, ROW1_VAL, C_TURB, "%-6.1f", (float)doc["turbidity"]);
  PRINT_VAL(COL2, ROW1_VAL, C_PH,   "%-6.2f", (float)doc["ph"]);
  PRINT_VAL(COL3, ROW1_VAL, C_TDS,  "%-6.1f", (float)doc["tds"]);

  PRINT_VAL(COL1, ROW2_VAL, C_PM,  "%-6d", (int)doc["pm25"]);
  PRINT_VAL(COL2, ROW2_VAL, C_CO2, "%-6d", (int)doc["co2"]);
  PRINT_VAL(COL3, ROW2_VAL, C_VOC, "%-6d", (int)doc["voc"]);

  PRINT_VAL(COL1, ROW3_VAL, C_TEMP, "%-6.1f", (float)doc["temp"]);
  PRINT_VAL(COL2, ROW3_VAL, C_HUM,  "%-6.1f", (float)doc["humidity"]);
  PRINT_VAL(COL3, ROW3_VAL, C_CH2O, "%-6.3f", (float)doc["ch2o"]);

  PRINT_VAL(COL1, ROW4_VAL, C_CO,  "%-6.1f", (float)doc["co"]);
  PRINT_VAL(COL2, ROW4_VAL, C_O3,  "%-6.2f", (float)doc["o3"]);
  PRINT_VAL(COL3, ROW4_VAL, C_NO2, "%-6.2f", (float)doc["no2"]);
}

// ═══════════════════════════════════════════════
//  PAYLOAD TRANSLATOR (Raw Data -> AWS Format)
// ═══════════════════════════════════════════════
String buildAwsPayload(StaticJsonDocument<1024>& in) {
  StaticJsonDocument<1536> out;
  
  // Header Meta
  out["imei"] = "860710081332028";
  out["d"] = THING_NAME; 
  out["meter"] = 2;
  out["offline"] = 1;
  
  // Dynamic Timestamp
  time_t now = time(nullptr);
  struct tm t; 
  localtime_r(&now, &t);
  char tbuf[22]; 
  strftime(tbuf, sizeof(tbuf), "%Y-%m-%d,%H:%M:%S", &t);
  out["date"] = String(tbuf);

  // Status fields
  out["d38"] = 25; 
  out["d39"] = 22; 
  out["d40"] = 23;
  out["device_type"] = 1;
  out["s1"] = 0;
  out["s2"] = WiFi.RSSI(); // Uses actual WiFi strength 
  out["s3"] = 183;

  // Re-mapping and scaling to integers as requested
  out["d1"]  = (int)((float)in["turbidity"] * 10);
  out["d2"]  = (int)((float)in["ph"] * 100);
  out["d3"]  = (int)((float)in["tds"] * 10);
  out["d4"]  = (int)in["pm1"];
  out["d5"]  = (int)in["pm25"];
  out["d6"]  = (int)in["pm10"];
  out["d7"]  = (int)in["co2"];
  out["d8"]  = (int)in["voc"];
  out["d9"]  = (int)((float)in["temp"] * 10);
  out["d10"] = (int)((float)in["humidity"] * 10);
  out["d11"] = (int)((float)in["ch2o"] * 1000);
  out["d12"] = (int)((float)in["co"] * 10);
  out["d13"] = (int)((float)in["o3"] * 100);
  out["d14"] = (int)((float)in["no2"] * 100);
  
  out["d15"] = 0; out["d16"] = 0; out["d17"] = 0; out["d18"] = 0;

  // Validity Flags (1 if non-zero)
  out["i1"]  = ((float)in["turbidity"] != 0) ? 1 : 0;
  out["i2"]  = ((float)in["ph"] != 0) ? 1 : 0;
  out["i3"]  = ((float)in["tds"] != 0) ? 1 : 0;
  out["i4"]  = ((int)in["pm25"] != 0) ? 1 : 0; 
  out["i5"]  = ((int)in["co2"] != 0) ? 1 : 0;
  out["i6"]  = ((float)in["temp"] != 0) ? 1 : 0;
  out["i7"]  = ((float)in["humidity"] != 0) ? 1 : 0;
  out["i8"]  = ((float)in["ch2o"] != 0) ? 1 : 0;
  out["i9"]  = ((float)in["co"] != 0) ? 1 : 0;
  out["i10"] = ((float)in["o3"] != 0) ? 1 : 0;

  String payload;
  serializeJson(out, payload);
  return payload;
}

// ═══════════════════════════════════════════════
//  DATA PARSING
// ═══════════════════════════════════════════════
void processIncomingData(String jsonStr) {
  jsonStr.trim();
  if (!jsonStr.startsWith("{") || !jsonStr.endsWith("}")) return;

  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, jsonStr);
  if (err) return; 

  updateDisplay(doc); // Instant UI Update
  
  // Re-package the payload silently in the background
  awsPayload = buildAwsPayload(doc); 
  hasFreshData = true;
}

// ═══════════════════════════════════════════════
//  AWS PUBLISHING LOGIC
// ═══════════════════════════════════════════════
void pubSubCheckConnect() {
  if (!mqttClient.connected()) {
    Serial.print("[AWS] Connecting to IoT Core...");
    if (mqttClient.connect(THING_NAME)) {
      Serial.println(" Connected!");
    } else {
      Serial.print(" Failed! rc=");
      Serial.println(mqttClient.state());
    }
  }
  mqttClient.loop();
}

void publishToAWS() {
  pubSubCheckConnect(); 
  if (mqttClient.connected()) {
    if (mqttClient.publish(MQTT_TOPIC, awsPayload.c_str())) {
      Serial.println("[AWS] Published successfully: " + awsPayload);
    } else {
      Serial.println("[AWS] Publish failed.");
    }
  }
}

// ═══════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  
  stm32Serial.setRxBufferSize(1024); 
  stm32Serial.begin(115200, SERIAL_8N1, STM32_RX, STM32_TX);

  initDisplay();

  Serial.print("[WiFi] Connecting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\n[WiFi] Connected! IP: " + WiFi.localIP().toString());

  syncTime();
  wifiClient.setCACert(AWS_ROOT_CA);
  wifiClient.setCertificate(DEVICE_CERT);
  wifiClient.setPrivateKey(PRIVATE_KEY);
  mqttClient.setServer(AWS_ENDPOINT, 8883);
  
  mqttClient.setBufferSize(1536); 
  mqttClient.setKeepAlive(60);

  delay(100);
  while (stm32Serial.available()) stm32Serial.read();
  inputBuffer = "";
}

// ═══════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }

  // 1. Read instantly (Updates Screen in Real-Time)
  while (stm32Serial.available()) {
    char c = stm32Serial.read();
    if (c == '\n') {
      processIncomingData(inputBuffer);    
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }

  // 2. Publish quietly every 10 seconds (10000ms)
  if (strlen(AWS_ENDPOINT) > 30) { 
    if (hasFreshData && (millis() - lastAwsPublish >= AWS_INTERVAL)) {
      lastAwsPublish = millis(); 
      publishToAWS(); 
      hasFreshData = false; 
    }
  }
}

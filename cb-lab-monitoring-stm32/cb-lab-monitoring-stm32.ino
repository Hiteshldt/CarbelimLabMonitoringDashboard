#include <Wire.h>

// ================= UART (Fixed Pins for STM32duino) =================
HardwareSerial ZphsSerial(PA_10, PA_9);   // Air sensor
HardwareSerial EspSerial (PC_7,  PC_6);   // ESP32

// ================= PINS =================
#define TURBIDITY_PIN  A2
#define PH_PIN         A1
#define TDS_PIN        A0

// ================= CONSTANTS =================
#define VREF            3.3f
#define ADC_RES         4095.0f
#define SAMPLES         10
#define DIVIDER_SCALE   1.5f

#define ZPHS_BAUD       9600
#define ZPHS_RESP_LEN   26

const byte ZPHS_CMD[9] = {0xFF,0x01,0x86,0,0,0,0,0,0x79};

// ================= TIMING (Relaxed 2-Second Update) =================
#define SENSOR_INTERVAL   500    // Read analog sensors every 0.5 seconds
#define PUBLISH_INTERVAL  2000   // Send to ESP32 every 2.0 seconds
#define AIR_REQ_INTERVAL  1000   // Request air sensor every 1.0 seconds

unsigned long lastSensor = 0;
unsigned long lastSend   = 0;
unsigned long lastAirReq = 0;

// ================= GLOBALS =================
float g_turbidity = 0, g_ph = 0, g_tds = 0;
float ambientTemp = 25.0f;

struct AirData {
  bool valid;
  int pm1, pm25, pm10, co2, voc;
  float temp, hum, ch2o, co, o3, no2;
} air;

// ================= HELPERS =================
float safeF(float v) {
  if (isnan(v) || isinf(v)) return 0.0f;
  return v;
}

float readAvgVoltage(int pin, bool divider=false) {
  long sum = 0;
  for(int i=0;i<SAMPLES;i++) sum += analogRead(pin);
  float v = (sum / (float)SAMPLES) * (VREF / ADC_RES);
  return divider ? v * DIVIDER_SCALE : v;
}

// ── CUSTOM TURBIDITY (Raw Voltage * 100) ──────────────
float readTurbidity() {
  float v = readAvgVoltage(TURBIDITY_PIN, false); 
  return safeF(v * 100.0f);
}

// ── CUSTOM pH CALIBRATION (Piecewise Linear) ──────────
float readPH() {
  float v = readAvgVoltage(PH_PIN, false);
  float ph = 7.0f;

  if (v <= 1.92f) {
    // Map between 1.71V (4.0 pH) and 1.92V (7.0 pH)
    ph = 7.0f - ((1.92f - v) / 0.21f) * 3.0f;
  } else {
    // Map between 1.92V (7.0 pH) and 2.10V (9.25 pH)
    ph = 7.0f + ((v - 1.92f) / 0.18f) * 2.25f;
  }
  return safeF(ph);
}

// ── TDS CALIBRATION ───────────────────────────────────
float readTDS() {
  float v = readAvgVoltage(TDS_PIN);
  float coeff = 1.0f + 0.02f*(ambientTemp - 25.0f);
  float cv = v / coeff;
  float tds = (133.42f*cv*cv*cv - 255.86f*cv*cv + 857.39f*cv)*0.5f;
  return safeF(tds);
}

// ================= NON-BLOCKING AIR READ =================
void requestAirSensor() {
  ZphsSerial.write(ZPHS_CMD, sizeof(ZPHS_CMD));
}

void readAirSensor() {
  static byte buf[ZPHS_RESP_LEN];
  static int index = 0;

  while (ZphsSerial.available()) {
    byte b = ZphsSerial.read();

    if (index == 0 && b != 0xFF) continue;

    buf[index++] = b;

    if (index == ZPHS_RESP_LEN) {
      index = 0;

      if (buf[1] == 0x86) {
        air.valid = true;
        air.pm1  = (buf[2]<<8)|buf[3];
        air.pm25 = (buf[4]<<8)|buf[5];
        air.pm10 = (buf[6]<<8)|buf[7];
        air.co2  = (buf[8]<<8)|buf[9];
        air.voc  = buf[10];
        air.temp = (((buf[11]<<8)|buf[12]) - 500)*0.1f;
        air.hum  = (buf[13]<<8)|buf[14];
        air.ch2o = ((buf[15]<<8)|buf[16])*0.001f;
        air.co   = ((buf[17]<<8)|buf[18])*0.1f;
        air.o3   = ((buf[19]<<8)|buf[20])*0.01f;
        air.no2  = ((buf[21]<<8)|buf[22])*0.01f;

        ambientTemp = air.temp;
      }
    }
  }
}

// ================= SERIAL PRINT =================
void printData() {
  Serial.print("Turbidity: "); Serial.print(g_turbidity);
  Serial.print(" | pH: "); Serial.print(g_ph);
  Serial.print(" | TDS: "); Serial.print(g_tds);
  Serial.print(" | CO2: "); Serial.println(air.co2);
}

// ================= SEND JSON (Memory Optimized) =================
void sendESP() {
  char buf[256]; 
  char tStr[10], phStr[10], tdsStr[10];
  
  dtostrf(g_turbidity, 1, 1, tStr);
  dtostrf(g_ph, 1, 2, phStr);
  dtostrf(g_tds, 1, 1, tdsStr);

  if (air.valid) {
    char tempStr[10], humStr[10], ch2oStr[10], coStr[10], o3Str[10], no2Str[10];
    
    dtostrf(air.temp, 1, 1, tempStr);
    dtostrf(air.hum,  1, 1, humStr);
    dtostrf(air.ch2o, 1, 3, ch2oStr);
    dtostrf(air.co,   1, 1, coStr);
    dtostrf(air.o3,   1, 2, o3Str);
    dtostrf(air.no2,  1, 2, no2Str);

    snprintf(buf, sizeof(buf),
      "{\"turbidity\":%s,\"ph\":%s,\"tds\":%s,\"pm1\":%d,\"pm25\":%d,\"pm10\":%d,\"co2\":%d,\"voc\":%d,\"temp\":%s,\"humidity\":%s,\"ch2o\":%s,\"co\":%s,\"o3\":%s,\"no2\":%s}",
      tStr, phStr, tdsStr, air.pm1, air.pm25, air.pm10, air.co2, air.voc, tempStr, humStr, ch2oStr, coStr, o3Str, no2Str
    );
  } else {
    // Failsafe string if air sensor isn't ready
    snprintf(buf, sizeof(buf),
      "{\"turbidity\":%s,\"ph\":%s,\"tds\":%s,\"pm1\":0,\"pm25\":0,\"pm10\":0,\"co2\":0,\"voc\":0,\"temp\":0.0,\"humidity\":0.0,\"ch2o\":0.0,\"co\":0.0,\"o3\":0.0,\"no2\":0.0}",
      tStr, phStr, tdsStr
    );
  }

  EspSerial.println(buf);
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  ZphsSerial.begin(ZPHS_BAUD);
  EspSerial.begin(115200);

  analogReadResolution(12);

  Serial.println("SYSTEM STARTED");
}

// ================= LOOP =================
void loop() {
  unsigned long now = millis();

  // ---- Continuous air sensor read ----
  readAirSensor();

  // ---- Request air data periodically ----
  if (now - lastAirReq >= AIR_REQ_INTERVAL) {
    lastAirReq = now;
    requestAirSensor();
  }

  // ---- Read analog sensors ----
  if (now - lastSensor >= SENSOR_INTERVAL) {
    lastSensor = now;
    g_turbidity = readTurbidity();
    g_ph        = readPH();
    g_tds       = readTDS();
  }

  // ---- Print to Serial ----
  if (now - lastSend >= PUBLISH_INTERVAL) {
    lastSend = now;
    printData();
    sendESP();
  }
}
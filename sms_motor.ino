#include <SoftwareSerial.h>

// D7=SIM7600 TX->Arduino RX, D8=Arduino TX->SIM7600 RX
SoftwareSerial sim(7, 8);

#define PWRKEY_PIN  12
#define STEP_PIN     2
#define DIR_PIN      5
#define ENABLE_PIN   8   // shared with SoftwareSerial TX
#define STATUS_LED  13
// 1/8 microstepping, 200-step motor: 200*8 = 1600 steps/rev, 90deg = 400 steps
// Hardware: remove BOTH MS1 and MS2 jumpers from CNC shield (TMC2208 MS1=OUT, MS2=OUT = 1/8)
#define STEPS_90    400
#define STEP_DELAY  1000  // microseconds per half-step pulse

static char     rxBuf[256];
static uint16_t rxLen;

void rxFlush() {
  while (sim.available()) sim.read();
  rxLen = 0; rxBuf[0] = '\0';
}

bool rxUntil(const char* token, unsigned long ms) {
  unsigned long t0 = millis();
  rxLen = 0;
  while (millis() - t0 < ms) {
    while (sim.available() && rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = (char)sim.read();
      rxBuf[rxLen]   = '\0';
    }
    if (strstr(rxBuf, token)) return true;
  }
  return false;
}

void simSend(const char* cmd) {
  rxFlush();
  digitalWrite(ENABLE_PIN, HIGH);
  delayMicroseconds(1000);
  sim.println(cmd);
}

bool atCmd(const char* cmd, const char* expect, unsigned long ms = 3000) {
  simSend(cmd);
  Serial.print(F(">> ")); Serial.println(cmd);
  bool ok = rxUntil(expect, ms);
  Serial.print(F("<< ")); Serial.println(rxBuf);
  return ok;
}

void blink(int n, int ms) {
  for (int i = 0; i < n; i++) {
    digitalWrite(STATUS_LED, HIGH); delay(ms);
    digitalWrite(STATUS_LED, LOW);  delay(ms);
  }
}

void pwrkeyPulse() {
  pinMode(PWRKEY_PIN, OUTPUT);
  digitalWrite(PWRKEY_PIN, HIGH);
  delay(1500);
  digitalWrite(PWRKEY_PIN, LOW);
}

bool wakeModule() {
  for (int attempt = 0; attempt < 3; attempt++) {
    if (atCmd("AT", "OK", 2000)) return true;
    Serial.println(F("Pulsing PWRKEY..."));
    pwrkeyPulse();
    delay(12000);
  }
  return false;
}

void moveStepper(int steps) {
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);
  delayMicroseconds(500);
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH); delayMicroseconds(STEP_DELAY);
    digitalWrite(STEP_PIN, LOW);  delayMicroseconds(STEP_DELAY);
  }
}

float nmeaToDecimal(const char* s) {
  float raw = atof(s);
  int   deg = (int)(raw / 100);
  float min = raw - (float)deg * 100.0f;
  return (float)deg + min / 60.0f;
}

bool parseGPS(char* out, int maxLen) {
  char* p = strstr(rxBuf, "+CGPSINFO:");
  if (!p) return false;
  p += 10;
  while (*p == ' ') p++;
  if (*p == ',') return false;  // no fix
  char latS[16] = {0}, lonS[16] = {0};
  int i = 0;
  while (*p && *p != ',' && i < 15) latS[i++] = *p++;
  if (*p == ',') p++;
  char ns = *p; p += 2;
  i = 0;
  while (*p && *p != ',' && i < 15) lonS[i++] = *p++;
  if (*p == ',') p++;
  char ew = *p;
  float lat = nmeaToDecimal(latS), lon = nmeaToDecimal(lonS);
  if (ns == 'S') lat = -lat;
  if (ew == 'W') lon = -lon;
  char lats[14], lons[14];
  dtostrf(lat, 1, 6, lats); dtostrf(lon, 1, 6, lons);
  snprintf(out, maxLen, "%s,%s", lats, lons);
  return true;
}

bool sendSMS(const char* number, const char* msg) {
  char cmd[40];
  snprintf(cmd, sizeof(cmd), "AT+CMGS=\"%s\"", number);
  simSend(cmd);
  if (!rxUntil(">", 5000)) return false;
  sim.print(msg);
  sim.write(0x1A);
  return rxUntil("+CMGS:", 30000);
}

void setup() {
  Serial.begin(115200);
  sim.begin(9600);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  Serial.println(F("=== SMS Motor + GPS ==="));
  delay(3000);
  if (!wakeModule()) { while (true) blink(1, 100); }
  atCmd("ATE0", "OK");
  atCmd("AT+CMGF=1", "OK");
  atCmd("AT+CMGD=1,4", "OK", 5000);
  atCmd("AT+CNUM", "OK");
  atCmd("AT+CNMI=2,2,0,0,0", "OK");
  Serial.println(F("Starting GPS (cold fix ~60-120s outdoors)..."));
  atCmd("AT+CGPS=0", "OK", 3000);
  delay(1000);
  bool gpsOk = false;
  for (int i = 0; i < 3 && !gpsOk; i++) {
    gpsOk = atCmd("AT+CGPS=1", "OK", 5000);
    if (!gpsOk) delay(2000);
  }
  if (!gpsOk) Serial.println(F("GPS start failed"));
  blink(3, 400);
  digitalWrite(STATUS_LED, HIGH);
  rxLen = 0; rxBuf[0] = '\0';
}

void loop() {
  while (sim.available() && rxLen < sizeof(rxBuf) - 1) {
    rxBuf[rxLen++] = (char)sim.read();
    rxBuf[rxLen] = '\0';
  }
  char* p = strstr(rxBuf, "+CMT:");
  if (!p) return;
  char* nl1 = strchr(p, '\n'); if (!nl1) return;
  char* nl2 = strchr(nl1 + 1, '\n'); if (!nl2) return;

  // Extract sender number from +CMT: "+1XXXXXXXXXX",...
  char sender[24] = {0};
  char* q = strchr(p, '"');
  if (q) { q++; int i = 0; while (*q && *q != '"' && i < 23) sender[i++] = *q++; }

  // Parse command (lowercase, first word of message body)
  char* body = nl1 + 1; while (*body == '\r') body++;
  char cmd[8] = {0}; int i = 0;
  while (*body && *body != '\r' && *body != '\n' && *body != ' ' && i < 7) {
    char c = *body++;
    cmd[i++] = (c >= 'A' && c <= 'Z') ? c + 32 : c;
  }

  rxLen = 0; rxBuf[0] = '\0';
  atCmd("AT+CMGD=1,4", "OK", 5000);
  atCmd("AT+CNMI=2,2,0,0,0", "OK");
  rxLen = 0; rxBuf[0] = '\0';

  if (cmd[0] == 'f' || cmd[0] == 'b') {
    // 'f' = forward (unlock), 'b' = backward (lock)
    digitalWrite(STATUS_LED, LOW);
    digitalWrite(DIR_PIN, cmd[0] == 'f' ? HIGH : LOW);
    moveStepper(STEPS_90);
    digitalWrite(STATUS_LED, HIGH);
  } else if (strcmp(cmd, "gps") == 0) {
    atCmd("AT+CGPSINFO", "OK", 5000);
    char coords[40];
    if (parseGPS(coords, sizeof(coords))) {
      if (sender[0]) sendSMS(sender, coords);
    } else {
      if (sender[0]) sendSMS(sender, "No GPS fix yet. Try again in a minute.");
    }
    rxLen = 0; rxBuf[0] = '\0';
  }
}

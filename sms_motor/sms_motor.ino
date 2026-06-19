#include <SoftwareSerial.h>

// D7=SIM7600 TXâ†’Arduino RX, D8=Arduino TXâ†’SIM7600 RX
SoftwareSerial sim(7, 8);

#define PWRKEY_PIN  12
#define STEP_PIN     2
#define DIR_PIN      5
#define ENABLE_PIN   8   // shared with SoftwareSerial TX
#define STATUS_LED  13
#define STEPS_90    400
#define STEP_DELAY 1000

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
  // D8 idles HIGH; moveStepper() may have left it LOW (ENABLE asserted).
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
  Serial.println(F("[PWRKEY pulse]"));
  pinMode(PWRKEY_PIN, OUTPUT);
  digitalWrite(PWRKEY_PIN, HIGH); delay(1500);
  digitalWrite(PWRKEY_PIN, LOW);  delay(12000);
}

void moveStepper(int steps) {
  // Enable driver â€” D8 LOW. simSend() restores HIGH before next AT cmd.
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);
  delayMicroseconds(500);
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH); delayMicroseconds(STEP_DELAY);
    digitalWrite(STEP_PIN, LOW);  delayMicroseconds(STEP_DELAY);
  }
}

bool wakeModule() {
  Serial.println(F("Trying AT..."));
  unsigned long t0 = millis();
  while (millis() - t0 < 8000) {
    if (atCmd("AT", "OK", 1500)) { Serial.println(F("Awake!")); return true; }
  }
  for (int i = 0; i < 2; i++) {
    blink(i + 1, 200);
    pwrkeyPulse();
    t0 = millis();
    while (millis() - t0 < 8000) {
      if (atCmd("AT", "OK", 1500)) { Serial.println(F("Awake after pulse!")); return true; }
    }
  }
  Serial.println(F("FAILED: no response"));
  return false;
}

// Convert NMEA DDDMM.MMMMM â†’ decimal degrees
float nmeaToDecimal(const char* s) {
  float raw = atof(s);
  int   deg = (int)(raw / 100);
  float min = raw - (float)deg * 100.0f;
  return (float)deg + min / 60.0f;
}

// Parse +CGPSINFO from rxBuf into a "lat,lon" decimal string.
// Returns true on a valid fix, false if no fix.
bool parseGPS(char* out, int maxLen) {
  // +CGPSINFO: [lat],[N/S],[lon],[E/W],[date],[UTC],[alt],[speed],[course]
  // No fix:    +CGPSINFO: ,,,,,,,,
  char* p = strstr(rxBuf, "+CGPSINFO:");
  if (!p) return false;
  p += 10;
  while (*p == ' ') p++;
  if (*p == ',') return false;  // empty lat = no fix

  char latS[16] = {0}, lonS[16] = {0};
  int  i = 0;
  while (*p && *p != ',' && i < 15) latS[i++] = *p++;
  if (*p == ',') p++;
  char ns = *p; p += 2;
  i = 0;
  while (*p && *p != ',' && i < 15) lonS[i++] = *p++;
  if (*p == ',') p++;
  char ew = *p;

  float lat = nmeaToDecimal(latS);
  float lon = nmeaToDecimal(lonS);
  if (ns == 'S') lat = -lat;
  if (ew == 'W') lon = -lon;

  // dtostrf avoids the AVR printf %f limitation
  char lats[14], lons[14];
  dtostrf(lat, 1, 6, lats);
  dtostrf(lon, 1, 6, lons);
  snprintf(out, maxLen, "%s,%s", lats, lons);
  return true;
}

// Send an SMS to number with message body.
bool sendSMS(const char* number, const char* msg) {
  char cmd[40];
  snprintf(cmd, sizeof(cmd), "AT+CMGS=\"%s\"", number);
  simSend(cmd);
  Serial.print(F(">> ")); Serial.println(cmd);
  if (!rxUntil(">", 5000)) { Serial.println(F("no '>' prompt")); return false; }
  sim.print(msg);
  sim.write(0x1A);  // Ctrl+Z sends the SMS
  bool ok = rxUntil("+CMGS:", 30000);
  Serial.print(F("<< ")); Serial.println(rxBuf);
  return ok;
}

void setup() {
  Serial.begin(115200);
  sim.begin(9600);
  pinMode(STEP_PIN,   OUTPUT);
  pinMode(DIR_PIN,    OUTPUT);
  pinMode(STATUS_LED, OUTPUT);

  Serial.println(F("=== SMS Motor + GPS ==="));
  delay(3000);

  if (!wakeModule()) { while (true) blink(1, 100); }
  atCmd("ATE0", "OK");

  atCmd("AT+CMGF=1", "OK");             // text mode
  atCmd("AT+CMGD=1,4", "OK", 5000);    // delete all stored SMS
  atCmd("AT+CNUM", "OK");              // print SIM phone number (info)
  atCmd("AT+CNMI=2,2,0,0,0", "OK");   // new SMS â†’ +CMT: URC on serial

  // Start GPS (standalone mode; needs GPS antenna on GPS ANT connector)
  Serial.println(F("Starting GPS (cold fix ~60-120s outdoors)..."));
  atCmd("AT+CGPS=0", "OK", 3000);   // stop first in case left on from a prior run
  delay(1000);
  bool gpsOk = false;
  for (int i = 0; i < 3 && !gpsOk; i++) {
    gpsOk = atCmd("AT+CGPS=1", "OK", 5000);
    if (!gpsOk) delay(2000);
  }
  if (!gpsOk) Serial.println(F("GPS start failed â€” 'gps' cmd will report no fix"));

  Serial.println(F("Ready! SMS commands:"));
  Serial.println(F("  f   = forward 90 degrees"));
  Serial.println(F("  b   = backward 90 degrees"));
  Serial.println(F("  gps = reply with coordinates"));
  blink(3, 400);
  digitalWrite(STATUS_LED, HIGH);

  rxLen = 0; rxBuf[0] = '\0';
}

void loop() {
  // Drain SoftwareSerial ring buffer into rxBuf continuously
  while (sim.available() && rxLen < sizeof(rxBuf) - 1) {
    rxBuf[rxLen++] = (char)sim.read();
    rxBuf[rxLen]   = '\0';
  }

  // New SMS arrives as two-line URC:
  //   +CMT: "+1XXXXXXXXXX","","YY/MM/DD,HH:MM:SS+ZZ"\r\n
  //   <message text>\r\n
  char* p = strstr(rxBuf, "+CMT:");
  if (!p) return;

  // Wait until both lines have arrived
  char* nl1 = strchr(p, '\n');
  if (!nl1) return;
  char* nl2 = strchr(nl1 + 1, '\n');
  if (!nl2) return;

  // Extract sender number from +CMT: "+1XXXXXXXXXX",...
  char sender[24] = {0};
  char* q = strchr(p, '"');
  if (q) {
    q++;
    int i = 0;
    while (*q && *q != '"' && i < 23) sender[i++] = *q++;
  }

  // Parse message body (lowercase, first word only)
  char* body = nl1 + 1;
  while (*body == '\r') body++;
  char cmd[8] = {0};
  int i = 0;
  while (*body && *body != '\r' && *body != '\n' && *body != ' ' && i < 7) {
    char c = *body++;
    cmd[i++] = (c >= 'A' && c <= 'Z') ? c + 32 : c;
  }

  Serial.print(F("SMS from ")); Serial.print(sender);
  Serial.print(F(": '")); Serial.print(cmd); Serial.println(F("'"));

  rxLen = 0; rxBuf[0] = '\0';
  atCmd("AT+CMGD=1,4", "OK", 5000);     // delete SMS
  atCmd("AT+CNMI=2,2,0,0,0", "OK");     // re-arm URC
  rxLen = 0; rxBuf[0] = '\0';

  if (cmd[0] == 'f' || cmd[0] == 'b') {
    bool fwd = (cmd[0] == 'f');
    Serial.println(fwd ? F("FORWARD") : F("BACKWARD"));
    digitalWrite(STATUS_LED, LOW);
    digitalWrite(DIR_PIN, fwd ? HIGH : LOW);
    moveStepper(STEPS_90);
    digitalWrite(STATUS_LED, HIGH);
    Serial.println(F("Done."));

  } else if (strcmp(cmd, "gps") == 0) {
    Serial.println(F("Querying GPS..."));
    atCmd("AT+CGPSINFO", "OK", 5000);
    char coords[40];
    if (parseGPS(coords, sizeof(coords))) {
      Serial.print(F("Fix: ")); Serial.println(coords);
      if (sender[0]) sendSMS(sender, coords);
    } else {
      Serial.println(F("No fix yet â€” move outdoors and try again"));
      if (sender[0]) sendSMS(sender, "No GPS fix yet. Try again in a minute.");
    }
    rxLen = 0; rxBuf[0] = '\0';

  } else {
    Serial.println(F("Unknown cmd â€” send: f, b, or gps"));
  }
}

// Flight Tracker for Cheap Yellow Display (ESP32 + ILI9341 320x240)
// Polls local dump1090 every 10s, displays the closest aircraft.
// Swipe anywhere on the screen to toggle between flight detail and radar views.

#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <SPI.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <math.h>
#include "airlines.h"
#include "config.h"     // WiFi credentials, API_URL, HOME_LAT/LON — see config.h.example

// ─── Timing / filter config ───────────────────────────────────
#define POLL_MS  10000UL
const int MAX_AGE = 60;   // ignore aircraft silent for more than 60s

// ─── Touch pins (CYD ESP32-2432S028R, XPT2046 on VSPI) ────────
#define TOUCH_CS   33
#define TOUCH_IRQ  36
#define TOUCH_CLK  25
#define TOUCH_MISO 39
#define TOUCH_MOSI 32

// ─── Flight view layout ───────────────────────────────────────
//  Left panel  : x 0..206   (data)
//  Right panel : x 208..319 (compass)
//  Status bar  : y 210..239
#define DIVIDER_X   207
#define DIVIDER_Y   208
#define STATUS_Y    218
#define CMP_CX      264     // compass center x
#define CMP_CY      104     // compass center y
#define CMP_R        50     // compass radius

// ─── Radar view layout ────────────────────────────────────────
//  Radar circle: left-justified, center at (97, 103), radius 90px
//    left edge = 7px margin, right edge = 187px
//    90px / 40mi = 2.25 px/mi; rings at ~23/45/68/90 px
//  Right panel: x 195..319  (vertical callsign list)
//  Status bar : y 209..239  (shared with flight view)
#define RAD_CX          97
#define RAD_CY          103
#define RADAR_RANGE_MI  40.0f
#define RADAR_R         90      // pixels to outermost ring
#define RAD_PANEL_X     195     // x start of right callsign panel
#define MAX_RADAR_BLIPS 30

// ─── Colors ───────────────────────────────────────────────────
#define C_BG        TFT_BLACK
#define C_LABEL     0x07FF   // cyan
#define C_VALUE     TFT_WHITE
#define C_CALL      TFT_YELLOW
#define C_DIVIDER   0x2965   // dark blue-grey
#define C_ARROW     TFT_GREEN
#define C_STATUS    0x7BEF   // mid grey
#define C_NORTH     TFT_RED
#define C_RADAR     0x07E0   // phosphor green — rings & cardinal labels
#define C_CLOSEST   0xF800   // = BLIP_COLORS[0]; used in status bar

// 10 distinct bright colors, index 0 = closest (red matches status bar)
static const uint16_t BLIP_COLORS[10] = {
  0xF800,   // 0 red       — closest
  0x07E0,   // 1 green
  0x07FF,   // 2 cyan
  0xFFE0,   // 3 yellow
  0xFD20,   // 4 orange
  0xF81F,   // 5 magenta
  0xFFFF,   // 6 white
  0x867D,   // 7 sky blue
  0xAFE5,   // 8 lime
  0xFB56,   // 9 hot pink
};
#define C_BLIP_DIM  0x4208   // dim grey for aircraft beyond top 10

// ─── Globals ──────────────────────────────────────────────────
TFT_eSPI tft = TFT_eSPI();

SPIClass touchSPI(VSPI);
XPT2046_Touchscreen ts(TOUCH_CS, TOUCH_IRQ);

struct Aircraft {
  char  flight[12];
  int   altFt;
  int   speedKts;
  int   trackDeg;
  float distMi;
  bool  valid;
};

struct RadarBlip {
  float  lat, lon, distMi;
  char   flight[12];
  int8_t colorIdx;   // 0-9 = colored & listed; -1 = beyond top 10
};

Aircraft  g_ac        = {};
RadarBlip g_blips[MAX_RADAR_BLIPS];
int       g_blipCount = 0;

unsigned long g_lastPoll = 0;
unsigned long g_drawTime = 0;
bool g_wifiOk = false;

// ─── View / swipe state ───────────────────────────────────────
enum View { VIEW_FLIGHT = 0, VIEW_RADAR = 1 };
View g_view = VIEW_FLIGHT;

bool          g_touching      = false;
int16_t       g_touchStartX   = 0;
int16_t       g_touchStartY   = 0;
int16_t       g_touchLastX    = 0;
int16_t       g_touchLastY    = 0;
unsigned long g_lastSwipe     = 0;
#define SWIPE_THRESHOLD   400   // raw XPT2046 units (~10% of 4096 range)
#define SWIPE_COOLDOWN_MS 800   // minimum ms between view toggles

// ─── Haversine distance (miles) ───────────────────────────────
float haversineMi(double lat1, double lon1, double lat2, double lon2) {
  const float R = 3958.8f;
  double dLat = (lat2 - lat1) * M_PI / 180.0;
  double dLon = (lon2 - lon1) * M_PI / 180.0;
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) *
             sin(dLon / 2) * sin(dLon / 2);
  return R * 2.0f * (float)atan2(sqrt(a), sqrt(1.0 - a));
}

// ─── Bearing from HOME to aircraft (degrees, 0=N, clockwise) ──
float bearingDeg(float acLat, float acLon) {
  double dLon = (acLon - HOME_LON) * M_PI / 180.0;
  double lat1 = HOME_LAT * M_PI / 180.0;
  double lat2 = (double)acLat * M_PI / 180.0;
  float y = sinf((float)dLon) * cosf((float)lat2);
  float x = cosf((float)lat1) * sinf((float)lat2)
            - sinf((float)lat1) * cosf((float)lat2) * cosf((float)dLon);
  float b = atan2f(y, x) * 180.0f / (float)M_PI;
  return fmodf(b + 360.0f, 360.0f);
}

// ─── Compass drawing ──────────────────────────────────────────
void drawCompass(int cx, int cy, int r, int trackDeg) {
  float rad  = trackDeg * M_PI / 180.0f;
  float sinA = sinf(rad);
  float cosA = cosf(rad);

  // Outer ring
  tft.fillCircle(cx, cy, r + 4, C_BG);
  tft.drawCircle(cx, cy, r + 2, C_DIVIDER);
  tft.drawCircle(cx, cy, r + 3, C_DIVIDER);

  // Cardinal tick marks (every 90°)
  for (int d = 0; d < 360; d += 90) {
    float a = d * M_PI / 180.0f;
    int x1 = cx + (int)((r - 5) * sinf(a));
    int y1 = cy - (int)((r - 5) * cosf(a));
    int x2 = cx + (int)(r * sinf(a));
    int y2 = cy - (int)(r * cosf(a));
    tft.drawLine(x1, y1, x2, y2, C_DIVIDER);
  }

  // "N" label above circle
  tft.setTextSize(1);
  tft.setTextColor(C_NORTH, C_BG);
  tft.setCursor(cx - 3, cy - r - 12);
  tft.print("N");

  // Arrow: tip at r, base at -r*0.3, wing width ±r*0.28
  float tipX  = cx + r * sinA;
  float tipY  = cy - r * cosA;
  float baseX = cx - r * 0.30f * sinA;
  float baseY = cy + r * 0.30f * cosA;
  float lwX   = baseX + r * 0.28f * cosA;
  float lwY   = baseY + r * 0.28f * sinA;
  float rwX   = baseX - r * 0.28f * cosA;
  float rwY   = baseY - r * 0.28f * sinA;

  tft.fillTriangle((int16_t)tipX, (int16_t)tipY,
                   (int16_t)lwX,  (int16_t)lwY,
                   (int16_t)rwX,  (int16_t)rwY,
                   C_ARROW);

  // Tail stub (opposite direction, narrower)
  float tailX = cx - r * 0.65f * sinA;
  float tailY = cy + r * 0.65f * cosA;
  float tlX   = tailX + r * 0.13f * cosA;
  float tlY   = tailY + r * 0.13f * sinA;
  float trX   = tailX - r * 0.13f * cosA;
  float trY   = tailY - r * 0.13f * sinA;

  tft.fillTriangle((int16_t)baseX, (int16_t)baseY,
                   (int16_t)tlX,   (int16_t)tlY,
                   (int16_t)trX,   (int16_t)trY,
                   0x4208);  // dark grey tail
}

// ─── Format altitude with comma separator ─────────────────────
void fmtAlt(char* buf, size_t sz, int alt) {
  if (alt >= 1000)
    snprintf(buf, sz, "%d,%03d ft", alt / 1000, alt % 1000);
  else
    snprintf(buf, sz, "%d ft", alt);
}

// ─── Draw full flight detail screen ───────────────────────────
void drawFlight(const Aircraft& ac) {
  tft.fillScreen(C_BG);

  // ── No aircraft case ──
  if (!ac.valid) {
    tft.setTextColor(TFT_YELLOW, C_BG);
    tft.setTextSize(2);
    tft.setCursor(28, 95);
    tft.print("No aircraft in range");
    tft.setTextSize(1);
    tft.setTextColor(C_STATUS, C_BG);
    tft.setCursor(4, STATUS_Y);
    tft.print("Polling every 10s...");
    return;
  }

  // ── Callsign ──────────────────────────────────────────────
  tft.setTextColor(C_CALL, C_BG);
  tft.setTextSize(4);
  tft.setCursor(6, 5);
  tft.print(ac.flight);

  // ── Dividers ─────────────────────────────────────────────
  tft.drawFastHLine(0, 46, DIVIDER_X, C_DIVIDER);
  tft.drawFastHLine(0, DIVIDER_Y, 320, C_DIVIDER);
  tft.drawFastVLine(DIVIDER_X, 0, DIVIDER_Y, C_DIVIDER);

  char buf[24];

  // ── ALTITUDE  (row 1) ─────────────────────────────────────
  tft.setTextSize(1);
  tft.setTextColor(C_LABEL, C_BG);
  tft.setCursor(6, 51);
  tft.print("ALTITUDE");

  tft.setTextSize(2);
  tft.setTextColor(C_VALUE, C_BG);
  tft.setCursor(6, 61);
  fmtAlt(buf, sizeof(buf), ac.altFt);
  tft.print(buf);

  // ── SPEED  (row 2) ────────────────────────────────────────
  tft.setTextSize(1);
  tft.setTextColor(C_LABEL, C_BG);
  tft.setCursor(6, 91);
  tft.print("SPEED");

  tft.setTextSize(2);
  tft.setTextColor(C_VALUE, C_BG);
  tft.setCursor(6, 101);
  snprintf(buf, sizeof(buf), "%d kts", ac.speedKts);
  tft.print(buf);

  // ── DISTANCE  (row 3) ─────────────────────────────────────
  tft.setTextSize(1);
  tft.setTextColor(C_LABEL, C_BG);
  tft.setCursor(6, 131);
  tft.print("DISTANCE");

  tft.setTextSize(2);
  tft.setTextColor(C_VALUE, C_BG);
  tft.setCursor(6, 141);
  snprintf(buf, sizeof(buf), "%.1f mi", ac.distMi);
  tft.print(buf);

  // ── AIRLINE  (row 4) ──────────────────────────────────────
  const char* airline = lookupAirline(ac.flight);
  tft.setTextSize(1);
  tft.setTextColor(C_LABEL, C_BG);
  tft.setCursor(6, 171);
  tft.print(airline ? "AIRLINE" : "AIRCRAFT TYPE");

  tft.setTextSize(2);
  tft.setTextColor(0xFD20, C_BG);  // orange
  tft.setCursor(6, 181);
  if (airline) {
    char name[17];
    strncpy(name, airline, 16);
    name[16] = '\0';
    if (strlen(airline) > 16) { name[13]='.'; name[14]='.'; name[15]='.'; }
    tft.print(name);
  } else {
    tft.print("Privately Owned");
  }

  // ── TRACK label (below compass) ───────────────────────────
  tft.setTextSize(1);
  tft.setTextColor(C_STATUS, C_BG);
  snprintf(buf, sizeof(buf), "%3d\xF7", ac.trackDeg);
  tft.setCursor(CMP_CX - (int)(strlen(buf) * 3), CMP_CY + CMP_R + 10);
  tft.print(buf);

  // ── Compass ───────────────────────────────────────────────
  drawCompass(CMP_CX, CMP_CY, CMP_R, ac.trackDeg);
}

// ─── Draw radar view ──────────────────────────────────────────
//  Left : radar circle, left-justified (center x=97, r=90).
//  Right: vertical list of up to 10 closest callsigns, color-coded.
//         Each dot on the radar uses the same color as its list entry.
void drawRadar() {
  tft.fillScreen(C_BG);

  // Status bar divider and right panel divider
  tft.drawFastHLine(0,           DIVIDER_Y,  320, C_DIVIDER);
  tft.drawFastVLine(RAD_PANEL_X - 2, 0, DIVIDER_Y, C_DIVIDER);

  // ── Range rings ───────────────────────────────────────────
  const float pxPerMi = (float)RADAR_R / RADAR_RANGE_MI;  // 2.25 px/mi
  tft.setTextSize(1);
  tft.setTextColor(C_RADAR, C_BG);
  for (int ring = 1; ring <= 4; ring++) {
    int r = (int)(ring * 10.0f * pxPerMi + 0.5f);  // ~23, 45, 68, 90
    tft.drawCircle(RAD_CX, RAD_CY, r, C_RADAR);
    // Mile label just inside the ring at ~3-o'clock, shifted up
    char lb[4];
    snprintf(lb, sizeof(lb), "%d", ring * 10);
    tft.setCursor(RAD_CX + r - 14, RAD_CY - 9);
    tft.print(lb);
  }

  // ── Cardinal indicators ───────────────────────────────────
  tft.setTextColor(C_RADAR, C_BG);
  tft.setCursor(RAD_CX - 3, 4);                            // N (top)
  tft.print("N");
  tft.setCursor(RAD_CX - 3, RAD_CY + RADAR_R + 3);         // S (bottom)
  tft.print("S");
  tft.setCursor(RAD_CX + RADAR_R + 3, RAD_CY - 4);         // E (right of ring)
  tft.print("E");
  tft.setCursor(2, RAD_CY - 4);                            // W (near left edge)
  tft.print("W");

  // ── Home position dot (white, center) ─────────────────────
  tft.fillCircle(RAD_CX, RAD_CY, 3, TFT_WHITE);

  // ── Aircraft blips ────────────────────────────────────────
  for (int i = 0; i < g_blipCount; i++) {
    float bear    = bearingDeg(g_blips[i].lat, g_blips[i].lon);
    float bearRad = bear * (float)M_PI / 180.0f;
    int   bx      = RAD_CX + (int)(g_blips[i].distMi * pxPerMi * sinf(bearRad));
    int   by      = RAD_CY - (int)(g_blips[i].distMi * pxPerMi * cosf(bearRad));

    if (bx < 2 || bx > RAD_PANEL_X - 4 || by < 2 || by > DIVIDER_Y - 2) continue;

    int8_t ci = g_blips[i].colorIdx;
    uint16_t color = (ci >= 0 && ci < 10) ? BLIP_COLORS[ci] : C_BLIP_DIM;
    tft.fillCircle(bx, by, 3, color);
  }

  // ── Right panel: "Flight Board" label + 10 closest callsigns ─
  // Panel spans x=RAD_PANEL_X..319 (124px), center at x=257.
  // Label at y=1; rows start at y=14, spaced 19px (10 rows fit before DIVIDER_Y).
  const int panelCX  = (RAD_PANEL_X + 319) / 2;   // 257
  const int panelW   = 319 - RAD_PANEL_X;          // 124

  // "Flight Board" header centered in panel
  const char* header = "Flight Board";
  int hx = panelCX - (int)(strlen(header) * 3);    // size-1 char = 6px, so half = 3px/char
  tft.setTextSize(1);
  tft.setTextColor(C_DIVIDER, C_BG);
  tft.setCursor(hx, 1);
  tft.print(header);
  tft.drawFastHLine(RAD_PANEL_X - 2, 11, 320 - (RAD_PANEL_X - 2), C_DIVIDER);

  int listed = 0;
  for (int i = 0; i < g_blipCount && listed < 10; i++) {
    if (g_blips[i].colorIdx < 0) continue;
    const char* label = g_blips[i].flight[0] ? g_blips[i].flight : "------";
    int rowY = 14 + listed * 19;
    uint16_t color = BLIP_COLORS[g_blips[i].colorIdx];

    // Center the dot+gap+text block in the panel
    int textW  = (int)strlen(label) * 6;  // size-1 text: 6px per char
    int blockW = 6 + 4 + textW;           // dot diameter + gap + text
    int startX = RAD_PANEL_X + (panelW - blockW) / 2;

    tft.fillCircle(startX + 3, rowY + 4, 3, color);   // dot centered vertically in row
    tft.setTextColor(color, C_BG);
    tft.setCursor(startX + 10, rowY);
    tft.print(label);
    listed++;
  }

  // ── No aircraft message ───────────────────────────────────
  if (g_blipCount == 0) {
    tft.setTextColor(TFT_YELLOW, C_BG);
    tft.setTextSize(2);
    tft.setCursor(10, 88);
    tft.print("No aircraft");
    tft.setCursor(10, 112);
    tft.print("in range");
  }
}

// ─── Update status bar only (called every second) ─────────────
void updateStatus(unsigned long secsSince) {
  tft.fillRect(0, DIVIDER_Y + 1, 320, 239 - DIVIDER_Y, C_BG);
  tft.setTextSize(1);
  tft.setTextColor(C_STATUS, C_BG);
  tft.setCursor(4, STATUS_Y);

  if (!g_wifiOk) {
    tft.setTextColor(TFT_RED, C_BG);
    tft.print("WiFi disconnected - reconnecting...");
    return;
  }

  char buf[64];
  unsigned long nextIn = POLL_MS / 1000 - (secsSince % (POLL_MS / 1000));
  if (secsSince == 0)
    snprintf(buf, sizeof(buf), "Just updated  |  Next in %lus", POLL_MS / 1000);
  else
    snprintf(buf, sizeof(buf), "Updated %lus ago  |  Next in %lus", secsSince, nextIn);
  tft.print(buf);

  // On radar view, show the closest callsign right-justified in red
  if (g_view == VIEW_RADAR && g_ac.valid && g_ac.flight[0] != '\0') {
    int x = 320 - (int)strlen(g_ac.flight) * 6 - 4;
    tft.setTextColor(C_CLOSEST, C_BG);
    tft.setCursor(x, STATUS_Y);
    tft.print(g_ac.flight);
  }
}

// ─── Fetch & parse aircraft.json ──────────────────────────────
//  Populates g_ac (closest) and g_blips[]/g_blipCount (all within 40mi),
//  sorted by distance with color indices 0-9 assigned to the 10 closest.
bool fetchAllAircraft() {
  if (WiFi.status() != WL_CONNECTED) {
    g_wifiOk = false;
    WiFi.reconnect();
    return false;
  }

  HTTPClient http;
  http.begin(API_URL);
  http.setTimeout(6000);
  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    http.end();
    return false;
  }

  // Filter to only parse fields we need — keeps heap use minimal
  StaticJsonDocument<192> filter;
  JsonObject fac = filter["aircraft"].createNestedObject();
  fac["flight"]   = true;
  fac["lat"]      = true;
  fac["lon"]      = true;
  fac["altitude"] = true;   // dump1090-mutability
  fac["alt_baro"] = true;   // dump1090-fa
  fac["speed"]    = true;   // dump1090-mutability
  fac["gs"]       = true;   // dump1090-fa (ground speed)
  fac["track"]    = true;
  fac["seen"]     = true;
  fac["seen_pos"] = true;

  DynamicJsonDocument doc(24576);
  DeserializationError err = deserializeJson(
      doc, http.getStream(), DeserializationOption::Filter(filter));
  http.end();

  if (err) return false;

  g_wifiOk = true;

  Aircraft best = {};
  best.distMi = 1e9f;
  g_blipCount = 0;

  for (JsonObject ac : doc["aircraft"].as<JsonArray>()) {
    if (!ac["lat"].is<float>() || !ac["lon"].is<float>()) continue;

    // Age check — use seen_pos if available, fall back to seen
    int age = ac.containsKey("seen_pos")
              ? (int)ac["seen_pos"]
              : (int)(ac["seen"] | 9999);
    if (age > MAX_AGE) continue;

    float lat  = ac["lat"].as<float>();
    float lon  = ac["lon"].as<float>();
    float dist = haversineMi(HOME_LAT, HOME_LON, lat, lon);

    // Collect radar blips within range
    if (dist <= RADAR_RANGE_MI && g_blipCount < MAX_RADAR_BLIPS) {
      RadarBlip& b = g_blips[g_blipCount++];
      b.lat      = lat;
      b.lon      = lon;
      b.distMi   = dist;
      b.colorIdx = -1;
      const char* fl = ac["flight"] | "";
      strncpy(b.flight, fl, sizeof(b.flight) - 1);
      b.flight[sizeof(b.flight) - 1] = '\0';
      for (int i = (int)strlen(b.flight) - 1; i >= 0 && b.flight[i] == ' '; i--)
        b.flight[i] = '\0';
    }

    // Track closest for flight detail view
    if (dist >= best.distMi) continue;

    int alt = 0;
    if (ac["altitude"].is<int>())      alt = ac["altitude"].as<int>();
    else if (ac["alt_baro"].is<int>()) alt = ac["alt_baro"].as<int>();

    int spd = 0;
    if (ac["speed"].is<int>())   spd = ac["speed"].as<int>();
    else if (ac["gs"].is<int>()) spd = ac["gs"].as<int>();

    best.distMi   = dist;
    best.valid    = true;
    best.altFt    = alt;
    best.speedKts = spd;
    best.trackDeg = ac["track"] | 0;

    const char* fl = ac["flight"] | "??????";
    strncpy(best.flight, fl, sizeof(best.flight) - 1);
    best.flight[sizeof(best.flight) - 1] = '\0';
    for (int i = (int)strlen(best.flight) - 1; i >= 0 && best.flight[i] == ' '; i--)
      best.flight[i] = '\0';
  }

  // Sort blips by distance ascending (insertion sort — max 30 elements)
  for (int i = 1; i < g_blipCount; i++) {
    RadarBlip key = g_blips[i];
    int j = i - 1;
    while (j >= 0 && g_blips[j].distMi > key.distMi) {
      g_blips[j + 1] = g_blips[j];
      j--;
    }
    g_blips[j + 1] = key;
  }

  // Assign color indices to the 10 closest (rest stay -1)
  for (int i = 0; i < g_blipCount && i < 10; i++)
    g_blips[i].colorIdx = (int8_t)i;

  g_ac = best;
  return true;
}

// ─── WiFi splash screen ───────────────────────────────────────
void connectWiFi() {
  tft.fillScreen(C_BG);
  tft.setTextColor(C_VALUE, C_BG);
  tft.setTextSize(2);
  tft.setCursor(10, 75);
  tft.print("Connecting to WiFi");
  tft.setTextSize(1);
  tft.setTextColor(C_STATUS, C_BG);
  tft.setCursor(10, 102);
  tft.print(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int dots = 0;
  while (WiFi.status() != WL_CONNECTED && dots < 40) {
    delay(500);
    tft.setTextColor(C_LABEL, C_BG);
    tft.setCursor(10 + dots * 7, 125);
    tft.print(".");
    dots++;
  }

  tft.fillScreen(C_BG);
  tft.setTextSize(2);

  if (WiFi.status() == WL_CONNECTED) {
    g_wifiOk = true;
    tft.setTextColor(TFT_GREEN, C_BG);
    tft.setCursor(55, 90);
    tft.print("Connected!");
    tft.setTextSize(1);
    tft.setTextColor(C_STATUS, C_BG);
    tft.setCursor(55, 115);
    tft.print(WiFi.localIP().toString());
    delay(1200);
  } else {
    tft.setTextColor(TFT_RED, C_BG);
    tft.setCursor(40, 90);
    tft.print("WiFi failed!");
    tft.setTextSize(1);
    tft.setTextColor(C_STATUS, C_BG);
    tft.setCursor(10, 118);
    tft.print("Will retry. Fill in WIFI_SSID / WIFI_PASSWORD.");
    delay(3000);
  }
}

// ─── setup ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  tft.init();
  tft.setRotation(1);   // landscape, USB port on left
  tft.fillScreen(C_BG);

  // Touch — XPT2046 on VSPI (CYD standard pinout)
  touchSPI.begin(TOUCH_CLK, TOUCH_MISO, TOUCH_MOSI, TOUCH_CS);
  ts.begin(touchSPI);
  ts.setRotation(1);

  connectWiFi();
}

// ─── loop ─────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  // ── Swipe detection ───────────────────────────────────────
  if (ts.touched()) {
    TS_Point p = ts.getPoint();
    if (!g_touching) {
      g_touching    = true;
      g_touchStartX = p.x;
      g_touchStartY = p.y;
    }
    g_touchLastX = p.x;
    g_touchLastY = p.y;
  } else if (g_touching) {
    g_touching = false;
    // Accept swipe on whichever axis has greater travel (handles any touch rotation)
    int16_t dx = abs(g_touchLastX - g_touchStartX);
    int16_t dy = abs(g_touchLastY - g_touchStartY);
    if (max(dx, dy) > SWIPE_THRESHOLD && now - g_lastSwipe > SWIPE_COOLDOWN_MS) {
      g_lastSwipe = now;
      g_view = (g_view == VIEW_FLIGHT) ? VIEW_RADAR : VIEW_FLIGHT;
      if (g_view == VIEW_FLIGHT) drawFlight(g_ac);
      else                       drawRadar();
      if (g_drawTime > 0) updateStatus((now - g_drawTime) / 1000);
    }
  }

  // ── Poll every POLL_MS ───────────────────────────────────
  if (g_lastPoll == 0 || now - g_lastPoll >= POLL_MS) {
    g_lastPoll = now;
    bool ok = fetchAllAircraft();
    if (!ok && g_ac.valid) {
      // Keep last data — don't blank the screen
    } else if (!ok) {
      g_ac.valid  = false;
      g_blipCount = 0;
    }
    if (g_view == VIEW_FLIGHT) drawFlight(g_ac);
    else                       drawRadar();
    g_drawTime = now;
  }

  // ── Status bar tick (every second) ───────────────────────
  static unsigned long lastSecTick = 0;
  if (g_drawTime > 0 && now - lastSecTick >= 1000) {
    lastSecTick = now;
    updateStatus((now - g_drawTime) / 1000);
  }

  delay(50);
}

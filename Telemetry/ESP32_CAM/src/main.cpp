/* UAV telemetry + PID configuration bridge, ESP32-CAM AI-Thinker.
 * GPIO13 <- STM32 LPUART1 TX (now PC1/A4); GPIO14 -> STM32 LPUART1 RX (now PC0/A5), 3.3V.
 * GPIO13/14 are shared with microSD and must not be used for SDMMC.
 * The MCU controls ARM/KILL/failsafe; Wi-Fi never drives motors.
 * Remote STM32 ROM flashing is disabled by default on the FC.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <Update.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <mbedtls/sha256.h>
#include <mbedtls/version.h>
#include <string.h>
#include <math.h>
#include <stdio.h>



namespace {
constexpr uint32_t FC_BAUD = 115200;
constexpr int FC_RX = 13, FC_TX = 14;
constexpr uint16_t UDP_PORT = 14550, HTTP_PORT = 80;
// Bench bring-up: 13 dBm is more practical than the old 2 dBm limit.
// IMPORTANT: Do not fly with Wi-Fi enabled near a 2.4 GHz RC receiver
// without doing RF coexistence/range tests. This is a BENCH configuration.
constexpr uint8_t WIFI_CHANNEL = 1, WIFI_TX_LIMIT_QUARTER_DBM = 52;
constexpr uint32_t PEER_TIMEOUT_MS = 3500;
constexpr uint32_t FC_TIMEOUT_MS = 1500;
constexpr uint32_t FLASH_MAX_BYTES = 508u * 1024u;
constexpr uint32_t CFG_COUNT = 13;
constexpr uint8_t FRAME_FLIGHT_LEN = 88;
// Direct AP at 192.168.4.1; no STA or router dependencies.
constexpr int ESP_OTA_BENCH_PIN = 2; // Ground GPIO2 for bench-service OTA without FC telemetry; no microSD.
constexpr uint32_t ESP_OTA_BENCH_HOLD_MS = 3000;
constexpr uint32_t ESP_OTA_REBOOT_DELAY_MS = 2200;
constexpr char ESP_FW_VERSION[] = "UAV-ESP32-DIRECT-AP-UI-v2";
// CHANGE WIFI only when DISARMED / CONFIG_READY if valid STM32 telemetry exists.
Preferences configNvs;
String apSsid, apPassword, apiToken;
bool setupAPActive = false, nvsReady = false;
uint32_t lastAPStartAttemptMs = 0, lastAPHealthLogMs = 0;
uint32_t lastTelemetryMs = 0;
uint16_t lastFlightFlags = 0;

HardwareSerial FcUart(1);
WiFiUDP udp;
WebServer http(HTTP_PORT);
IPAddress peerIP;
uint16_t peerPort = UDP_PORT;
bool peerKnown = false;
uint32_t peerLastSeen = 0;
uint8_t rxFrame[88];
uint8_t rxPos = 0, rxTarget = 0;
bool rxControl = false;
uint16_t reqSeq = 0, responseSeq = 0;
uint8_t responseType = 0, responseLen = 0, response[60];
bool responseReady = false;
uint32_t rxBytes = 0, validFrames = 0, crcErrors = 0, forwardedFrames = 0;
uint32_t udpErrors = 0, lastStats = 0;
bool uploadBad = false, uploadReady = false;
uint32_t uploadBytes = 0;
File uploadFile;
// ESP32 self-update (separate endpoint from the existing STM32 ROM OTA).
bool espOtaStarted = false, espOtaOk = false, espOtaRebootPending = false;
bool espOtaHashStarted = false;
uint32_t espOtaExpectedBytes = 0, espOtaReceivedBytes = 0, espOtaRebootAt = 0;
uint32_t espBenchLowSince = 0;
String espOtaExpectedSha, espOtaError;
mbedtls_sha256_context espOtaHash;

uint16_t read16(const uint8_t *p) { return uint16_t(p[0]) | (uint16_t(p[1]) << 8); }
uint32_t read32(const uint8_t *p) { return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24); }
uint16_t crc16(const uint8_t *data, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; ++i) {
    crc ^= uint16_t(data[i]) << 8;
    for (uint8_t b = 0; b < 8; ++b)
      crc = (crc & 0x8000u) ? uint16_t((crc << 1) ^ 0x1021u) : uint16_t(crc << 1);
  }
  return crc;
}
bool authorized() {
  const String got = http.header("X-Auth-Token");
  const String &expected = apiToken;
  /* Length and all bytes compared; there is no unauthenticated write API. */
  if (got.length() != expected.length() || expected.length() < 24) return false;
  uint8_t bad = 0;
  for (size_t i = 0; i < expected.length(); ++i) bad |= uint8_t(got[i] ^ expected[i]);
  return bad == 0;
}
bool requireAuth() {
  if (authorized()) return true;
  http.send(403, "application/json", "{\"error\":\"token invalid\"}");
  return false;
}

String makeRandomHex(size_t hexDigits) {
  static const char digits[] = "0123456789ABCDEF";
  String result;
  result.reserve(hexDigits);
  for (size_t i = 0; i < hexDigits; ++i)
    result += digits[esp_random() & 0x0F];
  return result;
}
String randomSecret(const char *key, size_t hexDigits) {
  String v = configNvs.getString(key, "");
  if (v.length() != hexDigits) {
    v = makeRandomHex(hexDigits);
    configNvs.putString(key, v);
  }
  return v;
}
String jsonQuote(const String &value) {
  String out = "\"";
  for (size_t i = 0; i < value.length(); ++i) {
    unsigned char c = static_cast<unsigned char>(value[i]);
    if (c == '"' || c == '\\') { out += '\\'; out += char(c); }
    else if (c < 0x20) {
      char tmp[7]; snprintf(tmp, sizeof(tmp), "\\u%04X", static_cast<unsigned>(c));
      out += tmp;
    } else out += char(c);
  }
  return out + '"';
}
bool configModeAllowsWiFiChange() {
  // During initial setup without STM32 telemetry, permit bench provisioning.
  if (validFrames == 0) return true;
  // With FC telemetry, stale/unknown state fails closed.
  if (uint32_t(millis() - lastTelemetryMs) > 1500) return false;
  constexpr uint16_t FLAG_ARMED = 1u << 2;
  constexpr uint16_t FLAG_KILL = 1u << 5;
  constexpr uint16_t FLAG_CONFIG = 1u << 11;
  return (lastFlightFlags & FLAG_ARMED) == 0 &&
         (lastFlightFlags & FLAG_KILL) != 0 &&
         (lastFlightFlags & FLAG_CONFIG) != 0;
}
void serviceOtaBenchPin() {
  const uint32_t now = millis();
  if (digitalRead(ESP_OTA_BENCH_PIN) == LOW) {
    if (espBenchLowSince == 0) espBenchLowSince = now ? now : 1;
  } else {
    espBenchLowSince = 0;
  }
}
// OTA does not rely on the permissive initial Wi-Fi provisioning condition.
// If the FC has ever sent a valid telemetry packet, ONLY fresh safe FC state
// can authorize an update. If the FC has not sent telemetry since boot,
// require the physical GPIO2-to-GND bench/service jumper for >= 3 s.
bool espOtaSafetyAllowed() {
  if (validFrames > 0) return configModeAllowsWiFiChange();
  return espBenchLowSince != 0 &&
         digitalRead(ESP_OTA_BENCH_PIN) == LOW &&
         uint32_t(millis() - espBenchLowSince) >= ESP_OTA_BENCH_HOLD_MS;
}
String espOtaSafetyReason() {
  if (validFrames > 0) return configModeAllowsWiFiChange() ? "stm32_config_ready" : "stm32_not_safe";
  return espOtaSafetyAllowed() ? "physical_bench_gpio2" : "no_fc_telemetry_or_service_jumper";
}
// Direct AP: this firmware NEVER joins a router or scans other Wi-Fi networks.
// The laptop must join the ESP32's network (192.168.4.1/24) directly.
void setupAccessPoint() {
  if (setupAPActive && WiFi.getMode() == WIFI_AP) return;
  lastAPStartAttemptMs = millis();
  const bool modeOk = WiFi.mode(WIFI_AP);
  delay(30);
  const IPAddress ap(192, 168, 4, 1), subnet(255, 255, 255, 0);
  const bool ipOk = WiFi.softAPConfig(ap, ap, subnet);
  const bool apOk = WiFi.softAP(apSsid.c_str(), apPassword.c_str(), WIFI_CHANNEL, 0, 4);
  setupAPActive = modeOk && ipOk && apOk;
  Serial.printf("[WIFI] DIRECT AP mode_ok=%d ip_ok=%d softAP_ok=%d ssid=%s ip=%s\n",
                int(modeOk), int(ipOk), int(apOk), apSsid.c_str(),
                WiFi.softAPIP().toString().c_str());
  if (!setupAPActive) Serial.println("[WIFI] AP initialization failed; retrying in 5 seconds. Check 5V power.");
}
void serviceWifi() {
  // Keep the direct AP running. No STA timeout, no router scans or switching.
  if (espOtaStarted) return; // Avoid Wi-Fi reconfiguration while writing flash.
  if ((!setupAPActive || WiFi.getMode() != WIFI_AP) &&
      uint32_t(millis() - lastAPStartAttemptMs) >= 5000u) {
    setupAPActive = false;
    setupAccessPoint();
  }
}
String wifiStateJSON() {
  return String("{\"mode\":\"DIRECT_AP_ONLY\",\"ap_ssid\":") + jsonQuote(apSsid) +
         ",\"ap_ip\":" + jsonQuote(setupAPActive ? WiFi.softAPIP().toString() : String("")) +
         ",\"ap_active\":" + (setupAPActive ? "true" : "false") +
         ",\"ap_clients\":" + String(WiFi.softAPgetStationNum()) +
         ",\"fc_frames\":" + String(validFrames) + "}";
}

void registerPeer() {
  int size = udp.parsePacket();
  if (size <= 0) return;
  char data[24] = {0};
  int n = udp.read(data, sizeof(data) - 1);
  if (n >= 10 && memcmp(data, "UAVT_HELLO", 10) == 0) {
    peerIP = udp.remoteIP(); peerPort = udp.remotePort();
    peerKnown = true; peerLastSeen = millis();
  }
  while (udp.available()) udp.read();
}
void forwardTelemetry(const uint8_t *data) {
  if (!peerKnown || (uint32_t)(millis() - peerLastSeen) > PEER_TIMEOUT_MS) {
    peerKnown = false; return;
  }
  if (udp.beginPacket(peerIP, peerPort) == 1 &&
      udp.write(data, FRAME_FLIGHT_LEN) == FRAME_FLIGHT_LEN && udp.endPacket() == 1)
    ++forwardedFrames;
  else ++udpErrors;
}
void onFrame() {
  if (rxFrame[2] != 1u) return;
  if (rxControl) {
    if (rxFrame[6] > 60) return;
    size_t n = 7u + rxFrame[6] + 2u;
    if (read16(&rxFrame[n - 2]) != crc16(rxFrame, n - 2)) { ++crcErrors; return; }
    responseSeq = read16(&rxFrame[4]); responseType = rxFrame[3];
    responseLen = rxFrame[6];
    memcpy(response, &rxFrame[7], responseLen);
    responseReady = true;
  } else if (rxFrame[3] == 1u && crc16(rxFrame, 86) == read16(&rxFrame[86])) {
    ++validFrames;
    lastTelemetryMs = millis();
    lastFlightFlags = read16(&rxFrame[10]);
    forwardTelemetry(rxFrame);
  } else ++crcErrors;
}
void receiveByte(uint8_t c) {
  if (!rxPos) {
    if (c == 0xA5 || c == 0xC3) { rxFrame[0] = c; rxControl = (c == 0xC3); rxPos = 1; }
    return;
  }
  if (rxPos == 1) {
    if (c == (rxControl ? 0x3C : 0x5A)) { rxFrame[rxPos++] = c; return; }
    rxPos = 0;
    if (c == 0xA5 || c == 0xC3) { rxFrame[0] = c; rxControl = (c == 0xC3); rxPos = 1; }
    return;
  }
  if (rxPos >= sizeof(rxFrame)) { rxPos = 0; return; }
  rxFrame[rxPos++] = c;
  if (rxPos == 7 && rxControl) {
    if (rxFrame[2] != 1u || rxFrame[6] > 60) { rxPos = 0; return; }
    rxTarget = 7u + rxFrame[6] + 2u;
  }
  if (rxPos == (rxControl ? rxTarget : FRAME_FLIGHT_LEN)) {
    onFrame(); rxPos = 0; rxTarget = 0;
  }
}
void serviceSerial() {
  /* Limit each pass to protect HTTP/UDP servicing from an ASCII log flood. */
  int budget = 512;
  while (budget-- && FcUart.available() > 0) {
    int c = FcUart.read(); if (c < 0) break;
    ++rxBytes; receiveByte(uint8_t(c));
  }
}
struct FcReply { uint8_t status = 0xFF, allowed = 0; float gains[13] = {}; };
bool fcCommand(uint8_t type, const uint8_t *payload, uint8_t len, FcReply &reply) {
  if (len > 52) return false;
  uint8_t req[7 + 52 + 2];
  uint16_t seq = ++reqSeq;
  req[0] = 0xC3; req[1] = 0x3C; req[2] = 1; req[3] = type;
  req[4] = uint8_t(seq); req[5] = uint8_t(seq >> 8); req[6] = len;
  if (len && payload) memcpy(&req[7], payload, len);
  uint16_t crc = crc16(req, 7 + len);
  req[7 + len] = uint8_t(crc); req[8 + len] = uint8_t(crc >> 8);
  responseReady = false;
  FcUart.write(req, 9 + len);
  const uint32_t started = millis();
  while (uint32_t(millis() - started) < FC_TIMEOUT_MS) {
    serviceSerial(); registerPeer();
    if (responseReady && responseSeq == seq && responseType == (type | 0x80)) {
      if (responseLen < 2) return false;
      reply.status = response[0]; reply.allowed = response[1];
      if (responseLen == 54) memcpy(reply.gains, &response[2], 52);
      return true;
    }
    delay(1);
  }
  return false;
}
String floatJSON(const FcReply &r) {
  String out = "{\"status\":" + String(r.status) + ",\"config_allowed\":" +
               String(r.allowed ? "true" : "false") + ",\"gains\":[";
  for (uint8_t i = 0; i < CFG_COUNT; ++i) {
    if (i) out += ',';
    out += String(r.gains[i], 6);
  }
  out += "]}";
  return out;
}
bool parseCSV(const String &src, float out[13]) {
  const char *cur = src.c_str();
  for (uint8_t i = 0; i < 13; ++i) {
    char *end = nullptr;
    float val = strtof(cur, &end);
    if (end == cur || !isfinite(val)) return false;
    out[i] = val;
    cur = end;
    if (i != 12) { if (*cur++ != ',') return false; }
  }
  return *cur == 0 || *cur == '\n' || *cur == '\r';
}
void apiPidGet() {
  if (!requireAuth()) return;
  FcReply r;
  if (!fcCommand(0x10, nullptr, 0, r)) { http.send(504, "application/json", "{\"error\":\"STM32 no reply\"}"); return; }
  http.send(r.status == 0 ? 200 : 409, "application/json", floatJSON(r));
}
void apiPidApply() {
  if (!requireAuth()) return;
  float gains[13];
  if (!parseCSV(http.arg("plain"), gains)) { http.send(400, "application/json", "{\"error\":\"need 13 comma-separated floats\"}"); return; }
  FcReply r;
  if (!fcCommand(0x11, reinterpret_cast<const uint8_t *>(gains), 52, r)) {
    http.send(504, "application/json", "{\"error\":\"STM32 no reply\"}"); return;
  }
  http.send(r.status == 0 ? 200 : 409, "application/json", floatJSON(r));
}
void apiPidSave() {
  if (!requireAuth()) return;
  FcReply r;
  if (!fcCommand(0x12, nullptr, 0, r)) { http.send(504, "application/json", "{\"error\":\"STM32 no reply\"}"); return; }
  http.send(r.status == 0 ? 200 : 409, "application/json", floatJSON(r));
}

/* ST AN3155 USART bootloader. Bench-only and OFF in STM32 build by default.
 * Upload is checked before requesting a bootloader handoff. No mass erase.
 * Deliberately avoid last 2 pages (0x0807F000..0x0807FFFF) containing PID. */
constexpr uint8_t ROM_ACK = 0x79;
bool romByte(uint8_t expected, uint32_t timeoutMs) {
  uint32_t start = millis();
  while (uint32_t(millis() - start) < timeoutMs) {
    if (FcUart.available()) return FcUart.read() == expected;
    delay(1);
  }
  return false;
}
bool romCommand(uint8_t cmd, uint32_t timeout = 1500) {
  uint8_t b[2] = {cmd, uint8_t(cmd ^ 0xFF)};
  FcUart.write(b, 2); FcUart.flush();
  return romByte(ROM_ACK, timeout);
}
bool romAddress(uint32_t addr) {
  uint8_t b[5] = {uint8_t(addr >> 24), uint8_t(addr >> 16), uint8_t(addr >> 8), uint8_t(addr), 0};
  b[4] = b[0] ^ b[1] ^ b[2] ^ b[3];
  FcUart.write(b, 5); FcUart.flush();
  return romByte(ROM_ACK, 1000);
}
bool romErasePages(uint16_t count) {
  if (!count || count > 254 || !romCommand(0x44)) return false;
  /* Extended Erase: N-1 big-endian, global 16-bit page indices 0..253. */
  uint8_t nHi = uint8_t((count - 1) >> 8), nLo = uint8_t(count - 1);
  uint8_t checksum = nHi ^ nLo;
  FcUart.write(nHi); FcUart.write(nLo);
  for (uint16_t p = 0; p < count; ++p) {
    uint8_t hi = uint8_t(p >> 8), lo = uint8_t(p);
    FcUart.write(hi); FcUart.write(lo); checksum ^= hi ^ lo;
  }
  FcUart.write(checksum); FcUart.flush();
  return romByte(ROM_ACK, 20000);
}
bool romWrite(uint32_t addr, const uint8_t *data, uint16_t n) {
  if (!n || n > 256 || !romCommand(0x31) || !romAddress(addr)) return false;
  uint8_t checksum = uint8_t(n - 1);
  FcUart.write(uint8_t(n - 1));
  for (uint16_t i = 0; i < n; ++i) checksum ^= data[i];
  FcUart.write(data, n); FcUart.write(checksum); FcUart.flush();
  return romByte(ROM_ACK, 2000);
}
bool romRead(uint32_t addr, uint8_t *data, uint16_t n) {
  if (!n || n > 256 || !romCommand(0x11) || !romAddress(addr)) return false;
  FcUart.write(uint8_t(n - 1)); FcUart.write(uint8_t((n - 1) ^ 0xFF)); FcUart.flush();
  if (!romByte(ROM_ACK, 1000)) return false;
  for (uint16_t i = 0; i < n; ++i) {
    uint32_t t = millis();
    while (!FcUart.available() && uint32_t(millis() - t) < 2000) delay(1);
    if (!FcUart.available()) return false;
    data[i] = uint8_t(FcUart.read());
  }
  return true;
}
bool firmwareValid(const char *filename, uint32_t &size) {
  File f = SPIFFS.open(filename, FILE_READ);
  if (!f) return false;
  size = f.size();
  if (size < 8 || size > FLASH_MAX_BYTES) { f.close(); return false; }
  uint8_t vector[8];
  bool ok = f.read(vector, 8) == 8;
  f.close();
  if (!ok) return false;
  uint32_t sp = read32(vector), entry = read32(vector + 4);
  return sp >= 0x20000000u && sp <= 0x20020000u &&
         (entry & 1u) && (entry & ~1u) >= 0x08000000u && (entry & ~1u) < 0x0807F000u;
}
/* mbedTLS 2.x uses *_ret(), mbedTLS 3.x renamed them without *_ret(). */
#if MBEDTLS_VERSION_MAJOR >= 3
  #define UAV_SHA256_START(ctx)         mbedtls_sha256_starts((ctx), 0)
  #define UAV_SHA256_UPDATE(ctx, p, n)  mbedtls_sha256_update((ctx), (p), (n))
  #define UAV_SHA256_FINISH(ctx, out)   mbedtls_sha256_finish((ctx), (out))
#else
  #define UAV_SHA256_START(ctx)         mbedtls_sha256_starts_ret((ctx), 0)
  #define UAV_SHA256_UPDATE(ctx, p, n)  mbedtls_sha256_update_ret((ctx), (p), (n))
  #define UAV_SHA256_FINISH(ctx, out)   mbedtls_sha256_finish_ret((ctx), (out))
#endif

bool checkSha256(const String &want) {
  if (want.length() != 64) return false;
  File f = SPIFFS.open("/stm32.bin", FILE_READ);
  if (!f) return false;
  uint8_t bytes[512], digest[32];
  mbedtls_sha256_context h;
  mbedtls_sha256_init(&h);
  bool ok = (UAV_SHA256_START(&h) == 0);
  while (ok && f.available()) {
    size_t n = f.read(bytes, sizeof(bytes));
    if (n == 0) { ok = false; break; }
    ok = (UAV_SHA256_UPDATE(&h, bytes, n) == 0);
  }
  if (ok) ok = (UAV_SHA256_FINISH(&h, digest) == 0);
  mbedtls_sha256_free(&h);
  f.close();
  if (!ok) return false;
  char hex[65];
  for (uint8_t i = 0; i < 32; ++i) snprintf(&hex[i*2], 3, "%02x", digest[i]);
  hex[64] = 0;
  return want.equalsIgnoreCase(hex);
}
bool flashStm32(uint32_t size) {
  FcReply r;
  if (!fcCommand(0x13, nullptr, 0, r) || r.status != 0 || !r.allowed) return false;
  /* Note: STM32 needs ~200 ms after ACK, plus ROM startup time. */
  delay(450);
  FcUart.end();
  FcUart.begin(FC_BAUD, SERIAL_8E1, FC_RX, FC_TX);
  while (FcUart.available()) FcUart.read();
  FcUart.write(uint8_t(0x7F)); FcUart.flush();
  bool ok = romByte(ROM_ACK, 2500);
  if (ok) ok = romErasePages(uint16_t((size + 2047u) / 2048u));
  File bin;
  if (ok) bin = SPIFFS.open("/stm32.bin", FILE_READ);
  if (ok && !bin) ok = false;
  uint8_t data[256], check[256];
  uint32_t address = 0x08000000;
  while (ok && bin.available()) {
    size_t n = bin.read(data, sizeof(data));
    if (!n) { ok = false; break; }
    /* Write Memory permits 1..256 bytes, multiple of 4; pad final block. */
    uint16_t padded = uint16_t((n + 3u) & ~3u);
    for (size_t j = n; j < padded; ++j) data[j] = 0xFF;
    ok = romWrite(address, data, padded) && romRead(address, check, padded) &&
         memcmp(data, check, padded) == 0;
    address += padded;
    delay(0);
  }
  if (bin) bin.close();
  if (ok) ok = romCommand(0x21) && romAddress(0x08000000u);
  FcUart.end();
  FcUart.begin(FC_BAUD, SERIAL_8N1, FC_RX, FC_TX);
  rxPos = 0; responseReady = false;
  return ok;
}
void onUpload() {
  HTTPUpload &u = http.upload();
  if (u.status == UPLOAD_FILE_START) {
    uploadBad = !authorized() || !SPIFFS.begin(false);
    uploadReady = false; uploadBytes = 0;
    if (!uploadBad) { SPIFFS.remove("/stm32.bin"); uploadFile = SPIFFS.open("/stm32.bin", FILE_WRITE); uploadBad = !uploadFile; }
  } else if (u.status == UPLOAD_FILE_WRITE) {
    if (!uploadBad) {
      if (uploadBytes + u.currentSize > FLASH_MAX_BYTES ||
          uploadFile.write(u.buf, u.currentSize) != u.currentSize) uploadBad = true;
      else uploadBytes += u.currentSize;
    }
  } else if (u.status == UPLOAD_FILE_END) {
    if (uploadFile) uploadFile.close();
    uploadReady = !uploadBad && uploadBytes >= 8;
  } else if (u.status == UPLOAD_FILE_ABORTED) {
    if (uploadFile) uploadFile.close();
    uploadBad = true; uploadReady = false;
  }
}
void apiOTA() {
  if (!requireAuth()) return;
  uint32_t size = 0;
  if (uploadBad || !uploadReady || !firmwareValid("/stm32.bin", size) ||
      !checkSha256(http.header("X-SHA256"))) {
    http.send(400, "application/json", "{\"error\":\"upload invalid, vector/size/SHA256 mismatch\"}");
    SPIFFS.remove("/stm32.bin"); return;
  }
  bool ok = flashStm32(size);
  SPIFFS.remove("/stm32.bin"); uploadReady = false;
  if (!ok) {
    http.send(409, "application/json", "{\"error\":\"STM32 boot disabled/locked or ROM flashing failed; use SWD to recover if needed\"}");
  } else {
    http.send(200, "application/json", "{\"ok\":true,\"bytes\":" + String(size) + "}");
  }
}

// ================= ESP32-CAM OTA via Wi-Fi =================
// Uses the Arduino Update library's inactive OTA app partition, not SPIFFS.
// Browser/Python client must provide firmware length and the expected SHA256.
void espOtaAbort(const String &message) {
  espOtaError = message;
  espOtaOk = false;
  if (espOtaStarted) Update.abort();
  espOtaStarted = false;
  if (espOtaHashStarted) { mbedtls_sha256_free(&espOtaHash); espOtaHashStarted = false; }
  Serial.printf("[ESP OTA] Rejected: %s\n", message.c_str());
}
bool validHexSha256(const String &v) {
  if (v.length() != 64) return false;
  for (size_t i = 0; i < 64; ++i) {
    const char c = v[i];
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'))) return false;
  }
  return true;
}
void espOtaOnUpload() {
  HTTPUpload &u = http.upload();
  if (u.status == UPLOAD_FILE_START) {
    espOtaOk = false;
    espOtaError = "";
    espOtaReceivedBytes = 0;
    espOtaStarted = false;
    espOtaRebootPending = false;
    if (!authorized()) { espOtaError = "invalid admin token"; return; }
    serviceSerial(); serviceOtaBenchPin();
    if (!espOtaSafetyAllowed()) { espOtaError = "FC must be DISARM+KILL+CONFIG; if FC telemetry absent, hold GPIO2 LOW for 3s on bench"; return; }
    if (!u.filename.endsWith(".bin")) { espOtaError = "upload a firmware.bin file, not a partitions or bootloader image"; return; }
    const String bytesHeader = http.header("X-Firmware-Size");
    char *end = nullptr;
    unsigned long declared = strtoul(bytesHeader.c_str(), &end, 10);
    if (bytesHeader.isEmpty() || !end || *end || declared < 1024u || declared > ESP.getFreeSketchSpace()) {
      espOtaError = "invalid size or no free OTA slot: check partition table and bin size"; return;
    }
    const String sha = http.header("X-SHA256");
    if (!validHexSha256(sha)) { espOtaError = "X-SHA256 must contain 64 hex characters"; return; }
    espOtaExpectedBytes = uint32_t(declared);
    espOtaExpectedSha = sha;
    if (!Update.begin(espOtaExpectedBytes, U_FLASH)) {
      espOtaError = "Update.begin failed: no OTA partition, insufficient space or flash error";
      return;
    }
    mbedtls_sha256_init(&espOtaHash);
#if MBEDTLS_VERSION_MAJOR >= 3
    const int hashStatus = mbedtls_sha256_starts(&espOtaHash, 0);
#else
    const int hashStatus = mbedtls_sha256_starts_ret(&espOtaHash, 0);
#endif
    if (hashStatus != 0) { Update.abort(); mbedtls_sha256_free(&espOtaHash); espOtaError = "SHA256 init failed"; return; }
    espOtaHashStarted = true;
    espOtaStarted = true;
    Serial.printf("[ESP OTA] Begin %lu bytes; safety=%s\n", (unsigned long)declared, espOtaSafetyReason().c_str());
    return;
  }
  if (u.status == UPLOAD_FILE_ABORTED) {
    if (espOtaStarted) espOtaAbort("client upload aborted");
    return;
  }
  if (!espOtaStarted) return;
  serviceSerial(); serviceOtaBenchPin();
  if (!espOtaSafetyAllowed()) { espOtaAbort("safety state lost while uploading; not rebooting"); return; }
  if (u.status == UPLOAD_FILE_WRITE) {
    if (espOtaReceivedBytes + u.currentSize > espOtaExpectedBytes) { espOtaAbort("upload exceeds declared file size"); return; }
    if (espOtaReceivedBytes == 0 && (u.currentSize == 0 || u.buf[0] != 0xE9)) {
      espOtaAbort("not an ESP32 application image (bad magic byte)"); return;
    }
#if MBEDTLS_VERSION_MAJOR >= 3
    const int hashStatus = mbedtls_sha256_update(&espOtaHash, u.buf, u.currentSize);
#else
    const int hashStatus = mbedtls_sha256_update_ret(&espOtaHash, u.buf, u.currentSize);
#endif
    if (hashStatus != 0 || Update.write(u.buf, u.currentSize) != u.currentSize) {
      espOtaAbort("SHA256 calculation or flash write failed"); return;
    }
    espOtaReceivedBytes += uint32_t(u.currentSize);
  } else if (u.status == UPLOAD_FILE_END) {
    if (espOtaReceivedBytes != espOtaExpectedBytes) { espOtaAbort("uploaded size differs from declared size"); return; }
    uint8_t digest[32] = {};
#if MBEDTLS_VERSION_MAJOR >= 3
    const int hashStatus = mbedtls_sha256_finish(&espOtaHash, digest);
#else
    const int hashStatus = mbedtls_sha256_finish_ret(&espOtaHash, digest);
#endif
    mbedtls_sha256_free(&espOtaHash); espOtaHashStarted = false;
    if (hashStatus != 0) { espOtaAbort("SHA256 finalize failed"); return; }
    char hex[65];
    for (size_t i = 0; i < 32; ++i) snprintf(&hex[2 * i], 3, "%02x", digest[i]);
    hex[64] = 0;
    if (!espOtaExpectedSha.equalsIgnoreCase(hex)) {
      espOtaAbort("SHA256 mismatch (firmware not committed)"); return;
    }
    serviceSerial(); serviceOtaBenchPin();
    if (!espOtaSafetyAllowed()) { espOtaAbort("FC no longer safe at commit"); return; }
    if (!Update.end(false)) { espOtaAbort("Update.end failed: invalid firmware or flash verification error"); return; }
    espOtaStarted = false;
    espOtaOk = true;
    Serial.printf("[ESP OTA] Verified %lu bytes, awaiting safe reboot\n", (unsigned long)espOtaReceivedBytes);
  }
}
void apiEspOtaFinish() {
  if (!requireAuth()) return;
  if (!espOtaOk) {
    const String reason = espOtaError.isEmpty() ? String("upload not accepted") : espOtaError;
    http.send(409, "application/json", String("{\"ok\":false,\"error\":") + jsonQuote(reason) + "}");
    return;
  }
  espOtaOk = false;
  espOtaRebootPending = true;
  espOtaRebootAt = millis() + ESP_OTA_REBOOT_DELAY_MS;
  http.send(200, "application/json", "{\"ok\":true,\"message\":\"ESP32 OTA verified; will reboot if bench/config safety remains valid\"}");
}
void apiEspOtaInfo() {
  if (!requireAuth()) return;
  serviceSerial(); serviceOtaBenchPin();
  String info = String("{\"version\":") + jsonQuote(ESP_FW_VERSION) +
      ",\"ota_bytes_free\":" + String(ESP.getFreeSketchSpace()) +
      ",\"can_update\":" + (espOtaSafetyAllowed() ? "true" : "false") +
      ",\"safety\":" + jsonQuote(espOtaSafetyReason()) +
      ",\"uart_valid_frames\":" + String(validFrames) + "}";
  http.send(200, "application/json", info);
}

const char HOME_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="vi">
<head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>UAV Flight Console | ESP32-CAM</title>
<style>
:root{color-scheme:dark;--bg:#080e1b;--panel:#111c30;--panel2:#15233a;--panel3:#0d1729;--line:#283a55;--line2:#354962;--text:#ecf3ff;--muted:#90a4bd;--sky:#58b9ff;--blue:#3b82f6;--mint:#4cdbb5;--amber:#ffbf68;--red:#ff7888;--shadow:0 18px 45px #02071265}
*{box-sizing:border-box}html{scroll-behavior:smooth}body{margin:0;min-height:100vh;background:radial-gradient(800px 360px at 77% -12%,#183c6b79,transparent 70%),radial-gradient(640px 480px at -5% 19%,#0f3e5370,transparent 75%),var(--bg);color:var(--text);font:14px/1.5 system-ui,-apple-system,"Segoe UI",Roboto,Arial,sans-serif}
a{color:#8dd4ff;text-decoration:none}a:hover{text-decoration:underline}button,input{font:inherit}button{cursor:pointer}button:disabled{opacity:.48;cursor:not-allowed;transform:none!important}.wrap{max-width:1190px;padding:0 24px 60px;margin:0 auto}
.topbar{height:82px;display:flex;align-items:center;justify-content:space-between;border-bottom:1px solid #2639517c;gap:15px}.brand{display:flex;align-items:center;gap:13px}.brand-mark{width:43px;height:43px;display:grid;place-items:center;border-radius:13px;background:linear-gradient(145deg,#2873ce,#35a5ad);box-shadow:0 6px 24px #1f83c53d}.brand-mark svg{width:26px;height:26px}.brand-name{font-size:19px;font-weight:800;letter-spacing:.055em}.brand-name em{font-style:normal;color:#72cafb}.brand-sub{font-size:10px;letter-spacing:.19em;color:var(--muted);font-weight:700}.top-right{display:flex;align-items:center;gap:11px}.pill{display:inline-flex;gap:7px;align-items:center;border-radius:30px;background:#16263b;border:1px solid #38546b;padding:7px 12px;color:#cbd9e9;font-size:11px;font-weight:800;letter-spacing:.045em;white-space:nowrap}.pill.good{color:#92ebc8;border-color:#286651;background:#0e352b}.pill.warn{color:#ffd4a0;border-color:#805c33;background:#382a1b}.pulse{width:7px;height:7px;background:#8298b3;border-radius:50%;display:inline-block;box-shadow:0 0 0 3px #8098b31c}.pill.good .pulse{background:#57dfad;box-shadow:0 0 0 3px #57dfad29}
.hero{display:flex;justify-content:space-between;align-items:flex-end;gap:22px;margin:36px 0 24px}.eyebrow{font-size:11px;letter-spacing:.18em;font-weight:800;color:#72c7f9;margin-bottom:9px}.hero h1{font-size:clamp(29px,3.3vw,43px);line-height:1.15;letter-spacing:-.042em;margin:0 0 10px}.hero p{margin:0;color:#a8b9cd;max-width:590px}.hero-tag{display:flex;align-items:center;gap:9px;border:1px solid var(--line);border-radius:13px;background:#0e1a2b;padding:11px 14px;white-space:nowrap}.hero-tag small{color:var(--muted);display:block;font-size:10px}.hero-tag b{display:block;letter-spacing:.02em}
.safety{display:flex;align-items:center;gap:12px;border-radius:13px;background:#3c2b1945;border:1px solid #80603886;padding:13px 17px;margin-bottom:22px;color:#efc797;font-size:12px}.safety strong{color:#ffe0b2}.safety-icon{width:28px;height:28px;display:grid;place-items:center;flex:none;border-radius:9px;background:#6041227d;font-size:17px}
.metrics{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:14px;margin-bottom:21px}.metric{border:1px solid var(--line);border-radius:17px;padding:18px 18px 16px;background:linear-gradient(150deg,#16253b,#111b2e);box-shadow:0 9px 25px #040a1760;min-width:0;position:relative;overflow:hidden}.metric::after{content:"";position:absolute;right:-20px;top:-25px;width:95px;height:95px;border-radius:50%;background:radial-gradient(circle,#228fcd16,transparent 70%)}.metric-label{font-size:11px;font-weight:700;letter-spacing:.06em;color:#9aacc5;text-transform:uppercase}.metric-main{display:flex;align-items:baseline;gap:6px;margin-top:12px}.metric-value{font-size:30px;font-weight:800;letter-spacing:-.035em;line-height:1.12;font-variant-numeric:tabular-nums;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}.metric-unit{font-size:12px;color:#829bb6}.metric-foot{font-size:11px;color:#88a4bf;margin-top:10px}.metric-foot.ok{color:#70dab3}.metric-foot.alert{color:#ffc17c}
.layout{display:grid;grid-template-columns:minmax(0,1.67fr) minmax(312px,1fr);gap:20px;align-items:start}.stack{display:flex;flex-direction:column;gap:20px;min-width:0}.panel{background:linear-gradient(180deg,#152239 0%,#101a2b 100%);border:1px solid var(--line);border-radius:18px;box-shadow:var(--shadow);overflow:hidden}.panel-head{display:flex;align-items:start;justify-content:space-between;gap:15px;padding:23px 23px 20px;border-bottom:1px solid #293b51}.panel-head h2{font-size:18px;letter-spacing:-.018em;margin:0 0 5px}.panel-head p{margin:0;color:#8da3bc;font-size:12px}.panel-body{padding:22px 23px}.section-mark{width:34px;height:34px;border:1px solid #3a6486;background:#1b3e5b;border-radius:11px;display:grid;place-items:center;flex:none;color:#8cd6ff;font-weight:800;font-size:15px}.head-title{display:flex;gap:12px;align-items:center}.title-actions{display:flex;gap:8px;align-items:center;flex-wrap:wrap;justify-content:flex-end}
.btn{min-height:38px;padding:9px 13px;display:inline-flex;align-items:center;justify-content:center;gap:7px;border-radius:9px;border:1px solid transparent;background:#27415e;color:#e6f0fb;font-weight:750;font-size:12px;transition:background .17s,transform .17s;white-space:nowrap}.btn:hover:not(:disabled){background:#355577;transform:translateY(-1px)}.btn.primary{background:linear-gradient(135deg,#3e9beb,#246ed0);box-shadow:0 4px 13px #187fe22a}.btn.primary:hover:not(:disabled){background:linear-gradient(135deg,#66b9f8,#297bda)}.btn.outline{background:#122239;border-color:#3e526c}.btn.outline:hover:not(:disabled){background:#1d3550}.btn.amber{background:#654521;border:1px solid #ae7a3e;color:#ffe2b8}.btn.amber:hover:not(:disabled){background:#85572a}.btn .ico{font-size:15px;line-height:1}
.subhead{font-size:10px;letter-spacing:.15em;font-weight:850;color:#779fbd;text-transform:uppercase;margin:2px 0 14px}.pid-block{border-bottom:1px solid #26374a;padding-bottom:19px;margin-bottom:22px}.pid-block:last-of-type{border:0;margin-bottom:0;padding-bottom:0}.input-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:13px}.input-grid.limit{grid-template-columns:repeat(2,minmax(0,1fr))}.form-field{display:block;min-width:0}.form-field span{display:block;font-size:11px;color:#a7bbd2;margin-bottom:7px;font-weight:650}.form-field input{width:100%;height:43px;border-radius:10px;background:#0d182a;border:1px solid #34506b;color:#f0f6fe;padding:0 12px;outline:none;font-weight:680;font-size:14px;font-variant-numeric:tabular-nums;transition:border-color .15s,box-shadow .15s}.form-field input:hover:not(:disabled){border-color:#50738f}.form-field input:focus{border-color:#65b8ed;box-shadow:0 0 0 3px #429af129}.form-field input:disabled{color:#7c93ab;background:#101a2b;opacity:.85}.axis-row{display:grid;grid-template-columns:58px repeat(3,minmax(0,1fr));gap:10px;align-items:end;margin-bottom:12px}.axis-title{padding-bottom:13px;font-size:11px;letter-spacing:.075em;color:#aac3db;font-weight:850}.pid-foot{display:flex;justify-content:space-between;align-items:center;gap:8px;margin-top:19px;padding-top:18px;border-top:1px solid #2c3d52;flex-wrap:wrap}.help{color:#8fa6c0;font-size:11px}.checkbox{display:flex;align-items:flex-start;gap:10px;color:#b6c8da;font-size:12px;cursor:pointer}.checkbox input{width:17px;height:17px;accent-color:#3ba5e8;flex:none;margin:1px 0}.button-row{display:flex;flex-wrap:wrap;gap:9px;margin-top:18px}
.auth-input{display:flex;gap:9px;align-items:stretch}.auth-input input{min-width:0;flex:1;height:43px;border-radius:10px;background:#0d182a;border:1px solid #344a64;color:#f0f6ff;padding:0 12px;outline:none;letter-spacing:.07em}.auth-input input:focus{border-color:var(--sky);box-shadow:0 0 0 3px #3fa6e826}.auth-note{color:#839db9;font-size:11px;margin:10px 0 0}.info-row{display:flex;justify-content:space-between;gap:12px;padding:11px 0;border-bottom:1px solid #29394a;font-size:12px}.info-row:last-child{border:0}.info-row span{color:#91a9c3}.info-row strong{font-weight:800;text-align:right;word-break:break-all}.info-row strong.green{color:#7ee3bd}.status-line{margin-top:17px;font-size:12px;color:#b5c8db;background:#0b1729;border:1px solid #2c415a;border-radius:11px;padding:11px 13px;min-height:43px}.status-line.error{color:#ffb8bc;border-color:#86414b}.status-line.success{color:#9eeac9;border-color:#326d56}
.ota-state{font-size:12px;color:#b4c9df;background:#0d182b;border-radius:12px;border:1px solid #344860;padding:13px 14px;min-height:48px;margin-bottom:17px}.ota-state.good{border-color:#3f8c6d;color:#8de9c0}.ota-state.bad{border-color:#a26543;color:#ffd09e}.file-drop{display:block;border:1px dashed #557390;background:#101d31;border-radius:12px;padding:16px 13px;text-align:center;cursor:pointer}.file-drop:hover{border-color:#83c6f4;background:#132940}.file-drop input{width:100%;font-size:11px;max-width:100%;color:#b5c9e1}.file-drop input::file-selector-button{padding:8px 10px;border-radius:8px;background:#294765;color:#eaf4ff;border:1px solid #466787;cursor:pointer;margin-right:10px}.ota-small{font-size:11px;color:#96a8ba;margin-top:12px}.progress{height:5px;background:#243c55;border-radius:15px;overflow:hidden;margin:13px 0 0;display:none}.progress.active{display:block}.progress::before{content:"";display:block;width:35%;height:100%;border-radius:20px;background:linear-gradient(90deg,#36b3ed,#69dfba);animation:move 1.15s linear infinite}@keyframes move{from{transform:translateX(-120%)}to{transform:translateX(420%)}}
.console{margin-top:21px;border:1px solid #293b53;border-radius:14px;background:#0b1526;padding:14px 17px}.console-head{display:flex;align-items:center;justify-content:space-between;gap:8px;color:#7f9dbb;font-size:10px;font-weight:800;letter-spacing:.13em;margin-bottom:8px}.console pre{margin:0;white-space:pre-wrap;word-break:break-word;font:11px/1.7 ui-monospace,SFMono-Regular,Consolas,monospace;color:#c4d8ec;max-height:150px;overflow:auto}.muted{color:var(--muted)}.tiny{font-size:11px;color:#8297af}.footer{display:flex;justify-content:space-between;gap:12px;align-items:center;color:#718aa7;font-size:11px;padding-top:21px}.sr{position:absolute!important;height:1px;width:1px;overflow:hidden;clip:rect(0,0,0,0)}
@media(max-width:990px){.layout{grid-template-columns:1fr}.metrics{grid-template-columns:repeat(2,minmax(0,1fr))}.hero{margin-top:26px}}@media(max-width:610px){.wrap{padding:0 15px 36px}.topbar{height:71px}.brand-name{font-size:17px}.brand-sub{font-size:9px}.top-right .pill:last-child{display:none}.hero{display:block;margin:26px 0 20px}.hero-tag{margin-top:16px;display:inline-flex}.hero h1{font-size:30px}.safety{align-items:flex-start}.metric{padding:15px}.metrics{gap:10px}.metric-value{font-size:25px}.metric-label{font-size:10px}.panel-head{padding:18px 16px}.panel-body{padding:18px 16px}.axis-row{grid-template-columns:43px repeat(3,minmax(0,1fr));gap:7px}.axis-title{font-size:10px}.form-field input{padding:0 7px;font-size:12px}.form-field span{font-size:10px}.title-actions{justify-content:flex-start}.head-title{align-items:start}.footer{display:block}.footer span{display:block;margin-bottom:6px}}
</style>
</head>
<body>
<div class="wrap">
<header class="topbar">
  <div class="brand"><div class="brand-mark" aria-hidden="true"><svg viewBox="0 0 28 28" fill="none" stroke="white" stroke-width="1.8"><path d="M14 2 3 8v12l11 6 11-6V8z"/><path d="m14 7 5 7-5 7-5-7z"/><path d="M3 8l6 6-6 6m22-12-6 6 6 6"/></svg></div><div><div class="brand-name">UAV <em>CONSOLE</em></div><div class="brand-sub">ESP32-CAM / FLIGHT LINK</div></div></div>
  <div class="top-right"><span id="linkPill" class="pill"><i class="pulse"></i> CHƯA XÁC THỰC</span><span class="pill">DIRECT AP · 2.4 GHz</span></div>
</header>
<section class="hero"><div><div class="eyebrow">TRẠM GIÁM SÁT & CẤU HÌNH</div><h1>Flight Control Dashboard</h1><p>Giám sát đường truyền UART / UDP, đọc và điều chỉnh PID STM32, cập nhật firmware ESP32-CAM — tất cả qua Wi-Fi trực tiếp.</p></div><div class="hero-tag"><div style="font-size:20px;color:#77dfc2" aria-hidden="true">◉</div><div><small>ĐỊA CHỈ THIẾT BỊ</small><b>192.168.4.1</b></div></div></section>
<div class="safety"><span class="safety-icon" aria-hidden="true">⚠</span><div><strong>Chỉ cấu hình trên bàn:</strong> tháo cánh quạt, ngắt nguồn động lực, ARM OFF, KILL ON, ga thấp, CH8 CONFIG ON. Wi-Fi 2.4 GHz có thể ảnh hưởng bộ thu RC cùng băng tần.</div></div>
<section class="metrics" aria-label="Chỉ số đường truyền">
  <article class="metric"><div class="metric-label">UART RX</div><div class="metric-main"><span id="mUart" class="metric-value">—</span></div><div id="fUart" class="metric-foot">Tổng số byte STM32 → ESP32</div></article>
  <article class="metric"><div class="metric-label">Telemetry frames</div><div class="metric-main"><span id="mFrames" class="metric-value">—</span></div><div id="fFrames" class="metric-foot">Chưa đọc dữ liệu</div></article>
  <article class="metric"><div class="metric-label">UDP chuyển tiếp</div><div class="metric-main"><span id="mUdp" class="metric-value">—</span></div><div id="fUdp" class="metric-foot">Cần Ground Station gửi HELLO</div></article>
  <article class="metric"><div class="metric-label">CRC lỗi</div><div class="metric-main"><span id="mCrc" class="metric-value">—</span></div><div id="fCrc" class="metric-foot">Lỗi khung UART</div></article>
</section>
<div class="layout">
 <div class="stack">
   <section class="panel" id="pidPanel"><div class="panel-head"><div class="head-title"><span class="section-mark">01</span><div><h2>PID Tuning / STM32</h2><p>Đọc thông số hiện tại, áp dụng RAM hoặc lưu vào Flash.</p></div></div><div id="pidBadge" class="pill">CHƯA ĐỌC PID</div></div>
     <div class="panel-body">
       <div class="pid-block"><div class="subhead">Điều khiển góc và tốc độ</div><div class="input-grid" id="attitudeInputs"></div></div>
       <div class="pid-block"><div class="subhead">Rate controller · Trục / Kp / Ki / Kd</div><div id="rateInputs"></div></div>
       <div class="pid-block"><div class="subhead">Giới hạn bộ điều khiển</div><div class="input-grid limit" id="limitInputs"></div></div>
       <label class="checkbox"><input id="benchConfirm" type="checkbox"><span>Tôi xác nhận đã <b>tháo cánh, ngắt nguồn động lực</b> và đặt STM32 ở trạng thái cấu hình an toàn.</span></label>
       <div class="button-row"><button id="pidReadBtn" class="btn primary" onclick="loadPid()"><span class="ico">↻</span> Đọc PID</button><button id="pidApplyBtn" class="btn outline" onclick="applyPid()" disabled>Áp dụng RAM</button><button id="pidSaveBtn" class="btn amber" onclick="savePid()" disabled>Lưu Flash</button></div>
       <div class="pid-foot"><span id="pidHint" class="help">Đọc PID để lấy thông số từ STM32.</span><span class="tiny">13 thông số · Không tự ghi Flash</span></div>
     </div>
   </section>
 </div>
 <aside class="stack">
  <section class="panel"><div class="panel-head"><div class="head-title"><span class="section-mark">02</span><div><h2>Xác thực quản trị</h2><p>Token chỉ gửi tới ESP32-CAM cục bộ.</p></div></div></div><div class="panel-body">
   <label for="token" class="subhead" style="display:block;margin:0 0 10px">API Token</label>
   <div class="auth-input"><input id="token" type="password" autocomplete="off" spellcheck="false" placeholder="Nhập token quản trị"><button class="btn outline" type="button" onclick="toggleToken()" aria-label="Ẩn hoặc hiện token">◉</button></div>
   <div class="button-row"><button class="btn primary" onclick="init()">Kết nối & đọc trạng thái</button><button class="btn outline" onclick="wifiStatus()">Làm mới</button></div>
   <p class="auth-note">Lưu trong phiên của tab này. Không có dịch vụ đám mây hoặc truy cập Internet.</p>
   <div class="status-line" id="authState">Chưa kết nối đến API quản trị.</div>
  </div></section>
  <section class="panel"><div class="panel-head"><div class="head-title"><span class="section-mark">03</span><div><h2>Đường truyền trực tiếp</h2><p>Wi-Fi nội bộ · không cần router.</p></div></div></div><div class="panel-body">
   <div class="info-row"><span>Chế độ</span><strong id="apMode" class="green">DIRECT AP ONLY</strong></div>
   <div class="info-row"><span>Tên mạng</span><strong id="ssidValue">UAV-DIRECT-XXXXXX</strong></div>
   <div class="info-row"><span>Địa chỉ ESP32</span><strong id="ipValue">192.168.4.1</strong></div>
   <div class="info-row"><span>Thiết bị truy cập</span><strong id="clientValue">—</strong></div>
   <div class="info-row"><span>Cập nhật trạng thái</span><strong id="lastUpdate">—</strong></div>
   <div class="button-row"><button class="btn outline" onclick="copyIp()">Sao chép IP</button><button class="btn outline" onclick="wifiStatus()">Làm mới số liệu</button></div>
   <p class="ota-small">Số liệu tự cập nhật mỗi 8 giây. Tốc độ ước tính từ bộ đếm frames, không phải tốc độ UDP Python nhận thực tế.</p>
  </div></section>
  <section class="panel"><div class="panel-head"><div class="head-title"><span class="section-mark">04</span><div><h2>ESP32 Firmware OTA</h2><p>Nạp ứng dụng <code>firmware.bin</code> qua mạng nội bộ.</p></div></div></div><div class="panel-body">
   <div id="espota_info" class="ota-state">Chưa kiểm tra điều kiện cập nhật firmware.</div>
   <button id="otaCheckBtn" class="btn outline" onclick="espOtaInfo()">Kiểm tra điều kiện OTA</button>
   <div style="height:18px"></div>
   <label class="subhead" for="espbin">Chọn file firmware (.bin)</label>
   <label class="file-drop"><input id="espbin" type="file" accept=".bin,application/octet-stream" onchange="showFileInfo()"></label>
   <p id="fileInfo" class="ota-small">Chỉ dùng firmware.bin từ đúng bản ESP32-CAM; không dùng bootloader hoặc partitions.</p>
   <button id="otaUploadBtn" class="btn primary" onclick="espOtaUpload()" disabled>Cập nhật ESP32</button>
   <div id="otaProgress" class="progress"></div>
   <p class="ota-small">OTA chỉ được cho phép khi điều kiện an toàn phía firmware xác nhận. Không ngắt nguồn trong khi đang nạp.</p>
  </div></section>
 </aside>
</div>
<section class="console"><div class="console-head"><span>NHẬT KÝ GIAO TIẾP</span><span id="logClock">—</span></div><pre id="msg" aria-live="polite">Sẵn sàng. Nhập API token rồi bấm “Kết nối & đọc trạng thái”.</pre></section>
<footer class="footer"><span>UAV Flight Console v2.0 · ESP32-CAM / STM32G474RE</span><span>Local dashboard · UDP 14550 · 115200 UART</span></footer>
</div>
<script>
const specs=[
 {id:0,key:'angle_kp',name:'Angle Kp'},{id:1,key:'max_rate_dps',name:'Max rate (deg/s)'},
 {id:2,key:'roll_kp',name:'Kp'},{id:3,key:'pitch_kp',name:'Kp'},{id:4,key:'yaw_kp',name:'Kp'},
 {id:5,key:'roll_ki',name:'Ki'},{id:6,key:'pitch_ki',name:'Ki'},{id:7,key:'yaw_ki',name:'Ki'},
 {id:8,key:'roll_kd',name:'Kd'},{id:9,key:'pitch_kd',name:'Kd'},{id:10,key:'yaw_kd',name:'Kd'},
 {id:11,key:'integral_limit',name:'Integral limit'},{id:12,key:'correction_limit',name:'Correction limit'}
];
const $=id=>document.getElementById(id);
const fmt=n=>Number(n).toLocaleString('vi-VN');
const inputMarkup=(i,label)=>`<label class="form-field"><span>${label}</span><input id="f${i}" type="number" step="any" inputmode="decimal" disabled aria-label="${specs[i].key}"></label>`;
$('attitudeInputs').innerHTML=inputMarkup(0,'Angle Kp')+inputMarkup(1,'Max rate (°/s)');
$('rateInputs').innerHTML=[['ROLL',2,5,8],['PITCH',3,6,9],['YAW',4,7,10]].map(r=>`<div class="axis-row"><div class="axis-title">${r[0]}</div>${inputMarkup(r[1],'Kp')}${inputMarkup(r[2],'Ki')}${inputMarkup(r[3],'Kd')}</div>`).join('');
$('limitInputs').innerHTML=inputMarkup(11,'Integral limit')+inputMarkup(12,'Correction limit');
let pidLoaded=false,pidWriteAllowed=false,pidDirty=false,otaAllowed=false,otaBusy=false,workBusy=false,prevStatus=null,pollTimer=null;
const tokenInput=$('token');
try{tokenInput.value=sessionStorage.getItem('uavtoken')||''}catch(e){}
function logMessage(message){$('msg').textContent=typeof message==='string'?message:JSON.stringify(message,null,2);$('logClock').textContent=new Date().toLocaleTimeString('vi-VN')}
function show(message){logMessage(message)}
function authBadge(kind,message){$('linkPill').className='pill '+kind;$('linkPill').innerHTML='<i class="pulse"></i> '+message;$('authState').className='status-line '+(kind==='good'?'success':kind==='warn'?'error':'');$('authState').textContent=message}
function applyButtons(){const confirmed=$('benchConfirm').checked;const canWrite=pidLoaded&&pidWriteAllowed&&confirmed&&!workBusy;$('pidApplyBtn').disabled=!canWrite;$('pidSaveBtn').disabled=!canWrite;$('pidReadBtn').disabled=workBusy;$('otaUploadBtn').disabled=!otaAllowed||!confirmed||otaBusy||!$('espbin').files.length;$('otaCheckBtn').disabled=otaBusy}
$('benchConfirm').addEventListener('change',applyButtons);
for(let i=0;i<13;i++)$('f'+i).addEventListener('input',()=>{pidDirty=true;$('pidHint').textContent='Có thay đổi chưa áp dụng vào STM32.'});
function toggleToken(){tokenInput.type=tokenInput.type==='password'?'text':'password'}
async function call(path,method='GET',body=null,contentType=null){const token=tokenInput.value.trim();if(!token)throw Error('Chưa nhập API token');try{sessionStorage.setItem('uavtoken',token)}catch(e){}const headers={'X-Auth-Token':token};if(contentType)headers['Content-Type']=contentType;const abort=new AbortController();const timeout=setTimeout(()=>abort.abort(),8000);try{const r=await fetch(path,{method,headers,body,signal:abort.signal,cache:'no-store'});let data;const raw=await r.text();try{data=JSON.parse(raw)}catch(e){throw Error('Máy chủ trả dữ liệu không hợp lệ')}if(!r.ok)throw Error(data.error||'HTTP '+r.status);return data} catch(e){if(e.name==='AbortError')throw Error('Thiết bị không phản hồi trong 8 giây');throw e}finally{clearTimeout(timeout)}}
function setMeter(id,n){$(id).textContent=typeof n==='number'?fmt(n):'—'}
async function wifiStatus(silent=false){if(otaBusy||workBusy)return;try{const d=await call('/api/status');const wifi=d.wifi||{};setMeter('mUart',d.uart_bytes);setMeter('mFrames',d.frames);setMeter('mUdp',d.udp_sent);setMeter('mCrc',d.crc_bad);$('ssidValue').textContent=wifi.ap_ssid||'—';$('ipValue').textContent=wifi.ap_ip||'192.168.4.1';$('clientValue').textContent=wifi.ap_clients??'—';$('apMode').textContent=wifi.mode||'DIRECT AP';$('lastUpdate').textContent=new Date().toLocaleTimeString('vi-VN');if(prevStatus){const seconds=(Date.now()-prevStatus.at)/1000;if(seconds>0.3){const hz=(Math.max(0,d.frames-prevStatus.frames)/seconds);$('fFrames').textContent='Nhận UART ~ '+hz.toFixed(1)+' frames/s';$('fFrames').className='metric-foot '+(hz>15?'ok':hz<2?'alert':'');const uhz=Math.max(0,d.udp_sent-prevStatus.udp)/seconds;$('fUdp').textContent='Gửi UDP ~ '+uhz.toFixed(1)+' gói/s';$('fUdp').className='metric-foot '+(uhz>10?'ok':'')}}else{$('fFrames').textContent=d.frames>0?'Đã nhận dữ liệu STM32':'Chưa có frame UART'}$('fCrc').textContent=d.crc_bad===0?'Không có lỗi CRC':'Kiểm tra đường UART';$('fCrc').className='metric-foot '+(d.crc_bad===0?'ok':'alert');$('fUart').textContent=d.uart_bytes>0?'ESP32 đang có dữ liệu UART':'Chưa nhận byte UART';prevStatus={at:Date.now(),frames:d.frames,udp:d.udp_sent};authBadge(wifi.ap_active?'good':'warn',wifi.ap_active?'LIÊN KẾT AP HOẠT ĐỘNG':'AP KHÔNG HOẠT ĐỘNG');if(!silent)logMessage('Cập nhật trạng thái thành công. UART frames: '+d.frames+'; UDP: '+d.udp_sent+'.');return d}catch(e){authBadge('warn','KHÔNG ĐỌC ĐƯỢC TRẠNG THÁI');if(!silent)logMessage('Trạng thái: '+e.message);return null}}
function init(){wifiStatus(false)}
function checkSafe(){if(!$('benchConfirm').checked)throw Error('Chưa xác nhận tháo cánh và ngắt nguồn động lực')}
function fillPid(data){if(!Array.isArray(data.gains)||data.gains.length!==13)throw Error('Gói PID thiếu 13 giá trị');data.gains.forEach((v,i)=>{if(!Number.isFinite(Number(v)))throw Error('PID không hợp lệ tại '+specs[i].key);$('f'+i).value=String(v)});pidLoaded=true;pidWriteAllowed=!!data.config_allowed;pidDirty=false;for(let i=0;i<13;i++)$('f'+i).disabled=!pidWriteAllowed;$('pidBadge').className='pill '+(pidWriteAllowed?'good':'warn');$('pidBadge').textContent=pidWriteAllowed?'CONFIG READY':'CHỈ ĐỌC / KHÓA GHI';$('pidHint').textContent=pidWriteAllowed?'Đã đọc PID. Có thể chỉnh giá trị sau khi xác nhận an toàn.':'Đã đọc PID. STM32 chưa cho phép ghi; cần DISARM + KILL + CH8 CONFIG.';applyButtons()}
async function loadPid(){if(workBusy||otaBusy)return;workBusy=true;applyButtons();try{const d=await call('/api/pid');fillPid(d);logMessage('Đọc PID thành công. STM32: '+(pidWriteAllowed?'cho phép cấu hình':'chưa cho phép ghi')+'.')}catch(e){logMessage('Đọc PID lỗi: '+e.message)}finally{workBusy=false;applyButtons()}}
function collectGains(){const values=specs.map(s=>Number($('f'+s.id).value));if(values.some((v,i)=>!Number.isFinite(v)||$('f'+i).value.trim()===''))throw Error('Phải nhập đầy đủ 13 giá trị số hữu hạn');return values}
async function applyPid(){if(workBusy||otaBusy)return;try{checkSafe();if(!pidLoaded||!pidWriteAllowed)throw Error('Cần Đọc PID và STM32 phải cho phép cấu hình');const values=collectGains();if(!confirm('Áp dụng 13 giá trị PID vào RAM STM32?\nThay đổi có thể ảnh hưởng điều khiển; UAV đã tháo cánh và ngắt nguồn động lực?'))return;workBusy=true;applyButtons();const d=await call('/api/pid','POST',values.join(','),'text/plain');fillPid(d);logMessage('Áp dụng PID vào RAM thành công. Chưa ghi Flash.')}catch(e){logMessage('Áp dụng RAM lỗi: '+e.message)}finally{workBusy=false;applyButtons()}}
async function savePid(){if(workBusy||otaBusy)return;try{checkSafe();if(!pidLoaded||!pidWriteAllowed)throw Error('STM32 chưa cho phép cấu hình');if(pidDirty)throw Error('Có giá trị thay đổi chưa áp dụng RAM. Hãy bấm Áp dụng RAM trước.');if(!confirm('GHI PID XUỐNG FLASH STM32?\nChỉ làm khi nguồn ổn định, đã tháo cánh, ngắt nguồn động lực, ARM OFF/KILL ON/CH8 CONFIG ON.'))return;workBusy=true;applyButtons();const d=await call('/api/save','POST');if(d.status!==0)throw Error('STM32 từ chối lưu PID');logMessage('STM32 xác nhận đã lưu PID vào Flash.')}catch(e){logMessage('Save Flash lỗi: '+e.message)}finally{workBusy=false;applyButtons()}}
async function espOtaInfo(){if(otaBusy)return;try{const d=await call('/api/esp32/ota/info');otaAllowed=!!d.can_update;$('espota_info').textContent='Firmware '+d.version+' · Slot '+fmt(d.ota_bytes_free)+' bytes · '+(otaAllowed?'ĐỦ ĐIỀU KIỆN OTA':'CHƯA ĐỦ ĐIỀU KIỆN: '+d.safety);$('espota_info').className='ota-state '+(otaAllowed?'good':'bad');applyButtons();return d}catch(e){otaAllowed=false;$('espota_info').textContent='Lỗi kiểm tra OTA: '+e.message;$('espota_info').className='ota-state bad';applyButtons();return null}}
function showFileInfo(){const f=$('espbin').files[0];$('fileInfo').textContent=f?('Đã chọn: '+f.name+' · '+fmt(f.size)+' bytes'):'Chưa chọn file firmware.';applyButtons()}
function sha256Local(input){
 const k=[0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2];
 const h=[0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19];
 const bytes=new Uint8Array(input), n=bytes.length;
 const padded=new Uint8Array(Math.ceil((n+9)/64)*64);padded.set(bytes);padded[n]=128;
 const bits=BigInt(n)*8n;for(let j=0;j<8;j++)padded[padded.length-1-j]=Number((bits>>BigInt(8*j))&255n);
 const rr=(x,s)=>((x>>>s)|(x<<(32-s)))>>>0;
 for(let p=0;p<padded.length;p+=64){let w=new Uint32Array(64);
   for(let j=0;j<16;j++){let q=p+4*j;w[j]=((padded[q]<<24)|(padded[q+1]<<16)|(padded[q+2]<<8)|padded[q+3])>>>0;}
   for(let j=16;j<64;j++){let x=w[j-15],y=w[j-2];let s0=rr(x,7)^rr(x,18)^(x>>>3),s1=rr(y,17)^rr(y,19)^(y>>>10);w[j]=(w[j-16]+s0+w[j-7]+s1)>>>0;}
   let [a,b,c,d,e,f,g,z]=h;
   for(let j=0;j<64;j++){let s1=rr(e,6)^rr(e,11)^rr(e,25),ch=(e&f)^(~e&g),t1=(z+s1+ch+k[j]+w[j])>>>0;
     let s0=rr(a,2)^rr(a,13)^rr(a,22),maj=(a&b)^(a&c)^(b&c),t2=(s0+maj)>>>0;
     z=g;g=f;f=e;e=(d+t1)>>>0;d=c;c=b;b=a;a=(t1+t2)>>>0;}
   [a,b,c,d,e,f,g,z].forEach((v,i)=>h[i]=(h[i]+v)>>>0);
 }
 return h.map(x=>x.toString(16).padStart(8,'0')).join('');
}
async function espOtaUpload(){if(otaBusy)return;const f=$('espbin').files[0];if(!f){logMessage('Chưa chọn firmware.bin');return}try{checkSafe();if(!f.name.toLowerCase().endsWith('.bin'))throw Error('Chỉ nhận firmware .bin');if(!confirm('CẬP NHẬT ESP32 VÀ KHỞI ĐỘNG LẠI?\nUAV đã tháo cánh, ngắt nguồn ESC và đang ở CONFIG an toàn?'))return;otaBusy=true;applyButtons();const d=await call('/api/esp32/ota/info');if(!d.can_update)throw Error('OTA bị khóa: '+d.safety);if(f.size>d.ota_bytes_free)throw Error('Firmware lớn hơn vùng OTA');logMessage('Đang kiểm tra SHA-256 cho '+f.name);$('otaProgress').classList.add('active');const b=await f.arrayBuffer();const sha=(window.crypto&&crypto.subtle)?Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',b))).map(x=>x.toString(16).padStart(2,'0')).join(''):sha256Local(b);const fd=new FormData();fd.append('firmware',f,f.name);logMessage('Đang upload firmware; tuyệt đối không ngắt nguồn.');const r=await fetch('/api/esp32/ota',{method:'POST',headers:{'X-Auth-Token':tokenInput.value.trim(),'X-SHA256':sha,'X-Firmware-Size':String(f.size)},body:fd});let res=await r.json();if(!r.ok)throw Error(res.error||'HTTP '+r.status);logMessage('Đã xác minh firmware. ESP32 sẽ reboot nếu điều kiện an toàn vẫn hợp lệ.')}catch(e){logMessage('OTA lỗi: '+e.message)}finally{otaBusy=false;$('otaProgress').classList.remove('active');applyButtons()}}
function copyIp(){const ip=$('ipValue').textContent;if(navigator.clipboard&&navigator.clipboard.writeText){navigator.clipboard.writeText(ip).then(()=>logMessage('Đã sao chép IP: '+ip)).catch(()=>window.prompt('Sao chép IP:',ip))}else window.prompt('Sao chép IP:',ip)}
applyButtons();pollTimer=setInterval(()=>{if(tokenInput.value.trim()&&!otaBusy&&!workBusy)wifiStatus(true)},8000);
</script>
</body></html>

)HTML";
} // namespace

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.printf("\n[BOOT] ESP32-CAM DIRECT-AP; version=%s reset=%d flash=%lu heap=%lu\n",
      ESP_FW_VERSION, int(esp_reset_reason()), (unsigned long)ESP.getFlashChipSize(),
      (unsigned long)ESP.getFreeHeap());
  // NVS errors must not silently kill the recovery AP. Temporary secrets are
  // generated in RAM if NVS fails (web saving remains unavailable until repaired).
  nvsReady = configNvs.begin("uav-net", false);
  if (!nvsReady) Serial.println("[NVS] ERROR: failed to open uav-net; temporary credentials will be printed; Wi-Fi settings will NOT persist");
  const uint64_t mac = ESP.getEfuseMac();
  char suffix[7]; snprintf(suffix, sizeof(suffix), "%06X", unsigned(mac & 0xFFFFFFu));
  apSsid = String("UAV-DIRECT-") + suffix;
  // Separate NVS keys rotate the old disclosed Setup credentials ONCE.
  // These new direct-AP credentials persist across reboots and firmware updates.
  apPassword = nvsReady ? randomSecret("direct_ap", 16) : makeRandomHex(16);
  apiToken = nvsReady ? randomSecret("direct_tok", 32) : makeRandomHex(32);
  Serial.println("[WIFI] DIRECT AP ONLY: no STA, no router, no Wi-Fi configuration");
  Serial.println("\n===== UAV DIRECT CREDENTIALS (KEEP PRIVATE) =====");
  Serial.printf("Direct SSID: %s\nDirect password: %s\nAPI token: %s\n", apSsid.c_str(), apPassword.c_str(), apiToken.c_str());
  Serial.println("Web URL: http://192.168.4.1/");
  Serial.println("===================================================");
  pinMode(ESP_OTA_BENCH_PIN, INPUT_PULLUP);
  serviceOtaBenchPin();
  FcUart.setRxBufferSize(2048);
  FcUart.begin(FC_BAUD, SERIAL_8N1, FC_RX, FC_TX);
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP); WiFi.setSleep(false);
  WiFi.setAutoReconnect(false);
  setupAccessPoint();
  esp_wifi_set_max_tx_power(WIFI_TX_LIMIT_QUARTER_DBM);
  Serial.println("[WIFI] Bench Wi-Fi TX cap: 13 dBm; do not fly before RF coexistence tests");
  udp.begin(UDP_PORT);
  const char *extra[] = {"X-Auth-Token", "X-SHA256", "X-Firmware-Size"};
  http.collectHeaders(extra, 3);
  http.on("/health", HTTP_GET, [](){
    http.send(200, "text/plain; charset=utf-8", String("ESP32 UAV RUNNING; AP=") +
              (setupAPActive ? "ON" : "OFF") + "; STA=" + String(int(WiFi.status())) +
              "; DIRECT_AP=1");
  });
  http.on("/", HTTP_GET, [](){ http.send_P(200, "text/html; charset=utf-8", HOME_HTML); });
  http.on("/api/status", HTTP_GET, [](){
    if (!requireAuth()) return;
    String json = "{\"wifi\":" + wifiStateJSON() + ",\"uart_bytes\":" + String(rxBytes) + ",\"frames\":" + String(validFrames) +
                  ",\"crc_bad\":" + String(crcErrors) + ",\"udp_sent\":" + String(forwardedFrames) + "}";
    http.send(200, "application/json", json);
  });
  http.on("/api/wifi", HTTP_GET, [](){ if (!requireAuth()) return; http.send(200, "application/json", wifiStateJSON()); });
  http.on("/api/pid", HTTP_GET, apiPidGet);
  http.on("/api/pid", HTTP_POST, apiPidApply);
  http.on("/api/save", HTTP_POST, apiPidSave);
  http.on("/api/ota", HTTP_POST, apiOTA, onUpload); // STM32 OTA (unchanged)
  http.on("/api/esp32/ota/info", HTTP_GET, apiEspOtaInfo);
  http.on("/api/esp32/ota", HTTP_POST, apiEspOtaFinish, espOtaOnUpload);
  http.begin();
  SPIFFS.begin(true);
  Serial.printf("UAV bridge ready. AP=%s UDP=%u HTTP=%u UART RX=%d TX=%d\n",
      WiFi.softAPIP().toString().c_str(), UDP_PORT, HTTP_PORT, FC_RX, FC_TX);
}
void loop() {
  registerPeer();
  serviceSerial();
  serviceOtaBenchPin();
  http.handleClient();
  serviceWifi();
  if (espOtaRebootPending && int32_t(millis() - espOtaRebootAt) >= 0) {
    espOtaRebootPending = false;
    serviceSerial(); serviceOtaBenchPin();
    if (espOtaSafetyAllowed()) { Serial.println("[ESP OTA] Safe reboot now"); delay(100); ESP.restart(); }
    else Serial.println("[ESP OTA] Firmware staged; auto-reboot canceled because safety state changed");
  }
  const uint32_t now = millis();
  // Independently report AP state, including if the network goes missing.
  if (uint32_t(now - lastAPHealthLogMs) >= 5000u) {
    lastAPHealthLogMs = now;
    Serial.printf("[AP CHECK] uptime=%lus mode=%d ap_active=%d ssid=%s ip=%s clients=%d heap=%lu sta=%d\n",
        (unsigned long)(now / 1000u), int(WiFi.getMode()), int(setupAPActive),
        apSsid.c_str(), WiFi.softAPIP().toString().c_str(),
        int(WiFi.softAPgetStationNum()), (unsigned long)ESP.getFreeHeap(), int(WiFi.status()));
  }
  if ((uint32_t)(now - lastStats) > 2000) {
    lastStats = now;
    Serial.printf("[UAV] RX=%lu frames=%lu CRC_BAD=%lu UDP=%lu ERR=%lu\n",
                  (unsigned long)rxBytes, (unsigned long)validFrames,
                  (unsigned long)crcErrors, (unsigned long)forwardedFrames,
                  (unsigned long)udpErrors);
  }
  delay(1);
}
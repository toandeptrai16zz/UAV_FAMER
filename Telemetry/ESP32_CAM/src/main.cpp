/* UAV telemetry + PID configuration bridge, ESP32-CAM AI-Thinker.
 * GPIO13 <- STM32 PA2 TX; GPIO14 -> STM32 PA3 RX (3.3V).
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
constexpr char ESP_FW_VERSION[] = "UAV-ESP32-DIRECT-AP-v1";
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
<!doctype html><html lang="vi"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>UAV Direct - PID / Telemetry / OTA</title>
<style>
body{max-width:900px;margin:24px auto;padding:0 16px;background:#0b1220;color:#dfe9f8;font:15px system-ui}
h1{font-size:25px}h2{font-size:18px;margin:3px 0 12px}
.card{border:1px solid #354766;background:#132038;padding:18px;border-radius:12px;margin:14px 0}
label{display:block;margin:8px 0} input,select{background:#23344c;color:white;padding:10px;border:1px solid #536783;border-radius:6px;max-width:100%;box-sizing:border-box;width:100%}
button{padding:10px 14px;background:#2f6bb0;color:white;border:0;border-radius:6px;cursor:pointer;margin:4px 4px 4px 0}
button.ghost{background:#354766}.row{display:grid;grid-template-columns:repeat(auto-fit,minmax(190px,1fr));gap:10px}
#msg{white-space:pre-wrap;background:#071123;border-radius:6px;padding:12px;min-height:35px;overflow-wrap:anywhere}
small{color:#a5b5ce}.danger{color:#ffd59c}
</style>
<h1>UAV ESP32-CAM · Direct Wi-Fi</h1>
<p class="danger">Wi-Fi phát trực tiếp, không dùng router. Thử trên bàn: tháo cánh, ngắt nguồn động lực, DISARM, KILL ON, CH8 CONFIG.</p>
<section class="card">
<h2>Quyền quản trị</h2>
<label>API token (in trên Serial Monitor lần đầu)<input id="token" type="password" autocomplete="off" placeholder="Token quản trị ESP32"></label>
<button onclick="init()">Kết nối & đọc trạng thái</button>
<small>Token giữ trong tab đang mở, không tải lên dịch vụ bên ngoài.</small>
</section>
<section class="card">
<h2>1. Kết nối trực tiếp</h2>
<p>Thiết bị luôn phát mạng <strong>UAV-DIRECT-XXXXXX</strong>.</p>
<p>Điện thoại/laptop kết nối mạng này rồi mở <a href="http://192.168.4.1/" style="color:#80bfff">http://192.168.4.1/</a>. Không cần Internet.</p>
<p id="netinfo">Nhập API token và nhấn Kết nối để xem thông tin UART/telemetry.</p>
<button onclick="wifiStatus()">Làm mới trạng thái</button>
</section>
<section class="card">
<h2>2. Cập nhật firmware ESP32 qua Wi-Fi (OTA)</h2>
<p class="danger">Chỉ thực hiện khi UAV tháo cánh, ngắt nguồn ESC, DISARM + KILL + CH8 CONFIG. Nếu STM32 không gửi telemetry, phải nối jumper GPIO2 → GND ít nhất 3 giây trên bàn; không dùng microSD.</p>
<p id="espota_info">Chưa kiểm tra điều kiện ESP32 OTA.</p>
<button onclick="espOtaInfo()">Kiểm tra OTA ESP32</button>
<label>File ứng dụng ESP32 (firmware.bin)<input id="espbin" type="file" accept=".bin,application/octet-stream"></label>
<button onclick="espOtaUpload()">Cập nhật ESP32</button>
<small>Không tải bootloader.bin, partitions.bin hay .elf. Trình duyệt tự tính SHA-256; thiết bị kiểm tra lại trước khi chuyển vùng boot. Không tắt nguồn khi đang upload.</small>
</section>
<section class="card">
<h2>3. PID (STM32)</h2>
<p>Chỉ sử dụng khi <strong>ARM OFF · KILL ON · ga thấp · CH8 CONFIG ON</strong>.</p>
<button onclick="loadPid()">Đọc PID</button><button onclick="applyPid()">Apply RAM</button><button onclick="savePid()">Save Flash</button>
<div class="row" id="fields"></div><p><small>OTA STM32 chỉ dành cho test trên bàn và đang khóa từ firmware STM32 mặc định.</small></p>
</section>
<pre id="msg">Sẵn sàng.</pre>
<script>
const labels=['angle_kp','max_rate_dps','roll_kp','pitch_kp','yaw_kp','roll_ki','pitch_ki','yaw_ki','roll_kd','pitch_kd','yaw_kd','integral_limit','correction_limit'];
document.getElementById('fields').innerHTML=labels.map((name,i)=>`<label>${name}<input id="f${i}" type="number" step="any"></label>`).join('');
const t=document.getElementById('token'); t.value=sessionStorage.getItem('uavtoken')||'';
function show(obj){document.getElementById('msg').textContent=typeof obj==='string'?obj:JSON.stringify(obj,null,2);}
async function call(path,method='GET',body=null,contentType='application/x-www-form-urlencoded'){sessionStorage.setItem('uavtoken',t.value);const r=await fetch(path,{method,headers:{'X-Auth-Token':t.value,'Content-Type':contentType},body});let d=await r.json();show(d);if(!r.ok&&r.status!==202)throw Error(d.error||`HTTP ${r.status}`);return d;}
async function wifiStatus(){
 try{
  const d=await call('/api/status');
  document.getElementById('netinfo').textContent=`AP: ${d.wifi.ap_ssid} / ${d.wifi.ap_ip} | Thiết bị kết nối: ${d.wifi.ap_clients} | UART bytes: ${d.uart_bytes} | Frames: ${d.frames} | UDP: ${d.udp_sent}`;
 }catch(e){show(e.message)}
}
async function init(){await wifiStatus();}
async function loadPid(){try{let d=await call('/api/pid');d.gains.forEach((v,i)=>document.getElementById('f'+i).value=v);}catch(e){show(e.message)}}
async function applyPid(){try{let vals=labels.map((_,i)=>document.getElementById('f'+i).value);await call('/api/pid','POST',vals.join(','),'text/plain');}catch(e){show(e.message)}}
async function savePid(){if(!confirm('Ghi PID vào STM32 Flash? Chỉ khi nguồn ổn định và tháo cánh.'))return;try{await call('/api/save','POST','');}catch(e){show(e.message)}}

async function espOtaInfo(){
 try{let d=await call('/api/esp32/ota/info');
 document.getElementById('espota_info').textContent=`Version: ${d.version} | OTA slot: ${d.ota_bytes_free} bytes | ${d.can_update?'CHO PHEP CAP NHAT':'DANG KHOA · '+d.safety}`;
 }catch(e){show('OTA: '+e.message)}
}
// Self-contained SHA-256 fallback for browsers accessing http://192.168.x.x
// (Web Crypto subtle.digest requires a secure context on many browsers).
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
async function espOtaUpload(){
 const f=document.getElementById('espbin').files[0];if(!f){show('Chọn firmware.bin trước');return;}
 if(!f.name.endsWith('.bin')){show('Phải dùng file firmware.bin');return;}
 if(!confirm('ESP32 sẽ tự cập nhật và reboot. Đã THÁO CÁNH, NGẮT NGUỒN ESC, ARM OFF + KILL ON + CONFIG ON?'))return;
 try{
  const st=await call('/api/esp32/ota/info');if(!st.can_update)throw Error('OTA bị khóa: '+st.safety);
  if(f.size>st.ota_bytes_free)throw Error('Firmware lớn hơn OTA slot ('+st.ota_bytes_free+' bytes)');
  show('Đang đọc và tính SHA256 cho '+f.name+' ...');let b=await f.arrayBuffer();
  let sha=((window.crypto && crypto.subtle)?Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',b))).map(x=>x.toString(16).padStart(2,'0')).join(''):sha256Local(b));
  const fd=new FormData();fd.append('firmware',f,f.name);
  show('SHA256: '+sha+'\nĐang upload... không ngắt nguồn');
  const r=await fetch('/api/esp32/ota',{method:'POST',headers:{'X-Auth-Token':t.value,'X-SHA256':sha,'X-Firmware-Size':String(f.size)},body:fd});
  const body=await r.json();show(body);if(!r.ok)throw Error(body.error||'HTTP '+r.status);
  show('Đã xác minh firmware. ESP32 sẽ reboot nếu trạng thái an toàn vẫn hợp lệ. Đợi ESP32 vào Wi-Fi lại.');
 }catch(e){show('ESP32 OTA lỗi: '+e.message)}
}
</script></html>
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
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
#include <esp_wifi.h>
#include <mbedtls/sha256.h>
#include "secrets.h"
#include <string.h>
#include <math.h>

namespace {
constexpr uint32_t FC_BAUD = 115200;
constexpr int FC_RX = 13, FC_TX = 14;
constexpr uint16_t UDP_PORT = 14550, HTTP_PORT = 80;
constexpr uint8_t WIFI_CHANNEL = 1, WIFI_TX_LIMIT_QUARTER_DBM = 8;
constexpr uint32_t PEER_TIMEOUT_MS = 3500;
constexpr uint32_t FC_TIMEOUT_MS = 1500;
constexpr uint32_t FLASH_MAX_BYTES = 508u * 1024u;
constexpr uint32_t CFG_COUNT = 13;
constexpr uint8_t FRAME_FLIGHT_LEN = 88, CTRL_MAX = 80;
constexpr char SSID[] = "UAV-TELEM";

HardwareSerial FcUart(1);
WiFiUDP udp;
WebServer http(HTTP_PORT);
IPAddress peerIP;
uint16_t peerPort = UDP_PORT;
bool peerKnown = false;
uint32_t peerLastSeen = 0;
uint8_t packet[88];
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
  const String expected(UAV_API_TOKEN);
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
    ++validFrames; forwardTelemetry(rxFrame);
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
bool checkSha256(const String &want) {
  if (want.length() != 64) return false;
  File f = SPIFFS.open("/stm32.bin", FILE_READ);
  if (!f) return false;
  uint8_t bytes[512], digest[32];
  mbedtls_sha256_context h; mbedtls_sha256_init(&h);
  mbedtls_sha256_starts_ret(&h, 0);
  while (f.available()) {
    size_t n = f.read(bytes, sizeof(bytes));
    if (!n) break;
    mbedtls_sha256_update_ret(&h, bytes, n);
  }
  mbedtls_sha256_finish_ret(&h, digest); mbedtls_sha256_free(&h); f.close();
  char hex[65];
  for (uint8_t i = 0; i < 32; ++i) sprintf(&hex[i*2], "%02x", digest[i]);
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
const char HOME_HTML[] PROGMEM = R"HTML(
<!doctype html><html lang="vi"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>UAV · PID Config</title>
<style>body{max-width:850px;margin:35px auto;padding:0 18px;font:16px system-ui;background:#0b1220;color:#e5eefb}h1{font-size:25px}input,button{padding:10px;border:1px solid #50617c;border-radius:6px;background:#1c2b43;color:white}button{cursor:pointer;margin:4px}fieldset{border:1px solid #42516d;border-radius:10px;display:grid;grid-template-columns:repeat(auto-fit,minmax(165px,1fr));gap:13px}label{font-size:14px;display:grid;gap:5px}input{width:95%;box-sizing:border-box}#status{white-space:pre-wrap;padding:12px;background:#15223a;border-radius:6px}small{color:#a4b1c7}</style>
<h1>UAV · STM32 PID Config</h1><p>Wi-Fi chỉ cấu hình khi <b>ARM OFF + KILL ON + ga thấp + CH8 ON</b>. Tháo cánh quạt.</p>
<label>API token<input type="password" id="token" autocomplete="off" placeholder="Token từ secrets.h"></label>
<p><button onclick="load()">Đọc STM32</button> <button onclick="apply()">Apply (RAM)</button> <button onclick="save()">Save (Flash)</button></p>
<fieldset id="fields"></fieldset><p id="status">Chưa kết nối. Nhấn Đọc STM32.</p>
<small>OTA STM32: dùng script GroundStation/uav_control.py sau khi kiểm tra bo thực tế. Khi hệ thống chưa đủ điều kiện, firmware từ chối OTA.</small>
<script>
const labels=['angle_kp','max_rate_dps','roll_kp','pitch_kp','yaw_kp','roll_ki','pitch_ki','yaw_ki','roll_kd','pitch_kd','yaw_kd','integral_limit','correction_limit'];
const field=document.getElementById('fields'); field.innerHTML=labels.map((name,i)=>`<label>${name}<input type="number" step="any" id="f${i}"></label>`).join('');
const headers=()=>({'X-Auth-Token':document.getElementById('token').value});
async function call(path,method='GET',body){let r=await fetch(path,{method,headers:{...headers(),'Content-Type':'text/plain'},body});let data=await r.json();document.getElementById('status').textContent=JSON.stringify(data,null,2); if(!r.ok)throw Error(data.error||'STM32 reject '+data.status);return data;}
async function load(){try{let x=await call('/api/pid');x.gains.forEach((v,i)=>document.getElementById('f'+i).value=v);}catch(e){alert(e.message)}}
async function apply(){try{let s=labels.map((_,i)=>document.getElementById('f'+i).value).join(',');await call('/api/pid','POST',s)}catch(e){alert(e.message)}}
async function save(){if(!confirm('Lưu PID vào Flash? Chỉ khi tháo cánh và nguồn ổn định.'))return;try{await call('/api/save','POST','')}catch(e){alert(e.message)}}
</script></html>
)HTML";
} // namespace

void setup() {
  Serial.begin(115200);
  if (strcmp(UAV_WIFI_PASSWORD, "CHANGE_WIFI_PASS_26") == 0 ||
      strcmp(UAV_API_TOKEN, "CHANGE_THIS_TOKEN_32_CHARS_BEFORE_USE") == 0) {
    Serial.println("ERROR: change both secrets.h settings before enabling Wi-Fi");
    for (;;) delay(1000);
  }
  FcUart.setRxBufferSize(2048);
  FcUart.begin(FC_BAUD, SERIAL_8N1, FC_RX, FC_TX);
  WiFi.mode(WIFI_AP); WiFi.setSleep(false);
  const IPAddress ap(192,168,4,1), subnet(255,255,255,0);
  WiFi.softAPConfig(ap, ap, subnet);
  if (!WiFi.softAP(SSID, UAV_WIFI_PASSWORD, WIFI_CHANNEL, 0, 1)) Serial.println("Wi-Fi AP failed");
  esp_wifi_set_max_tx_power(WIFI_TX_LIMIT_QUARTER_DBM);
  udp.begin(UDP_PORT);
  const char *extra[] = {"X-Auth-Token", "X-SHA256"};
  http.collectHeaders(extra, 2);
  http.on("/", HTTP_GET, [](){ http.send_P(200, "text/html; charset=utf-8", HOME_HTML); });
  http.on("/api/status", HTTP_GET, [](){
    if (!requireAuth()) return;
    String json = "{\"uart_bytes\":" + String(rxBytes) + ",\"frames\":" + String(validFrames) +
                  ",\"crc_bad\":" + String(crcErrors) + ",\"udp_sent\":" + String(forwardedFrames) + "}";
    http.send(200, "application/json", json);
  });
  http.on("/api/pid", HTTP_GET, apiPidGet);
  http.on("/api/pid", HTTP_POST, apiPidApply);
  http.on("/api/save", HTTP_POST, apiPidSave);
  http.on("/api/ota", HTTP_POST, apiOTA, onUpload);
  http.begin();
  SPIFFS.begin(true);
  Serial.printf("UAV bridge ready. SSID=%s AP=%s UDP=%u HTTP=%u UART RX=%d TX=%d\n",
      SSID, WiFi.softAPIP().toString().c_str(), UDP_PORT, HTTP_PORT, FC_RX, FC_TX);
}
void loop() {
  registerPeer();
  serviceSerial();
  http.handleClient();
  const uint32_t now = millis();
  if ((uint32_t)(now - lastStats) > 2000) {
    lastStats = now;
    Serial.printf("[UAV] RX=%lu frames=%lu CRC_BAD=%lu UDP=%lu ERR=%lu\n",
                  (unsigned long)rxBytes, (unsigned long)validFrames,
                  (unsigned long)crcErrors, (unsigned long)forwardedFrames,
                  (unsigned long)udpErrors);
  }
  delay(1);
}
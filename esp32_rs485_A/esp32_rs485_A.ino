/*
  ESP32 RS485 CHUNK ECHO TEST - Board A (controller)
  Hardware: ESP32 dev module + auto-direction TTL-to-RS485 module
  Use with esp32_rs485_B on Board B.

  Default test (Test 1): 1000 chunks of 342 bytes at 19200 baud, the RockBLOCK
  9603's largest burst (340-byte SBD message + 2-byte checksum) at its default speed.

  For every chunk (numbered 1..N):
    1. A sends an 'S' frame: chunk number, timestamp, RockBLOCK-style data type
    2. B checks it and replies with an 'R' frame carrying the exact same bytes
    3. A checks the 'R' frame byte by byte, plus CRC, chunk number, timestamp and type
    4. PASS only if everything is correct. No retries: failures are counted.

  Log lines:
    [S]  frame Board A sent to Board B
    [R]  echo (reply) frame received from Board B
    [!]  no echo received from Board B
    RUN_START / ROW / RUN_END   machine-readable lines for the Excel report

  Commands (USB serial, 115200, line ending "New Line"):
    run            send 1000 chunks
    run <n>        send n chunks (1-60000)
    size <n>       chunk size in bytes (16-8192, default 342)
    gap <ms>       pause between chunks (0-5000, default 20)
    baud <rate>    RS485 baud rate (set the SAME rate on Board B)
    label <text>   name for this run, e.g. label Test1 19200 100m
    help           show commands and current settings
  Press any key during a run to stop early (the report is still printed).
*/

#include <Arduino.h>
#include <strings.h>

// Defined before any function: the Arduino IDE adds function prototypes at the
// top of the sketch, and runChunk()'s prototype needs this type.
struct ChunkResult {
  uint8_t  reason;
  uint32_t tSendMs, tEchoMs, rttUs;
  uint16_t abBad, baBad, echoed;
  bool     gap, replied;
  uint8_t  abStatus;      // B's check of A->B (valid if replied)
  const char *baText;     // A's check of B->A
};
// ===================== shared by Board A and Board B =====================
// RS485 module (auto-direction) on UART2:
//   ESP32 GPIO17 (TX2) -> module TXD / DI   (data the module sends on the bus)
//   ESP32 GPIO16 (RX2) <- module RXD / RO   (data the module received)
//   Logs and commands go over the ESP32's USB port (Serial, 115200).

HardwareSerial &RS485 = Serial2;
uint32_t       rs485Baud       = 19200;    // RockBLOCK default; change with "baud <rate>" on BOTH boards
const int      RS485_RX_PIN    = 16;
const int      RS485_TX_PIN    = 17;
const uint32_t LOG_BAUD        = 115200;   // USB serial speed for logs and commands
const int      LED_PIN         = 2;        // on-board LED of most ESP32 dev modules
const uint16_t MAX_LEN         = 8192;     // largest payload either board accepts
const uint16_t ECHO_DELAY_MS   = 5;        // Board B waits this long before echoing

// Frame types, readable in a hex dump and in the logs
const uint8_t F_SEND  = 'S';               // A -> B : chunk sent by Board A
const uint8_t F_REPLY = 'R';               // B -> A : echo (reply) from Board B

// RockBLOCK-style data types (rotate chunk by chunk)
enum : uint8_t { DT_ATCMD = 0, DT_ATRSP, DT_RING, DT_MO, DT_MT, NUM_DT };

// Board B's result for the A->B direction, carried in the 'R' frame
enum : uint8_t { ST_OK = 0, ST_BAD = 1, ST_INCOMPLETE = 2, ST_CRC = 3, ST_TOO_LARGE = 4 };
const uint8_t ST_SEQ_FLAG = 0x80;          // B saw a gap in chunk numbers

// Frame layout:
//   AA 55 | type('S'/'R') chunk(2) timestamp(4) dtype status info(2) len(2) | payload | CRC16(2)
//   13 header bytes after AA 55, little-endian. CRC-16/XMODEM over header + payload,
//   sent high byte first. Frame size = payload + 17 bytes.

// Silence inside a frame longer than this = data lost (at least 50 ms or 20 byte times)
uint32_t byteTimeoutMs() {
  uint32_t t = (20UL * 10UL * 1000UL) / rs485Baud;
  return t < 50 ? 50 : t;
}

void openRs485(uint32_t baud) {
  RS485.end();
  rs485Baud = baud;
  RS485.setRxBufferSize(16384);            // must be set before begin()
  RS485.begin(rs485Baud, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
}

bool validBaud(long b) {
  const long ok[] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
  for (long v : ok) if (v == b) return true;
  return false;
}

uint16_t crcUpdate(uint16_t crc, uint8_t b) {   // CRC-16/XMODEM, poly 0x1021, init 0
  crc ^= (uint16_t)b << 8;
  for (uint8_t i = 0; i < 8; i++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  return crc;
}

struct Prng {                                  // xorshift32
  uint32_t s;
  void begin(uint16_t seed) { s = 0x9E3779B9UL ^ seed; next(); next(); }
  uint8_t next() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return (uint8_t)s; }
};

// Builds the payload of a chunk one byte at a time, the same way on both boards.
//   DT_ATCMD : "AT+SBDWT=CHUNK<n>,T=<n*7>\r"                      then random bytes
//   DT_ATRSP : "+SBDIX: 0, <n>, 1, <n&255>, 270, 0\r\n\r\nOK\r\n"  then random bytes
//   DT_RING  : "SBDRING\r\n"                                      then random bytes
//   DT_MO    : 340 data bytes + 2-byte checksum (like AT+SBDWB=340), then random
//   DT_MT    : 2-byte length 270 + 270 data + 2-byte checksum (like AT+SBDRB), then random
struct ChunkGen {
  uint8_t  type, segLen, seg[48];
  uint16_t idx, csum;
  Prng     prng;

  void begin(uint16_t chunk, uint8_t t) {
    type = t; idx = 0; csum = 0;
    uint16_t seed = (uint16_t)(chunk * 40503u + t * 9973u + 1u);
    prng.begin(seed);
    int n = 0;
    if (t == DT_ATCMD)
      n = snprintf((char *)seg, sizeof(seg), "AT+SBDWT=CHUNK%u,T=%u\r", chunk, (uint16_t)(chunk * 7u));
    else if (t == DT_ATRSP)
      n = snprintf((char *)seg, sizeof(seg), "+SBDIX: 0, %u, 1, %u, 270, 0\r\n\r\nOK\r\n", chunk, chunk & 0xFF);
    else if (t == DT_RING)
      n = snprintf((char *)seg, sizeof(seg), "SBDRING\r\n");
    segLen = (n > 0) ? (uint8_t)n : 0;
  }

  uint8_t next() {
    uint16_t i = idx++;
    switch (type) {
      case DT_MO:                               // checksum = sum of the data bytes
        if (i < 340)  { uint8_t v = prng.next(); csum += v; return v; }
        if (i == 340) return (uint8_t)(csum >> 8);
        if (i == 341) return (uint8_t)csum;
        return prng.next();
      case DT_MT:
        if (i == 0)   return 0x01;              // 270 = 0x010E
        if (i == 1)   return 0x0E;
        if (i < 272)  { uint8_t v = prng.next(); csum += v; return v; }
        if (i == 272) return (uint8_t)(csum >> 8);
        if (i == 273) return (uint8_t)csum;
        return prng.next();
      default:
        if (i < segLen) return seg[i];
        return prng.next();
    }
  }
};

// Puts a complete frame into out[] and returns its length
size_t buildFrame(uint8_t *out, uint8_t ftype, uint16_t chunk, uint32_t ts, uint8_t dtype,
                  uint8_t status, uint16_t info, uint16_t len, const uint8_t *payload) {
  size_t n = 0;
  out[n++] = 0xAA;
  out[n++] = 0x55;
  out[n++] = ftype;
  out[n++] = (uint8_t)chunk;
  out[n++] = (uint8_t)(chunk >> 8);
  for (int i = 0; i < 4; i++) out[n++] = (uint8_t)(ts >> (8 * i));
  out[n++] = dtype;
  out[n++] = status;
  out[n++] = (uint8_t)info;
  out[n++] = (uint8_t)(info >> 8);
  out[n++] = (uint8_t)len;
  out[n++] = (uint8_t)(len >> 8);
  if (len) memcpy(out + n, payload, len);
  n += len;
  uint16_t crc = 0;
  for (size_t i = 2; i < n; i++) crc = crcUpdate(crc, out[i]);
  out[n++] = (uint8_t)(crc >> 8);
  out[n++] = (uint8_t)crc;
  return n;
}

// Receives a frame byte by byte, stores the payload and checks it against ChunkGen
struct RxFrame {
  enum { S_SOF1, S_SOF2, S_HDR, S_PAY, S_CRC } st = S_SOF1;
  uint8_t  hdr[13], hn = 0, cb[2], cn = 0;
  uint8_t  ftype = 0, dtype = 0, status = 0;
  uint16_t chunk = 0, info = 0, len = 0, idx = 0, bad = 0, crc = 0;
  uint32_t ts = 0;
  bool     crcOk = false, hdrOk = false;
  uint8_t *store = nullptr;               // payload buffer, MAX_LEN bytes
  ChunkGen gen;

  void reset()      { st = S_SOF1; hdrOk = false; }
  bool busy() const { return st != S_SOF1; }

  bool feed(uint8_t b) {                   // true when a complete frame has arrived
    switch (st) {
      case S_SOF1:
        if (b == 0xAA) st = S_SOF2;
        break;
      case S_SOF2:
        if (b == 0x55) { st = S_HDR; hn = 0; crc = 0; hdrOk = false; }
        else if (b != 0xAA) st = S_SOF1;
        break;
      case S_HDR:
        hdr[hn++] = b;
        crc = crcUpdate(crc, b);
        if (hn == 13) {
          ftype  = hdr[0];
          chunk  = hdr[1] | ((uint16_t)hdr[2] << 8);
          ts     = hdr[3] | ((uint32_t)hdr[4] << 8) | ((uint32_t)hdr[5] << 16) | ((uint32_t)hdr[6] << 24);
          dtype  = hdr[7];
          status = hdr[8];
          info   = hdr[9]  | ((uint16_t)hdr[10] << 8);
          len    = hdr[11] | ((uint16_t)hdr[12] << 8);
          if ((ftype != F_SEND && ftype != F_REPLY) || dtype >= NUM_DT || len > MAX_LEN) {
            st = S_SOF1;
            break;
          }
          idx = 0; bad = 0; hdrOk = true;
          gen.begin(chunk, dtype);
          cn = 0;
          st = len ? S_PAY : S_CRC;
        }
        break;
      case S_PAY:
        crc = crcUpdate(crc, b);
        if (b != gen.next()) bad++;
        if (store) store[idx] = b;
        if (++idx == len) { st = S_CRC; cn = 0; }
        break;
      case S_CRC:
        cb[cn++] = b;
        if (cn == 2) {
          crcOk = ((((uint16_t)cb[0]) << 8) | cb[1]) == crc;
          st = S_SOF1;
          return true;
        }
        break;
    }
    return false;
  }
};

// Padded name for readable log lines, and a plain code for CSV lines
const char *typeName(uint8_t t) {
  switch (t) {
    case DT_ATCMD: return "AT_CMD";
    case DT_ATRSP: return "AT_RSP";
    case DT_RING:  return "RING  ";
    case DT_MO:    return "SBD_MO";
    case DT_MT:    return "SBD_MT";
    default:       return "?     ";
  }
}
const char *typeCode(uint8_t t) {
  switch (t) {
    case DT_ATCMD: return "AT_CMD";
    case DT_ATRSP: return "AT_RSP";
    case DT_RING:  return "RING";
    case DT_MO:    return "SBD_MO";
    case DT_MT:    return "SBD_MT";
    default:       return "UNKNOWN";
  }
}
const char *statusCode(uint8_t s) {
  switch (s & 0x7F) {
    case ST_OK:         return "OK";
    case ST_BAD:        return "BAD";
    case ST_INCOMPLETE: return "INCOMPLETE";
    case ST_CRC:        return "CRC";
    case ST_TOO_LARGE:  return "TOO_LARGE";
    default:            return "UNKNOWN";
  }
}

// Prints the start of a payload: text for AT/RING types, hex for SBD types
void printPreview(const uint8_t *p, uint16_t len, uint8_t dtype) {
  if (dtype == DT_MO || dtype == DT_MT) {
    Serial.print(" data=[");
    uint16_t n = len < 10 ? len : 10;
    for (uint16_t i = 0; i < n; i++) Serial.printf(i ? " %02X" : "%02X", p[i]);
    Serial.print(len > 10 ? " ...]" : "]");
  } else {
    Serial.print(" data=\"");
    uint16_t n = len < 24 ? len : 24;
    for (uint16_t i = 0; i < n; i++) {
      uint8_t c = p[i];
      if (c == '\r')               Serial.print("\\r");
      else if (c == '\n')          Serial.print("\\n");
      else if (c >= 32 && c < 127) Serial.print((char)c);
      else                         Serial.print('.');
    }
    Serial.print(len > 24 ? "...\"" : "\"");
  }
}
// =================== end of shared part ===================

const uint16_t REPLY_TIMEOUT_MS = 500;    // no 'R' frame started within this time = no reply

// ------------------------------------------------------------ settings
uint16_t chunkSize = 342;
uint16_t gapMs     = 20;
char     label[40] = "Test1";
char     cmd[64];
uint8_t  cmdLen = 0;

uint8_t payBuf[MAX_LEN];
uint8_t txBuf[MAX_LEN + 20];
uint8_t rxBuf[MAX_LEN];
RxFrame rx;

// ------------------------------------------------------------ results
enum : uint8_t { R_PASS = 0, R_AB_CORRUPT, R_AB_INCOMPLETE, R_BA_CORRUPT,
                 R_BA_INCOMPLETE, R_NO_REPLY, R_HEADER, NUM_R };

uint32_t cntReason[NUM_R];
uint32_t tSent[NUM_DT], tPass[NUM_DT], rttMin[NUM_DT], rttMax[NUM_DT], rttN[NUM_DT], rttSum[NUM_DT];
uint32_t bytesAB, bytesBA, badAB, badBA, tStart, tEnd;
uint32_t sentTotal, seqGaps, replyFrames, ownEchoFrames;
uint16_t failList[20];
uint8_t  failN;

const char *reasonText(uint8_t r) {
  switch (r) {
    case R_PASS:          return "PASS";
    case R_AB_CORRUPT:    return "FAIL A->B corrupted";
    case R_AB_INCOMPLETE: return "FAIL A->B incomplete";
    case R_BA_CORRUPT:    return "FAIL B->A corrupted";
    case R_BA_INCOMPLETE: return "FAIL B->A incomplete";
    case R_NO_REPLY:      return "FAIL no reply";
    case R_HEADER:        return "FAIL wrong chunk/header in echo";
    default:              return "?";
  }
}
const char *reasonCode(uint8_t r) {
  switch (r) {
    case R_PASS:          return "OK";
    case R_AB_CORRUPT:    return "AB_CORRUPT";
    case R_AB_INCOMPLETE: return "AB_INCOMPLETE";
    case R_BA_CORRUPT:    return "BA_CORRUPT";
    case R_BA_INCOMPLETE: return "BA_INCOMPLETE";
    case R_NO_REPLY:      return "NO_REPLY";
    case R_HEADER:        return "HEADER";
    default:              return "UNKNOWN";
  }
}

void clearStats() {
  memset(cntReason, 0, sizeof(cntReason));
  memset(tSent, 0, sizeof(tSent));
  memset(tPass, 0, sizeof(tPass));
  memset(rttMax, 0, sizeof(rttMax));
  memset(rttN, 0, sizeof(rttN));
  memset(rttSum, 0, sizeof(rttSum));
  for (int i = 0; i < NUM_DT; i++) rttMin[i] = 0xFFFFFFFF;
  bytesAB = bytesBA = badAB = badBA = 0;
  sentTotal = seqGaps = replyFrames = ownEchoFrames = 0;
  failN = 0;
}

// Waits for Board B's 'R' frame.
// 0 = 'R' frame received, 1 = nothing from B, 2 = 'R' frame started but incomplete.
// 'S' frames seen here are A's own transmission heard back through the
// auto-direction module (local echo); they are counted and ignored.
uint8_t waitReply() {
  rx.reset();
  uint32_t last = millis();
  uint32_t tmo  = byteTimeoutMs();
  while (true) {
    while (RS485.available()) {
      uint8_t b = RS485.read();
      last = millis();
      if (rx.feed(b)) {
        if (rx.ftype == F_SEND)  { ownEchoFrames++; continue; }
        if (rx.ftype == F_REPLY) { replyFrames++;  return 0; }
      }
    }
    uint32_t idle = millis() - last;
    if (rx.busy() && idle > tmo) {
      if (rx.hdrOk && rx.ftype == F_REPLY) return 2;
      rx.reset();
    }
    if (!rx.busy() && idle > REPLY_TIMEOUT_MS) return 1;
    yield();
  }
}

// Sends one chunk, waits for the echo, fills in the result
void runChunk(uint16_t c, uint8_t dt, ChunkResult &res) {
  res = ChunkResult();
  res.baText = "NONE";

  ChunkGen g;
  g.begin(c, dt);
  for (uint16_t i = 0; i < chunkSize; i++) payBuf[i] = g.next();

  Serial.printf("[S] #%05u SENT  A->B  %s %5u B ", c, typeName(dt), chunkSize);
  printPreview(payBuf, chunkSize, dt);
  Serial.println();

  res.tSendMs = millis();
  uint32_t t0us = micros();
  size_t   n    = buildFrame(txBuf, F_SEND, c, res.tSendMs, dt, 0, 0, chunkSize, payBuf);
  digitalWrite(LED_PIN, HIGH);
  RS485.write(txBuf, n);
  RS485.flush();                           // wait until the last byte has left the UART
  digitalWrite(LED_PIN, LOW);

  uint8_t w = waitReply();
  res.rttUs = micros() - t0us;
  res.tEchoMs = millis();

  if (w == 1 || !rx.hdrOk) { res.reason = R_NO_REPLY; res.rttUs = 0; res.tEchoMs = 0; return; }

  res.replied  = true;
  res.echoed   = (rx.idx < rx.len) ? rx.idx : rx.len;
  res.gap      = rx.status & ST_SEQ_FLAG;
  res.abStatus = rx.status & 0x7F;
  res.abBad    = (res.abStatus == ST_BAD) ? rx.info : 0;
  if (res.gap) seqGaps++;

  if (w == 2)              res.baText = "INCOMPLETE";
  else if (rx.chunk != c || rx.ts != res.tSendMs || rx.dtype != dt || rx.len != chunkSize)
                           res.baText = "HEADER";
  else if (rx.bad)         res.baText = "BAD";
  else if (!rx.crcOk)      res.baText = "CRC";
  else                     res.baText = "OK";
  if (w == 0 && rx.len == chunkSize) res.baBad = rx.bad;

  if (res.abStatus == ST_INCOMPLETE)                       res.reason = R_AB_INCOMPLETE;
  else if (res.abStatus == ST_BAD || res.abStatus == ST_CRC) res.reason = R_AB_CORRUPT;
  else if (res.abStatus == ST_TOO_LARGE)                   res.reason = R_HEADER;
  else if (w == 2)                                         res.reason = R_BA_INCOMPLETE;
  else if (strcmp(res.baText, "HEADER") == 0)              res.reason = R_HEADER;
  else if (rx.bad || !rx.crcOk)                            res.reason = R_BA_CORRUPT;
  else                                                     res.reason = R_PASS;
}

// ------------------------------------------------------------ report
void printReport(bool aborted) {
  uint32_t pass = cntReason[R_PASS];
  uint32_t fail = sentTotal - pass;
  float    secs = (tEnd - tStart) / 1000.0f;
  uint32_t rttAllSum = 0, rttAllN = 0;
  for (int t = 0; t < NUM_DT; t++) { rttAllSum += rttSum[t]; rttAllN += rttN[t]; }
  float    rttAvg = rttAllN ? rttAllSum / 1000.0f / rttAllN : 0;
  float    theo   = 2.0f * (chunkSize + 17) * 10.0f * 1000.0f / rs485Baud + ECHO_DELAY_MS;

  Serial.println();
  Serial.println("============= ESP32 RS485 CHUNK ECHO TEST REPORT =============");
  Serial.printf("Label            : %s\n", label);
  Serial.println("Hardware         : 2x ESP32 dev module + auto-direction RS485 modules");
  Serial.printf("Chunk size       : %u bytes payload, %u bytes per frame\n", chunkSize, chunkSize + 17);
  Serial.println("Data types       : AT_CMD, AT_RSP, RING, SBD_MO, SBD_MT (rotating)");
  Serial.printf("Baud             : %lu 8N1\n", (unsigned long)rs485Baud);
  Serial.printf("Gap between      : %u ms\n", gapMs);
  Serial.printf("Duration         : %.1f s\n", secs);
  if (aborted) Serial.println("NOTE: run stopped early by user");
  Serial.println("--------------------------------------------------------------");
  Serial.printf("Chunks sent ([S] frames)       : %lu\n", (unsigned long)sentTotal);
  Serial.printf("Echoes received ([R] frames)   : %lu\n", (unsigned long)replyFrames);
  Serial.printf("PASS                           : %lu\n", (unsigned long)pass);
  Serial.printf("FAIL                           : %lu\n", (unsigned long)fail);
  Serial.printf("SUCCESS RATE                   : %.2f %%\n", sentTotal ? pass * 100.0f / sentTotal : 0.0f);
  Serial.println("Failures by reason:");
  Serial.printf("  A->B corrupted       : %lu\n", (unsigned long)cntReason[R_AB_CORRUPT]);
  Serial.printf("  A->B incomplete      : %lu\n", (unsigned long)cntReason[R_AB_INCOMPLETE]);
  Serial.printf("  B->A corrupted       : %lu\n", (unsigned long)cntReason[R_BA_CORRUPT]);
  Serial.printf("  B->A incomplete      : %lu\n", (unsigned long)cntReason[R_BA_INCOMPLETE]);
  Serial.printf("  No reply (lost)      : %lu\n", (unsigned long)cntReason[R_NO_REPLY]);
  Serial.printf("  Wrong chunk/header   : %lu\n", (unsigned long)cntReason[R_HEADER]);
  Serial.printf("Sequence gaps seen by B        : %lu\n", (unsigned long)seqGaps);
  Serial.printf("Own [S] frames heard on A's RX : %lu  (module local echo, ignored)\n",
                (unsigned long)ownEchoFrames);
  Serial.println("--------------------------------------------------------------");
  Serial.println("  Type     Sent   Pass   Fail   RTT min    avg    max (ms)");
  for (int t = 0; t < NUM_DT; t++) {
    if (tSent[t] == 0) continue;
    Serial.printf("  %s %6lu %6lu %6lu   %7.1f %6.1f %6.1f\n", typeName(t),
                  (unsigned long)tSent[t], (unsigned long)tPass[t], (unsigned long)(tSent[t] - tPass[t]),
                  rttN[t] ? rttMin[t] / 1000.0f : 0.0f,
                  rttN[t] ? rttSum[t] / 1000.0f / rttN[t] : 0.0f,
                  rttMax[t] / 1000.0f);
  }
  Serial.println("--------------------------------------------------------------");
  Serial.printf("Bytes A->B : %lu   bad bytes: %lu\n", (unsigned long)bytesAB, (unsigned long)badAB);
  Serial.printf("Bytes B->A : %lu   bad bytes: %lu\n", (unsigned long)bytesBA, (unsigned long)badBA);
  Serial.printf("Average round trip     : %.1f ms (theory %.1f ms)\n", rttAvg, theo);
  Serial.printf("Throughput (both directions, incl. gaps) : %lu bytes/s\n",
                (unsigned long)(secs > 0 ? (bytesAB + bytesBA) / secs : 0));
  Serial.print("Failed chunks (first 20) : ");
  if (failN == 0) Serial.println("none");
  else { for (int i = 0; i < failN; i++) Serial.printf("%u ", failList[i]); Serial.println(); }
  Serial.printf("RESULT : %s\n", (fail == 0 && sentTotal > 0)
                ? "PASS - all chunks correct in both directions" : "FAIL - see failures above");
  Serial.println("==============================================================");
}

// ------------------------------------------------------------ test run
void runTest(uint16_t n) {
  bool aborted = false;
  clearStats();
  delay(20);
  while (Serial.available()) Serial.read();   // drop the rest of the command line

  Serial.printf("Sending %u chunks of %u bytes at %lu baud. Press any key to stop.\n",
                n, chunkSize, (unsigned long)rs485Baud);
  Serial.println("Legend: [S] sent by A   [R] echo received from B   [!] no echo from B");
  // RUN_START,label,baud,size,count,gap_ms,echo_delay_ms,board_ms
  Serial.printf("RUN_START,%s,%lu,%u,%u,%u,%u,%lu\n", label, (unsigned long)rs485Baud,
                chunkSize, n, gapMs, ECHO_DELAY_MS, (unsigned long)millis());
  tStart = millis();

  for (uint16_t c = 1; c <= n; c++) {
    if (Serial.available()) { aborted = true; break; }
    uint8_t     dt = (c - 1) % NUM_DT;
    ChunkResult res;
    runChunk(c, dt, res);
    uint8_t r = res.reason;

    sentTotal++;
    tSent[dt]++;
    cntReason[r]++;
    bytesAB += chunkSize;
    badAB   += res.abBad;
    if (res.replied) {
      bytesBA += res.echoed;
      badBA   += res.baBad;
      rttN[dt]++;
      rttSum[dt] += res.rttUs;
      if (res.rttUs < rttMin[dt]) rttMin[dt] = res.rttUs;
      if (res.rttUs > rttMax[dt]) rttMax[dt] = res.rttUs;
    }
    if (r == R_PASS) tPass[dt]++;
    else if (failN < 20) failList[failN++] = c;

    // readable echo line
    if (!res.replied) {
      Serial.printf("[!] #%05u NO ECHO from B within %u ms  %s\n", c, REPLY_TIMEOUT_MS, reasonText(r));
    } else {
      Serial.printf("[R] #%05u ECHO  B->A  %s %5u B  rtt %6.1f ms  %s", rx.chunk,
                    typeName(rx.dtype), res.echoed, res.rttUs / 1000.0f, reasonText(r));
      if (res.abBad) Serial.printf(" (A->B bad %u)", res.abBad);
      if (res.baBad) Serial.printf(" (B->A bad %u)", res.baBad);
      Serial.print(" ");
      printPreview(rxBuf, res.echoed, rx.dtype);
      Serial.println();
    }

    // ROW,chunk,type,bytes_sent,bytes_echoed,t_send_ms,t_echo_ms,rtt_us,ab_check,ba_check,
    //     ab_bad,ba_bad,seq_gap,result,reason
    Serial.printf("ROW,%u,%s,%u,%u,%lu,%lu,%lu,%s,%s,%u,%u,%u,%s,%s\n",
                  c, typeCode(dt), chunkSize, res.echoed,
                  (unsigned long)res.tSendMs, (unsigned long)res.tEchoMs, (unsigned long)res.rttUs,
                  res.replied ? statusCode(res.abStatus) : "NONE", res.baText,
                  res.abBad, res.baBad, res.gap ? 1 : 0,
                  r == R_PASS ? "PASS" : "FAIL", reasonCode(r));

    delay(gapMs);
  }
  tEnd = millis();
  if (aborted) while (Serial.available()) Serial.read();
  // RUN_END,sent,pass,fail,own_echo_frames,aborted,board_ms
  Serial.printf("RUN_END,%lu,%lu,%lu,%lu,%u,%lu\n", (unsigned long)sentTotal,
                (unsigned long)cntReason[R_PASS], (unsigned long)(sentTotal - cntReason[R_PASS]),
                (unsigned long)ownEchoFrames, aborted ? 1 : 0, (unsigned long)millis());
  printReport(aborted);
}

// ------------------------------------------------------------ commands
void printHelp() {
  Serial.println("Commands:");
  Serial.println("  run            send 1000 chunks");
  Serial.println("  run <n>        send n chunks (1-60000)");
  Serial.println("  size <n>       chunk size in bytes (16-8192)");
  Serial.println("  gap <ms>       pause between chunks (0-5000)");
  Serial.println("  baud <rate>    RS485 baud rate (set the SAME rate on Board B)");
  Serial.println("  label <text>   name for this run, e.g. label Test1 19200 100m");
  Serial.println("  help           show this list");
  Serial.printf("Current: baud %lu, size %u B, gap %u ms, label %s\n",
                (unsigned long)rs485Baud, chunkSize, gapMs, label);
}

void handleCommand(char *c) {
  Serial.printf("> %s\n", c);
  if (strcasecmp(c, "run") == 0) {
    runTest(1000);
  } else if (strncasecmp(c, "run ", 4) == 0) {
    long n = atol(c + 4);
    if (n < 1 || n > 60000) Serial.println("Count must be 1-60000");
    else runTest((uint16_t)n);
  } else if (strncasecmp(c, "size ", 5) == 0) {
    long n = atol(c + 5);
    if (n < 16 || n > MAX_LEN) Serial.println("Size must be 16-8192");
    else { chunkSize = (uint16_t)n; Serial.printf("Chunk size: %u\n", chunkSize); }
  } else if (strncasecmp(c, "gap ", 4) == 0) {
    long n = atol(c + 4);
    if (n < 0 || n > 5000) Serial.println("Gap must be 0-5000");
    else { gapMs = (uint16_t)n; Serial.printf("Gap: %u ms\n", gapMs); }
  } else if (strncasecmp(c, "baud ", 5) == 0) {
    long b = atol(c + 5);
    if (!validBaud(b)) Serial.println("Baud must be one of 1200 2400 4800 9600 19200 38400 57600 115200 230400 460800 921600");
    else { openRs485((uint32_t)b); Serial.printf("RS485 baud: %lu  (set the same on Board B)\n", (unsigned long)rs485Baud); }
  } else if (strncasecmp(c, "label ", 6) == 0) {
    strncpy(label, c + 6, sizeof(label) - 1);
    label[sizeof(label) - 1] = '\0';
    for (char *p = label; *p; p++) if (*p == ',') *p = ' ';   // keep CSV lines valid
    Serial.printf("Label: %s\n", label);
  } else if (strcasecmp(c, "help") == 0) {
    printHelp();
  } else {
    Serial.println("Unknown command, type help");
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(LOG_BAUD);
  openRs485(rs485Baud);
  rx.store = rxBuf;
  delay(500);
  Serial.println();
  Serial.println("ESP32 RS485 chunk echo test - Board A (controller)");
  printHelp();
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r' || ch == '\n') {
      if (cmdLen > 0) {
        cmd[cmdLen] = '\0';
        handleCommand(cmd);
        cmdLen = 0;
      }
    } else if (cmdLen < sizeof(cmd) - 1) {
      cmd[cmdLen++] = ch;
    }
  }
}

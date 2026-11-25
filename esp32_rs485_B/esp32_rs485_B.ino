/*
  ESP32 RS485 CHUNK ECHO TEST - Board B (responder)
  Hardware: ESP32 dev module + auto-direction TTL-to-RS485 module
  Use with esp32_rs485_A on Board A.

  For every 'S' frame from Board A:
    - checks every byte against the expected RockBLOCK-style payload + CRC
    - checks that chunk numbers arrive in order (chunk 1 restarts the count)
    - replies with an 'R' frame carrying the exact same bytes, plus its result

  Log lines:
    [S]   frame received from Board A
    [R]   echo (reply) frame sent back to Board A
    BROW  machine-readable line for the Excel report
  'R' frames heard on B's own RX are B's transmission coming back through the
  auto-direction module (local echo); they are counted and ignored.
  On-board LED: ON while a chunk is arriving. Stays ON after any error (press EN to clear).

  Commands (USB serial, 115200, line ending "New Line"):
    baud <rate>   RS485 baud rate (must match Board A)
    status        show baud rate and totals
*/

#include <Arduino.h>
#include <strings.h>
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

uint8_t  rxBuf[MAX_LEN];
uint8_t  txBuf[MAX_LEN + 20];
RxFrame  rx;
uint32_t lastByteMs = 0;
uint32_t okTotal = 0, failTotal = 0, gapTotal = 0, ownEchoFrames = 0;
uint16_t expectedChunk = 1;
bool     failSeen = false;
char     cmd[48];
uint8_t  cmdLen = 0;

void handleSend(bool complete) {
  uint32_t tRecv = millis();
  uint8_t  st;
  uint16_t info = 0;
  if (!complete)              { st = ST_INCOMPLETE; info = rx.idx; }
  else if (rx.bad)            { st = ST_BAD;        info = rx.bad; }
  else if (!rx.crcOk)         { st = ST_CRC; }
  else                        { st = ST_OK; }

  if (rx.chunk == 1) {        // new run: reset counters
    expectedChunk = 1;
    okTotal = failTotal = gapTotal = ownEchoFrames = 0;
    failSeen = false;
    Serial.printf("BRUN_START,%lu,%lu\n", (unsigned long)rs485Baud, (unsigned long)tRecv);
  }
  bool gap = (rx.chunk != expectedChunk);
  if (gap) gapTotal++;
  expectedChunk = rx.chunk + 1;

  // Reply first, log afterwards, so logging does not add to the round-trip time
  uint16_t replyLen = complete ? rx.len : 0;
  delay(ECHO_DELAY_MS);
  size_t n = buildFrame(txBuf, F_REPLY, rx.chunk, rx.ts, rx.dtype,
                        st | (gap ? ST_SEQ_FLAG : 0), info, replyLen, rxBuf);
  RS485.write(txBuf, n);
  RS485.flush();

  if (st == ST_OK) okTotal++;
  else { failTotal++; failSeen = true; }

  // [S] line: what B received from A
  Serial.printf("[S] #%05u RECEIVED from A  %s %5u B  check ", rx.chunk, typeName(rx.dtype),
                complete ? rx.len : rx.idx);
  switch (st) {
    case ST_OK:         Serial.print("OK"); break;
    case ST_BAD:        Serial.printf("BAD BYTES %u", info); break;
    case ST_INCOMPLETE: Serial.printf("INCOMPLETE, received %u", info); break;
    case ST_CRC:        Serial.print("CRC ERROR"); break;
  }
  if (gap) Serial.print("  SEQUENCE GAP");
  Serial.print(" ");
  printPreview(rxBuf, complete ? rx.len : rx.idx, rx.dtype);
  Serial.println();

  // [R] line: the echo B sent back
  Serial.printf("[R] #%05u ECHO SENT to A   %s %5u B  totals %lu OK / %lu not OK / %lu gaps\n",
                rx.chunk, typeName(rx.dtype), replyLen,
                (unsigned long)okTotal, (unsigned long)failTotal, (unsigned long)gapTotal);

  // BROW,chunk,type,bytes_received,check,bad_bytes,seq_gap,bytes_echoed,t_recv_ms
  Serial.printf("BROW,%u,%s,%u,%s,%u,%u,%u,%lu\n", rx.chunk, typeCode(rx.dtype),
                complete ? rx.len : rx.idx, statusCode(st), st == ST_BAD ? info : 0,
                gap ? 1 : 0, replyLen, (unsigned long)tRecv);

  if (rx.chunk % 100 == 0)
    Serial.printf("    own [R] frames heard back on B's RX (module local echo): %lu\n",
                  (unsigned long)ownEchoFrames);

  digitalWrite(LED_PIN, failSeen ? HIGH : LOW);
}

void handleCommand(char *c) {
  Serial.printf("> %s\n", c);
  if (strncasecmp(c, "baud ", 5) == 0) {
    long b = atol(c + 5);
    if (!validBaud(b)) Serial.println("Baud must be one of 1200 2400 4800 9600 19200 38400 57600 115200 230400 460800 921600");
    else { openRs485((uint32_t)b); rx.reset(); Serial.printf("RS485 baud: %lu\n", (unsigned long)rs485Baud); }
  } else if (strcasecmp(c, "status") == 0 || strcasecmp(c, "help") == 0) {
    Serial.printf("RS485 baud %lu | totals %lu OK / %lu not OK / %lu gaps | own echo frames %lu\n",
                  (unsigned long)rs485Baud, (unsigned long)okTotal, (unsigned long)failTotal,
                  (unsigned long)gapTotal, (unsigned long)ownEchoFrames);
    Serial.println("Commands: baud <rate>, status");
  } else {
    Serial.println("Unknown command. Commands: baud <rate>, status");
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.begin(LOG_BAUD);
  openRs485(rs485Baud);
  rx.store = rxBuf;
  delay(500);
  Serial.println();
  Serial.println("ESP32 RS485 chunk echo test - Board B (responder)");
  Serial.printf("RS485 baud: %lu. Commands: baud <rate>, status\n", (unsigned long)rs485Baud);
  Serial.println("Legend: [S] received from A   [R] echo sent back to A");
  Serial.println("Waiting for chunks...");
}

void loop() {
  while (RS485.available()) {
    uint8_t b = RS485.read();
    lastByteMs = millis();
    bool wasIdle = !rx.busy();
    if (rx.feed(b)) {
      if (rx.ftype == F_SEND)       handleSend(true);
      else if (rx.ftype == F_REPLY) ownEchoFrames++;   // our own echo heard back
    } else if (wasIdle && rx.busy()) {
      digitalWrite(LED_PIN, HIGH);                     // a frame is arriving
    }
  }

  if (rx.busy() && millis() - lastByteMs > byteTimeoutMs()) {
    if (rx.hdrOk && rx.ftype == F_SEND) handleSend(false);   // header arrived, data lost
    rx.reset();
    if (!failSeen) digitalWrite(LED_PIN, LOW);
  }

  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r' || ch == '\n') {
      if (cmdLen > 0) { cmd[cmdLen] = '\0'; handleCommand(cmd); cmdLen = 0; }
    } else if (cmdLen < sizeof(cmd) - 1) {
      cmd[cmdLen++] = ch;
    }
  }
}

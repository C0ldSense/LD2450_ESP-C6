#include <Arduino.h>

HardwareSerial LD(0);

// ESP32-C6: 
static const int RX_PIN = 17;      // U0RXD <- Sensor TX
static const int TX_PIN = 16;      // U0TXD <- Sensor RX
static const uint32_t BAUD = 256000;

// LD2450 Protokoll (Manual)
static const uint8_t HDR[4] = {0xAA, 0xFF, 0x03, 0x00};
static const uint8_t FTR[2] = {0x55, 0xCC};
static const size_t FRAME_LEN = 30;  // 4 + 3*8 + 2

uint8_t frame[FRAME_LEN];

// Parser-Zustand
size_t hdrMatch = 0;
size_t idx = 0;
bool collecting = false;

// letzter gültiger Frame (wird bei jedem neuen gültigen Frame überschrieben)
uint8_t lastFrame[FRAME_LEN];
bool hasLastFrame = false;

// zuletzt gedruckter Frame (Spam-Schutz gegen identische Frames)
uint8_t lastPrinted[FRAME_LEN];
bool hasLastPrinted = false;

// ---- Hilfsfunktionen ----
static inline uint16_t u16le(const uint8_t* p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

// Manual: int16 mit "Sign-Bit": MSB=1 => positiv, MSB=0 => negativ, Betrag in unteren 15 Bits
static inline int16_t signed15_with_signbit(uint16_t raw) {
  int16_t mag = (int16_t)(raw & 0x7FFF);
  bool positive = (raw & 0x8000) != 0;
  return positive ? mag : -mag;
}

static inline bool framesEqual(const uint8_t* a, const uint8_t* b) {
  for (size_t i = 0; i < FRAME_LEN; i++) {
    if (a[i] != b[i]) return false;
  }
  return true;
}

void printFrameHex(const uint8_t* d) {
  Serial.print("RAW: ");
  for (size_t i = 0; i < FRAME_LEN; i++) {
    if (d[i] < 0x10) Serial.print("0");
    Serial.print(d[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void decodeAndPrint(const uint8_t* d) {
  // Goals starten bei Byte 4, je 8 Byte
  for (int goal = 0; goal < 3; goal++) {
    const uint8_t* g = d + 4 + goal * 8;

    uint16_t xraw = u16le(g + 0);
    uint16_t yraw = u16le(g + 2);
    uint16_t sraw = u16le(g + 4);
    uint16_t res  = u16le(g + 6);

    int16_t x_mm  = signed15_with_signbit(xraw);
    int16_t y_mm  = signed15_with_signbit(yraw);
    int16_t v_cms = signed15_with_signbit(sraw);

    // Existenz-Prüfung (Sonst Fälle wie speed != 0 aber x/y/res = 0)
    bool exists = (res != 0);

    Serial.print("Goal ");
    Serial.print(goal + 1);
    Serial.print(": ");

    if (!exists) {
      Serial.println("(none)");
      continue;
    }

    Serial.print("x=");
    Serial.print(x_mm);
    Serial.print(" mm, y=");
    Serial.print(y_mm);
    Serial.print(" mm, v=");
    Serial.print(v_cms);
    Serial.print(" cm/s, res=");
    Serial.print(res);
    Serial.println(" mm");
  }
}

// kontinuierlich einlesen, Frames extrahieren
// Rückgabe: true, wenn ein NEUER gültiger Frame gespeichert wurde
bool ingest() {
  bool newValidFrame = false;

  while (LD.available()) {
    uint8_t b = (uint8_t)LD.read();

    if (!collecting) {
      // Header suchen
      if (b == HDR[hdrMatch]) {
        hdrMatch++;
        if (hdrMatch == 4) {
          // Header komplett -> Frame starten
          frame[0] = HDR[0];
          frame[1] = HDR[1];
          frame[2] = HDR[2];
          frame[3] = HDR[3];
          idx = 4;
          collecting = true;
          hdrMatch = 0;
        }
      } else {
        hdrMatch = (b == HDR[0]) ? 1 : 0;
      }
      continue;
    }

    // Frame sammeln bis 30 Bytes
    frame[idx++] = b;

    if (idx >= FRAME_LEN) {
      // Footer prüfen
      bool okFooter = (frame[FRAME_LEN - 2] == FTR[0] && frame[FRAME_LEN - 1] == FTR[1]);

      if (okFooter) {
        memcpy(lastFrame, frame, FRAME_LEN);
        hasLastFrame = true;
        newValidFrame = true;  // <- hier: neuer gültiger Frame angekommen
      }

      // Zurücksetzen (egal ob gültig oder nicht)
      collecting = false;
      idx = 0;
      hdrMatch = 0;
    }
  }

  return newValidFrame;
}

void setup() {
  Serial.begin(115200);
  delay(300);

  LD.begin(BAUD, SERIAL_8N1, RX_PIN, TX_PIN);

  Serial.println("\nESP32-C6 + LD2450: Print only on new valid frame");
  Serial.println("Erwartet: AA FF 03 00 ... ... ... 55 CC");
}

void loop() {
  // Nur wenn neuer gültiger Frame
  if (!ingest()) {
    return; // nix Neues
  }

  if (!hasLastFrame) return;

  // Identische Frames nicht nochmal drucken
  if (hasLastPrinted && framesEqual(lastFrame, lastPrinted)) {
    return;
  }

  memcpy(lastPrinted, lastFrame, FRAME_LEN);
  hasLastPrinted = true;

  Serial.println("\n--- Neuer gueltiger Frame ---");
  printFrameHex(lastFrame);
  decodeAndPrint(lastFrame);
}

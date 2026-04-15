#include <SoftwareSerial.h>
#include <Servo.h>

#define RX_PIN     10
#define TX_PIN     11
#define DE_RE_PIN  2
#define SERVO_PIN  9

SoftwareSerial RS485Serial(RX_PIN, TX_PIN);
Servo platformServo;

// MG996R positions
#define FLAT_POS    0    // Platform horizontal (waiting)
#define DROP_POS    90   // Platform dropped 90° down

#define SLAVE_ID 1

// ── CRC ──
uint16_t calculateCRC(uint8_t *buf, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
      else crc >>= 1;
    }
  }
  return crc;
}

// ── Send Response ──
void sendResponse(uint8_t *data, uint8_t len) {
  digitalWrite(DE_RE_PIN, HIGH);
  delayMicroseconds(200);
  for (uint8_t i = 0; i < len; i++) RS485Serial.write(data[i]);
  RS485Serial.flush();
  delayMicroseconds(200);
  digitalWrite(DE_RE_PIN, LOW);
}

// ── Drop Platform ──
void dropPlatform() {
  Serial.println("▼ Platform DROPPING 90°...");
  
  platformServo.attach(SERVO_PIN, 500, 2500);  // Fix 2: attach with pulse limits
  platformServo.write(DROP_POS);               // Drop down
  delay(600);                                  // Wait for bolt to fall
  platformServo.write(FLAT_POS);              // Return to flat
  delay(600);                                  // Wait to reach flat
  platformServo.detach();                      // Fix 4: detach when idle = no jitter
  
  Serial.println("▲ Platform back to FLAT");
}

// ── Process Modbus Frame ──
void processModbus(uint8_t *frame, uint8_t len) {
  if (len < 8) return;
  if (frame[0] != SLAVE_ID) return;

  uint16_t receivedCRC = frame[len-2] | (frame[len-1] << 8);
  uint16_t calcCRC     = calculateCRC(frame, len - 2);
  if (receivedCRC != calcCRC) { Serial.println("✗ CRC Error"); return; }

  uint8_t  fc      = frame[1];
  uint16_t regAddr = (frame[2] << 8) | frame[3];
  uint16_t regVal  = (frame[4] << 8) | frame[5];

  if (fc == 0x06 && regAddr == 0) {
    sendResponse(frame, len);

    if (regVal == 1) {
      Serial.println("✓ ACCEPTED bolt → Dropping platform");
      dropPlatform();
    }
    else if (regVal == 2) {
      Serial.println("✗ REJECTED bolt → Dropping platform");
      dropPlatform();
    }
    else if (regVal == 0) {
      platformServo.attach(SERVO_PIN, 500, 2500);  // Fix 2: attach with limits
      platformServo.write(FLAT_POS);               // Fix 1: immediate write
      delay(500);
      platformServo.detach();                       // Fix 4: detach when idle
      Serial.println("○ IDLE: Platform flat");
    }
  }
}

// ── RX Buffer ──
uint8_t rxBuffer[32];
uint8_t rxIndex = 0;
unsigned long lastByteTime = 0;

void setup() {
  Serial.begin(9600);
  RS485Serial.begin(9600);
  pinMode(DE_RE_PIN, OUTPUT);
  digitalWrite(DE_RE_PIN, LOW);

  // Fix 1 + 2 + 4 combined
  platformServo.attach(SERVO_PIN, 500, 2500);  // Attach with pulse width limits
  platformServo.write(FLAT_POS);               // Write position immediately
  delay(500);                                  // Let servo reach position
  platformServo.detach();                      // Detach to prevent jitter

  Serial.println("Slave Ready. Trapdoor Platform Mode.");
  Serial.println("Waiting for Modbus commands...");
}

void loop() {
  while (RS485Serial.available()) {
    rxBuffer[rxIndex++] = RS485Serial.read();
    lastByteTime = millis();
    if (rxIndex >= 32) rxIndex = 0;
  }

  if (rxIndex > 0 && (millis() - lastByteTime) > 10) {
    processModbus(rxBuffer, rxIndex);
    rxIndex = 0;
    memset(rxBuffer, 0, sizeof(rxBuffer));
  }
}
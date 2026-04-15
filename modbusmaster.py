#include <SoftwareSerial.h>

#define RX_PIN     10
#define TX_PIN     11
#define DE_RE_PIN  2

// Ultrasonic Pins
#define TRIG_PIN   6
#define ECHO_PIN   7

SoftwareSerial RS485Serial(RX_PIN, TX_PIN);

#define SLAVE_ID 1

long duration;
uint16_t distance_cm;

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

// ── Ultrasonic Read ──
uint16_t readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout

  uint16_t dist = duration * 0.034 / 2; // cm
  return dist;
}

// ── Send Response ──
void sendResponse(uint8_t *data, uint8_t len) {
  digitalWrite(DE_RE_PIN, HIGH);
  delayMicroseconds(200);

  for (uint8_t i = 0; i < len; i++) {
    RS485Serial.write(data[i]);
  }

  RS485Serial.flush();
  delayMicroseconds(200);
  digitalWrite(DE_RE_PIN, LOW);
}

// ── Send Distance ──
void sendDistance(uint16_t distance) {
  uint8_t response[7];

  response[0] = SLAVE_ID;
  response[1] = 0x03; // Read Holding Register
  response[2] = 0x02; // Byte count
  response[3] = highByte(distance);
  response[4] = lowByte(distance);

  uint16_t crc = calculateCRC(response, 5);
  response[5] = crc & 0xFF;
  response[6] = crc >> 8;

  sendResponse(response, 7);
}

// ── Process Modbus ──
void processModbus(uint8_t *frame, uint8_t len) {
  if (len < 8) return;
  if (frame[0] != SLAVE_ID) return;

  uint16_t receivedCRC = frame[len-2] | (frame[len-1] << 8);
  uint16_t calcCRC     = calculateCRC(frame, len - 2);

  if (receivedCRC != calcCRC) {
    Serial.println("CRC Error");
    return;
  }

  uint8_t fc = frame[1];
  uint16_t regAddr = (frame[2] << 8) | frame[3];

  // ── READ DISTANCE ──
  if (fc == 0x03 && regAddr == 1) {
    uint16_t dist = readDistance();

    Serial.print("Distance: ");
    Serial.print(dist);
    Serial.println(" cm");

    sendDistance(dist);
  }

  // ── OPTIONAL WRITE ACK (if master sends something) ──
  if (fc == 0x06) {
    sendResponse(frame, len); // echo back
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

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.println("Ultrasonic Modbus Slave Ready");
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
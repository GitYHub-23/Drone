
#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

/*
 * DRONE STEP 13
 * ESP32-S3 + HW-725
 *
 * Full throttle transmission: 0-100%
 * Roll, Pitch, Yaw transmission
 * Automatic throttle direction detection
 *
 * BENCH TEST ONLY!
 *
 * STM32 Step 11 must keep motors disabled.
 * No propellers, motors disconnected.
 *
 * Joystick power detection is NOT
 * implemented in this version.
 */

// =========== NRF24 PINS ===========

#define NRF_CE     9
#define NRF_CSN    10
#define NRF_SCK    12
#define NRF_MOSI   11
#define NRF_MISO   13

// =========== JOYSTICK PINS ===========

#define PIN_THROTTLE  1
#define PIN_YAW       2
#define PIN_ROLL      3
#define PIN_PITCH     4

// =========== RADIO ===========

RF24 radio(NRF_CE, NRF_CSN);

const uint8_t ADDRESS[5] = {
  0xCC, 0xCC, 0xCC, 0xCC, 0xCC
};

// =========== AXIS SETTINGS ===========

// Keep as in your previous working test
const bool INVERT_YAW   = false;
const bool INVERT_ROLL  = false;
const bool INVERT_PITCH = false;

// =========== TIMING ===========

const uint32_t TX_INTERVAL_MS = 50;
const uint32_t PRINT_INTERVAL_MS = 250;

// =========== VARIABLES ===========

uint16_t centerYaw   = 2048;
uint16_t centerRoll  = 2048;
uint16_t centerPitch = 2048;

bool throttleMinAtLowADC = true;
bool calibrationOK = false;

uint32_t txCount = 0;
uint32_t lastTX = 0;
uint32_t lastPrint = 0;

uint8_t packet[9] = {
  0xE1,
  0x8D,
  0x70,
  0x70,
  0x05,
  0x38,
  0x00,
  0x00,
  0x00
};

// =========== ADC ===========

uint16_t readADC(uint8_t pin) {

  uint32_t sum = 0;

  for (int i = 0; i < 4; i++) {
    sum += analogRead(pin);
  }

  return (uint16_t)(sum / 4);
}

// =========== AXIS ENCODING ===========

uint8_t encodeAxis(
  uint16_t raw,
  uint16_t center,
  uint8_t neutral,
  bool invert
) {

  int value = raw;
  int mid = center;

  if (invert) {
    value = 4095 - value;
    mid = 4095 - mid;
  }

  value = constrain(value, 0, 4095);
  mid = constrain(mid, 1, 4094);

  if (abs(value - mid) <= 35) {
    return neutral;
  }

  int result;

  if (value < mid) {

    result = value * neutral / mid;

  } else {

    result = neutral +
      (value - mid) *
      (225 - neutral) /
      (4095 - mid);
  }

  return (uint8_t)constrain(
    result, 0, 225
  );
}

// =========== THROTTLE ===========

uint8_t encodeThrottle(uint16_t raw) {

  raw = constrain(raw, 0, 4095);

  uint32_t scaled =
    ((uint32_t)raw * 225U + 2047U)
    / 4095U;

  if (throttleMinAtLowADC) {

    // ADC 0    -> 0% gas -> 225 (E1)
    // ADC 4095 -> 100%   -> 0

    return (uint8_t)(225U - scaled);

  } else {

    // ADC 4095 -> 0% gas -> 225 (E1)
    // ADC 0    -> 100%   -> 0

    return (uint8_t)scaled;
  }
}

uint8_t throttlePercent(uint8_t value) {

  return (uint8_t)(
    ((225U - value) * 100U + 112U)
    / 225U
  );
}

// =========== CHECKSUM ===========

uint8_t checksum(const uint8_t *data) {

  uint16_t sum = 0x76;

  for (int i = 0; i < 8; i++) {
    sum += data[i];
  }

  return (uint8_t)(sum & 0xFF);
}

// =========== CALIBRATION ===========

bool calibrateJoysticks() {

  Serial.println();
  Serial.println("CALIBRATION START");

  Serial.println(
    "Throttle DOWN, other axes CENTER!"
  );

  uint32_t sumT = 0;
  uint32_t sumY = 0;
  uint32_t sumR = 0;
  uint32_t sumP = 0;

  const int samples = 80;

  for (int i = 0; i < samples; i++) {

    sumT += analogRead(PIN_THROTTLE);
    sumY += analogRead(PIN_YAW);
    sumR += analogRead(PIN_ROLL);
    sumP += analogRead(PIN_PITCH);

    delay(5);
  }

  uint16_t t = sumT / samples;

  centerYaw   = sumY / samples;
  centerRoll  = sumR / samples;
  centerPitch = sumP / samples;

  Serial.printf(
    "ADC: T=%u Y=%u R=%u P=%u\n",
    t,
    centerYaw,
    centerRoll,
    centerPitch
  );

  // Check neutral positions of 3 axes

  if (
    centerYaw < 1300 ||
    centerYaw > 2800 ||
    centerRoll < 1300 ||
    centerRoll > 2800 ||
    centerPitch < 1300 ||
    centerPitch > 2800
  ) {

    Serial.println(
      "CAL ERROR: Joysticks not centered "
      "or joystick power is OFF!"
    );

    return false;
  }

  // Throttle must be at one endpoint

  if (t < 700) {

    throttleMinAtLowADC = true;

    Serial.println(
      "Throttle minimum: LOW ADC"
    );

  } else if (t > 3395) {

    throttleMinAtLowADC = false;

    Serial.println(
      "Throttle minimum: HIGH ADC"
    );

  } else {

    Serial.println(
      "CAL ERROR: Put throttle fully DOWN!"
    );

    return false;
  }

  calibrationOK = true;

  Serial.printf(
    "CENTERS: Y=%u R=%u P=%u\n",
    centerYaw,
    centerRoll,
    centerPitch
  );

  Serial.println("CALIBRATION OK");

  return true;
}

// =========== SETUP ===========

void setup() {

  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("=========================");
  Serial.println("DRONE STEP 13");
  Serial.println("THROTTLE DIAGNOSTIC");
  Serial.println("=========================");

  analogReadResolution(12);

  pinMode(PIN_THROTTLE, INPUT);
  pinMode(PIN_YAW, INPUT);
  pinMode(PIN_ROLL, INPUT);
  pinMode(PIN_PITCH, INPUT);

  SPI.begin(
    NRF_SCK,
    NRF_MISO,
    NRF_MOSI,
    NRF_CSN
  );

  if (!radio.begin(&SPI)) {

    while (true) {

      Serial.println(
        "ERROR: NRF24 NOT FOUND!"
      );

      delay(1000);
    }
  }

  radio.setAddressWidth(5);
  radio.setChannel(45);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);

  radio.setAutoAck(false);
  radio.setRetries(0, 0);

  radio.disableDynamicPayloads();

  radio.setCRCLength(RF24_CRC_16);
  radio.setPayloadSize(9);

  radio.openWritingPipe(ADDRESS);
  radio.stopListening();

  Serial.println("NRF24 OK");
  Serial.println("CHANNEL: 45");
  Serial.println("PAYLOAD: 9 BYTES");

  Serial.println();
  Serial.println(
    "Turn joystick power ON."
  );

  Serial.println(
    "Throttle fully DOWN."
  );

  Serial.println(
    "Yaw, Roll, Pitch centered."
  );

  delay(2000);

  // Do not start TX before calibration

  while (!calibrateJoysticks()) {

    Serial.println(
      "Retrying calibration..."
    );

    delay(1500);
  }

  Serial.println();
  Serial.println("TX STARTED");

  Serial.println(
    "MOTORS MUST REMAIN DISCONNECTED!"
  );
}

// =========== MAIN LOOP ===========

void loop() {

  uint32_t now = millis();

  if ((uint32_t)(now - lastTX) <
      TX_INTERVAL_MS) {

    delay(1);
    return;
  }

  lastTX = now;

  if (!calibrationOK) {
    return;
  }

  uint16_t rawT = readADC(PIN_THROTTLE);
  uint16_t rawY = readADC(PIN_YAW);
  uint16_t rawR = readADC(PIN_ROLL);
  uint16_t rawP = readADC(PIN_PITCH);

  /*
   * Limited bench guard for the case
   * where unpowered joystick inputs
   * all read near zero.
   *
   * This is NOT reliable power detection.
   */

  if (rawY < 100 &&
      rawR < 100 &&
      rawP < 100) {

    if ((uint32_t)(now - lastPrint) >=
        PRINT_INTERVAL_MS) {

      lastPrint = now;

      Serial.println(
        "INVALID ADC INPUTS - TX BLOCKED"
      );
    }

    return;
  }

  // Full throttle range

  packet[0] = encodeThrottle(rawT);

  // Other axes

  packet[1] = encodeAxis(
    rawY,
    centerYaw,
    0x8D,
    INVERT_YAW
  );

  packet[2] = encodeAxis(
    rawR,
    centerRoll,
    0x70,
    INVERT_ROLL
  );

  packet[3] = encodeAxis(
    rawP,
    centerPitch,
    0x70,
    INVERT_PITCH
  );

  packet[4] = 0x05;
  packet[5] = 0x38;
  packet[6] = 0x00;

  // Sequence number

  packet[7] = (uint8_t)(
    txCount & 0xFF
  );

  // Checksum

  packet[8] = checksum(packet);

  bool sent = radio.write(packet, 9);

  txCount++;

  if ((uint32_t)(now - lastPrint) >=
      PRINT_INTERVAL_MS) {

    lastPrint = now;

    Serial.printf(
      "THROTTLE: %3u%% | ",
      throttlePercent(packet[0])
    );

    Serial.printf(
      "ADC: T=%u Y=%u R=%u P=%u | ",
      rawT,
      rawY,
      rawR,
      rawP
    );

    Serial.printf(
      "PACKET: %02X %02X %02X %02X "
      "%02X %02X %02X %02X %02X | ",
      packet[0],
      packet[1],
      packet[2],
      packet[3],
      packet[4],
      packet[5],
      packet[6],
      packet[7],
      packet[8]
    );

    Serial.printf(
      "TX:%lu %s\n",
      (unsigned long)txCount,
      sent ? "OK" : "FAIL"
    );
  }
}

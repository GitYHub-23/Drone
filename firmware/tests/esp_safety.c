
/*
 * =========================================
 * DRONE_ STEP 18
 * ESP32-S3-N16R8 + NRF24L01+ HW-725
 *
 * JOYSTICK POWER PROTECTION
 *
 * Features:
 * - Detect switched joystick power
 * - Stop TX if joystick power is lost
 * - Wait for stable power on recovery
 * - Calibrate joystick centers
 * - Require minimum throttle
 * - Require neutral Roll/Pitch/Yaw
 * - Recalibrate after power interruption
 * - Transmit original 9-byte protocol
 *
 * BENCH DIAGNOSTICS ONLY
 *
 * STM32 MOTOR LOCK MUST STAY ENABLED.
 * MOTORS PHYSICALLY DISCONNECTED.
 * PROPELLERS REMOVED.
 *
 * No arming or real motor operation.
 * =========================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

/* =========================================
 * NRF24 PIN MAPPING
 * ========================================= */

#define NRF_CE   9
#define NRF_CSN  10
#define NRF_SCK  12
#define NRF_MOSI 11
#define NRF_MISO 13

/* =========================================
 * JOYSTICK PIN MAPPING
 * ========================================= */

#define PIN_THROTTLE 1
#define PIN_YAW      2
#define PIN_ROLL     3
#define PIN_PITCH    4

/* =========================================
 * SWITCHED POWER SENSOR
 * ========================================= */

/*
 * GPIO14 receives the MIDPOINT
 * of a 10k / 10k resistor divider.
 *
 * Switched supply -> 10k -> GPIO14
 *                         |
 *                        10k
 *                         |
 *                        GND
 *
 * Supply must be 0-5V only.
 */

#define PIN_POWER_SENSE 14

/*
 * Divider outputs approximately:
 *
 * Supply OFF : 0 mV
 * Supply 3.3V: 1650 mV
 * Supply 5V  : 2500 mV
 */

const uint32_t POWER_ON_THRESHOLD_MV = 900;

/* =========================================
 * TIMING
 * ========================================= */

const uint32_t POWER_STABLE_MS = 1000;
const uint32_t SAFE_STICKS_MS = 1000;

const uint32_t TX_INTERVAL_MS = 50;
const uint32_t PRINT_INTERVAL_MS = 1000;

/* =========================================
 * CALIBRATION SETTINGS
 * ========================================= */

const uint16_t CENTER_MIN_ADC = 1300;
const uint16_t CENTER_MAX_ADC = 2800;

const uint16_t CENTER_TOLERANCE_ADC = 160;
const uint16_t THROTTLE_TOLERANCE_ADC = 150;

const uint16_t THROTTLE_LOW_LIMIT = 700;
const uint16_t THROTTLE_HIGH_LIMIT = 3395;

const uint16_t MAX_CALIBRATION_SPREAD = 180;

/* =========================================
 * RADIO CONFIGURATION
 * ========================================= */

RF24 radio(NRF_CE, NRF_CSN);

const uint8_t RADIO_ADDRESS[5] = {
  0xCC, 0xCC, 0xCC, 0xCC, 0xCC
};

/* =========================================
 * AXIS SETTINGS
 * ========================================= */

/*
 * These are the settings used for
 * the previous transmitter diagnostics.
 *
 * Axis directions remain subject to
 * final physical verification.
 */

const bool INVERT_YAW = false;
const bool INVERT_ROLL = false;
const bool INVERT_PITCH = false;

/* =========================================
 * SYSTEM STATES
 * ========================================= */

enum SystemState {
  WAIT_POWER,
  WAIT_POWER_STABLE,
  CALIBRATING,
  WAIT_SAFE_STICKS,
  TX_ACTIVE
};

SystemState state = WAIT_POWER;

/* =========================================
 * VARIABLES
 * ========================================= */

uint16_t centerYaw = 2048;
uint16_t centerRoll = 2048;
uint16_t centerPitch = 2048;

uint16_t throttleMinimumRaw = 0;
bool throttleMinAtLowADC = true;

uint32_t powerStableStartMs = 0;
uint32_t safeSticksStartMs = 0;

uint32_t lastTxMs = 0;
uint32_t lastPrintMs = 0;

uint32_t txCount = 0;
uint32_t powerLossCount = 0;

bool calibrationOK = false;

/* Original 9-byte packet */

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

/* =========================================
 * ADC HELPERS
 * ========================================= */

uint16_t readADC(uint8_t pin)
{
  uint32_t sum = 0;

  for (int i = 0; i < 4; i++) {
    sum += analogRead(pin);
  }

  return (uint16_t)(sum / 4);
}

/* =========================================
 * POWER MONITOR
 * ========================================= */

uint32_t powerSenseMilliVolts()
{
  uint32_t sum = 0;

  for (int i = 0; i < 4; i++) {
    sum += analogReadMilliVolts(
      PIN_POWER_SENSE
    );
  }

  return sum / 4;
}

bool joystickPowerOK()
{
  return powerSenseMilliVolts() >=
         POWER_ON_THRESHOLD_MV;
}

/* =========================================
 * STATE NAME
 * ========================================= */

const char *stateName(SystemState value)
{
  switch (value) {
    case WAIT_POWER:
      return "WAIT_POWER";

    case WAIT_POWER_STABLE:
      return "WAIT_POWER_STABLE";

    case CALIBRATING:
      return "CALIBRATING";

    case WAIT_SAFE_STICKS:
      return "WAIT_SAFE_STICKS";

    case TX_ACTIVE:
      return "TX_ACTIVE";

    default:
      return "UNKNOWN";
  }
}

/* =========================================
 * STOP VALID RADIO TRANSMISSION
 * ========================================= */

void stopTransmission()
{
  if (state != WAIT_POWER) {
    powerLossCount++;

    Serial.println();
    Serial.println(
      "!!! JOYSTICK POWER LOST !!!"
    );

    Serial.println(
      "CONTROL PACKETS STOPPED"
    );
  }

  state = WAIT_POWER;
  calibrationOK = false;

  safeSticksStartMs = 0;
  powerStableStartMs = 0;

  /*
   * There is intentionally NO
   * radio.write() in this state.
   *
   * STM32 should detect a timeout.
   */
}

/* =========================================
 * THROTTLE CHECK
 * ========================================= */

bool throttleAtMinimum(uint16_t raw)
{
  if (throttleMinAtLowADC) {

    return
      (uint32_t)raw <=
      (uint32_t)throttleMinimumRaw +
      THROTTLE_TOLERANCE_ADC;

  } else {

    return
      (uint32_t)raw +
      THROTTLE_TOLERANCE_ADC >=
      (uint32_t)throttleMinimumRaw;
  }
}

/* =========================================
 * CENTER CHECK
 * ========================================= */

bool sticksCentered(
  uint16_t yaw,
  uint16_t roll,
  uint16_t pitch
)
{
  if (
    abs((int)yaw - (int)centerYaw) >
      CENTER_TOLERANCE_ADC
  ) {
    return false;
  }

  if (
    abs((int)roll - (int)centerRoll) >
      CENTER_TOLERANCE_ADC
  ) {
    return false;
  }

  if (
    abs((int)pitch - (int)centerPitch) >
      CENTER_TOLERANCE_ADC
  ) {
    return false;
  }

  return true;
}

/* =========================================
 * CALIBRATION
 * ========================================= */

bool calibrateJoysticks()
{
  const uint8_t pins[3] = {
    PIN_YAW,
    PIN_ROLL,
    PIN_PITCH
  };

  uint32_t sums[3] = {0, 0, 0};

  uint16_t low[3] = {
    4095, 4095, 4095
  };

  uint16_t high[3] = {0, 0, 0};

  uint32_t throttleSum = 0;

  const int samples = 80;

  Serial.println();
  Serial.println("CALIBRATION START");
  Serial.println(
    "Keep throttle DOWN"
  );
  Serial.println(
    "Keep other sticks CENTERED"
  );

  for (int i = 0; i < samples; i++) {

    if (!joystickPowerOK()) {
      Serial.println(
        "CAL ABORT: POWER LOST"
      );
      return false;
    }

    throttleSum +=
      analogRead(PIN_THROTTLE);

    for (int a = 0; a < 3; a++) {

      uint16_t v = analogRead(pins[a]);

      sums[a] += v;

      if (v < low[a]) {
        low[a] = v;
      }

      if (v > high[a]) {
        high[a] = v;
      }
    }

    delay(5);
  }

  if (!joystickPowerOK()) {
    return false;
  }

  uint16_t throttleRaw =
    (uint16_t)(throttleSum / samples);

  uint16_t centers[3];

  for (int a = 0; a < 3; a++) {

    centers[a] =
      (uint16_t)(sums[a] / samples);

    if (
      centers[a] < CENTER_MIN_ADC ||
      centers[a] > CENTER_MAX_ADC
    ) {
      Serial.println(
        "CAL ERROR: INVALID CENTER"
      );
      return false;
    }

    if (
      (high[a] - low[a]) >
      MAX_CALIBRATION_SPREAD
    ) {
      Serial.println(
        "CAL ERROR: STICK MOVING"
      );
      return false;
    }
  }

  if (throttleRaw <= THROTTLE_LOW_LIMIT) {

    throttleMinAtLowADC = true;

  } else if (
    throttleRaw >= THROTTLE_HIGH_LIMIT
  ) {

    throttleMinAtLowADC = false;

  } else {

    Serial.println(
      "CAL ERROR: THROTTLE NOT MINIMUM"
    );

    return false;
  }

  centerYaw = centers[0];
  centerRoll = centers[1];
  centerPitch = centers[2];

  throttleMinimumRaw = throttleRaw;

  calibrationOK = true;

  Serial.printf(
    "CAL OK T=%u Y=%u R=%u P=%u\n",
    throttleMinimumRaw,
    centerYaw,
    centerRoll,
    centerPitch
  );

  Serial.printf(
    "THROTTLE MIN ADC: %s\n",
    throttleMinAtLowADC ? "LOW" : "HIGH"
  );

  return true;
}

/* =========================================
 * AXIS ENCODING
 * ========================================= */

uint8_t encodeAxis(
  uint16_t raw,
  uint16_t center,
  uint8_t neutral,
  bool invert
)
{
  int value = raw;
  int middle = center;

  if (invert) {

    value = 4095 - value;
    middle = 4095 - middle;
  }

  value = constrain(value, 0, 4095);
  middle = constrain(middle, 1, 4094);

  if (abs(value - middle) <= 35) {
    return neutral;
  }

  int result;

  if (value < middle) {

    result =
      value * neutral / middle;

  } else {

    result =
      neutral +
      (value - middle) *
      (225 - neutral) /
      (4095 - middle);
  }

  return (uint8_t)constrain(
    result,
    0,
    225
  );
}

/* =========================================
 * THROTTLE ENCODING
 * ========================================= */

uint8_t encodeThrottle(uint16_t raw)
{
  raw = constrain(raw, 0, 4095);

  if (throttleAtMinimum(raw)) {
    return 225;
  }

  uint32_t scaled =
    ((uint32_t)raw * 225U + 2047U) /
    4095U;

  if (throttleMinAtLowADC) {
    return (uint8_t)(225U - scaled);
  }

  return (uint8_t)scaled;
}

/* =========================================
 * THROTTLE PERCENT
 * ========================================= */

uint32_t throttlePercent(uint8_t value)
{
  return
    ((225U - value) * 100U + 112U) /
    225U;
}

/* =========================================
 * PACKET CHECKSUM
 * ========================================= */

uint8_t packetChecksum(
  const uint8_t *data
)
{
  uint16_t sum = 0x76U;

  for (int i = 0; i < 8; i++) {
    sum += data[i];
  }

  return (uint8_t)(sum & 0xFF);
}

/* =========================================
 * INPUT VALIDATION
 * ========================================= */

bool readValidInputs(
  uint16_t &t,
  uint16_t &y,
  uint16_t &r,
  uint16_t &p
)
{
  if (!joystickPowerOK()) {
    return false;
  }

  t = readADC(PIN_THROTTLE);
  y = readADC(PIN_YAW);
  r = readADC(PIN_ROLL);
  p = readADC(PIN_PITCH);

  if (!joystickPowerOK()) {
    return false;
  }

  /*
   * Additional diagnostic guard:
   * all three axes near zero together
   * are treated as invalid.
   *
   * This is not a substitute for
   * power sensing.
   */

  if (y < 60 && r < 60 && p < 60) {
    return false;
  }

  return true;
}

/* =========================================
 * RADIO INITIALIZATION
 * ========================================= */

bool initializeRadio()
{
  SPI.begin(
    NRF_SCK,
    NRF_MISO,
    NRF_MOSI,
    NRF_CSN
  );

  if (!radio.begin(&SPI)) {
    return false;
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

  radio.openWritingPipe(RADIO_ADDRESS);
  radio.stopListening();

  return true;
}

/* =========================================
 * SETUP
 * ========================================= */

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("==========================");
  Serial.println("DRONE_ STEP 18");
  Serial.println("JOYSTICK POWER PROTECTION");
  Serial.println("==========================");

  analogReadResolution(12);

  analogSetPinAttenuation(
    PIN_POWER_SENSE,
    ADC_11db
  );

  pinMode(PIN_THROTTLE, INPUT);
  pinMode(PIN_YAW, INPUT);
  pinMode(PIN_ROLL, INPUT);
  pinMode(PIN_PITCH, INPUT);

  pinMode(PIN_POWER_SENSE, INPUT);

  Serial.println("Initializing NRF24...");

  if (!initializeRadio()) {

    Serial.println(
      "ERROR: NRF24 NOT FOUND"
    );

    while (true) {
      delay(1000);
    }
  }

  Serial.println("NRF24 OK");
  Serial.println("Channel: 45");
  Serial.println("Payload: 9 bytes");

  Serial.println();
  Serial.println(
    "Waiting for switched joystick power"
  );

  state = WAIT_POWER;
}

/* =========================================
 * MAIN LOOP
 * ========================================= */

void loop()
{
  uint32_t now = millis();

  uint32_t senseMv =
    powerSenseMilliVolts();

  /* =====================================
   * HIGHEST PRIORITY: POWER VALIDATION
   * ===================================== */

  if (senseMv < POWER_ON_THRESHOLD_MV) {

    stopTransmission();

    if (
      (uint32_t)(now - lastPrintMs) >=
      PRINT_INTERVAL_MS
    ) {
      lastPrintMs = now;

      Serial.printf(
        "STATE=WAIT_POWER SENSE=%lu mV\n",
        (unsigned long)senseMv
      );
    }

    delay(5);
    return;
  }

  /* =====================================
   * NEW POWER DETECTED
   * ===================================== */

  if (state == WAIT_POWER) {

    powerStableStartMs = now;

    state = WAIT_POWER_STABLE;

    Serial.println();
    Serial.println(
      "JOYSTICK POWER DETECTED"
    );

    Serial.printf(
      "SENSE=%lu mV\n",
      (unsigned long)senseMv
    );
  }

  /* =====================================
   * WAIT FOR STABLE POWER
   * ===================================== */

  if (state == WAIT_POWER_STABLE) {

    if (
      (uint32_t)(
        millis() - powerStableStartMs
      ) < POWER_STABLE_MS
    ) {
      delay(5);
      return;
    }

    state = CALIBRATING;
  }

  /* =====================================
   * CALIBRATE AFTER POWER-ON
   * ===================================== */

  if (state == CALIBRATING) {

    calibrationOK = false;

    if (!calibrateJoysticks()) {

      if (!joystickPowerOK()) {

        stopTransmission();

      } else {

        state = WAIT_POWER_STABLE;
        powerStableStartMs = millis();

        Serial.println(
          "Calibration retry required"
        );
      }

      delay(50);
      return;
    }

    state = WAIT_SAFE_STICKS;
    safeSticksStartMs = 0;

    Serial.println();
    Serial.println(
      "CALIBRATION COMPLETE"
    );

    Serial.println(
      "Waiting for safe joystick positions"
    );
  }

  /* =====================================
   * REQUIRE SAFE STICK POSITIONS
   * ===================================== */

  if (state == WAIT_SAFE_STICKS) {

    uint16_t t, y, r, p;

    if (!readValidInputs(t, y, r, p)) {

      if (!joystickPowerOK()) {
        stopTransmission();
      } else {
        safeSticksStartMs = 0;
      }

      return;
    }

    bool safe =
      calibrationOK &&
      throttleAtMinimum(t) &&
      sticksCentered(y, r, p);

    if (!safe) {

      safeSticksStartMs = 0;

      if (
        (uint32_t)(now - lastPrintMs) >=
        PRINT_INTERVAL_MS
      ) {
        lastPrintMs = now;

        Serial.printf(
          "WAIT_SAFE T=%u Y=%u R=%u P=%u\n",
          t, y, r, p
        );
      }

      delay(10);
      return;
    }

    if (safeSticksStartMs == 0) {
      safeSticksStartMs = millis();
    }

    if (
      (uint32_t)(
        millis() - safeSticksStartMs
      ) < SAFE_STICKS_MS
    ) {
      delay(10);
      return;
    }

    state = TX_ACTIVE;

    lastTxMs = millis();

    Serial.println();
    Serial.println("SAFE INPUTS CONFIRMED");
    Serial.println("TX ACTIVE");
    Serial.println("MOTORS MUST STAY OFF");
  }

  /* =====================================
   * ACTIVE RADIO TRANSMISSION
   * ===================================== */

  if (state != TX_ACTIVE) {
    delay(2);
    return;
  }

  now = millis();

  if (
    (uint32_t)(now - lastTxMs) <
    TX_INTERVAL_MS
  ) {
    delay(1);
    return;
  }

  lastTxMs = now;

  uint16_t rawT;
  uint16_t rawY;
  uint16_t rawR;
  uint16_t rawP;

  if (!readValidInputs(
        rawT, rawY, rawR, rawP
      )) {

    if (!joystickPowerOK()) {

      stopTransmission();

    } else {

      /*
       * Unexpected invalid ADC inputs:
       * require a new calibration.
       */

      calibrationOK = false;
      state = WAIT_POWER_STABLE;
      powerStableStartMs = millis();

      Serial.println(
        "INVALID INPUTS - TX STOPPED"
      );
    }

    return;
  }

  /* =====================================
   * ENCODE ORIGINAL PROTOCOL
   * ===================================== */

  packet[0] = encodeThrottle(rawT);

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

  packet[7] = (uint8_t)(
    txCount & 0xFF
  );

  packet[8] = packetChecksum(packet);

  /*
   * Check switched power one more time
   * immediately before radio.write().
   */

  if (!joystickPowerOK()) {
    stopTransmission();
    return;
  }

  bool sent = radio.write(packet, 9);

  txCount++;

  /* =====================================
   * SERIAL MONITOR
   * ===================================== */

  if (
    (uint32_t)(now - lastPrintMs) >=
    PRINT_INTERVAL_MS
  ) {
    lastPrintMs = now;

    Serial.printf(
      "STATE=%s | POWER=%lu mV | ",
      stateName(state),
      (unsigned long)powerSenseMilliVolts()
    );

    Serial.printf(
      "THR=%lu%% | ADC=%u | ",
      (unsigned long)throttlePercent(packet[0]),
      rawT
    );

    Serial.printf(
      "Y=%u R=%u P=%u | ",
      rawY, rawR, rawP
    );

    Serial.printf(
      "TX=%lu | WRITE=%s | ",
      (unsigned long)txCount,
      sent ? "OK" : "FAIL"
    );

    Serial.printf(
      "LOSS=%lu\n",
      (unsigned long)powerLossCount
    );
  }
}

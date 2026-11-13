
/*
 * ============================================
 * DRONE_ STEP 18.1
 * ESP32-S3-N16R8 + nRF24L01+ (HW-725)
 *
 * TRANSMITTER POWER FAILSAFE
 * ============================================
 *
 * Corrections:
 * 1. Throttle minimum is always HIGH ADC.
 * 2. Full joystick travel is permitted.
 * 3. Detect power loss and voltage sag.
 * 4. Recalibrate after interruption.
 * 5. Require safe stick positions.
 *
 * Diagnostic firmware ONLY.
 *
 * STM32 must remain DISARMED.
 * Motors must remain disconnected.
 * Propellers must remain removed.
 */

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

// ============================================
// PINS
// ============================================

#define NRF_CE    9
#define NRF_CSN   10
#define NRF_SCK   12
#define NRF_MOSI  11
#define NRF_MISO  13

#define PIN_THROTTLE  1
#define PIN_YAW       2
#define PIN_ROLL      3
#define PIN_PITCH     4

// 10k + 10k voltage divider
#define PIN_POWER     14

// ============================================
// POWER CONFIGURATION
// ============================================

// Your normal measured voltage:
// approximately 1640-1645 mV.
//
// OFF:
// approximately 0 mV.

const uint32_t POWER_START_MV = 1560;
const uint32_t POWER_DROP_ALLOW_MV = 85;
const uint32_t POWER_RISE_ALLOW_MV = 100;

const uint32_t POWER_STABLE_SPREAD_MV = 50;

const uint32_t POWER_STABLE_TIME_MS = 1500;

// ============================================
// JOYSTICK CALIBRATION
// ============================================

// According to your verified measurements:
//
// Throttle DOWN = ADC ~4095
// Throttle UP   = ADC ~0

const uint16_t THROTTLE_CAL_MIN = 3900;
const uint16_t THROTTLE_DEADBAND = 125;

const uint16_t CENTER_MIN = 1500;
const uint16_t CENTER_MAX = 2500;

const uint16_t CENTER_TOLERANCE = 160;

const uint16_t MAX_CAL_SPREAD = 180;
const uint16_t MAX_THROTTLE_SPREAD = 180;

// ============================================
// TIMING
// ============================================

const uint32_t SAFE_STICKS_TIME_MS = 1000;
const uint32_t TX_INTERVAL_MS = 50;
const uint32_t PRINT_INTERVAL_MS = 1000;

// ============================================
// RADIO
// ============================================

RF24 radio(NRF_CE, NRF_CSN);

const uint8_t radioAddress[5] = {
  0xCC, 0xCC, 0xCC, 0xCC, 0xCC
};

// Keep the same directions as Step 18.

const bool INVERT_YAW = false;
const bool INVERT_ROLL = false;
const bool INVERT_PITCH = false;

// ============================================
// TRANSMITTER STATES
// ============================================

enum TxState {
  WAIT_POWER,
  WAIT_STABLE_POWER,
  CALIBRATING,
  WAIT_SAFE_STICKS,
  TX_ACTIVE
};

TxState state = WAIT_POWER;

// ============================================
// VARIABLES
// ============================================

uint16_t centerYaw = 2048;
uint16_t centerRoll = 2048;
uint16_t centerPitch = 2048;

uint16_t throttleMinRaw = 4095;

uint32_t powerBaselineMv = 0;
uint32_t minimumPowerMv = POWER_START_MV;

uint32_t stableStartMs = 0;
uint32_t stableMinMv = 0;
uint32_t stableMaxMv = 0;

uint32_t safeStartMs = 0;

uint32_t lastTxMs = 0;
uint32_t lastPrintMs = 0;

uint32_t txCount = 0;
uint32_t powerFaultCount = 0;

uint8_t sequenceNumber = 0;

bool calibrationValid = false;

// Original receiver-compatible packet.
uint8_t packet[9] = {
  225, 141, 112, 112,
  5, 56, 0, 0, 0
};

// ============================================
// ADC FUNCTIONS
// ============================================

uint16_t readADC(uint8_t pin)
{
  uint32_t sum = 0;

  for (int i = 0; i < 4; i++) {
    sum += analogRead(pin);
  }

  return (uint16_t)(sum / 4);
}

uint32_t readPowerMv()
{
  uint32_t sum = 0;

  for (int i = 0; i < 4; i++) {
    sum += analogReadMilliVolts(PIN_POWER);
  }

  return sum / 4;
}

void readAllSticks(
  uint16_t &t,
  uint16_t &y,
  uint16_t &r,
  uint16_t &p
)
{
  t = readADC(PIN_THROTTLE);
  y = readADC(PIN_YAW);
  r = readADC(PIN_ROLL);
  p = readADC(PIN_PITCH);
}

// ============================================
// STATE NAMES
// ============================================

const char *stateName(TxState s)
{
  switch (s) {
    case WAIT_POWER:
      return "WAIT_POWER";

    case WAIT_STABLE_POWER:
      return "WAIT_STABLE_POWER";

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

// ============================================
// POWER STABILITY
// ============================================

void beginPowerStabilization(
  uint32_t now,
  uint32_t mv
)
{
  stableStartMs = now;
  stableMinMv = mv;
  stableMaxMv = mv;

  state = WAIT_STABLE_POWER;
}

bool runtimePowerOK(uint32_t mv)
{
  if (!calibrationValid) {
    return false;
  }

  if (mv < POWER_START_MV) {
    return false;
  }

  if (mv < minimumPowerMv) {
    return false;
  }

  if (
    mv >
    powerBaselineMv + POWER_RISE_ALLOW_MV
  ) {
    return false;
  }

  return true;
}

// ============================================
// TRANSMISSION STOP
// ============================================

void stopTransmission(
  const char *reason,
  uint32_t measuredMv
)
{
  if (state == TX_ACTIVE) {
    powerFaultCount++;
  }

  if (state != WAIT_POWER) {
    Serial.println();
    Serial.println(
      "!!! TRANSMISSION STOPPED !!!"
    );

    Serial.print("REASON: ");
    Serial.println(reason);

    Serial.printf(
      "POWER=%lu mV\n",
      (unsigned long)measuredMv
    );
  }

  calibrationValid = false;

  powerBaselineMv = 0;
  minimumPowerMv = POWER_START_MV;

  safeStartMs = 0;
  stableStartMs = 0;

  state = WAIT_POWER;

  /*
   * IMPORTANT:
   *
   * No radio.write() here.
   * The STM32 receiver must detect
   * missing packets and remain locked.
   */
}

// ============================================
// THROTTLE SAFETY
// ============================================

bool throttleAtMinimum(uint16_t raw)
{
  // The minimum is ALWAYS at high ADC.
  if (raw < THROTTLE_CAL_MIN) {
    return false;
  }

  return (
    (uint32_t)raw +
    THROTTLE_DEADBAND >=
    (uint32_t)throttleMinRaw
  );
}

// ============================================
// CENTERED STICKS
// ============================================

bool sticksCentered(
  uint16_t y,
  uint16_t r,
  uint16_t p
)
{
  return (
    abs((int)y - (int)centerYaw)
      <= CENTER_TOLERANCE &&

    abs((int)r - (int)centerRoll)
      <= CENTER_TOLERANCE &&

    abs((int)p - (int)centerPitch)
      <= CENTER_TOLERANCE
  );
}

// ============================================
// CALIBRATION
// ============================================

bool calibrateSticks()
{
  Serial.println();
  Serial.println("CALIBRATION START");
  Serial.println("Throttle FULL DOWN");
  Serial.println("Yaw/Roll/Pitch CENTER");

  const int SAMPLES = 80;

  uint32_t sums[4] = {0, 0, 0, 0};

  uint16_t minADC[4] = {
    4095, 4095, 4095, 4095
  };

  uint16_t maxADC[4] = {
    0, 0, 0, 0
  };

  uint32_t powerSum = 0;
  uint32_t powerMin = 10000;
  uint32_t powerMax = 0;

  for (int i = 0; i < SAMPLES; i++) {

    uint32_t mv = readPowerMv();

    if (mv < POWER_START_MV) {
      Serial.println(
        "CAL ERROR: LOW POWER"
      );
      return false;
    }

    powerSum += mv;

    if (mv < powerMin) {
      powerMin = mv;
    }

    if (mv > powerMax) {
      powerMax = mv;
    }

    uint16_t values[4];

    readAllSticks(
      values[0],
      values[1],
      values[2],
      values[3]
    );

    for (int a = 0; a < 4; a++) {

      sums[a] += values[a];

      if (values[a] < minADC[a]) {
        minADC[a] = values[a];
      }

      if (values[a] > maxADC[a]) {
        maxADC[a] = values[a];
      }
    }

    delay(5);
  }

  if (
    (powerMax - powerMin) >
    POWER_STABLE_SPREAD_MV
  ) {
    Serial.println(
      "CAL ERROR: UNSTABLE POWER"
    );
    return false;
  }

  uint16_t t = sums[0] / SAMPLES;
  uint16_t y = sums[1] / SAMPLES;
  uint16_t r = sums[2] / SAMPLES;
  uint16_t p = sums[3] / SAMPLES;

  // Never automatically reverse throttle.

  if (t < THROTTLE_CAL_MIN) {
    Serial.printf(
      "CAL ERROR: THROTTLE=%u\n", t
    );
    return false;
  }

  if (
    (maxADC[0] - minADC[0]) >
    MAX_THROTTLE_SPREAD
  ) {
    Serial.println(
      "CAL ERROR: THROTTLE MOVING"
    );
    return false;
  }

  const uint16_t centers[3] = {y, r, p};

  for (int a = 0; a < 3; a++) {

    if (
      centers[a] < CENTER_MIN ||
      centers[a] > CENTER_MAX
    ) {
      Serial.println(
        "CAL ERROR: INVALID CENTER"
      );
      return false;
    }

    if (
      (maxADC[a + 1] - minADC[a + 1]) >
      MAX_CAL_SPREAD
    ) {
      Serial.println(
        "CAL ERROR: STICK MOVING"
      );
      return false;
    }
  }

  uint32_t finalPower = readPowerMv();

  if (finalPower < POWER_START_MV) {
    Serial.println(
      "CAL ERROR: POWER LOST"
    );
    return false;
  }

  throttleMinRaw = t;

  centerYaw = y;
  centerRoll = r;
  centerPitch = p;

  powerBaselineMv = powerSum / SAMPLES;

  if (
    powerBaselineMv >
    POWER_DROP_ALLOW_MV
  ) {
    minimumPowerMv =
      powerBaselineMv -
      POWER_DROP_ALLOW_MV;
  } else {
    minimumPowerMv = POWER_START_MV;
  }

  if (minimumPowerMv < POWER_START_MV) {
    minimumPowerMv = POWER_START_MV;
  }

  if (!runtimePowerOK(finalPower)) {
    // calibrationValid is still false here;
    // check the voltage directly instead.

    if (
      finalPower < minimumPowerMv ||
      finalPower >
        powerBaselineMv + POWER_RISE_ALLOW_MV
    ) {
      Serial.println(
        "CAL ERROR: POWER CHANGED"
      );
      return false;
    }
  }

  calibrationValid = true;

  Serial.printf(
    "CAL OK T=%u Y=%u R=%u P=%u\n",
    t, y, r, p
  );

  Serial.printf(
    "POWER BASELINE=%lu mV\n",
    (unsigned long)powerBaselineMv
  );

  Serial.printf(
    "POWER MINIMUM=%lu mV\n",
    (unsigned long)minimumPowerMv
  );

  Serial.println(
    "THROTTLE MIN: HIGH ADC"
  );

  return true;
}

// ============================================
// AXIS ENCODING
// ============================================

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
    result, 0, 225
  );
}

// ============================================
// THROTTLE ENCODING
// ============================================

uint8_t encodeThrottle(uint16_t raw)
{
  // Original radio protocol:
  //
  // 225 = 0% gas
  //   0 = 100% gas

  if (throttleAtMinimum(raw)) {
    return 225;
  }

  uint32_t result =
    ((uint32_t)raw * 225U + 2047U)
    / 4095U;

  return (uint8_t)result;
}

// ============================================
// CHECKSUM
// ============================================

uint8_t calculateChecksum()
{
  uint16_t sum = 0x76;

  for (int i = 0; i < 8; i++) {
    sum += packet[i];
  }

  return (uint8_t)(sum & 0xFF);
}

// ============================================
// RADIO CONFIGURATION
// ============================================

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
  radio.setPayloadSize(9);
  radio.setCRCLength(RF24_CRC_16);

  radio.openWritingPipe(radioAddress);

  radio.stopListening();

  return true;
}

// ============================================
// SETUP
// ============================================

void setup()
{
  Serial.begin(115200);
  delay(1000);

  analogReadResolution(12);

  analogSetPinAttenuation(
    PIN_POWER, ADC_11db
  );

  analogSetPinAttenuation(
    PIN_THROTTLE, ADC_11db
  );

  analogSetPinAttenuation(
    PIN_YAW, ADC_11db
  );

  analogSetPinAttenuation(
    PIN_ROLL, ADC_11db
  );

  analogSetPinAttenuation(
    PIN_PITCH, ADC_11db
  );

  pinMode(PIN_POWER, INPUT);
  pinMode(PIN_THROTTLE, INPUT);
  pinMode(PIN_YAW, INPUT);
  pinMode(PIN_ROLL, INPUT);
  pinMode(PIN_PITCH, INPUT);

  Serial.println();
  Serial.println("=========================");
  Serial.println("DRONE_ STEP 18.1");
  Serial.println("POWER FAILSAFE UPDATED");
  Serial.println("=========================");

  if (!initializeRadio()) {
    Serial.println(
      "ERROR: NRF24 NOT FOUND"
    );

    while (true) {
      delay(1000);
    }
  }

  Serial.println("NRF24 OK");
  Serial.println("CHANNEL: 45");
  Serial.println("PAYLOAD: 9 BYTES");
  Serial.println("THROTTLE MIN ADC: HIGH");

  Serial.println();
  Serial.println("WAITING FOR POWER");

  state = WAIT_POWER;
}

// ============================================
// MAIN LOOP
// ============================================

void loop()
{
  uint32_t now = millis();
  uint32_t powerMv = readPowerMv();

  // ----------------------------------------
  // STATE 1: WAIT FOR POWER
  // ----------------------------------------

  if (state == WAIT_POWER) {

    if (powerMv >= POWER_START_MV) {

      Serial.println();
      Serial.println(
        "JOYSTICK POWER DETECTED"
      );

      Serial.printf(
        "POWER=%lu mV\n",
        (unsigned long)powerMv
      );

      beginPowerStabilization(
        now, powerMv
      );

    } else if (
      now - lastPrintMs >=
      PRINT_INTERVAL_MS
    ) {

      lastPrintMs = now;

      Serial.printf(
        "WAIT_POWER | %lu mV\n",
        (unsigned long)powerMv
      );
    }

    delay(5);
    return;
  }

  // ----------------------------------------
  // STATE 2: STABILIZE POWER
  // ----------------------------------------

  if (state == WAIT_STABLE_POWER) {

    if (powerMv < POWER_START_MV) {

      stopTransmission(
        "POWER BELOW START LIMIT",
        powerMv
      );

      delay(5);
      return;
    }

    if (powerMv < stableMinMv) {
      stableMinMv = powerMv;
    }

    if (powerMv > stableMaxMv) {
      stableMaxMv = powerMv;
    }

    if (
      stableMaxMv - stableMinMv >
      POWER_STABLE_SPREAD_MV
    ) {

      // Restart the stability interval.

      stableStartMs = now;
      stableMinMv = powerMv;
      stableMaxMv = powerMv;
    }

    if (
      now - stableStartMs <
      POWER_STABLE_TIME_MS
    ) {
      delay(5);
      return;
    }

    state = CALIBRATING;

    Serial.println(
      "POWER STABLE"
    );
  }

  // ----------------------------------------
  // STATE 3: CALIBRATION
  // ----------------------------------------

  if (state == CALIBRATING) {

    calibrationValid = false;

    if (!calibrateSticks()) {

      uint32_t mv = readPowerMv();

      if (mv < POWER_START_MV) {

        stopTransmission(
          "POWER LOST DURING CAL",
          mv
        );

      } else {

        beginPowerStabilization(
          millis(), mv
        );

        Serial.println(
          "CALIBRATION RETRY"
        );
      }

      delay(50);
      return;
    }

    state = WAIT_SAFE_STICKS;
    safeStartMs = 0;

    Serial.println();
    Serial.println(
      "CALIBRATION COMPLETE"
    );

    Serial.println(
      "WAITING FOR SAFE STICKS"
    );
  }

  // ----------------------------------------
  // STATE 4: SAFE STICK CHECK
  // ----------------------------------------

  if (state == WAIT_SAFE_STICKS) {

    if (!runtimePowerOK(powerMv)) {

      stopTransmission(
        "POWER UNSTABLE",
        powerMv
      );

      return;
    }

    uint16_t t, y, r, p;

    readAllSticks(t, y, r, p);

    uint32_t mv = readPowerMv();

    if (!runtimePowerOK(mv)) {

      stopTransmission(
        "POWER CHANGED",
        mv
      );

      return;
    }

    bool safe =
      throttleAtMinimum(t) &&
      sticksCentered(y, r, p);

    if (!safe) {

      safeStartMs = 0;

      if (
        now - lastPrintMs >=
        PRINT_INTERVAL_MS
      ) {

        lastPrintMs = now;

        Serial.printf(
          "WAIT_SAFE | T=%u Y=%u R=%u P=%u\n",
          t, y, r, p
        );
      }

      delay(5);
      return;
    }

    if (safeStartMs == 0) {
      safeStartMs = now;
    }

    if (
      now - safeStartMs <
      SAFE_STICKS_TIME_MS
    ) {
      delay(5);
      return;
    }

    state = TX_ACTIVE;

    lastTxMs = millis();

    Serial.println();
    Serial.println(
      "SAFE STICKS CONFIRMED"
    );

    Serial.println("TX ACTIVE");
    Serial.println(
      "MOTORS MUST REMAIN OFF"
    );
  }

  // ----------------------------------------
  // STATE 5: TRANSMISSION
  // ----------------------------------------

  if (state != TX_ACTIVE) {
    delay(2);
    return;
  }

  now = millis();
  powerMv = readPowerMv();

  if (!runtimePowerOK(powerMv)) {

    stopTransmission(
      "POWER DROP OR INSTABILITY",
      powerMv
    );

    return;
  }

  if (
    now - lastTxMs <
    TX_INTERVAL_MS
  ) {
    delay(1);
    return;
  }

  uint16_t rawT, rawY, rawR, rawP;

  readAllSticks(
    rawT, rawY, rawR, rawP
  );

  // Unlike the previous version,
  // ADC=0 and ADC=4095 are valid
  // endpoint values for Yaw/Roll/Pitch.

  uint32_t finalPower =
    readPowerMv();

  if (!runtimePowerOK(finalPower)) {

    stopTransmission(
      "POWER LOST BEFORE TX",
      finalPower
    );

    return;
  }

  // ----------------------------------------
  // ORIGINAL 9-BYTE RADIO PROTOCOL
  // ----------------------------------------

  packet[0] =
    encodeThrottle(rawT);

  packet[1] =
    encodeAxis(
      rawY, centerYaw, 141, INVERT_YAW
    );

  packet[2] =
    encodeAxis(
      rawR, centerRoll, 112, INVERT_ROLL
    );

  packet[3] =
    encodeAxis(
      rawP, centerPitch, 112, INVERT_PITCH
    );

  packet[4] = 0x05;
  packet[5] = 0x38;
  packet[6] = 0x00;

  packet[7] = sequenceNumber++;

  packet[8] = calculateChecksum();

  // Recheck the power before sending.

  finalPower = readPowerMv();

  if (!runtimePowerOK(finalPower)) {

    stopTransmission(
      "POWER FAULT AT TX",
      finalPower
    );

    return;
  }

  bool writeResult =
    radio.write(packet, sizeof(packet));

  lastTxMs = millis();
  txCount++;

  // ----------------------------------------
  // SERIAL DIAGNOSTICS
  // ----------------------------------------

  if (
    lastTxMs - lastPrintMs >=
    PRINT_INTERVAL_MS
  ) {

    lastPrintMs = lastTxMs;

    uint32_t throttlePercent =
      ((225U - packet[0]) * 100U +
       112U) / 225U;

    Serial.printf(
      "STATE=TX_ACTIVE | "
      "POWER=%lu mV | "
      "THR=%lu%% | "
      "ADC=%u | "
      "Y=%u R=%u P=%u | "
      "TX=%lu | "
      "WRITE=%s | "
      "POWER_FAULTS=%lu\n",

      (unsigned long)finalPower,
      (unsigned long)throttlePercent,

      rawT,
      rawY,
      rawR,
      rawP,

      (unsigned long)txCount,

      writeResult ? "OK" : "FAIL",

      (unsigned long)powerFaultCount
    );
  }
}

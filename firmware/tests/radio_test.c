
#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

// HW-725 -> ESP32-S3
#define NRF_CE    9
#define NRF_CSN   10
#define NRF_SCK   12
#define NRF_MOSI  11
#define NRF_MISO  13

RF24 radio(NRF_CE, NRF_CSN);

// Same address and channel as STM32 Step 8
const uint8_t radioAddress[5] = {
  0xCC, 0xCC, 0xCC, 0xCC, 0xCC
};

// Known valid 9-byte test packet
uint8_t packet[9] = {
  0xE1,  // Minimum throttle
  0x8D,  // Yaw center
  0x70,  // Roll center
  0x70,  // Pitch center
  0x05,
  0x38,
  0x00,
  0x21,
  0x22   // Checksum
};

uint32_t txCount = 0;

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("=== DRONE RADIO TX TEST ===");

  // SCK, MISO, MOSI, CSN
  SPI.begin(
    NRF_SCK,
    NRF_MISO,
    NRF_MOSI,
    NRF_CSN
  );

  if (!radio.begin(&SPI)) {
    Serial.println("ERROR: NRF24 not responding!");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("NRF24 SPI OK!");

  radio.setAddressWidth(5);
  radio.setChannel(45);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);

  radio.setAutoAck(false);
  radio.setRetries(0, 0);
  radio.disableDynamicPayloads();

  radio.setCRCLength(RF24_CRC_16);
  radio.setPayloadSize(9);

  radio.openWritingPipe(radioAddress);
  radio.stopListening();

  Serial.println("Radio configured!");
  Serial.println("Channel: 45");
  Serial.println("Address: CC CC CC CC CC");
  Serial.println("Data rate: 1 Mbps");
  Serial.println("Payload: 9 bytes");
  Serial.println("Starting TX...");
}

void loop() {
  bool sent = radio.write(packet, sizeof(packet));

  txCount++;

  if (txCount % 20 == 0) {
    Serial.print("TX #");
    Serial.print(txCount);

    Serial.print(" | Status: ");
    Serial.println(sent ? "SENT" : "FAILED");
  }

  delay(50);
}

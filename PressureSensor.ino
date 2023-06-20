#include <Adafruit_MPRLS.h>
#include <BluetoothSerial.h>

constexpr double PSI_to_KPA{PSI_to_HPA / 10.0};
constexpr double KPA_to_PSI{10.0 / PSI_to_HPA};

BluetoothSerial SerialBT;
Adafruit_MPRLS mpr = Adafruit_MPRLS{-1, -1, 0, 25, 10, 90, PSI_to_KPA};  // psi

// Packet structure for serial communication
struct {
  uint8_t startingBit{0xFA};
  float pressure;
  uint16_t crc;
} __attribute__((packed)) packet;

void calculateCRC() {
  constexpr size_t length{sizeof(packet) - sizeof(packet.startingBit) -
                          sizeof(packet.crc)};
  uint8_t *data{(uint8_t *)&packet + sizeof(packet.startingBit)};
  uint16_t crc = 0;
  for (size_t i = 0; i < length; i++) {
    crc = (uint8_t)(crc >> 8) | (crc << 8);
    crc ^= data[i];
    crc ^= (uint8_t)(crc & 0xff) >> 4;
    crc ^= crc << 12;
    crc ^= (crc & 0x00ff) << 5;
  }
  packet.crc = crc;
}

void setup() {
  SerialBT.begin("PressureSensor");
  Serial.begin(115200);
  if (!mpr.begin()) {
    delay(1000);
    return;
  }
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
}

void loop() {
  // Serial.Serial.printf("%.2f V\n", analogReadMilliVolts(A13) * 0.002f);
  packet.pressure = mpr.readPressure();
  calculateCRC();
  Serial.write((uint8_t *)&packet, sizeof(packet));

  SerialBT.println((packet.pressure - 100.0) * KPA_to_PSI);  // pressure in psi
  delay(20);
}

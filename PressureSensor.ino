#include <Adafruit_MPRLS.h>
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;
Adafruit_MPRLS mpr = Adafruit_MPRLS{-1, -1, 0, 25, 10, 90, 1};

void setup() {
  SerialBT.begin("PressureSensor");
  if (!mpr.begin()) {
    delay(1000);
    return;
  }
}

void loop() {
  SerialBT.println(mpr.readPressure());  // pressure in psi
  delay(5);
}

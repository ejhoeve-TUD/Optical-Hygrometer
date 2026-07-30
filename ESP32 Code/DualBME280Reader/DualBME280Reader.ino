++/*
  BME280 forced-mode readout at 1 Hz over I2C
  Uses Adafruit BME280 library

  Wiring (typical):
    BME280 VIN  -> 3.3V or 5V
    BME280 GND  -> GND
    BME280 SDA  -> SDA
    BME280 SCL  -> SCL

  Serial output:
    Temperature [C], Pressure [hPa], Humidity [%RH]
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

Adafruit_BME280 bme1;
Adafruit_BME280 bme2;

// Typical I2C addresses:
// 0x76 or 0x77
const uint8_t BME_ADDR1 = 0x76;
const uint8_t BME_ADDR2 = 0x77;

void setup() {
  Serial.begin(115200);

  Wire.begin();

  if (!bme1.begin(BME_ADDR1)) {
    Serial.println("Could not find BME280");
    while (1) {
      delay(1000);
    }
  }
  if (!bme2.begin(BME_ADDR2)) {
    Serial.println("Could not find BME280");
    while (1) {
      delay(1000);
    }
  }

    // Configure sensor 1
  bme1.setSampling(
    Adafruit_BME280::MODE_FORCED,
    Adafruit_BME280::SAMPLING_X1,
    Adafruit_BME280::SAMPLING_X1,
    Adafruit_BME280::SAMPLING_X1,
    Adafruit_BME280::FILTER_OFF
  );

  // Configure sensor 2
  bme2.setSampling(
    Adafruit_BME280::MODE_FORCED,
    Adafruit_BME280::SAMPLING_X1,
    Adafruit_BME280::SAMPLING_X1,
    Adafruit_BME280::SAMPLING_X1,
    Adafruit_BME280::FILTER_OFF
  );

  Serial.println("BME280 forced mode @ 1 Hz");
}

void loop() {
  // Trigger one measurement
  bme1.takeForcedMeasurement();
  bme2.takeForcedMeasurement();

  float temperature1 = bme1.readTemperature();      // °C
  float pressure1    = bme1.readPressure() / 100.0; // hPa
  float humidity1    = bme1.readHumidity();         // %RH

  float temperature2 = bme2.readTemperature();      // °C
  float pressure2    = bme2.readPressure() / 100.0; // hPa
  float humidity2    = bme2.readHumidity();         // %RH

  Serial.println("----- Sensor 1 -----");
  Serial.print("T = ");
  Serial.print(temperature1, 2);
  Serial.print(" C, ");

  Serial.print("P = ");
  Serial.print(pressure1, 2);
  Serial.print(" hPa, ");

  Serial.print("RH = ");
  Serial.print(humidity1, 2);
  Serial.println(" %");

  //Print 2

  Serial.println("----- Sensor 2 -----");
  Serial.print("T = ");
  Serial.print(temperature2, 2);
  Serial.print(" C, ");

  Serial.print("P = ");
  Serial.print(pressure2, 2);
  Serial.print(" hPa, ");

  Serial.print("RH = ");
  Serial.print(humidity2, 2);
  Serial.println(" %");

  // 1 Hz update rate
  delay(1000);
}
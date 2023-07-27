#include <Wire.h>                     // biblioteca requerida para rodar I2C
#include <SPI.h>                      // biblioteca requerida para rodar I2C
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>          // biblioteca BMP280

// Objects create
Adafruit_BMP280 sensor;               // cria o objeto sensor

void setup() {
  Serial.begin(9600);
  Serial.print("Teste modulo BMP280");
  sensor.begin();

  // verifica a conexão do sensor
  if (!sensor.begin()) {
    Serial.println("Sensor não encontrado. Verifique as conexoes do circuito!");
    //while (1);
  } else Serial.println(" - testando");
}

void loop() {
  Serial.print("Temeperature: ");
  Serial.print(sensor.readTemperature(), 2);
  Serial.print(" ºC - Pressure: ");
  Serial.print(sensor.readPressure()/101325, 2); // // converte pressão em hPa para atm
  Serial.print(" atm - Altitude: ");
  Serial.print(sensor.readAltitude(1013.25), 2);
  Serial.println(" m");

  delay(1000);
}
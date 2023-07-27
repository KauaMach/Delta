#include "TinyGPS++.h"
#include "SoftwareSerial.h"
#include <Wire.h>
#include <SPI.h>
#include <SD.h>


// Constantes
const float V_0 = 5.0; // Tensão de alimentação do sensor (V)
const float rho = 1.204; // Densidade do ar (kg/m^3)

// Parâmetros de calibração
const int offset_size = 10; // Número de leituras para cálculo do offset
const int veloc_mean_size = 20; // Número de leituras para média da velocidade
const int zero_span = 2; // Faixa de zero da leitura do ADC

// Pinos do Arduino
const int pressurePin = A0; // Pino analógico conectado à saída do sensor de pressão diferencial

// Variáveis globais
int offset = 0; // Offset do sensor
float veloc_mps = 0.0; // Velocidade em m/s
float veloc_kph = 0.0; // Velocidade em km/h


SoftwareSerial serial_connection(4, 3); //RX=pin 10, TX=pin 11
TinyGPSPlus gps;//This is the GPS object that will pretty much do all the grunt work with the NMEA data

uint8_t chipSelect = 8; // O pino CS da shield GPS

void setup()
{
  Serial.begin(9600);//This opens up communications to the Serial monitor in the Arduino IDE
  serial_connection.begin(9600);//This opens up communications to the GPS
  Serial.println("GPS Start");//Just show to the monitor that the sketch has started


   // Cálculo do offset
  for (int ii = 0; ii < offset_size; ii++) {
    offset += analogRead(pressurePin) - (1023 / 2);
  }
  offset /= offset_size;

   if (!SD.begin(chipSelect)) { // Se o cartão não estiver presente ou falhar....
    Serial.println("O MicroSD falhou ou nao esta presente");
    delay(1000);
  }
  Serial.println("O cartao foi inicializado corretamente.");

}

void loop()
{
  while(serial_connection.available())//While there are characters to come from the GPS
  {
    gps.encode(serial_connection.read());//This feeds the serial NMEA data into the library one char at a time
  }
  if(gps.location.isUpdated())//This will pretty much be fired all the time anyway but will at least reduce it to only after a package of NMEA data comes in
  {

    File dataFile = SD.open("GPSlog.txt", FILE_WRITE); // Associará o objeto dataFile ao arquivo GPSlog.txt. Se caso o arquivo não exista, será criado
    //O objeto dataFile foi setado como escrita, mas dá para utilizar de outras formas.
    Serial.println("<-------------------------------------------------------------------->");
     dataFile.println("<-------------------------------------------------------------------->");
    
    // Se caso não conseguir abrir o arquivo por qualquer razão, irá avisar no MonitorSerial
    if (!dataFile) Serial.println("Erro ao abrir o arquivo GPSlog.txt"); 

    //Get the latest info from the gps object which it derived from the data sent by the GPS unit

    Serial.println("\nDados GPS");
    Serial.print("Satellite: ");
    Serial.println(gps.satellites.value());
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("Velocidade (km/h): ");
    Serial.println(gps.speed.kmph());
    Serial.print("Velocidade (m/s): ");
    Serial.println(gps.speed.mps());
    Serial.print("Altitude Metros: ");
    Serial.println(gps.altitude.meters());

    dataFile.println("\nDados GPS");
    dataFile.print("Satellite: ");
    dataFile.println(gps.satellites.value());
    dataFile.print("Latitude: ");
    dataFile.println(gps.location.lat(), 6);
    dataFile.print("Longitude: ");
    dataFile.println(gps.location.lng(), 6);
    dataFile.print("Velocidade (km/h): ");
    dataFile.println(gps.speed.kmph());
    dataFile.print("Velocidade (m/s): ");
    dataFile.println(gps.speed.mps());
    dataFile.print("Altitude Metros: ");
    dataFile.println(gps.altitude.meters());

      float adc_avg = 0;
    
    // Leituras do ADC para média da velocidade
    for (int ii = 0; ii < veloc_mean_size; ii++) {
      adc_avg += analogRead(pressurePin) - offset;
    }
    adc_avg /= veloc_mean_size;
    
    // Cálculo da velocidade em m/s
    if (adc_avg > 512 - zero_span && adc_avg < 512 + zero_span) {
      // Velocidade próxima a zero
      veloc_mps = 0.0;
    } else {
      if (adc_avg < 512) {
        // Velocidade negativa
        veloc_mps = -sqrt((-10000.0 * ((adc_avg / 1023.0) - 0.5)) / rho);
      } else {
        // Velocidade positiva
        veloc_mps = sqrt((10000.0 * ((adc_avg / 1023.0) - 0.5)) / rho);
      }
    }

    // Cálculo da velocidade em km/h
    veloc_kph = veloc_mps * 3.6;
    
    // Impressão dos resultados na porta serial

    Serial.println("\nDados Tubo de Pitot");
    Serial.print("Velocidade (m/s): ");
    Serial.println(veloc_mps);
    Serial.print("Velocidade (km/h): ");
    Serial.println(veloc_kph);
    
    dataFile.println("\nDados Tubo de Pitot");
    dataFile.print("Velocidade (m/s): ");
    dataFile.println(veloc_mps);
    dataFile.print("Velocidade (km/h): ");
    dataFile.println(veloc_kph);
    dataFile.close();
  
   delay(100); // Ajuste o delay de acordo com a frequência de leitura desejada
  }  

}


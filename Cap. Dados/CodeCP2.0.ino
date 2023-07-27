#include "TinyGPS++.h"
#include "SoftwareSerial.h"
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>   


//----------------------------------------------------------------------------------------

const float V_0 = 5.0; // Tensão de alimentação do sensor pitot (V)
const float rho = 1.204; // Densidade do ar (kg/m^3)

// Parâmetros de calibração
const int offset_size = 10; // Número de leituras para cálculo do offset
const int veloc_mean_size = 20; // Número de leituras para média da velocidade
const int zero_span = 2; // Faixa de zero da leitura do ADC

const int pressurePin = A0; // Pino analógico conectado à saída do sensor de pressão diferencial

// Variáveis globais
int offset = 0; // Offset do sensor
float veloc_mps = 0.0; // Velocidade em m/s
float veloc_kph = 0.0; // Velocidade em km/h
//---------------------------------------------------------------------------------------

//GPS  
SoftwareSerial serial_connection(4, 3); //RX=pin 10, TX=pin 11
TinyGPSPlus gps;//This is the GPS object that will pretty much do all the grunt work with the NMEA data

uint8_t chipSelect = 8; // O pino CS da shield GPS
//----------------------------------------------------------------------------------------
  //SDcard
File dataFile;

//------------------------------------------------
// BM280
Adafruit_BMP280 sensor;       
float altset  = 74.15; 


void setup(){
  Serial.begin(9600);
  delay(100);
  Serial.println("Start Cap. Dados Delta");

  // ----------------------------------------------
    //BMP280
  //----------------------------------------------------------------

  // Pitot - Cálculo do offset
  for (int ii = 0; ii < offset_size; ii++) {
    offset += analogRead(pressurePin) - (1023 / 2);
  }
  offset /= offset_size;

  //----------------------------------------------------------------

  //Inicialização SD Card
  Serial.print("Inicializando cartão SD...");

  if (!SD.begin(8)) {
    Serial.println("Falha na inicialização!");
    //while (1);
  }else{
    Serial.println("Inicialização feita.");
  } 
 

  //----------------------------------------------------------------
  
  // GPS
  serial_connection.begin(9600);
  Serial.println("Inicializando GPS...");
}

void loop()
{
  
  while(serial_connection.available()){
    gps.encode(serial_connection.read());
  }
  if(gps.location.isUpdated()){
    dataFile = SD.open("CapDadosLOG.txt", FILE_WRITE); 
    if (!dataFile) Serial.println("Erro ao abrir o arquivo .txt"); 

    Serial.println("\n<-------------------------------------------------------------------->"); 
    //if (!dataFile) Serial.println("Erro ao abrir o arquivo .txt");

    Serial.println("\nDados GPS");
    Serial.print(gps.date.day() - 1 );
    Serial.print("/");
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.print(gps.date.year());
    Serial.print("  -  ");
    Serial.print(gps.time.hour() + 24 - 3 );
    Serial.print(":");
    Serial.print(gps.time.minute());
    Serial.print(":");
    Serial.println(gps.time.second());

    Serial.print("Satellite: ");
    Serial.println(gps.satellites.value());
    Serial.print("Latitude: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(" / ");
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
     
    Serial.print("Velocidade: ");
    Serial.print(gps.speed.kmph());
    Serial.print(" km/h");
    Serial.print(" / ");
    Serial.print(gps.speed.mps());
    Serial.println(" m/s");
    
    Serial.print("Altitude");
    Serial.print(gps.altitude.meters());
    Serial.print("m");
    
    //Salvando dados GPS no SD Card
    dataFile.println("\n\n<-------------------------------------------------------------------->");
    dataFile.println("\nDados GPS");   
    dataFile.print(gps.date.day());
    dataFile.print("/");
    dataFile.print(gps.date.month());
    dataFile.print("/");
    dataFile.print(gps.date.year());
    dataFile.print(" - ");
    dataFile.print(gps.time.hour() + 24 - 3);;    
    dataFile.print(":");
    dataFile.print(gps.time.minute());
    dataFile.print(":");
    dataFile.println(gps.time.second());
    dataFile.close();
    
    BMP280();  
    delay(100);
  }
}

/*
void Dados(){
  while(serial_connection.available()){
    gps.encode(serial_connection.read());
  }
  if(gps.location.isUpdated()){
    dataFile = SD.open("CapDadosLOG.txt", FILE_WRITE);

    Serial.println("<-------------------------------------------------------------------->"); 
    if (!dataFile) Serial.println("Erro ao abrir o arquivo .txt");

    Serial.println("\nDados GPS");
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.print(gps.date.year());
    Serial.print("  -  ");
    Serial.print(gps.time.hour());
    Serial.print(":");
    Serial.print(gps.time.minute());
    Serial.print(":");
    Serial.println(gps.time.second());

    Serial.print("Satellite: ");
    Serial.println(gps.satellites.value());
    Serial.print("Latitude: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(" / ");
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    
    Serial.print("Velocidade: ");
    Serial.print(gps.speed.kmph());
    Serial.print(" km/h");
    Serial.print(" / ");
    Serial.print(gps.speed.mps());
    Serial.println(" m/s");
    
    Serial.print("Altitude");
    Serial.print(gps.altitude.meters());
    Serial.print("m");


    //Salvando dados GPS no SD Card
    dataFile.println("\n\n<-------------------------------------------------------------------->");
    dataFile.println("\nDados GPS");   
    dataFile.print(gps.date.day());
    dataFile.print("/");
    dataFile.print(gps.date.month());
    dataFile.print("/");
    dataFile.print(gps.date.year());
    dataFile.print(" - ");
    dataFile.print(gps.time.hour() -3);;    
    dataFile.print(":");
    dataFile.print(gps.time.minute());
    dataFile.print(":");
    dataFile.println(gps.time.second());

    dataFile.println("Satelite: ");
    dataFile.println(gps.satellites.value());
    dataFile.print("Latitude: ");
    dataFile.print(gps.location.lat(), 6);
    dataFile.print(" / ");
    dataFile.print("Longitude: ");
    dataFile.println(gps.location.lng(), 6);

    dataFile.print("Velocidade");
    dataFile.print(gps.speed.kmph());
    dataFile.print(" km/h");
    dataFile.print(" / ");
    dataFile.print(gps.speed.mps());
    dataFile.println(" m/s");

    dataFile.print("Altitude: ");
    dataFile.print(gps.altitude.meters());
    dataFile.println(" M");
    dataFile.close();   
    delay(100);
    
  }
}*/

void BMP280(){
    dataFile = SD.open("CapDadosLOG.txt", FILE_WRITE);
    //if (!dataFile) Serial.println("Erro ao abrir o arquivo .txt"); 
    sensor.begin();
    Serial.println("\n\nBMP280");
    Serial.print("Altitude: ");
    Serial.print((sensor.readAltitude(1013.25)-altset), 2);
    Serial.println(" m");    
    Serial.print("Temeperatura: ");
    Serial.print(sensor.readTemperature(), 2);
    Serial.println(" ºC ");
    Serial.print("Pressão: ");
    Serial.print(sensor.readPressure()/101325, 2); // // converte pressão em hPa para atm
    Serial.println(" atm");
     
    dataFile.println("\n\nBMP280");
    dataFile.print("Altitude: ");
    dataFile.print(sensor.readAltitude(1013.25), 2);
    dataFile.println(" m"); 
    dataFile.print("Temeperatura: ");
    dataFile.print(sensor.readTemperature(), 2);
    dataFile.println(" ºC");
    dataFile.print("Pressão: ");
    dataFile.print(sensor.readPressure()/101325, 2); // // converte pressão em hPa para atm 
    dataFile.println(" atm");
    dataFile.close();
  
}
void Pitot(){
   dataFile = SD.open("CapDadosLOG.txt", FILE_WRITE);

  //Sensor de velocidade
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
  Serial.println("\n\nDados Tubo de Pitot:");
  Serial.print("Velocidade: "); 
  Serial.println(veloc_mps);
  Serial.print("m/s");
  Serial.print(" / ");
  Serial.println(veloc_kph);
  Serial.print(" (km/h)");
  
  dataFile.println("\n\nDados Tubo de Pitot:");
  dataFile.print("Velocidade: ");  
  dataFile.print(veloc_mps);
  dataFile.print(" m/s");
  dataFile.print(" / ");
  dataFile.print(veloc_kph);
  dataFile.println(" km/h");
  dataFile.close();
}




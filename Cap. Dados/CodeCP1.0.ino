#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>    
#include <SPI.h>
#include <SD.h>
#include "TinyGPS++.h"
#include "SoftwareSerial.h"
#include <Wire.h>


//GPS  
SoftwareSerial serial_connection(4, 3); //RX=pin 10, TX=pin 11
TinyGPSPlus gps;

//----------------------------------------------------------------------------------------
  //SDcard
uint8_t chipSelect = 8; // O pino CS da shield GPS
File dataFile;
//------------------------------------------------
// BM280
Adafruit_BMP280 sensor;               
float altset  = 74.15; 


void setup() {
  Serial.begin(9600);
  serial_connection.begin(9600);
  while (!Serial) {
 // Espera a porta serial conectar
  }
  
  Serial.println("Start Cap. Dados Delta:");
  // ----------------------------------------------
  //BMP280
  //sensor.begin();
  //-----------------------------------------------

  Serial.print("Inicializando cartão SD...");

  if (!SD.begin(8)) {
    Serial.println("Falha na inicialização!");
    while (1);
  }
  Serial.println("Inicialização feita.");


   dataFile = SD.open("cpdados.txt", FILE_WRITE);
  // Se este arquivo abriu corretamente, escreva nele:
  if (dataFile) {
    Serial.print("Escrevendo para cpdados.txt...");
    dataFile.println("testando 1, 2, 3.");
    // fecha o arquivo:
    dataFile.close();
    Serial.println("feito.");
  } else {
    // Se este arquivo não abriu, imprima um erro:
    Serial.println("Erro ao abrir cpdados.txt");
  }

  //----------------------------------------------------------------
  // GPS
  Serial.println("Inicializando GPS...");

}

void loop() {
  
  while(serial_connection.available())//While there are characters to come from the GPS
  {
    gps.encode(serial_connection.read());//This feeds the serial NMEA data into the library one char at a time
  }
  if(gps.location.isUpdated())//This will pretty much be fired all the time anyway but will at least reduce it to only after a package of NMEA data comes in
  {

    File dataFile = SD.open("cpdados.txt", FILE_WRITE); // Associará o objeto dataFile ao arquivo GPSlog.txt. Se caso o arquivo não exista, será criado
    //O objeto dataFile foi setado como escrita, mas dá para utilizar de outras formas.
    Serial.println("<-------------------------------------------------------------------->");
     dataFile.println("<-------------------------------------------------------------------->");
    
    // Se caso não conseguir abrir o arquivo por qualquer razão, irá avisar no MonitorSerial
    if (!dataFile) Serial.println("Erro ao abrir o arquivo cpdados.txt"); 

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
  
   delay(100); // Ajuste o delay de acordo com a frequência de leitura desejada
  }  
}


void Dados(){
  while(serial_connection.available()){
    gps.encode(serial_connection.read());
  }
  if(gps.location.isUpdated()){
    dataFile = SD.open("cpdados.txt", FILE_WRITE);

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

    BMP280();
    delay(100);
    
  }
}
void BMP280(){
    dataFile = SD.open("cpdados.txt", FILE_WRITE);
    if (!dataFile) Serial.println("Erro ao abrir o arquivo cpdados.txt"); 
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
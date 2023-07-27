#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>    
#include <SPI.h>
#include <SD.h>

File dataFile;

Adafruit_BMP280 sensor;               // cria o objeto sensor
float altset  = 74.15; 


void setup() {
  Serial.begin(9600);
  while (!Serial) {
 // Espera a porta serial conectar
  }

  Serial.println("Start :");
  // ----------------------------------------------
    //BMP280
  sensor.begin();
  //----------------------------------------------------

  Serial.print("Inicializando cartão SD...");

  if (!SD.begin(8)) {
    Serial.println("Falha na inicialização!");
    while (1);
  }
  Serial.println("Inicialização feita.");


   dataFile = SD.open("test.txt", FILE_WRITE);
  // Se este arquivo abriu corretamente, escreva nele:
  if (dataFile) {
    Serial.print("Escrevendo para test.txt...");
    dataFile.println("testando 1, 2, 3.");
    // fecha o arquivo:
    dataFile.close();
    Serial.println("feito.");
  } else {
    // Se este arquivo não abriu, imprima um erro:
    Serial.println("Erro ao abrir test.txt");
  }

}

void loop() {
  
  BMP280(); 
  delay(1000);
}

void BMP280(){
    dataFile = SD.open("test.txt", FILE_WRITE);
    if (!dataFile) Serial.println("Erro ao abrir o arquivo .txt"); 

    Serial.println("\nBMP280");
    Serial.print("Altitude: ");
    Serial.print((sensor.readAltitude(1013.25)-altset), 2);
    Serial.println(" m");    
    Serial.print("Temeperature: ");
    Serial.print(sensor.readTemperature(), 2);
    Serial.print(" ºC - Pressure: ");
    Serial.print(sensor.readPressure()/101325, 2); // // converte pressão em hPa para atm
    Serial.println();

    dataFile.println("\nBMP280");
    dataFile.print(" atm - Altitude: ");
    dataFile.print(sensor.readAltitude(1013.25), 2);
    dataFile.println(" m"); 
    dataFile.print("Temeperature: ");
    dataFile.print(sensor.readTemperature(), 2);
    dataFile.print(" ºC - Pressure: ");
    dataFile.print(sensor.readPressure()/101325, 2); // // converte pressão em hPa para atm
    dataFile.print("\n"); 
    
    dataFile.close();
   
}
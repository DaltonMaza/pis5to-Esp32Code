#include <WiFi.h> // Conexión Wi-Fi
// Librerías para la comunicación con la placa Maesto
#include <ESPAsyncWebServer.h>
#include "esp_now.h"
#include <ArduinoJson.h> // Librería para manejar datos tipo JSON

AsyncWebServer server(80); // Se levanta un servidor en el puerto 80

const char* ssid = "Internet_UNL"; // SSID de la red
const char* password = "UNL1859WiFi"; // Contraseña de la red

//Libreria para el sensor DHT11
#include "DHTesp.h"
//Inicializamos el controlador del sensor
DHTesp dht;
const int Dhtpin = 16; //Pin para el Sensor

float getTemperatura(){
  float temperatura = dht.getTemperature(); // Recuperar la temperatura del sensor
  
  Serial.print("Temperatura: ");
  Serial.print(temperatura);
  Serial.println(" °C");

  return temperatura;
}

float getHumedad(){
  float humedad = dht.getHumidity(); // Recuperar la humedad del sensor

  Serial.print("Humedad: ");
  Serial.print(humedad);
  Serial.println(" %");
  Serial.println();

  return humedad;
}

//Sensor de presión
//Libreria para comunicación I2C
#include <Wire.h>
//Librerias del sensor
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>

//Inicializar el controlador del sensor
Adafruit_BMP085 bmp;

//variables para presion
float constanteDivision = 100.0;
double constanteConversion = (4.0000D / 3.0000D);

float getPresion(){
  float presion = ((bmp.readPressure() / constanteDivision)); // Leer la presión
  presion = (presion * constanteConversion); // Convertir la presión a Pa

  Serial.print("Presion: ");
  Serial.print(presion);
  Serial.println(" mlb");
  Serial.println();

  return presion;
}

void setup() {
  //DHT11
  Serial.begin(115200);
  Serial2.begin(115200);
  dht.setup(Dhtpin, DHTesp::DHT11);

  //BMP180 
  bmp.begin();

  Serial.begin(115200); // Comunicación serial para la debug
  delay(1000);

  WiFi.begin(ssid, password); // Iniciar la conexión Wi-Fi

  Serial.print("Conectando a Wifi.");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("Conectado a Wifi");
  Serial.println();
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("Dirección MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println();

  // Disponemos los datos recuperados, esperando ser leidos por el maestro
  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){
    float temperatura = getTemperatura();
    float humedad = getHumedad();
    float presion = getPresion();

    AsyncResponseStream *response = request->beginResponseStream("application/json");
    JsonDocument jsonDoc;
    jsonDoc["temperature"] = temperatura;
    jsonDoc["humidity"] = humedad;
    jsonDoc["pressure"] = presion;
    serializeJson(jsonDoc, *response);
    request->send(response);
  });

  server.begin();
}

void loop() {
}
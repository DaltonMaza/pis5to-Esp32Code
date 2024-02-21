#include <WiFi.h> // Librería para la comunicación Wi-Fi de la placa
#include <HTTPClient.h> // Librería para realizar peticiones HTTP al servidor
#include <ArduinoJson.h> // Librería para manejar datos tipo JSON

volatile int LED_FAILED_REQUEST = 26;   // D26
volatile int LED_SUCCESS_REQUEST = 14;  // D14

const char* slaveEndpoint = "http://192.168.0.100/data"; // Dirección IP de la placa hija
const char* serverEndpoint = "https://api-pis5to.fly.dev/weatherdatas"; // Dirección del servidor
const char* ssid = "Internet_UNL"; // SSID de la red
const char* password = "UNL1859WiFi"; // Contraseña de la red

unsigned long previousMillis = 0; // Variable para el control de las solicitudes
const long interval = 60000;  // Intervalo de tiempo a realizar petición (1 minuto)

WiFiClient client; // Inicialización de el manejador del Wi-Fi

//Variables para la medición del sensor de viento
unsigned long tiempoInicioViento = 0;
const unsigned long duracionMedicionViento = 1000000;  // 1 segundo en microsegundos

//Sensor de viento
int vueltas = 0;
bool helicePasando = false;
const float distanciaHelicesCM = 25.0;
const float diametroRodamiento = 100.0;
const int pinEntrada = 35; // Pin para recibir datos
const int pinLED = 13;  // Pin para el LED

float getVelocidadViento(){
  float velocidadViento = 0;
   // Verificar si ha pasado 1 segundo desde la última llamada
  if (esp_timer_get_time() - tiempoInicioViento >= duracionMedicionViento) {
    tiempoInicioViento = esp_timer_get_time();  // Reiniciar el tiempo de inicio

    // Código para medir la velocidad del viento durante 1 segundo
    vueltas = 0;
    while (esp_timer_get_time() - tiempoInicioViento < duracionMedicionViento) {
      // if (digitalRead(pinEntrada) == HIGH) {
      if (digitalRead(pinEntrada) == HIGH & !helicePasando) {
        vueltas++;
        helicePasando = true;
      }
      if(digitalRead(pinEntrada) == LOW){
        helicePasando = false;
      }
    }
    velocidadViento = ((distanciaHelicesCM * vueltas) / diametroRodamiento) / (duracionMedicionViento / 1000000);
    // Imprimir resultados después de 1 segundo
    Serial.print("Velocidad Viento: ");
    Serial.print(velocidadViento);
    Serial.println("m/s");
    Serial.println();
  }
  return velocidadViento;
}

void setup() {
  pinMode(pinEntrada, INPUT);
  pinMode(pinLED, OUTPUT);

  Serial.begin(115200); // Iniciar la comunicación Serial (UART)
  delay(1000);
  WiFi.begin(ssid, password); // Iniciar la conexión de red

  // Mensajes informativos mientras se establece conexión wifi
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

  // Configuración sensores y pulsadores o leds
  pinMode(LED_FAILED_REQUEST, OUTPUT);   // Led petición fallida
  pinMode(LED_SUCCESS_REQUEST, OUTPUT);  // Led petición exitosa

  digitalWrite(LED_FAILED_REQUEST, LOW);
  digitalWrite(LED_SUCCESS_REQUEST, LOW);
}

void loop() {
  unsigned long currentMillis = millis();
  digitalWrite(LED_FAILED_REQUEST, LOW);
  digitalWrite(LED_SUCCESS_REQUEST, LOW);

  // Comprobar si ha pasado el intervalo de tiempo
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    sendData();
    // Un delay para que se mantenga encendido cierto tiempo el led
    delay(2000);
  }
}

// Función a ejecutar cada cierto intervalo
// Envía los datos al servidor
void sendData() {
  float windSpeed = getVelocidadViento();
  float temperature, humidity, pressure;
  getTemperatureAndHumidity(temperature, humidity, pressure);

  // Crear JSON
  StaticJsonDocument<200> doc;
  doc["temperature"] = temperature;
  doc["humidity"] = humidity;
  doc["barometricPressure"] = pressure;
  doc["windSpeed"] = windSpeed;

  // Serializar JSON a string
  String jsonStr;
  serializeJson(doc, jsonStr);

  // Envio JSON al servidor
  HTTPClient http;
  http.begin(serverEndpoint);
  // Configuro cabezeras de solicitud
  http.addHeader("Content-Type", "application/json");

  int httpResponseCode = http.POST(jsonStr);

  Serial.println(httpResponseCode);
  Serial.println(jsonStr);

  if (httpResponseCode > 0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);

    digitalWrite(LED_SUCCESS_REQUEST, HIGH);
  } else {
    Serial.println("Error en la solicitud HTTP");
    digitalWrite(LED_FAILED_REQUEST, HIGH);
  }
  Serial.println("Respuesta servidor:");
  Serial.println(http.getString());
  Serial.println();
  Serial.println();
  Serial.println();

  http.end();
}

// Obtener los datos de la placa hija
void getTemperatureAndHumidity(float& temperature, float& humidity, float& pressure) {
  HTTPClient http;
  http.begin(slaveEndpoint);

  int httpResponseCode = http.GET();

  if (httpResponseCode == HTTP_CODE_OK) {
    String payload = http.getString();
    JsonDocument doc;
    deserializeJson(doc, payload);

    temperature = doc["temperature"];
    humidity = doc["humidity"];
    pressure = doc["pressure"];
  } else {
    Serial.print("Error en la solicitud HTTP: ");
    Serial.println(httpResponseCode);

    temperature = -1; // Valor por defecto para temperatura
    humidity = -1; // Valor por defecto para humedad
    pressure = -1; //Valor por defecto para la presión
  }

  http.end();
}
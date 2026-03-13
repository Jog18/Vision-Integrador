#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "JOSUE's Galaxy A52"; //nombre de la red
const char* password = "jog18030"; //contraseña de nuestra red
const char* mqtt_server = "10.91.115.191"; // broker

WiFiClient espClient;
PubSubClient client(espClient);

long lastMsg = 0;
//Encendido y stop
const int PIN_BOTON_OFF = 4; // botón para apagar
const int PIN_BOTON_ON  = 18;   // botón para encender
const int PIN_LED = 2;    // LED si esta encendido o en paro
bool ledState = false;   // estado del LED para enviar a MQTT
// estados estables de los botones
bool buttonStateOn  = HIGH;
bool buttonStateOff = HIGH;
// lecturas anteriores (para detectar rebotes)
bool lastReadingOn  = HIGH;
bool lastReadingOff = HIGH;
// tiempos de debounce
unsigned long lastDebounceTimeOn  = 0;
unsigned long lastDebounceTimeOff = 0;
// tiempo de eliminación de rebote
const unsigned long debounceDelay = 50;

//Direccion
const int ledizq = 16;
const int ledder = 5;
const int izq = 17;
const int der = 0;

//Leer sensores. POT
const int pinPot1 = 34; //GPI34 para leer el pot 1
int valorADCpot1 = 0; //variable para almacenar la lectura del ADC
float voltaje1 = 0.0; // variable para guardar el voltaje equivalente
//LM35
const int pinLM35 = 36;//GPI36 para leer LM35
int adcTemp = 0;// //variable para almacenar el valor del ADC del LM35
float voltajeLM35 = 0.0; // voltaje del lm35
float temperatura = 0.0; // temperatura equivalente

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);

  setup_wifi();

  client.setServer(mqtt_server, 1883); //concetamos al servidor mosquitto en el puerto 1883
  client.setCallback(callback);

  //Potenciometro
  analogReadResolution(12);      // Resolución de 12 bits (0–4095)
  analogSetAttenuation(ADC_11db); // Permite leer hasta ~3.3V

  //pinMode(ledPin, OUTPUT); // definimos pinled como salida
  pinMode(ledizq, OUTPUT);
  pinMode(ledder, OUTPUT);
  pinMode(izq, OUTPUT);
  pinMode(der, OUTPUT);

  //Arranque y paro
  pinMode(PIN_BOTON_ON, INPUT_PULLUP);
  pinMode(PIN_BOTON_OFF, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
}

// ---------- LOOP ----------
void loop() {

  if (!client.connected()) {
    reconnect();
  }

  client.loop();
  startStop();

  //POTENCIOMETRO1
  valorADCpot1 = analogRead(pinPot1); //leemos el ADC del pot1
  voltaje1 = (valorADCpot1 * 3.3) / 4095.0; // Convertir a voltaje 0 - 3.3

  //LM35
  adcTemp = analogRead(pinLM35); // lectura del ADC DE LM35
  voltajeLM35 = (adcTemp * 5) / 4095.0; //convertimos a voltaje 
  temperatura = voltajeLM35 * 100.0; //calculamos el equivalente a temperatura

  long now = millis();
  if (now - lastMsg > 1000) {
    lastMsg = now;
    //enviar datos pot
    char vol1[10]; //variable tipo caracter a enviar por MQTT
    dtostrf(voltaje1, 1, 3, vol1); //convertimos el valor de voltaje a tipo char para que se pueda enviar a mosquitto
    client.publish("pot/uno", vol1); // mandamos la variable "mensaje"  al topic "pot/uno"
    //client.publish("prueba/uno", "hola"); 

    //enviar datos LM35
    char temp[10]; //variable tipo caracter a enviar por MQTT
    dtostrf(temperatura, 1, 3, temp); //convertimos el valor de temperatura a tipo char para que se pueda enviar a mosquitto
    client.publish("LM35/uno", temp); // mandamos la variable "temp" al topic "LM35/uno"

   if(ledState) {
    client.publish("arranque/paro", "MOVIMIENTO");
    }
   else {
        client.publish("arranque/paro", "PARO");
    }
  }

}
// ---------- WIFI ----------
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

// ---------- CALLBACK MQTT ----------
void callback(char* topic, byte* message, unsigned int length) {

  Serial.print("Mensaje recibido en topic: ");
  Serial.print(topic);
  Serial.print(". Mensaje: ");

  String messageTemp; //variable para almacenar el dato que recibe el esp32 decodificado

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i]; 
  }
  Serial.println();


  // Controlar motores y LEDs según posición de la línea
  if (messageTemp == "CENTRO") { // Línea dentro de la zona segura: ambos motores y LEDs encendidos
    digitalWrite(ledizq, HIGH);
    digitalWrite(ledder, HIGH);
    digitalWrite(izq, HIGH);
    digitalWrite(der, HIGH);
  }
  if (messageTemp == "IZQ") { // Línea salió a la izquierda: corregir activando solo lado izquierdo
    digitalWrite(ledizq, HIGH);
    digitalWrite(ledder, LOW);
    digitalWrite(izq, HIGH);
    digitalWrite(der, LOW);
  }
  if (messageTemp == "DER") { // Línea salió a la derecha: corregir activando solo lado derecho
    digitalWrite(ledder, HIGH);
    digitalWrite(ledizq, LOW);
    digitalWrite(der, HIGH);
    digitalWrite(izq, LOW);
  }

  if (messageTemp == "STOP") { // PARO desde interfaz
    digitalWrite(ledizq, LOW);
    digitalWrite(ledder, LOW);
    digitalWrite(izq, LOW);
    digitalWrite(der, LOW);
    ledState = false;                // apagar LED
        digitalWrite(PIN_LED, ledState);
  }

  if (messageTemp == "GO") { // ENCENDIDO desde interfaz
    digitalWrite(ledizq, LOW);
    digitalWrite(ledder, LOW);
    digitalWrite(izq, LOW);
    digitalWrite(der, LOW);
    ledState = true;                // apagar LED
        digitalWrite(PIN_LED, ledState);
  }

}

// ---------- RECONNECT MQTT ----------
void reconnect() {
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT... ");

    if (client.connect("ESP32Client")) {
      Serial.println("conectado");
      client.subscribe("esp32/control");
      client.subscribe("esp32/arranque");
      client.subscribe("esp32/stop");
    } else {
      Serial.print("falló, rc=");
      Serial.print(client.state());
      Serial.println(" intentando en 5 segundos");
      delay(5000);
    }
  }
}

void startStop() {
  bool readingOn = digitalRead(PIN_BOTON_ON);

  if (readingOn != lastReadingOn) {
    lastDebounceTimeOn = millis();
  }

  if ((millis() - lastDebounceTimeOn) > debounceDelay) {

    if (readingOn != buttonStateOn) {

      buttonStateOn = readingOn;

      // si se presiona el botón de encender
      if (buttonStateOn == LOW) {

        ledState = true;                 // encender LED
        digitalWrite(PIN_LED, ledState);

        Serial.println("LED ENCENDIDO");

      }

    }

  }

  lastReadingOn = readingOn;

  bool readingOff = digitalRead(PIN_BOTON_OFF);

  if (readingOff != lastReadingOff) {
    lastDebounceTimeOff = millis();
  }

  if ((millis() - lastDebounceTimeOff) > debounceDelay) {

    if (readingOff != buttonStateOff) {

      buttonStateOff = readingOff;

      // si se presiona el botón de apagar
      if (buttonStateOff == LOW) {

        ledState = false;                // apagar LED
        digitalWrite(PIN_LED, ledState);

        Serial.println("LED APAGADO");

      }

    }

  }

  lastReadingOff = readingOff;

}

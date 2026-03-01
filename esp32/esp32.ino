#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "JOSUE's Galaxy A52"; //nombre de la red
const char* password = "jog18030"; //contraseña de nuestra red
const char* mqtt_server = "10.165.252.191"; // broker

WiFiClient espClient;
PubSubClient client(espClient);

long lastMsg = 0;
//const int ledPin = 4; // GPIO4 de la ESP32
const int ledizq = 16;
const int ledder = 4;
const int izq = 17;
const int der = 0;

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);

  setup_wifi();

  client.setServer(mqtt_server, 1883); //concetamos al servidor mosquitto en el puerto 1883
  client.setCallback(callback);

  //pinMode(ledPin, OUTPUT); // definimos pinled como salida
  pinMode(ledizq, OUTPUT);
  pinMode(ledder, OUTPUT);
  pinMode(izq, OUTPUT);
  pinMode(der, OUTPUT);
}

// ---------- LOOP ----------
void loop() {

  if (!client.connected()) {
    reconnect();
  }

  client.loop();

  long now = millis();
  if (now - lastMsg > 1000) {
    lastMsg = now;
    client.publish("prueba/uno", "hola"); 
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

  // controlar LED
  if (messageTemp == "IZQ") { //si el mensaje recibido es ON, enciende led
    digitalWrite(ledizq, HIGH);
    digitalWrite(ledder, LOW);
    digitalWrite(izq, HIGH);
    digitalWrite(der, LOW);
  }
  if (messageTemp == "DER") { //si el mensaje recibido es OFF, apaga el led
    digitalWrite(ledder, HIGH);
    digitalWrite(ledizq, LOW);
    digitalWrite(der, HIGH);
    digitalWrite(izq, LOW);
  }

}

// ---------- RECONNECT MQTT ----------
void reconnect() {
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT... ");

    if (client.connect("ESP32Client")) {
      Serial.println("conectado");
      client.subscribe("esp32/control");
    } else {
      Serial.print("falló, rc=");
      Serial.print(client.state());
      Serial.println(" intentando en 5 segundos");
      delay(5000);
    }
  }
}

#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

const char* ssid = "JOSUE's Galaxy A52"; //nombre de la red
const char* password = "jog18030"; //contraseña de nuestra red
const char* mqtt_server = "10.218.99.191"; // broker

WiFiClient espClient;
PubSubClient client(espClient);

long lastMsg = 0;
//Encendido y stop
const int PIN_BOTON_OFF = 4; // botón para apagar
const int PIN_BOTON_ON  = 18;   // botón para encender
const int PIN_LED = 2;    // LED si esta encendido
const int PIN_LEDPARO = 23;    // LED si esta encendido en paro
bool ledState = false;   // estado del LED para enviar a MQTT
bool lastLedState = false; // estado anterior para detectar cambios
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

// E-Stop (Paro de emergencia) - Botón normalmente cerrado
const int PIN_ESTOP = 13;
volatile bool emergencia = false; // flag de emergencia (volatile porque se modifica en ISR)
bool flag = false;

// ---- Servo de dirección Ackermann ----
const int PIN_SERVO = 5;  // Pin PWM para el servo de dirección
Servo servoDir;
int anguloActual = 90;    // 90° = recto

// LEDs indicadores de dirección (se mantienen para feedback visual)
const int ledizq = 16;
const int ledder = 15;

//Leer sensores. POT
const int pinPot1 = 34; //GPI34 para leer el pot 1
int valorADCpot1 = 0; //variable para almacenar la lectura del ADC
float voltaje1 = 0.0; // variable para guardar el voltaje equivalente
//LM35
const int pinLM35 = 36;//GPI36 para leer LM35
int adcTemp = 0;// //variable para almacenar el valor del ADC del LM35
float voltajeLM35 = 0.0; // voltaje del lm35
float temperatura = 0.0; // temperatura equivalente

// ---------- ISR PARO DE EMERGENCIA ----------
void IRAM_ATTR isrEmergencia() {
  emergencia = true;
  // Apagar LEDs y centrar servo inmediatamente
  digitalWrite(ledizq, LOW);
  digitalWrite(ledder, LOW);
  digitalWrite(PIN_LED, LOW);
  digitalWrite(PIN_LEDPARO, HIGH);
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);

  setup_wifi();

  client.setServer(mqtt_server, 1883); //concetamos al servidor mosquitto en el puerto 1883
  client.setCallback(callback);

  //Potenciometro
  analogReadResolution(12);      // Resolución de 12 bits (0–4095)
  analogSetAttenuation(ADC_11db); // Permite leer hasta ~3.3V

  // LEDs indicadores de dirección
  pinMode(ledizq, OUTPUT);
  pinMode(ledder, OUTPUT);

  // Servo de dirección
  servoDir.attach(PIN_SERVO, 500, 2400); // min/max pulse width en microsegundos
  servoDir.write(90); // Posición inicial: recto

  //Arranque y paro
  pinMode(PIN_BOTON_ON, INPUT_PULLUP);
  pinMode(PIN_BOTON_OFF, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_LEDPARO, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  digitalWrite(PIN_LEDPARO, HIGH);

  // E-Stop: botón NC conecta GPIO 13 a GND. Al presionar o cable roto → HIGH → emergencia
  pinMode(PIN_ESTOP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ESTOP), isrEmergencia, RISING);

  // Verificar estado inicial del E-Stop (por si arranca con el botón presionado o cable roto)
  if (digitalRead(PIN_ESTOP) == HIGH) {
    emergencia = true;
  }
}

// ---------- LOOP ----------
void loop() {

  if (!client.connected()) {
    reconnect();
  }

  client.loop();

  // Si hay emergencia, mantener todo apagado y publicar estado
  if (emergencia) {
    apagarTodo();
    ledState = false;

    // Publicar emergencia solo cuando cambia el estado
    if (lastLedState != ledState) {
      lastLedState = ledState;
      client.publish("arranque/paro", "EMERGENCIA");
      Serial.println("PARO DE EMERGENCIA ACTIVADO");
    }

    // Verificar si el E-Stop fue liberado (botón NC vuelve a cerrar → GPIO LOW)
    // NO reanuda automáticamente, solo limpia el flag para permitir rearranque manual
    if (flag == false && digitalRead(PIN_ESTOP) == LOW) {
      emergencia = false;
      Serial.println("E-Stop liberado. Presione ON o envie GO para reanudar.");
    }
    return; // No ejecutar nada más mientras haya emergencia
  }

  startStop();

  //POTENCIOMETRO1
  valorADCpot1 = analogRead(pinPot1); //leemos el ADC del pot1
  voltaje1 = (valorADCpot1 * 3.3) / 4095.0; // Convertir a voltaje 0 - 3.3

  //LM35
  adcTemp = analogRead(pinLM35); // lectura del ADC DE LM35
  voltajeLM35 = (adcTemp * 5) / 4095.0; //convertimos a voltaje
  temperatura = voltajeLM35 * 100.0; //calculamos el equivalente a temperatura

  // Publicar estado arranque/paro SOLO cuando cambia
  if (ledState != lastLedState) {
    lastLedState = ledState;
    if (ledState) {
      client.publish("arranque/paro", "MOVIMIENTO");
      digitalWrite(PIN_LEDPARO, LOW);
      Serial.println("Estado: MOVIMIENTO");
    } else {
      client.publish("arranque/paro", "PARO");
      digitalWrite(PIN_LEDPARO, HIGH);
      apagarTodo();
      Serial.println("Estado: PARO");
    }
  }

  // Mantener servo centrado y LEDs apagados si el sistema está en paro
  if (!ledState) {
    apagarTodo();
  }

  long now = millis();
  if (now - lastMsg > 1000) {
    lastMsg = now;
    //enviar datos pot
    char vol1[10]; //variable tipo caracter a enviar por MQTT
    dtostrf(voltaje1, 1, 3, vol1); //convertimos el valor de voltaje a tipo char para que se pueda enviar a mosquitto
    client.publish("pot/uno", vol1); // mandamos la variable "mensaje"  al topic "pot/uno"

    //enviar datos LM35
    char temp[10]; //variable tipo caracter a enviar por MQTT
    dtostrf(temperatura, 1, 3, temp); //convertimos el valor de temperatura a tipo char para que se pueda enviar a mosquitto
    client.publish("LM35/uno", temp); // mandamos la variable "temp" al topic "LM35/uno"
  }
}

// ---------- APAGAR TODO ----------
void apagarTodo() {
  digitalWrite(ledizq, LOW);
  digitalWrite(ledder, LOW);
  servoDir.write(90); // Centrar servo (recto)
  anguloActual = 90;
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

  // E-Stop remoto desde interfaz: activa emergencia igual que el botón físico
  if (messageTemp == "EMERGENCIA") {
    emergencia = true;
    flag = true;
    apagarTodo();
    ledState = false;
    digitalWrite(PIN_LED, LOW);
    digitalWrite(PIN_LEDPARO, HIGH);
    client.publish("arranque/paro", "EMERGENCIA");
    Serial.println("PARO DE EMERGENCIA REMOTO");
    return;
  }

  // Si hay emergencia física, solo permitir liberar con "RESET_EMERGENCIA"
  if (emergencia) {
    if (messageTemp == "RESET_EMERGENCIA" && digitalRead(PIN_ESTOP) == LOW) {
      emergencia = false;
      flag = false;
      Serial.println("Emergencia reseteada desde interfaz");
    }
    return;
  }

  // ---- Control del servo de dirección (desde control difuso) ----
  if (String(topic) == "esp32/servo/control" && ledState) {
    int angulo = messageTemp.toInt();
    // Validar rango
    if (angulo < 0) angulo = 0;
    if (angulo > 180) angulo = 180;

    servoDir.write(angulo);
    anguloActual = angulo;

    // LEDs indicadores de dirección
    if (angulo < 80) {
      // Girando a la izquierda
      digitalWrite(ledizq, HIGH);
      digitalWrite(ledder, LOW);
    } else if (angulo > 100) {
      // Girando a la derecha
      digitalWrite(ledder, HIGH);
      digitalWrite(ledizq, LOW);
    } else {
      // Recto: ambos LEDs encendidos
      digitalWrite(ledizq, HIGH);
      digitalWrite(ledder, HIGH);
    }

    Serial.print("Servo -> ");
    Serial.print(angulo);
    Serial.println(" grados");
    return;
  }

  // ---- Comandos de arranque/paro desde interfaz ----
  if (!ledState) {
    apagarTodo();
    digitalWrite(PIN_LEDPARO, HIGH);
  }

  if (messageTemp == "STOP") { // PARO desde interfaz
    apagarTodo();
    ledState = false;
    digitalWrite(PIN_LED, ledState);
  }

  if (messageTemp == "GO") { // ENCENDIDO desde interfaz
    apagarTodo();
    ledState = true;
    digitalWrite(PIN_LED, ledState);
  }
}

// ---------- RECONNECT MQTT ----------
void reconnect() {
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT... ");

    if (client.connect("ESP32Client")) {
      Serial.println("conectado");
      client.subscribe("esp32/servo/control");
      client.subscribe("esp32/arranque");
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

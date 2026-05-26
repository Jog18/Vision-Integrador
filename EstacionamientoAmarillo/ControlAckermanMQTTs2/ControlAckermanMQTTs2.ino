#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

const char* ssid = "INFINITUM60B6";
const char* password = "NUPatFq39h";
const char* mqtt_server = "192.168.1.68";

WiFiClient espClient;
PubSubClient client(espClient);

long lastMsg = 0;

// Encendido y stop
const int PIN_BOTON_OFF = 4;
const int PIN_BOTON_ON  = 12;
const int PIN_LED = 2;
const int PIN_LEDPARO = 23;
bool ledState = false;
bool lastLedState = false;
bool buttonStateOn  = HIGH;
bool buttonStateOff = HIGH;
bool lastReadingOn  = HIGH;
bool lastReadingOff = HIGH;
unsigned long lastDebounceTimeOn  = 0;
unsigned long lastDebounceTimeOff = 0;
const unsigned long debounceDelay = 50;

// E-Stop (Paro de emergencia) - Boton normalmente cerrado
const int PIN_ESTOP = 13;
volatile bool emergencia = false;
bool flag = false;

// ---- Servo de direccion Ackermann (SIMETRICO) ----
const int PIN_SERVO = 5;
Servo servoDir;
const int SERVO_MIN   = 65;
const int SERVO_MAX   = 160;
const int SERVO_RECTO = 113;
int anguloActual = SERVO_RECTO;

// LEDs indicadores de direccion
const int ledizq = 16;
const int ledder = 15;

// ---- Motor (TB6612FNG) ----
const int AIN1 = 19;
const int AIN2 = 18;
const int PWMA = 26;
const int STBY = 22;
const int PWM_FREQ = 1000;
const int PWM_RESOLUTION = 8;
const int VELOCIDAD_FIJA = 50;

// Leer sensores. BATERIA
const int pinBateria = 34;
const float voltajeMaxBat = 11.3;
const float voltajeMinBat = 8;
const float factorDivisor = 5.37;
float porcentajeBat = 0.0;
// LM35
const int pinLM35 = 36;
int adcTemp = 0; 
float voltajeLM35 = 0.0;
float temperatura = 0.0;

// ---------- FUNCIONES MOTOR ----------
void motorForward(int speed) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWMA, speed);
}

void motorBackward(int speed) {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  ledcWrite(PWMA, speed);
}

void stopMotor() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWMA, 0);
}

// ---------- ISR PARO DE EMERGENCIA ----------
void IRAM_ATTR isrEmergencia() {
  static unsigned long ultimaISR = 0;
  unsigned long ahora = millis();
  if (ahora - ultimaISR < 50) return;
  ultimaISR = ahora;

  if (digitalRead(PIN_ESTOP) == LOW) return;

  emergencia = true;
  digitalWrite(ledizq, LOW);
  digitalWrite(ledder, LOW);
  digitalWrite(PIN_LED, LOW);
  digitalWrite(PIN_LEDPARO, HIGH);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);

  setup_wifi();

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // LEDs indicadores de direccion
  pinMode(ledizq, OUTPUT);
  pinMode(ledder, OUTPUT);

  // Motor
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(STBY, HIGH);

  // Servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servoDir.setPeriodHertz(50);
  servoDir.attach(PIN_SERVO, 500, 2400);
  servoDir.write(SERVO_RECTO);

  // PWM del motor
  ledcAttach(PWMA, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(PWMA, 0);

  // Arranque y paro
  pinMode(PIN_BOTON_ON, INPUT_PULLUP);
  pinMode(PIN_BOTON_OFF, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_LEDPARO, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  digitalWrite(PIN_LEDPARO, HIGH);

  // E-Stop
  pinMode(PIN_ESTOP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ESTOP), isrEmergencia, RISING);

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

  // Si hay emergencia, mantener todo apagado
  if (emergencia) {
    apagarTodo();
    ledState = false;

    if (lastLedState != ledState) {
      lastLedState = ledState;
      client.publish("arranque/paro", "EMERGENCIA");
      Serial.println("PARO DE EMERGENCIA ACTIVADO");
    }

    if (flag == false && digitalRead(PIN_ESTOP) == LOW) {
      emergencia = false;
      Serial.println("E-Stop liberado. Presione ON o envie GO para reanudar.");
    }
    return;
  }

  // --- Operacion normal ---
  startStop();

  // Publicar estado arranque/paro SOLO cuando cambia
  if (ledState != lastLedState) {
    lastLedState = ledState;
    if (ledState) {
      client.publish("arranque/paro", "MOVIMIENTO");
      digitalWrite(PIN_LEDPARO, LOW);
      motorForward(VELOCIDAD_FIJA);
      Serial.println("Estado: MOVIMIENTO (motor adelante)");
    } else {
      client.publish("arranque/paro", "PARO");
      digitalWrite(PIN_LEDPARO, HIGH);
      apagarTodo();
      Serial.println("Estado: PARO");
    }
  }

  if (!ledState) {
    apagarTodo();
  }

  long now = millis();
  if (now - lastMsg > 1000) {
    lastMsg = now;
    enviarSensores();
  }
}

// ---------- ENVIAR SENSORES ----------
void enviarSensores() {
  // BATERIA
  long sumaADC = 0;
  for (int i = 0; i < 20; i++) {
    sumaADC += analogRead(pinBateria);
    delayMicroseconds(500);
  }
  float promedioADC = sumaADC / 20.0;
  float voltajePin = promedioADC * (3.3 / 4095.0);
  float voltajeBateria = voltajePin * factorDivisor;
  porcentajeBat = (voltajeBateria - voltajeMinBat) / (voltajeMaxBat - voltajeMinBat) * 100.0;
  porcentajeBat = constrain(porcentajeBat, 0.0, 100.0);

  // LM35
  adcTemp = analogRead(pinLM35);
  voltajeLM35 = (adcTemp * 5) / 4095.0;
  temperatura = voltajeLM35 * 100.0;

  char bat[10];
  dtostrf(porcentajeBat, 1, 1, bat);
  client.publish("bateria/porcentaje", bat);

  char temp[10];
  dtostrf(temperatura, 1, 3, temp);
  client.publish("LM35/uno", temp);
}

// ---------- APAGAR TODO ----------
void apagarTodo() {
  digitalWrite(ledizq, LOW);
  digitalWrite(ledder, LOW);
  servoDir.write(SERVO_RECTO);
  anguloActual = SERVO_RECTO;
  stopMotor();
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

  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // E-Stop remoto desde interfaz
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

  // Si hay emergencia, solo permitir RESET
  if (emergencia) {
    if (messageTemp == "RESET_EMERGENCIA" && digitalRead(PIN_ESTOP) == LOW) {
      emergencia = false;
      flag = false;
      Serial.println("Emergencia reseteada desde interfaz");
    }
    return;
  }

  // ---- Control del servo de direccion (desde control difuso) ----
  if (String(topic) == "esp32/servo/control" && ledState) {
    int angulo = messageTemp.toInt();
    if (angulo < SERVO_MIN) angulo = SERVO_MIN;
    if (angulo > SERVO_MAX) angulo = SERVO_MAX;

    servoDir.write(angulo);
    anguloActual = angulo;

    if (angulo < SERVO_RECTO - 3) {
      digitalWrite(ledizq, HIGH);
      digitalWrite(ledder, LOW);
    } else if (angulo > SERVO_RECTO + 3) {
      digitalWrite(ledder, HIGH);
      digitalWrite(ledizq, LOW);
    } else {
      digitalWrite(ledizq, HIGH);
      digitalWrite(ledder, HIGH);
    }

    Serial.print("Servo -> ");
    Serial.print(angulo);
    Serial.println(" grados");
    return;
  }

  // ---- Comandos de arranque/paro ----
  if (!ledState) {
    apagarTodo();
    digitalWrite(PIN_LEDPARO, HIGH);
  }

  if (messageTemp == "STOP") {
    apagarTodo();
    ledState = false;
    digitalWrite(PIN_LED, ledState);
  }

  if (messageTemp == "GO") {
    apagarTodo();
    ledState = true;
    digitalWrite(PIN_LED, ledState);
    motorForward(VELOCIDAD_FIJA);
  }
}

// ---------- RECONNECT MQTT ----------
void reconnect() {
  while (!client.connected()) {
    Serial.print("Intentando conexion MQTT... ");

    if (client.connect("ESP32Client")) {
      Serial.println("conectado");
      client.subscribe("esp32/servo/control");
      client.subscribe("esp32/arranque");
    } else {
      Serial.print("fallo, rc=");
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
      if (buttonStateOn == LOW) {
        ledState = true;
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

      if (buttonStateOff == LOW) {
        ledState = false;
        digitalWrite(PIN_LED, ledState);
        Serial.println("LED APAGADO");
      }
    }
  }
  lastReadingOff = readingOff;
}

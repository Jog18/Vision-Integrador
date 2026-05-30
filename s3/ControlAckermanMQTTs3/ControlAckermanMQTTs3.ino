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

// LEDs indicadores de bateria
const int LED_BAT1 = 22;
const int LED_BAT2 = 16;
const int LED_BAT3 = 15;

// ---- Motor (TB6612FNG) ---- STBY puenteado a 3.3V
const int AIN1 = 19;
const int AIN2 = 18;
const int PWMA = 26;
const int PWM_FREQ = 1000;
const int PWM_RESOLUTION = 8;
const int VELOCIDAD_FIJA = 50;

// ---- Direccion del motor (s3: soporte para reversa) ----
bool enReversa = false;
bool lastEnReversa = false;

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

// Parpadeo LED PARO en emergencia
unsigned long lastBlinkTime = 0;
bool blinkState = false;

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

// ---------- ACTUALIZAR LEDs DE BATERIA ----------
void actualizarLedsBateria() {
  if (porcentajeBat >= 80.0) {
    digitalWrite(LED_BAT1, HIGH);
    digitalWrite(LED_BAT2, HIGH);
    digitalWrite(LED_BAT3, HIGH);
  } else if (porcentajeBat >= 40.0) {
    digitalWrite(LED_BAT1, HIGH);
    digitalWrite(LED_BAT2, HIGH);
    digitalWrite(LED_BAT3, LOW);
  } else {
    digitalWrite(LED_BAT1, HIGH);
    digitalWrite(LED_BAT2, LOW);
    digitalWrite(LED_BAT3, LOW);
  }
}

// ---------- ISR PARO DE EMERGENCIA ----------
void IRAM_ATTR isrEmergencia() {
  static unsigned long ultimaISR = 0;
  unsigned long ahora = millis();
  if (ahora - ultimaISR < 300) return;
  ultimaISR = ahora;

  if (digitalRead(PIN_ESTOP) == LOW) return;

  emergencia = true;
  digitalWrite(PIN_LED, LOW);
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

  // LEDs indicadores de bateria
  pinMode(LED_BAT1, OUTPUT);
  pinMode(LED_BAT2, OUTPUT);
  pinMode(LED_BAT3, OUTPUT);
  digitalWrite(LED_BAT1, LOW);
  digitalWrite(LED_BAT2, LOW);
  digitalWrite(LED_BAT3, LOW);

  // Motor
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);

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

  // Si hay emergencia, mantener motor apagado y parpadear LED PARO
  if (emergencia) {
    stopMotor();
    enReversa = false;

    if (lastLedState != ledState || ledState) {
      ledState = false;
      lastLedState = false;
      client.publish("arranque/paro", "EMERGENCIA");
      digitalWrite(PIN_LED, LOW);
      Serial.println("PARO DE EMERGENCIA ACTIVADO");
    }

    // Parpadeo del LED PARO en emergencia (300ms ON / 300ms OFF)
    unsigned long ahora = millis();
    if (ahora - lastBlinkTime >= 300) {
      lastBlinkTime = ahora;
      blinkState = !blinkState;
      digitalWrite(PIN_LEDPARO, blinkState ? HIGH : LOW);
    }

    if (flag == false && digitalRead(PIN_ESTOP) == LOW) {
      emergencia = false;
      blinkState = false;
      digitalWrite(PIN_LEDPARO, HIGH);
      Serial.println("E-Stop liberado. Presione ON o envie GO para reanudar.");
    }
    return;
  }

  // --- Operacion normal ---
  startStop();

  // Publicar estado y activar motor cuando cambia ledState o enReversa
  if (ledState != lastLedState || enReversa != lastEnReversa) {
    lastLedState = ledState;
    lastEnReversa = enReversa;
    if (ledState) {
      client.publish("arranque/paro", "MOVIMIENTO");
      digitalWrite(PIN_LEDPARO, LOW);
      if (enReversa) {
        motorBackward(VELOCIDAD_FIJA);
        Serial.println("Estado: MOVIMIENTO (motor reversa)");
      } else {
        motorForward(VELOCIDAD_FIJA);
        Serial.println("Estado: MOVIMIENTO (motor adelante)");
      }
    } else {
      client.publish("arranque/paro", "PARO");
      digitalWrite(PIN_LEDPARO, HIGH);
      stopMotor();
      Serial.println("Estado: PARO");
    }
  }

  // Seguridad: mantener motor apagado si no esta en movimiento
  if (!ledState) {
    stopMotor();
  }

  long now = millis();
  if (now - lastMsg > 1000) {
    lastMsg = now;
    enviarSensores();
    actualizarLedsBateria();
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
  servoDir.write(SERVO_RECTO);
  anguloActual = SERVO_RECTO;
  enReversa = false;
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
    stopMotor();
    enReversa = false;
    ledState = false;
    digitalWrite(PIN_LED, LOW);
    client.publish("arranque/paro", "EMERGENCIA");
    Serial.println("PARO DE EMERGENCIA REMOTO");
    return;
  }

  // Si hay emergencia, solo permitir RESET
  if (emergencia) {
    if (messageTemp == "RESET_EMERGENCIA" && digitalRead(PIN_ESTOP) == LOW) {
      emergencia = false;
      flag = false;
      blinkState = false;
      digitalWrite(PIN_LEDPARO, HIGH);
      Serial.println("Emergencia reseteada desde interfaz");
    }
    return;
  }

  // ---- Control del servo de direccion (desde control difuso o maniobra) ----
  if (String(topic) == "esp32/servo/control" && ledState) {
    int angulo = messageTemp.toInt();
    if (angulo < SERVO_MIN) angulo = SERVO_MIN;
    if (angulo > SERVO_MAX) angulo = SERVO_MAX;

    servoDir.write(angulo);
    anguloActual = angulo;

    Serial.print("Servo -> ");
    Serial.print(angulo);
    Serial.println(" grados");
    return;
  }

  // ---- Comandos de arranque/paro/reversa ----
  if (messageTemp == "STOP") {
    ledState = false;
    enReversa = false;
    digitalWrite(PIN_LED, LOW);
  }

  if (messageTemp == "GO") {
    enReversa = false;
    ledState = true;
    digitalWrite(PIN_LED, HIGH);
  }

  if (messageTemp == "REVERSE") {
    enReversa = true;
    ledState = true;
    digitalWrite(PIN_LED, HIGH);
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
        enReversa = false;
        ledState = true;
        digitalWrite(PIN_LED, ledState);
        Serial.println("LED ENCENDIDO");
      }
    }
  }
  lastReadingOn = readingOn;

  // Boton OFF con doble lectura para filtrar ruido en GPIO4
  bool readingOff = digitalRead(PIN_BOTON_OFF);
  if (readingOff != lastReadingOff) {
    lastDebounceTimeOff = millis();
  }
  if ((millis() - lastDebounceTimeOff) > debounceDelay) {
    if (readingOff != buttonStateOff) {
      // Confirmar con segunda lectura para evitar falsos disparos
      delay(5);
      bool confirmacion = digitalRead(PIN_BOTON_OFF);
      if (confirmacion == readingOff && readingOff == LOW) {
        buttonStateOff = readingOff;
        enReversa = false;
        ledState = false;
        digitalWrite(PIN_LED, ledState);
        Serial.println("LED APAGADO");
      } else if (confirmacion == readingOff) {
        buttonStateOff = readingOff;
      }
    }
  }
  lastReadingOff = readingOff;
}

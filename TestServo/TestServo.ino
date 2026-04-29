/*
 * TestServo.ino — Prueba de limites del servo para carro Ackermann
 *
 * Recibe angulos desde la Raspberry (MQTT topic: esp32/servo/control)
 * y tambien permite control manual por Serial Monitor.
 *
 * Comandos por Serial Monitor (115200 baud):
 *   <numero>    -> Mover servo a ese angulo (0-180)
 *   +           -> Incrementar 1 grado
 *   -           -> Decrementar 1 grado
 *   s           -> Sweep: barrido lento de 0 a 180 (para encontrar limites)
 *   r           -> Sweep en rango actual (SERVO_MIN a SERVO_MAX)
 *   c           -> Centrar servo (ir a 113)
 *   ?           -> Mostrar angulo actual
 *
 * Proceso para encontrar limites mecanicos:
 *   1. Subir este sketch al ESP32
 *   2. Abrir Serial Monitor a 115200
 *   3. Enviar 's' para iniciar barrido completo (0-180, 1 grado cada 300ms)
 *   4. Observar el carro: anotar donde las ruedas empiezan a trabarse
 *   5. Usar '+' y '-' para ajustar con precision de 1 grado
 *   6. Anotar el angulo minimo y maximo que no traban las ruedas
 *   7. Esos son tus SERVO_MIN y SERVO_MAX reales
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

// ---- WiFi y MQTT ----
const char* ssid = "A52 de Roberto";
const char* password = "123456789";
const char* mqtt_server = "10.249.23.191";

WiFiClient espClient;
PubSubClient client(espClient);

// ---- Servo ----
const int PIN_SERVO = 5;
Servo servoDir;
int anguloActual = 113;  // posicion inicial (recto)

// Rango actual (ajustar despues de las pruebas)
int SERVO_MIN = 80;
int SERVO_MAX = 135;
const int SERVO_RECTO = 113;

// ---- Sweep ----
bool sweepActivo = false;
int sweepDesde = 0;
int sweepHasta = 180;
int sweepPos = 0;
int sweepDir = 1;  // 1 = subiendo, -1 = bajando
unsigned long sweepLastTime = 0;
const int SWEEP_DELAY = 300;  // ms entre cada grado

// ---- Funciones ----

void moverServo(int angulo) {
  if (angulo < 0) angulo = 0;
  if (angulo > 180) angulo = 180;
  servoDir.write(angulo);
  anguloActual = angulo;

  Serial.print("Servo -> ");
  Serial.print(angulo);
  Serial.print(" grados");

  // Indicar posicion relativa al centro
  if (angulo < SERVO_RECTO) {
    Serial.print("  [IZQUIERDA, ");
    Serial.print(SERVO_RECTO - angulo);
    Serial.println(" del centro]");
  } else if (angulo > SERVO_RECTO) {
    Serial.print("  [DERECHA, +");
    Serial.print(angulo - SERVO_RECTO);
    Serial.println(" del centro]");
  } else {
    Serial.println("  [CENTRO]");
  }

  // Publicar angulo por MQTT para que la Raspberry lo vea
  if (client.connected()) {
    char msg[10];
    itoa(angulo, msg, 10);
    client.publish("esp32/servo/feedback", msg);
  }
}

void iniciarSweep(int desde, int hasta) {
  sweepActivo = true;
  sweepDesde = desde;
  sweepHasta = hasta;
  sweepPos = desde;
  sweepDir = 1;
  sweepLastTime = millis();
  moverServo(sweepPos);
  Serial.println("=== SWEEP INICIADO ===");
  Serial.print("Rango: ");
  Serial.print(desde);
  Serial.print(" -> ");
  Serial.println(hasta);
  Serial.println("Enviar cualquier comando para detener el sweep");
}

void procesarSweep() {
  if (!sweepActivo) return;
  if (millis() - sweepLastTime < SWEEP_DELAY) return;

  sweepLastTime = millis();
  sweepPos += sweepDir;

  if (sweepPos >= sweepHasta) {
    sweepDir = -1;
  } else if (sweepPos <= sweepDesde) {
    sweepActivo = false;
    Serial.println("=== SWEEP COMPLETADO ===");
    Serial.println("Usa '+'/'-' para ajuste fino o envia un angulo directo");
    return;
  }

  moverServo(sweepPos);
}

// ---- WiFi ----
void setup_wifi() {
  delay(10);
  Serial.print("Conectando a WiFi ");
  Serial.print(ssid);
  WiFi.begin(ssid, password);

  int intentos = 0;
  while (WiFi.status() != WL_CONNECTED && intentos < 20) {
    delay(500);
    Serial.print(".");
    intentos++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" OK");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(" FALLO (continuando sin WiFi)");
  }
}

// ---- MQTT Callback ----
void callback(char* topic, byte* message, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)message[i];
  }

  Serial.print("[MQTT] ");
  Serial.print(topic);
  Serial.print(" -> ");
  Serial.println(msg);

  if (String(topic) == "esp32/servo/control") {
    sweepActivo = false;  // detener sweep si estaba activo
    int angulo = msg.toInt();
    moverServo(angulo);
  }
}

void reconnect() {
  if (!client.connected()) {
    Serial.print("Conectando MQTT... ");
    if (client.connect("ESP32_TestServo")) {
      Serial.println("OK");
      client.subscribe("esp32/servo/control");
    } else {
      Serial.print("fallo (rc=");
      Serial.print(client.state());
      Serial.println(")");
    }
  }
}

// ---- SETUP ----
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("================================");
  Serial.println(" TEST SERVO - Limites Ackermann");
  Serial.println("================================");
  Serial.println();
  Serial.println("Comandos:");
  Serial.println("  <numero>  Mover a angulo (0-180)");
  Serial.println("  +         +1 grado");
  Serial.println("  -         -1 grado");
  Serial.println("  s         Sweep completo (0-180)");
  Serial.println("  r         Sweep en rango (80-135)");
  Serial.println("  c         Centrar (113)");
  Serial.println("  ?         Mostrar angulo actual");
  Serial.println();

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  // Servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servoDir.setPeriodHertz(50);
  servoDir.attach(PIN_SERVO, 500, 2400);

  // Posicion inicial: centro
  moverServo(SERVO_RECTO);
  Serial.println("Servo centrado en 113. Listo para pruebas.");
  Serial.println();
}

// ---- LOOP ----
void loop() {
  // MQTT
  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
  }

  // Sweep automatico
  procesarSweep();

  // Comandos por Serial
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() == 0) return;

    // Detener sweep con cualquier comando
    if (sweepActivo && input != "s" && input != "r") {
      sweepActivo = false;
      Serial.println("Sweep detenido.");
    }

    if (input == "+") {
      moverServo(anguloActual + 1);
    }
    else if (input == "-") {
      moverServo(anguloActual - 1);
    }
    else if (input == "s") {
      iniciarSweep(0, 180);
    }
    else if (input == "r") {
      iniciarSweep(SERVO_MIN, SERVO_MAX);
    }
    else if (input == "c") {
      moverServo(SERVO_RECTO);
    }
    else if (input == "?") {
      Serial.print("Angulo actual: ");
      Serial.print(anguloActual);
      Serial.print(" | Rango config: [");
      Serial.print(SERVO_MIN);
      Serial.print(", ");
      Serial.print(SERVO_MAX);
      Serial.print("] | Centro: ");
      Serial.println(SERVO_RECTO);
    }
    else {
      // Intentar interpretar como numero
      int angulo = input.toInt();
      if (angulo > 0 || input == "0") {
        moverServo(angulo);
      } else {
        Serial.println("Comando no reconocido. Usa ? para ayuda.");
      }
    }
  }
}

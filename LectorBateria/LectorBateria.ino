const int pinBateria = 34; // Pin ADC para leer voltaje de batería
const float voltajeMaximo = 12.6; // Voltaje máximo típico batería 12V Li-ion (3 celdas)
const float voltajeMinimo = 9.0;  // Voltaje mínimo seguro batería 12V Li-ion
const float factorDivisor = 5.0;  // Divisor de voltaje: (30k + 7.5k) / 7.5k = 5.0

void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // Resolución ADC 12 bits
}

void loop() {
  long sumaADC = 0;
  for(int i = 0; i < 20; i++) {
    sumaADC += analogRead(pinBateria);
    delay(5);
  }
  float promedioADC = sumaADC / 20.0;

  float voltajePin = promedioADC * (3.3 / 4095.0);
  float voltajeBateria = voltajePin * factorDivisor;

  float porcentaje = (voltajeBateria - voltajeMinimo) / (voltajeMaximo - voltajeMinimo) * 100;
  porcentaje = constrain(porcentaje, 0, 100);

  Serial.printf("Voltaje pin: %.2f V | Voltaje batería: %.2f V | Estado: ", voltajePin, voltajeBateria);

  if (porcentaje > 80) {
    Serial.printf("Alta (%.1f%%)\n", porcentaje);
  } else if (porcentaje > 50) {
    Serial.printf("Media (%.1f%%)\n", porcentaje);
  } else if (porcentaje > 20) {
    Serial.printf("Baja (%.1f%%)\n", porcentaje);
  } else {
    Serial.printf("Muy baja - Cargar batería (%.1f%%)\n", porcentaje);
  }

  delay(5000);
}
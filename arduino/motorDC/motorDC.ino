#include <Arduino.h>

// Configuración del PWM para la velocidad
const int PWM_CHANNEL = 0;         // Canal PWM 
const int MOTOR_PWM_PIN = 32;        // Pin de salida PWM al puente H
const int PWM_FREQUENCY = 5000;      // Frecuencia PWM en Hz 
const int PWM_RESOLUTION = 8;        // Resolución de 8 bits (0-255)

// Pines para controlar la dirección en el puente H
const int DIR_PIN1 = 25;             // Primer pin de dirección
const int DIR_PIN2 = 26;             // Segundo pin de dirección

// Variable para almacenar el valor de velocidad recibido (0-255)
int speedValue = 0;

void setup() {
  Serial.begin(115200);
  delay(1000); // Espera breve para la conexión Serial

  // Configurar el canal PWM
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_PWM_PIN, PWM_CHANNEL);

  // Configurar los pines de dirección como salidas
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);

  // Fijar la dirección predeterminada (por ejemplo, avance)
  // Para avanzar: DIR_PIN1 HIGH y DIR_PIN2 LOW
  digitalWrite(DIR_PIN1, HIGH);
  digitalWrite(DIR_PIN2, LOW);

  Serial.println("ESP32 listo para controlar el motor DC con puente H...");
}

void loop() {
  // Revisar si hay datos disponibles por Serial
  if (Serial.available() > 0) {
    // Leer el valor (se espera que sea un número en formato ASCII terminado en salto de línea)
    speedValue = Serial.parseInt();

    // Limitar el valor entre 0 y 255 (por seguridad)
    if (speedValue < 0) speedValue = 0;
    if (speedValue > 255) speedValue = 255;

    // Controlar la dirección y velocidad:
    if (speedValue == 0) {
      // Detener el motor: PWM a 0
      ledcWrite(PWM_CHANNEL, 0);
    } else {
      // Para avanzar: 
      digitalWrite(DIR_PIN1, HIGH);
      digitalWrite(DIR_PIN2, LOW);
      ledcWrite(PWM_CHANNEL, speedValue);
    }

    // Imprimir en Monitor Serie para depuración
    Serial.print("Velocidad (PWM) enviada: ");
    Serial.println(speedValue);
  }
}

#include <Arduino.h>
#include <ESP32Servo.h>  // Librería específica para ESP32

Servo myServo;

// Definir el pin para el servo y los pulsos mínimo/máximo 
const int SERVO_PIN = 32;
const int SERVO_MIN_PULSE = 500;   // Pulso mínimo en microsegundos 
const int SERVO_MAX_PULSE = 2400;  // Pulso máximo en microsegundos 

void setup() {
  Serial.begin(115200);
  delay(1000);  // Espera para que se inicie la comunicación Serial

  // Adjuntar el servo al pin 32 con los pulsos especificados
  myServo.attach(SERVO_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);

  Serial.println("ESP32 listo para mover el servo en el pin 32...");
}

void loop() {
  // Verificar si hay datos disponibles en el puerto Serial
  if (Serial.available() > 0) {
    // Leer el ángulo enviado por Python
    int angleValue = Serial.parseInt();

    // Validar que el ángulo esté en el rango de 0 a 180 grados
    if (angleValue >= 0 && angleValue <= 180) {
      myServo.write(angleValue);
      Serial.print("Servo movido a: ");
      Serial.println(angleValue);
    }
  }
}

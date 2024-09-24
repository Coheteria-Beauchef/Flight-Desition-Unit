#include <Wire.h>
#include <MPU6050.h>

int cal;
MPU6050 mpu;

void setup() {


  pinMode(13, OUTPUT);  // Pin 13 como salida
  Serial.begin(9600);
  
  Wire.begin();
  mpu.initialize();

  Serial.println("Don t move calibration is on");
  delay(300);
  cal = mpu.getAccelerationZ();
  Serial.println(cal);

}

void loop() {

  int16_t gyroX = mpu.getRotationX();
  int16_t gyroY = mpu.getRotationY();
  int16_t gyroZ = mpu.getRotationZ();
  
  float dt = 0.1;  // Intervalo de tiempo en segundos (10ms)
  static float angleX = 0, angleY = 0, angleZ = 0;  // Acumulación de ángulos

  // Integrar los valores de giroscopio para obtener los ángulos
  angleX += gyroX * dt / 131;  // 131: sensibilidad para giroscopio configurado a 250°/s
  angleY += gyroY * dt / 131;
  angleZ += gyroZ * dt / 131;
  
  // Leer aceleraciones y convertir a metros por segundo cuadrado
  float accelX = mpu.getAccelerationX() * 9.81 / cal;
  float accelY = mpu.getAccelerationY() * 9.81 / cal;
  float accelZ = mpu.getAccelerationZ() * 9.81 / cal;

  // Magnitud total de la aceleración
  float accel = sqrt(pow(accelX, 2) + pow(accelY, 2) + pow(accelZ, 2));
  
  // Calcular Pitch y Roll
  float pitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * (180.0 / PI);
  float roll = atan2(accelX, sqrt(accelY * accelY + accelZ * accelZ)) * (180.0 / PI);

   /*
  Serial.print("Posición: ");
  Serial.print("Pitch = ");
  Serial.print(pitch);
  Serial.print("°   Roll = ");
  Serial.print(roll);
  Serial.println("°");
  
  Serial.print("Ángulo: ");
  Serial.print("X = ");
  Serial.print(angleX);
  Serial.print("°   Y = ");
  Serial.print(angleY);
  Serial.print("°   Z = ");
  Serial.print(angleZ);
  Serial.println("°");
  */


  // Codigo  para mostrar las aceleraciones en todos los ejes

  /*
  Serial.print("Ax: ");
  Serial.print(accelX);
  Serial.print("  Ay:  ");
  Serial.print(accelY);
  Serial.print("  Az:  ");
  Serial.print(accelZ);
  Serial.print("  At: ");
  Serial.println(accel);
  */

  // Condiciones para abrir paracaídas

  if (9.81 - accelZ < -0.5) {
    Serial.print(accelZ);
    Serial.println(" (caida) Abrir paracaidas");
    digitalWrite(13, HIGH);  // Acción a tomar
  }
  else if (abs(pitch) > 25) {
    Serial.print(pitch);
    Serial.println("  desvio (pitch) Abrir paracaidas");
    digitalWrite(13, HIGH);
  }
  else if (abs(roll) > 25) {
    Serial.print(roll);
    Serial.println("  desvio (roll) Abrir paracaidas");
    digitalWrite(13, HIGH);
  }
  else {
    digitalWrite(13, LOW);
    Serial.print("ok  pitch: ");
    Serial.print(pitch);
    Serial.print("  roll:  ");
    Serial.print(roll);
    Serial.print("  Az:  ");
    Serial.println(9.81-accelZ);
  }


  delay(dt * 1000);  // Esperar el tiempo definido por dt antes de la siguiente lectura
}





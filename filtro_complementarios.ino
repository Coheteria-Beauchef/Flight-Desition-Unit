#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;
unsigned long previousTime = 0;
float gyroXrate, angleX = 0;
float gyroYrate, angleY = 0;
float gyroXOffset = 0;
float gyroYOffset = 0;
float ax, ay, az = 0;
float alpha = 0.996;

// Función para calcular los offsets del giroscopio
void calibrateGyro() {
  int numReadings = 1000;
  long gyroXSum = 0;
  long gyroYSum = 0;

  Serial.println("Calibrando giroscopio...");
  
  for (int i = 0; i < numReadings; i++) {
    int16_t gyroXRaw, gyroYRaw, gyroZRaw;
    mpu.getRotation(&gyroXRaw, &gyroYRaw, &gyroZRaw);

    // Sumar lecturas del giroscopio en X
    gyroXSum += gyroXRaw;
    gyroYSum += gyroYRaw;

    delay(3);  // Pequeño retardo entre lecturas
  }

  // Calcular el promedio de las lecturas
  gyroXOffset = gyroXSum / numReadings;
  gyroYOffset = gyroYSum / numReadings;

  Serial.print("Offset del giroscopio en X: ");
  Serial.println(gyroXOffset);
}

void setup() {
  Wire.begin();
  Serial.begin(115200);

  // Inicializar el MPU6050
  mpu.initialize();

  // Calibrar giroscopio para obtener los offsets
  calibrateGyro();
}

void loop() {
  // Tiempo actual en milisegundos
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0; // Tiempo entre lecturas en segundos

  // Leer las velocidades angulares del giroscopio
  int16_t gyroXRaw, gyroYRaw, gyroZRaw;
  int16_t axr, ayr, azr;
  mpu.getRotation(&gyroXRaw, &gyroYRaw, &gyroZRaw);
  mpu.getAcceleration(&axr, &ayr, &azr);

  // Compensar el offset del giroscopio
  gyroXRaw -= gyroXOffset;
  gyroYRaw -= gyroYOffset;

  // Convertir la lectura cruda del giroscopio a grados/segundo
  gyroXrate = gyroXRaw / 131; // 131.0 para un rango de 250 grados/segundo (predeterminado)
  gyroYrate = gyroYRaw / 131;


  // Integrar para obtener el ángulo en el eje X
  angleX += gyroXrate * deltaTime;
  angleY += gyroYrate * deltaTime;

  ax = axr* 9.81 / 17480;
  ay = ayr * 9.81 / 17480;
  az = azr * 9.81 / 17480;

  float pitch = atan2(ay, sqrt(ax * ax + az * az)) * (180.0 / PI);
  float roll = atan2(ax, sqrt(ay * ay + az * az)) * (180.0 / PI);

  float angleXc=  alpha*(angleXc + angleX) + (1-alpha)*pitch;
  float angleYc=  alpha*(angleYc + angleY) + (1-alpha)*roll;
  Serial.print(angleX);Serial.print("\t");Serial.print(pitch); Serial.print("\t");Serial.print(angleXc);Serial.print("\t");Serial.print(angleY);Serial.print("\t");Serial.print(roll);Serial.print("\t");Serial.println(angleYc);

 
  // Actualizar el tiempo anterior
  previousTime = currentTime;

  delay(10); // Pequeño retardo para suavizar la salida
}





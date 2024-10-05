#include <Wire.h>
#include <MPU6050.h>
#include <MadgwickAHRS.h>

// Inicializar el sensor MPU6050 y el filtro de Madgwick
MPU6050 mpu;
Madgwick filter;

// Variables para almacenar los ángulos
float roll, pitch, yaw;

// Variable para el vector de aceleración
float a_cohete[3]; // [ax, ay, az]
float a_tierra[3]; // [ax, ay, az] transformado

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Inicializar el MPU6050
  mpu.initialize();
  filter.begin(100); // Inicializa el filtro a 100 Hz
}

void loop() {
  // Leer datos del MPU6050
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Convertir los valores de aceleración a g (1 g = 9.81 m/s²)
  a_cohete[0] = ax / 17548.0; // Factor de escala para MPU6050
  a_cohete[1] = ay / 17548.0;
  a_cohete[2] = az / 17548.0;

  // Actualizar el filtro de Madgwick (sin magnetómetro)
  filter.update(float(gx) / 131.0, float(gy) / 131.0, float(gz) / 131.0, 
                a_cohete[0], a_cohete[1], a_cohete[2], 0,0,0);

  // Obtener los ángulos de Euler
  roll = filter.getRoll();
  pitch = filter.getPitch();
  yaw = filter.getYaw();

  // Calcular la matriz de rotación
  float R[3][3]; // Matriz de rotación 3x3
  float cos_roll = cos(radians(roll));
  float sin_roll = sin(radians(roll));
  float cos_pitch = cos(radians(pitch));
  float sin_pitch = sin(radians(pitch));
  float cos_yaw = cos(radians(yaw));
  float sin_yaw = sin(radians(yaw));

  // Matriz de rotación
  R[0][0] = cos_pitch * cos_yaw;
  R[0][1] = cos_pitch * sin_yaw;
  R[0][2] = -sin_pitch;
  R[1][0] = sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw;
  R[1][1] = sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw;
  R[1][2] = sin_roll * cos_pitch;
  R[2][0] = cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw;
  R[2][1] = cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw;
  R[2][2] = cos_roll * cos_pitch;

  // Transformar el vector de aceleración
  a_tierra[0] = R[0][0] * a_cohete[0] + R[0][1] * a_cohete[1] + R[0][2] * a_cohete[2];
  a_tierra[1] = R[1][0] * a_cohete[0] + R[1][1] * a_cohete[1] + R[1][2] * a_cohete[2];
  a_tierra[2] = R[2][0] * a_cohete[0] + R[2][1] * a_cohete[1] + R[2][2] * a_cohete[2];

  // Imprimir los resultados
  Serial.print(a_tierra[0] * 9.81); // Convertir a m/s²
  Serial.print(", ");
  Serial.print(a_tierra[1] * 9.81);
  Serial.print(", ");
  Serial.print(a_tierra[2] * 9.81);
  Serial.println();

  delay(100); // Espera un poco antes de la siguiente lectura
}



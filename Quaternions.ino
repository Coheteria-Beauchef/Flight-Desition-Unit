//
// MPU-6050 Mahony IMU  (yaw angle is relative to starting orientation)
// last update 7/9/2020 
//

#include "Wire.h"

// Dirección I2C del MPU-6050
#define MPU_ADDR 0x68

// Offset y factores de escala para el acelerómetro y el giroscopio
float A_cal[6] = {265.0, -80.0, -700.0, 0.994, 1.000, 1.014}; // 0..2 offset xyz, 3..5 scale xyz
float G_off[3] = { -499.5, -17.7, -82.0}; // offsets para el giroscopio
#define G_SCALE ((250.0 / 32768.0) * (PI / 180.0)) // escala para giroscopio a rad/s

// Filtros de Mahony
float q[4] = {1.0, 0.0, 0.0, 0.0}; // vector quaternion
float Kp = 30.0; // ganancia proporcional
float Ki = 0.0; // ganancia integral

// Variables globales
unsigned long now_ms, last_ms = 0; // temporizadores de millis
unsigned long print_ms = 200; // intervalo de impresión
float yaw, pitch, roll; // salida de ángulos Euler

void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println("Iniciando...");

  // Inicializar el sensor
  wakeUpMPU();
}

// Función para despertar el MPU-6050
void wakeUpMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // Registro PWR_MGMT_1
  Wire.write(0);     // Despertar el MPU-6050
  Wire.endTransmission(true);
}

// Función para leer los datos del sensor
void readSensorData(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz, int16_t &Tmp) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // Registro de inicio ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // Leer 14 registros
  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();
}

// Bucle principal
void loop() {
  static unsigned long last = 0; // temporizador de micros
  static float deltat = 0;  // tiempo de bucle en segundos
  int16_t ax, ay, az, gx, gy, gz, Tmp; // datos crudos

  readSensorData(ax, ay, az, gx, gy, gz, Tmp); // leer datos del sensor

  // Escalar y aplicar offsets
  float Axyz[3] = { (float) ax, (float) ay, (float) az };
  float Gxyz[3] = { ((float) gx - G_off[0]) * G_SCALE, ((float) gy - G_off[1]) * G_SCALE, ((float) gz - G_off[2]) * G_SCALE };

  // Aplicar offsets y factores de escala al acelerómetro
  for (int i = 0; i < 3; i++) {
    Axyz[i] = (Axyz[i] - A_cal[i]) * A_cal[i + 3];
  }

  unsigned long now = micros();
  deltat = (now - last) * 1.0e-6; // segundos desde la última actualización
  last = now;

  Mahony_update(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat);

  // Calcular ángulos Tait-Bryan
  roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
  pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
  yaw   = -atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - (q[2] * q[2] + q[3] * q[3]));

  // Convertir a grados
  yaw   = yaw * 180.0 / PI;
  pitch = pitch * 180.0 / PI;
  roll  = roll * 180.0 / PI;

  if (yaw < 0) yaw += 360.0; // círculo de brújula

  now_ms = millis(); // tiempo para imprimir
  if (now_ms - last_ms >= print_ms) {
    last_ms = now_ms;
    // Imprimir ángulos
    Serial.print(yaw, 0);
    Serial.print(", ");
    Serial.print(pitch, 0);
    Serial.print(", ");
    Serial.println(roll, 0);
  }
}

// Función de actualización del filtro Mahony
void Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat) {
  float recipNorm;
  float vx, vy, vz;
  float ex, ey, ez;  // términos de error
  float qa, qb, qc;
  static float ix = 0.0, iy = 0.0, iz = 0.0;  // términos de retroalimentación integral

  // Comprobar si las medidas del acelerómetro son válidas
  float tmp = ax * ax + ay * ay + az * az;
  if (tmp > 0.0) {
    // Normalizar acelerómetro
    recipNorm = 1.0 / sqrt(tmp);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Dirección estimada de la gravedad en el marco del cuerpo
    vx = q[1] * q[3] - q[0] * q[2];
    vy = q[0] * q[1] + q[2] * q[3];
    vz = q[0] * q[0] - 0.5f + q[3] * q[3];

    // Error entre dirección estimada y medida de la gravedad
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    // Aplicar retroalimentación integral si está habilitada
    if (Ki > 0.0f) {
      ix += Ki * ex * deltat;  // error integral escalado por Ki
      iy += Ki * ey * deltat;
      iz += Ki * ez * deltat;
      gx += ix;  // aplicar retroalimentación integral
      gy += iy;
      gz += iz;
    }

    // Aplicar retroalimentación proporcional al término del giroscopio
    gx += Kp * ex;
    gy += Kp * ey;
    gz += Kp * ez;
  }

  // Integrar la tasa de cambio del quaternion
  deltat = 0.5 * deltat;
  gx *= deltat;   // pre-multiplicar factores comunes
  gy *= deltat;
  gz *= deltat;
  qa = q[0];
  qb = q[1];
  qc = q[2];
  q[0] += (-qb * gx - qc * gy - q[3] * gz);
  q[1] += (qa * gx + qc * gz - q[3] * gy);
  q[2] += (qa * gy - qb * gz + q[3] * gx);
  q[3] += (qa * gz + qb * gy - qc * gx);

  // Renormalizar quaternion
  recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] *= recipNorm;
  q[1] *= recipNorm;
  q[2] *= recipNorm;
  q[3] *= recipNorm;
}



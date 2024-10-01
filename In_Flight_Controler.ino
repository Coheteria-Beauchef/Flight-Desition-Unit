/*
Idea utilizar un sensor de presion para detectar la variación de la presion que
cuandos sea nativa se detecta la caida  
*/

#include <Wire.h>
#include <MPU6050.h>

#define MPU6050_ADDR 0x68  // Dirección I2C del MPU6050
#define MAX_RETRIES 5       // Número máximo de intentos
MPU6050 mpu;


float gravityX = 0, gravityY = 0, gravityZ = 9.81; //
unsigned long previousTime = 0;
float azl, axl, ayl = 0;
float gyroXrate, angleX = 0;
float gyroYrate, angleY = 0;
float gyroXOffset = 0;
float gyroYOffset = 0;
float ax, ay, az = 0;
float alpha = 0.996; // Factor de suavizado
int cal;


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

  pinMode(2, OUTPUT);  // Pin 13 como salida
  pinMode(3, OUTPUT);
  Serial.begin(115200);
  Serial.println("Starting...");
  
  Wire.begin();
  bool connected = false;

    for (int i = 0; i < MAX_RETRIES; i++) {
        Wire.beginTransmission(MPU6050_ADDR);
        if (Wire.endTransmission() == 0) {
            connected = true;
            break; // Salir si está conectado
        }
        delay(100); // Espera un poco antes de reintentar
    }

    if (connected) {
        Serial.println("MPU6050 conectado correctamente");
    } else {
        Serial.println("No se pudo conectar al MPU6050");
        while(1);
    }

  mpu.initialize();

  Serial.println("Don t move calibration is on");
  calibrateGyro(); 
  cal = mpu.getAccelerationZ();
  Serial.println(cal);
  delay(300);

}

void loop() {

  unsigned long currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0; // Tiempo entre lecturas en segundos

  int16_t gyroXRaw, gyroYRaw, gyroZRaw;
  int16_t axr, ayr, azr ;
  

  mpu.getRotation(&gyroXRaw, &gyroYRaw, &gyroZRaw);
  mpu.getAcceleration(&axr, &ayr, &azr);

  // Compensar el offset del giroscopio
  gyroXRaw -= gyroXOffset;
  gyroYRaw -= gyroYOffset;

  // Convertir la lectura cruda del giroscopio a grados/segundo
  gyroXrate = gyroXRaw / 131; // 131.0 para un rango de 250 grados/segundo (predeterminado)
  gyroYrate = gyroYRaw / 131;

 
  static float angleX = 0, angleY = 0, angleZ = 0;  // Acumulación de ángulos

  // Integrar los valores de giroscopio para obtener los ángulos
  angleX += gyroXrate * deltaTime;
  angleY += gyroYrate * deltaTime;

  // Leer aceleraciones y convertir a metros por segundo cuadrado
  
  
  float ax = axr* 9.81 / cal;
  float ay = ayr * 9.81 / cal;
  float az = azr * 9.81 / cal;

  gravityX = alpha * gravityX + (1 - alpha) * ax;
  gravityY = alpha * gravityY + (1 - alpha) * ay;
  gravityZ = alpha * gravityZ + (1 - alpha) * az;

  float vz= vz + (gravityZ-az)*deltaTime;
  float altitude = altitude + vz*deltaTime;


  // Magnitud total de la aceleración

  // Calcular el angulos angulos con la aceleracion
  float gyroAccelX = atan2(ay, sqrt(ax * ax + az * az)) * (180.0 / PI);
  float gyroAccelY = atan2(ax, sqrt(ay * ay + az * az)) * (180.0 / PI);

  //angulos calculados con filtro complementario
  float pitch=  alpha*(pitch + angleX) + (1-alpha)*gyroAccelX;
  float roll=  alpha*(roll + angleY) + (1-alpha)*gyroAccelY;

  //sistema linealizado
  azl = gravityZ -az;
  ayl = ay - gravityY;
  axl = ax - gravityX;

  // Condiciones para abrir paracaídas

  if (azl < -0.9) {
    Serial.print(az - gravityZ);
    Serial.println(" (caida) Abrir paracaidas");
    digitalWrite(2, HIGH);  // Acción a tomar
  }
  else if (abs(pitch) > 20) {
    Serial.print(pitch);Serial.println("  desvio (pitch) Abrir paracaidas");
    digitalWrite(3, HIGH);
  }
  else if (abs(roll) > 20) {
    Serial.print(roll);Serial.println("  desvio (roll) Abrir paracaidas");
    digitalWrite(3, HIGH);
  }
  else {
    digitalWrite(2, LOW);digitalWrite(3, LOW);
    Serial.print("ok  pitch: ");  Serial.print(pitch); Serial.print("  roll:  "); Serial.print(roll);Serial.print("  Az:  ");   Serial.println(gravityZ-az);
  }


  //datos para la interfaz

 /*
 String a =  String(gyroY * PI /180) + "|" + String(roll) + "|" + String(pitch) + "|" + 
             String(altitude) + "|" + String(gravityZ) + "|" + 
             String(gravityZ - az) + "|" + String(vz) + "|0";
  Serial.println(a);
  */

  // Actualizar el tiempo anterior
  previousTime = currentTime;
  delay(10);  // Esperar el tiempo definido por dt antes de la siguiente lectura

}


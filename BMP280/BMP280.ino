#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp;

float previousAltitude = 0;
/*
el valor para configuracion en m/s para detectar la caida debe ser ser multiplicada por la frecuencia de muestreo del sensor
*/
const float descentThreshold = -0.32; // Umbral de velocidad de descenso en m

void setup() {
  Serial.begin(115200);
  if (!bmp.begin(0x76)) {
    Serial.println("No se pudo encontrar el sensor BMP280.");
    while (1);
  }
  previousAltitude = bmp.readAltitude(1013.25);
}

void loop() {
  float currentAltitude = bmp.readAltitude(1013.25);

  // Cálculo del cambio de altura y tiempo
  float altitudeDrift = (currentAltitude - previousAltitude);// Deriva de altura en m/s

  // Mostrar la altitud y la deriva de altura en el monitor serie
  Serial.println(altitudeDrift);

  // Detectar caída si la deriva es menor que el umbral
  if (altitudeDrift < descentThreshold) {
    Serial.println("¡Caída detectada!");
    while (1);
  }
  // Actualizar valores anteriores
  previousAltitude = currentAltitude;
  delay(1);
}


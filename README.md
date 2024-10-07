# Flight Decision Unit

## Descripción

El **Flight Decision Unit** es un proyecto desarrollado por el equipo de Cohetería Beauchef que tiene como objetivo procesar y analizar datos de telemetría obtenidos durante el vuelo de cohetes. Este sistema está diseñado para mejorar la toma de decisiones en tiempo real y optimizar el rendimiento de las misiones espaciales.

## Estructura del Proyecto

En la rama principal, el repositorio contiene los siguientes archivos:

- **In_Flight_Controller.ino**: Controlador en vuelo que gestiona la telemetría y el comportamiento del cohete.
- **Quaternions.ino**: Implementación de cálculos utilizando cuaterniones para la orientación.
- **Rotation_Matrix.ino**: Cálculos de matrices de rotación para la reorientación del sistema.
- **filtro_complementarios.ino**: Algoritmo de filtro complementario para mejorar la precisión de los datos de sensores.

### Carpeta `Telemetry-Data-Processing`

- **TPD.py**: Procesa los datos de telemetría utilizando matrices de rotación y filtros pasa altos para optimizar el análisis y la visualización de los datos.
- **Ejemplo**: Incluye un archivo con datos de ejemplo.
- **Programa de conexión**: Un script que permite conectarse por el puerto serial a la placa para obtener datos en tiempo real.

## Instalación

Para instalar las dependencias necesarias, ejecuta el siguiente comando:

```bash
pip install scipy numpy pandas matplotlib PySimpleGUI pyserial

# 🏎️ F1Tenth Navigation and Lap Timing System

Este repositorio contiene dos nodos ROS 2 desarrollados para un vehículo autónomo F1Tenth en simulación. El objetivo del sistema es:

1. **Navegar sin colisiones utilizando el algoritmo Follow-The-Gap (FTG).**
2. **Registrar los tiempos de vuelta completados durante la simulación, e imprimir en consola la vuelta más rápida.**

---

## 📦 Requisitos

- ROS 2 Humble
- Python 3
- Simulador F1Tenth previamente instalado

---

## 📁 Contenido del repositorio

| Archivo                  | Descripción                                                       |
|--------------------------|-------------------------------------------------------------------|
| `gap.py`                 | Nodo principal de navegación. Implementa el algoritmo Follow-The-Gap. |
| `estadisticas.py`        | Nodo de cronometraje. Detecta cruces de línea de meta y registra los tiempos por vuelta. |

---

## 1. Nodo de Navegación: `gap_follower.py`

Este nodo implementa el algoritmo **Follow-The-Gap**, diseñado para permitir que un vehículo autónomo navegue evitando obstáculos detectados por el LiDAR. El principio clave es identificar el mayor espacio libre (gap) en el entorno, seleccionar el punto más seguro dentro de ese espacio, y dirigir el vehículo hacia él con una velocidad adaptativa.

### Estructura general del nodo

El nodo está construido sobre `rclpy` (el cliente ROS 2 en Python) y sus principales funciones son:

- Suscribirse al topic `/scan` para recibir los datos del LiDAR.
- Procesar estos datos para encontrar el espacio navegable.
- Publicar comandos de velocidad y dirección en el topic `/drive`.

### Preprocesamiento del LiDAR (`preprocess_lidar()`)

```python
proc_ranges = np.convolve(proc_ranges, np.ones(self.PREPROCESS_CONV_SIZE), 'same') / self.PREPROCESS_CONV_SIZE
```

- Suaviza los datos del LiDAR con una media móvil para eliminar ruido.
- Aplica un límite de distancia (`MAX_LIDAR_DIST = 3 m`) para evitar valores extremos que afecten la lógica.

Además, se calcula:

```python
self.radians_per_elem = (4.71239) / len(ranges)
```

- Esto permite convertir un índice del array de LiDAR en un ángulo real del entorno.

### Generación del "Bubble" de seguridad

```python
closest = proc_ranges.argmin()
proc_ranges[min_index:max_index] = 0
```

- Se identifica el punto más cercano detectado por el LiDAR.
- A su alrededor se elimina una "burbuja" de seguridad (`BUBBLE_RADIUS = 160` elementos) para forzar al vehículo a evitar obstáculos cercanos y no tomar decisiones que lo acerquen demasiado a estos puntos.

### Detección del mayor espacio libre (`find_max_gap()`)

```python
masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
slices = np.ma.notmasked_contiguous(masked)
```

- Se enmascaran los valores igual a cero (correspondientes a zonas peligrosas o no navegables).
- Se identifican todos los tramos contiguos donde sí hay espacio libre.
- Se selecciona el más largo para maximizar la seguridad del trayecto.

### Selección del mejor punto (`find_best_point()`)

```python
averaged_max_gap = np.convolve(ranges[start_i:end_i], np.ones(self.BEST_POINT_CONV_SIZE), 'same') / self.BEST_POINT_CONV_SIZE
return averaged_max_gap.argmax() + start_i
```

- Dentro del espacio libre, se aplica otra media móvil con una ventana amplia para evitar esquinas o zonas angostas.
- Se escoge el punto con mayor distancia dentro del espacio como la dirección objetivo.

### Cálculo del ángulo de dirección (`get_angle()`)

```python
lidar_angle = (range_index - (range_len / 2)) * self.radians_per_elem
steering_angle = lidar_angle / 2
```

- Se transforma el índice del mejor punto seleccionado en un ángulo.
- El ángulo se reduce a la mitad para suavizar el giro y evitar maniobras bruscas.

### Control de velocidad adaptativo

```python
if abs(steering_angle) > self.STRAIGHTS_STEERING_ANGLE:
    speed = self.CORNERS_SPEED
else:
    speed = self.STRAIGHTS_SPEED
```

- Si el ángulo de giro requerido es pequeño, se considera un tramo recto y se acelera.
- Si se requiere un giro pronunciado, se reduce la velocidad para mayor estabilidad.

### Publicación de comandos al vehículo (`publish_drive()`)

```python
drive_msg = AckermannDriveStamped()
drive_msg.drive.steering_angle = angle
drive_msg.drive.speed = speed
self.drive_pub.publish(drive_msg)
```

- Se genera un mensaje `AckermannDriveStamped` con los valores de dirección y velocidad calculados.
- Se publica el mensaje en el topic `/drive`, que controla el movimiento del vehículo en el simulador.

### Tópicos utilizados

- **Suscribe a**: `/scan`  
  - Tipo: `sensor_msgs/msg/LaserScan`  
  - Fuente: Sensor LiDAR del vehículo

- **Publica en**: `/drive`  
  - Tipo: `ackermann_msgs/msg/AckermannDriveStamped`  
  - Destino: Controlador del vehículo (simulador o robot real)

---

### Parámetros configurables

Puedes ajustar estos parámetros dentro del nodo para cambiar su comportamiento:

| Parámetro                 | Descripción                                                       | Valor por defecto |
|--------------------------|-------------------------------------------------------------------|-------------------|
| `BUBBLE_RADIUS`          | Radio del área a eliminar alrededor del obstáculo más cercano     | 160 elementos     |
| `PREPROCESS_CONV_SIZE`   | Tamaño de la ventana para suavizado del LiDAR                    | 3                 |
| `BEST_POINT_CONV_SIZE`   | Tamaño de la ventana para selección del mejor punto               | 80                |
| `MAX_LIDAR_DIST`         | Distancia máxima aceptada del LiDAR                              | 3.0 m             |
| `STRAIGHTS_SPEED`        | Velocidad en tramos rectos                                       | 6.0 m/s           |
| `CORNERS_SPEED`          | Velocidad en curvas pronunciadas                                 | 3.0 m/s           |
| `STRAIGHTS_STEERING_ANGLE` | Umbral angular para definir si es un tramo recto                | π/18 rad (~10°)   |

---

## ⏱️ 2. Nodo de Cronometraje: `lap_timer_node.py`

Este nodo implementa un sistema de cronometraje para vehículos autónomos que registra el tiempo entre cruces de una **línea de meta virtual**, lo cual permite calcular el tiempo por vuelta en un circuito simulado o real.

### ⚙️ Lógica de funcionamiento

1. **Definición de la línea de meta virtual**:
   - Se establecen coordenadas fijas `(x, y)` como ubicación de la línea de meta.
   - Se utiliza una tolerancia configurable para detectar cuándo el vehículo cruza dicha línea.

2. **Detección de cruces con `/odom`**:
   - El nodo se suscribe al tópico `/ego_racecar/odom`.
   - Cada vez que la posición del vehículo se encuentra dentro del área de tolerancia, se considera un cruce de línea.

3. **Registro de tiempos de vuelta**:
   - Se mide el tiempo entre cruces consecutivos, evitando registros duplicados mediante un tiempo de enfriamiento (`cooldown_time`).
   - Cada tiempo se almacena y se imprime en consola en el momento en que se completa una vuelta.


### 📌 Parámetros personalizables

| Parámetro            | Descripción                                                     | Valor por defecto |
|---------------------|-----------------------------------------------------------------|-------------------|
| `finish_line_x`     | Coordenada X de la línea de meta                                | 0.0               |
| `finish_line_y`     | Coordenada Y de la línea de meta                                | 0.0               |
| `line_tolerance`    | Margen de tolerancia para detección de cruce (en metros)        | 0.5               |
| `cooldown_time`     | Tiempo mínimo entre cruces para evitar falsos positivos (s)     | 3.0               |


### 📤 Tópico utilizado

- **Suscribe a**: `/ego_racecar/odom`  
  - Tipo: `nav_msgs/msg/Odometry`  
  - Fuente: Odómetro del vehículo simulado o real

### 🧪 Ejecución y finalización

El nodo se ejecuta de manera continua hasta que se interrumpe manualmente (`Ctrl+C`). Al momento de la interrupción, se imprime el resumen con los tiempos por vuelta:

- Total de vueltas completadas.
- Tiempo por cada vuelta.
- Mejor vuelta destacada.

### 🧩 Fragmento clave del código (`odom_callback`)

Este método es el corazón del nodo `lap_timer_node.py`. Se ejecuta automáticamente cada vez que se recibe un nuevo mensaje de odometría del vehículo (a través del tópico `/ego_racecar/odom`). A continuación, se detalla cada parte del código:

```python
def odom_callback(self, msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    current_time = time()
```

- Se extraen las coordenadas actuales del vehículo (`x` e `y`) desde el mensaje de odometría.
- Se captura el tiempo actual del sistema para calcular duraciones.

```python
    if abs(x - self.finish_line_x) < self.line_tolerance and abs(y - self.finish_line_y) < self.line_tolerance:
```

- Se verifica si el vehículo está dentro de una zona definida alrededor de la línea de meta virtual.
- Esta comparación se hace considerando una tolerancia en ambas coordenadas, para no depender de una posición exacta.

```python
        if current_time - self.last_crossing_time > self.cooldown_time:
```

- Se comprueba que haya pasado un tiempo suficiente desde el último cruce (en segundos), para evitar registrar múltiples vueltas por pequeñas oscilaciones de posición.

```python
            lap_time = current_time - self.lap_start_time
            self.lap_start_time = current_time
            self.last_crossing_time = current_time
            self.lap_count += 1
            self.lap_times.append(lap_time)
```

- Si se cumple la condición de cruce y tiempo, se considera una vuelta completa.
- Se calcula el tiempo transcurrido desde el último cruce (`lap_time`).
- Se actualizan los registros internos:
  - `lap_start_time` para la próxima vuelta.
  - `last_crossing_time` para reiniciar el cooldown.
  - Se incrementa el contador de vueltas (`lap_count`).
  - Se almacena el tiempo de la vuelta en la lista `lap_times`.

```python
            self.get_logger().info(f'🏁 Vuelta {self.lap_count} completada en {lap_time:.2f} segundos')
```

- Finalmente, se imprime un mensaje en consola con el número de vuelta y el tiempo registrado.

---
# 📂 Resumen de Nodos del Proyecto

Este repositorio incluye dos nodos fundamentales para la navegación y evaluación del rendimiento de un vehículo autónomo. Ambos se encuentran en la carpeta `nodes/` del repositorio.

### 🚗 `gap_follower.py`

Este nodo implementa el algoritmo **Follow-The-Gap**, el cual permite al vehículo evitar obstáculos identificando el mayor espacio libre dentro del rango del LiDAR y navegando hacia él con una velocidad adaptativa.

### ⏱️ `lap_timer_node.py`

Este nodo actúa como cronómetro, detectando cuándo el vehículo cruza una **línea de meta virtual** para medir tiempos por vuelta. Registra cada vuelta y muestra un resumen al finalizar la ejecución.

---

Ambos nodos están diseñados para facilitar pruebas de navegación autónoma en simuladores o entornos reales.

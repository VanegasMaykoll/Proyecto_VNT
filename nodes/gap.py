#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class GapFollower(Node):
    BUBBLE_RADIUS = 160
    PREPROCESS_CONV_SIZE = 3
    BEST_POINT_CONV_SIZE = 80
    MAX_LIDAR_DIST = 3
    STRAIGHTS_SPEED = 6.0
    CORNERS_SPEED = 3.0
    STRAIGHTS_STEERING_ANGLE = np.pi / 18

    def __init__(self):
        super().__init__('gap_follower')

        self.scan_topic = '/scan'
        self.drive_topic = '/drive'

        # Crear suscriptor y publicador
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)

        self.radians_per_elem = None
        self.get_logger().info("GapFollower node initialized")

    def preprocess_lidar(self, ranges):
        self.radians_per_elem = (4.71239) / len(ranges)
        proc_ranges = np.array(ranges)
        proc_ranges = np.convolve(proc_ranges, np.ones(self.PREPROCESS_CONV_SIZE), 'same') / self.PREPROCESS_CONV_SIZE
        proc_ranges = np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST)
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """
        Retorna el índice de inicio y fin del mayor espacio libre en free_space_ranges.
        free_space_ranges: lista de datos LiDAR que contiene un "bubble" de ceros.
        """
        # Máscara el "bubble" para ignorar esos datos
        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
        # Obtiene una secuencia contigua de datos no enmascarados (espacio libre)
        slices = np.ma.notmasked_contiguous(masked)
        max_len = slices[0].stop - slices[0].start
        chosen_slice = slices[0]

        # Verifica si hay otras secuencias y selecciona la más larga
        for sl in slices[1:]:
            sl_len = sl.stop - sl.start
            if sl_len > max_len:
                max_len = sl_len
                chosen_slice = sl

        return chosen_slice.start, chosen_slice.stop

    def find_best_point(self, start_i, end_i, ranges):
        """
        start_i y end_i son los índices de inicio y fin del rango del mayor espacio libre, respectivamente.
        Retorna el índice del mejor punto dentro de ese rango.
        Estrategia: Escoge el punto más alejado dentro del rango y navega hacia allí.
        """
        # Realiza un promedio deslizante sobre los datos en el espacio libre para evitar golpear esquinas
        averaged_max_gap = np.convolve(ranges[start_i:end_i], np.ones(self.BEST_POINT_CONV_SIZE),
                                    'same') / self.BEST_POINT_CONV_SIZE

        return averaged_max_gap.argmax() + start_i

    def get_angle(self, range_index, range_len):
        """
        Obtiene el ángulo de un elemento particular en los datos del LiDAR y lo transforma en un ángulo de dirección apropiado.
        """
        lidar_angle = (range_index - (range_len / 2)) * self.radians_per_elem
        steering_angle = lidar_angle / 2
        return steering_angle

    def scan_callback(self, scan):
        """
        Esta es la función principal que se llama desde la simulación.
        Procesa cada escaneo LiDAR según el algoritmo Follow Gap y publica un mensaje AckermannDriveStamped.
        """
        # Preprocesar la información del LiDAR
        proc_ranges = self.preprocess_lidar(scan.ranges)
        # Encontrar el punto más cercano al LiDAR
        closest = proc_ranges.argmin()

        # Eliminar todos los puntos dentro del "bubble" (configurarlos a cero)
        min_index = int(closest - self.BUBBLE_RADIUS)
        max_index = int(closest + self.BUBBLE_RADIUS)
        if min_index < 0: min_index = 0
        if max_index > len(proc_ranges): max_index = len(proc_ranges) - 1
        proc_ranges[min_index:max_index] = 0

        # Encontrar el mayor espacio libre
        gap_start, gap_end = self.find_max_gap(proc_ranges)

        # Encontrar el mejor punto en el espacio libre
        best = self.find_best_point(gap_start, gap_end, proc_ranges)

        # Obtener el ángulo de dirección final y el valor de velocidad
        steering_angle = self.get_angle(best, len(proc_ranges))
        if abs(steering_angle) > self.STRAIGHTS_STEERING_ANGLE:
            speed = self.CORNERS_SPEED
        else:
            speed = self.STRAIGHTS_SPEED

        # Enviar de vuelta la velocidad y el ángulo de dirección al simulador
        self.publish_drive(speed, steering_angle)

    def publish_drive(self, speed, angle):
        """
        Publica el mensaje de dirección y velocidad.
        """
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    gap_follower = GapFollower()
    rclpy.spin(gap_follower)
    gap_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from time import time

class LapTimer(Node):

    def __init__(self):
        super().__init__('lap_timer_node')

        # Coordenadas aproximadas de la línea de meta virtual
        self.finish_line_x = 0.0
        self.finish_line_y = 0.0
        self.line_tolerance = 0.5  # metros
        self.cooldown_time = 3.0   # segundos entre pasos por la meta para contar una nueva vuelta

        # Suscripción a la odometría
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

        # Inicialización de variables
        self.last_crossing_time = 0
        self.lap_times = []
        self.lap_start_time = time()
        self.lap_count = 0

        self.get_logger().info("Lap timer initialized")

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        current_time = time()

        # Verifica si está cruzando la línea de meta
        if abs(x - self.finish_line_x) < self.line_tolerance and abs(y - self.finish_line_y) < self.line_tolerance:
            if current_time - self.last_crossing_time > self.cooldown_time:
                # Se detecta una nueva vuelta
                lap_time = current_time - self.lap_start_time
                self.lap_start_time = current_time
                self.last_crossing_time = current_time
                self.lap_count += 1
                self.lap_times.append(lap_time)

                self.get_logger().info(f'🏁 Vuelta {self.lap_count} completada en {lap_time:.2f} segundos')
    
    def destroy_node(self):
        # Mostrar resumen de vueltas al finalizar
        self.get_logger().info("Resumen de vueltas:")
        for i, t in enumerate(self.lap_times):
            self.get_logger().info(f"Vuelta {i+1}: {t:.2f} segundos")
        if self.lap_times:
            self.get_logger().info(f"⏱️ Mejor vuelta: {min(self.lap_times):.2f} segundos")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LapTimer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Lap timer node finalizado manualmente.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#! /usr/bin/env python3
import threading

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class RotaRecorder(Node):
    def __init__(self):
        super().__init__('rota_recorder')
        # Buffer para armazenar a posição (TF)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_current_pose(self):
        try:
            # Tenta pegar a posição do 'base_link' (robô) em relação ao 'map'
            t = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time())

            x = t.transform.translation.x
            y = t.transform.translation.y

            # Orientação (z e w são os mais importantes para rotação 2D)
            w_orient = t.transform.rotation.w

            return x, y, w_orient

        except TransformException as ex:
            self.get_logger().info(f'Ainda não consegui ler a posição: {ex}')
            return None, None, None


def spin_ros(node):
    rclpy.spin(node)


def main():
    rclpy.init()
    recorder = RotaRecorder()

    # Roda o ROS numa thread separada para não travar o input do teclado
    thread = threading.Thread(target=spin_ros, args=(recorder,), daemon=True)
    thread.start()

    print("\n" + "="*50)
    print("🚜 GRAVADOR DE ROTAS AGRÍCOLA 🚜")
    print("="*50)
    print("INSTRUÇÕES:")
    print("1. Dirija o robô usando o teleop em outro terminal.")
    print("2. Quando chegar num ponto importante, aperte [ENTER] aqui.")
    print("3. Digite 'fim' para encerrar e gerar o código.")
    print("="*50 + "\n")

    waypoints_code = []
    point_count = 1

    while True:
        try:
            user_input = input(f"Dirija até o Ponto {point_count} e aperte ENTER (ou 'fim'): ")

            if user_input.lower() == 'fim':
                break

            # Pega a posição atual
            x, y, w = recorder.get_current_pose()

            if x is not None:
                # Formata a linha de código exata para o seu script de missão
                line = f"    # Ponto {point_count}\n"
                line += f"    waypoints.append(create_pose(navigator, {x:.2f}, {y:.2f}, {w:.3f}))"
                waypoints_code.append(line)

                print(f"✅ Ponto {point_count} GRAVADO: x={x:.2f}, y={y:.2f}")
                point_count += 1
            else:
                print("⚠️  O robô ainda não sabe onde está (AMCL não localizou). Ande um pouco.")

        except KeyboardInterrupt:
            break

    print("\n" + "="*50)
    print("📋 COPIE O CÓDIGO ABAIXO PARA 'farm_mission.py':")
    print("="*50)
    print("\n    waypoints = []\n")
    for line in waypoints_code:
        print(line)
    print("\n" + "="*50)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

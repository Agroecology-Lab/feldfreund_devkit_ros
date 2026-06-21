#! /usr/bin/env python3
import math
import time

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy

"""
SCRIPT DE MISSÃO AGRÍCOLA - ROTA GRAVADA (CORRIGIDO)
"""


def create_pose(navigator, x, y, z_w):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.w = z_w

    # Cálculo automático do Z para validar o quaternion
    try:
        val_z = math.sqrt(1 - min(z_w**2, 1.0))
        pose.pose.orientation.z = val_z
    except ValueError:
        pose.pose.orientation.z = 0.0

    return pose


def main():
    rclpy.init()
    navigator = BasicNavigator()

    print("🚜 Aguardando o Nav2 ficar ativo...")
    navigator.waitUntilNav2Active()

    # --- PROTOCOLO DE SEGURANÇA ---
    print("📍 Definindo posição inicial (0,0)...")
    initial_pose = create_pose(navigator, 0.0, 0.0, 1.0)
    navigator.setInitialPose(initial_pose)

    print("🧹 Limpando mapas de custo...")
    navigator.clearAllCostmaps()
    time.sleep(1.0)

    print("✅ Nav2 Pronto! Iniciando Rota Gravada.")

    waypoints = []

    # --- LISTA DE PONTOS GRAVADOS ---
    # Ponto 1
    waypoints.append(create_pose(navigator, 0.01, 0.06, 1.000))
    # Ponto 2
    waypoints.append(create_pose(navigator, 0.68, 0.47, 0.859))
    # Ponto 3
    waypoints.append(create_pose(navigator, 5.05, 1.29, 1.000))
    # Ponto 4
    waypoints.append(create_pose(navigator, 8.90, 1.25, 1.000))
    # Ponto 5
    waypoints.append(create_pose(navigator, 11.39, 1.27, 1.000))
    # Ponto 6
    waypoints.append(create_pose(navigator, 15.25, 0.93, 1.000))
    # Ponto 7
    waypoints.append(create_pose(navigator, 15.30, 1.05, 0.897))
    # Ponto 8
    waypoints.append(create_pose(navigator, 16.07, 3.82, 0.718))
    # Ponto 9
    waypoints.append(create_pose(navigator, 12.90, 4.12, 0.017))
    # Ponto 10
    waypoints.append(create_pose(navigator, 10.28, 4.23, -0.002))
    # Ponto 11
    waypoints.append(create_pose(navigator, 5.58, 4.49, 0.006))
    # Ponto 12
    waypoints.append(create_pose(navigator, 2.55, 4.49, -0.009))
    # Ponto 13
    waypoints.append(create_pose(navigator, -0.48, 4.38, -0.007))

    print(f"🚀 Enviando {len(waypoints)} pontos de trabalho...")
    navigator.followWaypoints(waypoints)

    # --- LOOP DE MONITORAMENTO CORRIGIDO ---
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()

        if feedback and i % 5 == 0:
            # CORREÇÃO: Removido 'distance_remaining' que não existe no FollowWaypoints
            print(f"🔄 Executando Waypoint {feedback.current_waypoint + 1} de {len(waypoints)}")

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('\n🎉 Rota Finalizada com Sucesso!')
            break
        elif result == TaskResult.CANCELED:
            print('\n⚠️ Rota cancelada!')
            break
        elif result == TaskResult.FAILED:
            print('\n❌ Falha na rota!')
            break

        time.sleep(0.5)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

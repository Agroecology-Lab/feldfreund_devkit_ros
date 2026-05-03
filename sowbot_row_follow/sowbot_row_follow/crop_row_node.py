#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2022 Agricultural-Robotics-Bonn
# Copyright 2026 Agroecology Lab Ltd
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# crop_row_node.py
# ================
# Derivado de / Derived from:
#   visual-multi-crop-row-navigation
#   https://github.com/Agricultural-Robotics-Bonn/visual-multi-crop-row-navigation
#   Autores / Authors: Alireza Ahmadi, Michael Halstead, Chris McCool (Agricultural-Robotics-Bonn)
#   Porta ROS2 / ROS2 port: Agroecology Lab Ltd
#   Referência / Reference: Ahmadi et al., "Towards Autonomous Crop-Agnostic Visual Navigation
#              in Arable Fields", arXiv:2109.11936, 2021.
#              Ahmadi et al., ICRA 2020.
#
# Adaptado para o projeto Sowbot por Agroecology Lab Ltd.
# Adapted for the Sowbot project by Agroecology Lab Ltd.
#
# Modificações / Modifications:
#   Reduzido para câmera RGB única, monocular, seguimento de fileira apenas para frente.
#   Stripped to single RGB camera, monocular, forward row-following only.
#   Removido / Removed: profundidade, estéreo, correspondência de características,
#     modos de navegação, troca de fileiras, câmera dupla.
#     (depth, stereo, feature matching, navigation modes, row switching, dual-camera)
#   Preservado verbatim ou quase verbatim / Preserved verbatim or near-verbatim:
#     - Índice ExG + limiar Otsu         (imageProc.getExgMask)
#     - Extração de centros de contorno  (imageProc.processRGBImage)
#     - Ajuste de linha por janela       (imageProc.findLinesInImage)
#     - Transformação câmera→imagem      (imageProc.cameraToImage)
#     - Matriz de interação visual       (controller.visualServoingCtl)
#     - Modelo de câmera                 (camera.Camera)
#
# Subscreve / Subscribes:
#   /caatinga_vision/row_nav/image_raw  (sensor_msgs/Image) — da / from camera_source_node
#
# Publica / Publishes:
#   /aoc/conditions/row_offset        (std_msgs/Float32)  deslocamento lateral [-1,1] / lateral offset
#   /aoc/conditions/row_heading_error (std_msgs/Float32)  erro de rumo em radianos / heading error in radians
#   /aoc/heartbeat/neo_vision         (std_msgs/Bool)     True enquanto fileiras detectadas (M10)
#   /caatinga_vision/row_nav/debug_image (sensor_msgs/Image)  frame anotado / annotated frame (Foxglove)
#
# Publica sempre / Always publishes (gated by /row_follow/enable service):
#   /cmd_vel  (geometry_msgs/Twist)

from __future__ import division, print_function

import math

import cv2 as cv
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from std_srvs.srv import SetBool


# ===========================================================================
# Modelo de câmera / Camera model  (de / from camera.py — BSD-2)
# Apenas os campos usados pela matriz de interação do controlador.
# Only the fields used by the controller interaction matrix.
# ===========================================================================

class Camera:
    def __init__(self, f=1, deltaz=1.2, deltay=0, scale=1,
                 tilt_angle=np.deg2rad(-80), pixel_size=0.96,
                 cx=0, cy=0, ar=1):
        self.f = f
        self.deltaz = deltaz      # altura acima do solo (m) / height above ground (m)
        self.deltay = deltay      # deslocamento frontal da câmera / forward camera offset from wheel axis
        self.scale = scale
        self.tilt_angle = tilt_angle  # ângulo de inclinação (rad) / tilt angle (rad)
        self.pixel_size = pixel_size
        self.cx = cx
        self.cy = cy
        self.ar = ar


# ===========================================================================
# Controlador de servo visual / Visual servoing controller  (de / from controller.py — BSD-2)
# Matriz de interação de Cherubini & Chaumette, preservada exatamente.
# Cherubini & Chaumette interaction matrix, preserved exactly.
# ===========================================================================

def wrap_to_pi(theta: float) -> float:
    # Normaliza ângulo para [-pi, pi] / Wrap angle to [-pi, pi]
    while theta < -math.pi:
        theta += 2 * math.pi
    while theta > math.pi:
        theta -= 2 * math.pi
    return theta


def visual_servoing_ctl(camera: Camera,
                        desired_state: np.ndarray,
                        actual_state: np.ndarray,
                        v_des: float) -> float:
    """
    Calcula a correção de velocidade angular (rad/s).
    Compute angular velocity correction (rad/s).

    desired_state: [x_desejado, y_desejado, theta_desejado]  (características no frame da imagem)
                   [x_desired,  y_desired,  theta_desired]   (image-frame features)
    actual_state:  [x_atual,    y_atual,    theta_atual]
                   [x_actual,   y_actual,   theta_actual]
    v_des: velocidade frontal desejada (m/s) / desired forward speed (m/s)

    Preservado de / Preserved from controller.visualServoingCtl (BSD-2).
    """
    x = actual_state[0]
    y = actual_state[1]
    theta = actual_state[2]

    # Ganhos do controlador / Controller gains
    lambda_x_1 = 10
    lambda_w_1 = 3000
    lambdavec = np.array([lambda_x_1, lambda_w_1])

    # Tipo 0 = seguimento de fileira / Type 0 = row following (not column)
    controller_type = 0

    angle = camera.tilt_angle
    delta_z = camera.deltaz

    # Matriz de interação (Cherubini & Chaumette)
    # Relaciona variáveis de controle [v, w] às mudanças de características [x, y, theta]
    # Relates control variables [v, w] to feature changes [x, y, theta]
    IntMat = np.array([
        [(-np.sin(angle) - y * np.cos(angle)) / delta_z, 0,
         x * (np.sin(angle) + y * np.cos(angle)) / delta_z,
         x * y, -1 - x**2, y],
        [0, -(np.sin(angle) + y * np.cos(angle)) / delta_z,
         y * (np.sin(angle) + y * np.cos(angle)) / delta_z,
         1 + y**2, -x * y, -x],
        [np.cos(angle) * np.cos(theta)**2 / delta_z,
         np.cos(angle) * np.cos(theta) * np.sin(theta) / delta_z,
         -(np.cos(angle) * np.cos(theta) * (y * np.sin(theta) + x * np.cos(theta))) / delta_z,
         -(y * np.sin(theta) + x * np.cos(theta)) * np.cos(theta),
         -(y * np.sin(theta) + x * np.cos(theta)) * np.sin(theta), -1]
    ])

    # Transformação do frame do robô para o frame da câmera
    # Transformation from robot frame to camera frame
    delta_y = camera.deltay
    TransfMat = np.array([
        [0,               -delta_y],
        [-np.sin(angle),   0],
        [np.cos(angle),    0],
        [0,                0],
        [0,               -np.cos(angle)],
        [0,               -np.sin(angle)],
    ])
    Trans_vel = TransfMat[:, 0]  # componente linear / linear component
    Trans_ang = TransfMat[:, 1]  # componente angular / angular component

    # Jacobiano: relaciona controles do robô às mudanças de características
    # Jacobian: relates robot controls to feature changes
    Jac = np.array([IntMat[controller_type, :], IntMat[2, :]])
    Jac_vel = np.matmul(Jac, Trans_vel)
    Jac_ang = np.matmul(Jac, Trans_ang)
    Jac_ang_pi = np.linalg.pinv([Jac_ang])  # pseudoinversa / pseudoinverse

    # Erro entre estado atual e desejado / Error between actual and desired state
    trans_delta = actual_state[controller_type] - desired_state[controller_type]
    ang_delta = actual_state[2] - desired_state[2]
    delta = np.array([trans_delta, wrap_to_pi(ang_delta)])

    # Lei de controle de realimentação / Feedback control law
    temp = lambdavec * delta
    ang_fb = np.matmul(-Jac_ang_pi.T, (temp + Jac_vel * v_des))

    return float(np.squeeze(ang_fb))


# ===========================================================================
# Índice de vegetação ExG + extração de contornos
# ExG vegetation index + contour extraction  (de / from imageProc.py — BSD-2)
# ===========================================================================

def compute_exg_mask(bgr_img: np.ndarray):
    """
    Índice de Verde Excedente → máscara binária de vegetação.
    Excess Green Index → binary vegetation mask.
    De / From imageProc.getExgMask (BSD-2).
    """
    # Converte para int32 para permitir valores negativos no cálculo ExG
    # Cast to int32 to allow negative values in ExG calculation
    img = bgr_img.astype("int32")
    b, g, r = img[:, :, 0], img[:, :, 1], img[:, :, 2]

    # ExG = 2G - R - B  (realça vegetação verde / highlights green vegetation)
    exg = 2 * g - r - b
    exg[exg < 0] = 0  # descarta valores negativos / discard negative values
    exg = exg.astype("uint8")

    # Suavização gaussiana antes da limiarização para reduzir ruído
    # Gaussian blur before thresholding to reduce noise
    blur = cv.GaussianBlur(exg, (5, 5), 0)

    # Limiarização de Otsu: encontra automaticamente o melhor limiar
    # Otsu thresholding: automatically finds the best threshold
    _, mask = cv.threshold(blur, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)

    # Dilatação para conectar regiões de plantas próximas
    # Dilation to connect nearby plant regions
    kernel = np.ones((10, 10), np.uint8)
    mask = cv.dilate(mask, kernel, iterations=1)

    return mask


def get_plant_centers(mask: np.ndarray, min_area: float) -> np.ndarray:
    """
    Centros de contorno da máscara binária. Retorna array (N,2) de (x,y).
    Contour centers from binary mask. Returns (N,2) array of (x,y).
    De / From imageProc.processRGBImage (BSD-2).
    """
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    centers = []
    for c in contours:
        # Filtra contornos pequenos (ruído de solo, pedras, etc.)
        # Filter small contours (soil noise, stones, etc.)
        if cv.contourArea(c) >= min_area:
            M = cv.moments(c)
            if M["m00"] > 0:
                # Centro de massa do contorno / Contour centroid
                centers.append((int(M["m10"] / M["m00"]),
                                 int(M["m01"] / M["m00"])))
    return np.array(centers) if centers else np.empty((0, 2), dtype=int)


# ===========================================================================
# Ajuste de linha por janela de varredura
# Scan-window line fitting  (de / from imageProc.findLinesInImage — BSD-2)
# ===========================================================================

def fit_line(points_xy: np.ndarray):
    """
    Ajusta x = m*y + b aos centros das plantas.
    Fit x = m*y + b to plant centers.
    De / From helpers geométricos do imageProc (BSD-2).

    Usa y como variável independente pois as fileiras são aproximadamente verticais
    na imagem — ajuste padrão x=f(y) é mais estável que y=f(x) neste caso.
    Uses y as the independent variable because rows run approximately vertical
    in the image — fitting x=f(y) is more stable than y=f(x) here.
    """
    if len(points_xy) < 2:
        return None, None
    y = points_xy[:, 1].astype(float)
    x = points_xy[:, 0].astype(float)
    # Evita ajuste degenerado quando todos os pontos têm a mesma altura
    # Avoid degenerate fit when all points share the same row
    if np.std(y) < 1.0:
        return None, None
    m, b = np.polyfit(y, x, 1)  # inclinação e intercepto / slope and intercept
    return m, b


def detect_crop_rows(centers: np.ndarray,
                     img_w: int, img_h: int,
                     n_windows: int, win_w: int) -> list:
    """
    Divide a imagem em n_windows colunas verticais e ajusta uma linha por coluna.
    Divide the image into n_windows vertical columns and fit a line per column.
    De / From imageProc.findLinesInImage (BSD-2).

    Retorna / Returns: lista de (bottom_x, slope, intercept) por fileira detectada.
                       list of (bottom_x, slope, intercept) per detected row.
    """
    if len(centers) == 0:
        return []

    # Largura de cada passo de varredura / Width of each scan step
    step = img_w / n_windows
    rows = []

    for i in range(n_windows):
        # Limites da janela atual / Current window bounds
        left = i * step
        right = left + win_w

        # Pontos dentro desta janela / Points inside this window
        in_win = centers[(centers[:, 0] >= left) & (centers[:, 0] < right)]
        if len(in_win) < 2:
            continue  # pontos insuficientes para ajustar linha / not enough points to fit

        m, b = fit_line(in_win)
        if m is None:
            continue

        # x onde a linha cruza a borda inferior da imagem
        # x where the line crosses the bottom edge of the image
        bx = m * img_h + b
        if 0 <= bx <= img_w:  # dentro dos limites da imagem / within image bounds
            rows.append((bx, m, b))

    return rows


# ===========================================================================
# Visualização de depuração / Debug visualisation
# (de / from imageProc.drawGraphics — BSD-2)
# ===========================================================================

def draw_debug(bgr: np.ndarray, centers: np.ndarray,
               rows: list, img_w: int, img_h: int) -> np.ndarray:
    """
    Desenha centros de plantas, linhas de fileira detectadas e linha central de referência.
    Draw plant centers, detected row lines, and centre reference line.
    """
    out = bgr.copy()

    # Centros das plantas (magenta) / Plant centers (magenta)
    for (cx, cy) in centers:
        cv.circle(out, (cx, cy), 4, (255, 0, 255), -1)

    # Linhas de fileira detectadas (verde) / Detected row lines (green)
    for (bx, m, b) in rows:
        x_top = int(m * 0 + b)   # x no topo da imagem / x at top of image
        x_bot = int(bx)           # x na base da imagem / x at bottom of image
        cv.line(out, (x_top, 0), (x_bot, img_h), (0, 255, 0), 2)

    # Linha central de referência (vermelha) — onde o robô deve estar
    # Centre reference line (red) — where the robot should be
    cv.line(out, (img_w // 2, 0), (img_w // 2, img_h), (0, 0, 255), 1)

    return out


# ===========================================================================
# Nó ROS 2 / ROS 2 Node
# ===========================================================================

class CropRowNode(Node):

    def __init__(self):
        super().__init__("crop_row_node")
        self.bridge = CvBridge()

        # -------------------------------------------------------------------
        # Parâmetros — todos configuráveis via crop_row_params.yaml
        # Parameters — all settable from crop_row_params.yaml
        # -------------------------------------------------------------------
        self.declare_parameter("image_topic", "/caatinga_vision/row_nav/image_raw")
        self.declare_parameter("debug_image_topic", "/caatinga_vision/row_nav/debug_image")
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("n_scan_windows", 8)
        self.declare_parameter("window_width", 80)
        self.declare_parameter("min_contour_area", 10.0)
        self.declare_parameter("camera_height_m", 1.2)
        self.declare_parameter("camera_tilt_deg", -80.0)
        self.declare_parameter("linear_vel", 0.15)
        self.declare_parameter("omega_scaler", 0.1)
        self.declare_parameter("max_omega", 0.4)
        self.declare_parameter("heartbeat_timeout_s", 2.0)
        self.declare_parameter("use_direct_cmd_vel", True)  # kept for param-file compat; ignored — service is the gate

        p = self.get_parameter
        self.image_topic: str    = p("image_topic").value
        self.debug_topic: str    = p("debug_image_topic").value
        self.img_w: int          = p("image_width").value
        self.img_h: int          = p("image_height").value
        self.n_windows: int      = p("n_scan_windows").value
        self.win_w: int          = p("window_width").value
        self.min_area: float     = p("min_contour_area").value
        self.linear_vel: float   = p("linear_vel").value
        self.omega_scaler: float = p("omega_scaler").value
        self.max_omega: float    = p("max_omega").value
        self.hb_timeout: float   = p("heartbeat_timeout_s").value

        # Modelo de câmera para a matriz de interação do servo visual
        # Camera model for the visual servoing interaction matrix
        self.camera = Camera(
            deltaz=p("camera_height_m").value,
            tilt_angle=np.deg2rad(p("camera_tilt_deg").value),
        )

        self.get_logger().info(
            "crop_row_node iniciado / started | subscribing to %s | "
            "cmd_vel gated by /row_follow/enable service"
            % self.image_topic
        )

        # -------------------------------------------------------------------
        # Subscritor / Subscriber — recebe frames da camera_source_node
        #                           receives frames from camera_source_node
        # -------------------------------------------------------------------
        self.sub = self.create_subscription(
            Image, self.image_topic, self._on_image, 10)

        # -------------------------------------------------------------------
        # Publicadores / Publishers
        # -------------------------------------------------------------------
        # Condições AOC: deslocamento lateral e erro de rumo para a camada de navegação
        # AOC conditions: lateral offset and heading error for the navigation layer
        self.pub_offset = self.create_publisher(
            Float32, "/aoc/conditions/row_offset", 10)
        self.pub_heading = self.create_publisher(
            Float32, "/aoc/conditions/row_heading_error", 10)

        # Heartbeat para detecção de modo PERCEPTION_DEGRADED (M10)
        # Heartbeat for PERCEPTION_DEGRADED mode detection (M10)
        self.pub_heartbeat = self.create_publisher(
            Bool, "/aoc/heartbeat/neo_vision", 10)

        # Imagem de depuração para Foxglove / Debug image for Foxglove
        self.pub_debug = self.create_publisher(
            Image, self.debug_topic, 10)

        # cmd_vel publisher — always active; /row_follow/enable service is the gate
        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)

        # Enable/disable service — Limbic calls this to start/stop row following
        self._enabled = False
        self.srv_enable = self.create_service(
            SetBool, "/row_follow/enable", self._on_enable_service)

        # -------------------------------------------------------------------
        # Timer de heartbeat (5 Hz) — M10
        # Heartbeat timer (5 Hz) — M10
        # -------------------------------------------------------------------
        self._last_detection_time = None
        self._hb_timer = self.create_timer(0.2, self._publish_heartbeat)

    # -----------------------------------------------------------------------
    # Callback de imagem / Image callback
    # -----------------------------------------------------------------------

    def _on_image(self, msg: Image) -> None:
        # Converte mensagem ROS para array OpenCV BGR
        # Convert ROS message to OpenCV BGR array
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error("Falha no CV bridge: %s" % str(e))
            return

        # Redimensiona se a câmera não respeitou a resolução solicitada
        # Resize if camera did not honour the requested resolution
        h, w = bgr.shape[:2]
        if w != self.img_w or h != self.img_h:
            bgr = cv.resize(bgr, (self.img_w, self.img_h))

        # Pipeline: máscara ExG → centros de plantas → detecção de fileiras
        # Pipeline: ExG mask → plant centers → row detection
        mask = compute_exg_mask(bgr)
        centers = get_plant_centers(mask, self.min_area)
        detected_rows = detect_crop_rows(
            centers, self.img_w, self.img_h, self.n_windows, self.win_w)

        # Publica imagem de depuração independentemente da detecção
        # Always publish debug image regardless of detection result
        debug_img = draw_debug(bgr, centers, detected_rows, self.img_w, self.img_h)
        debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
        debug_msg.header = msg.header
        self.pub_debug.publish(debug_msg)

        # Sem fileiras detectadas — heartbeat silenciará em breve
        # No rows detected — heartbeat will go silent shortly
        if not detected_rows:
            return

        # Média das características de todas as fileiras detectadas
        # Average features across all detected rows
        avg_bx = float(np.mean([r[0] for r in detected_rows]))
        avg_slope = float(np.mean([r[1] for r in detected_rows]))
        heading_error_rad = float(np.arctan(avg_slope))

        # Vetor de características no frame da imagem para o controlador
        # De imageProc.cameraToImage (BSD-2): desloca origem para o centro da imagem
        # Feature vector in image frame for the controller
        # From imageProc.cameraToImage (BSD-2): shift origin to image centre
        actual_feature = np.array([
            avg_bx - self.img_w / 2.0,   # x: deslocamento lateral do centro / lateral offset from centre
            self.img_h / 2.0,             # y: metade da altura da imagem / image mid-height
            heading_error_rad,            # theta: ângulo da fileira / row angle
        ])

        # Estado desejado: fileira centrada, sem inclinação / Desired: centred row, no tilt
        desired_feature = np.array([0.0, actual_feature[1], 0.0])

        # Calcula correção angular via servo visual / Compute angular correction via visual servoing
        omega_raw = visual_servoing_ctl(
            self.camera, desired_feature, actual_feature, self.linear_vel)

        # Aplica escala e limita ao máximo permitido
        # Apply scale factor and clamp to maximum allowed
        omega = float(np.clip(
            self.omega_scaler * omega_raw, -self.max_omega, self.max_omega))

        # Deslocamento lateral normalizado para AOC [-1, 1]
        # Normalised lateral offset for AOC [-1, 1]
        # Negativo = fileiras à direita do centro → girar à esquerda
        # Negative = rows right of centre → steer left
        norm_offset = float(np.clip(
            actual_feature[0] / (self.img_w / 2.0), -1.0, 1.0))

        # Atualiza timestamp para o heartbeat / Update timestamp for heartbeat
        self._last_detection_time = self.get_clock().now()

        # Publica condições AOC / Publish AOC conditions
        off_msg = Float32()
        off_msg.data = norm_offset
        self.pub_offset.publish(off_msg)

        hdg_msg = Float32()
        hdg_msg.data = heading_error_rad
        self.pub_heading.publish(hdg_msg)

        # Publish cmd_vel only when enabled by Limbic via /row_follow/enable
        if self._enabled:
            twist = Twist()
            twist.linear.x = self.linear_vel   # velocidade frontal / forward speed
            twist.angular.z = omega             # correção de rumo / heading correction
            self.pub_cmd_vel.publish(twist)

        self.get_logger().debug(
            "fileiras=%d offset=%.3f rumo=%.1fgraus omega=%.3f | "
            "rows=%d offset=%.3f heading=%.1fdeg omega=%.3f"
            % (len(detected_rows), norm_offset, math.degrees(heading_error_rad), omega,
               len(detected_rows), norm_offset, math.degrees(heading_error_rad), omega)
        )

    # -----------------------------------------------------------------------
    # Enable / disable service — called by sowbot_row_follow on Limbic
    # -----------------------------------------------------------------------

    def _on_enable_service(self, request: SetBool.Request,
                           response: SetBool.Response) -> SetBool.Response:
        self._enabled = request.data
        if not self._enabled:
            self._publish_zero_cmd_vel()
        response.success = True
        response.message = "enabled" if self._enabled else "disabled"
        self.get_logger().info("row_follow/enable → %s" % response.message)
        return response

    def _publish_zero_cmd_vel(self) -> None:
        """Publish one zero-velocity Twist to guarantee the robot stops."""
        self.pub_cmd_vel.publish(Twist())

    # -----------------------------------------------------------------------
    # Heartbeat M10
    # Monitora perda de percepção — o Sistema Límbico observa este tópico
    # Monitors perception loss — the Limbic System watches this topic
    # -----------------------------------------------------------------------

    def _publish_heartbeat(self) -> None:
        alive = False
        if self._last_detection_time is not None:
            # Tempo desde a última detecção / Time since last detection
            elapsed = (
                self.get_clock().now() - self._last_detection_time
            ).nanoseconds * 1e-9
            alive = elapsed < self.hb_timeout

        if not alive and self._last_detection_time is not None:
            # Sem detecção de fileira — acionar modo PERCEPTION_DEGRADED no Sistema Límbico
            # No row detection — trigger PERCEPTION_DEGRADED mode in Limbic System
            self.get_logger().warn(
                "crop_row_node: sem detecção de fileira — heartbeat INATIVO. "
                "No row detection — heartbeat DEAD. "
                "Sistema Límbico deve entrar em PERCEPTION_DEGRADED (M10).",
                throttle_duration_sec=5.0,
            )

        hb = Bool()
        hb.data = alive
        self.pub_heartbeat.publish(hb)


def main(args=None):
    rclpy.init(args=args)
    node = CropRowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

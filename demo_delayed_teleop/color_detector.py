#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        self.subscription = self.create_subscription(
            Image,
            '/fleet_0/camera/image',   # adapte le topic à ta caméra
            self.listener_callback,
            10)
        self.bridge = CvBridge()

        # Crée une fenêtre avec sliders
        cv2.namedWindow("Sliders", cv2.WINDOW_NORMAL)
        cv2.createTrackbar("H min", "Sliders", 0, 179, lambda x: None)
        cv2.createTrackbar("H max", "Sliders", 179, 179, lambda x: None)
        cv2.createTrackbar("S min", "Sliders", 0, 255, lambda x: None)
        cv2.createTrackbar("S max", "Sliders", 255, 255, lambda x: None)
        cv2.createTrackbar("V min", "Sliders", 0, 255, lambda x: None)
        cv2.createTrackbar("V max", "Sliders", 255, 255, lambda x: None)

        # Définition des valeurs pour le rouge
        red = [170, 10, 50, 255, 30, 255]  # [H_min, H_max, S_min, S_max, V_min, V_max] si (Hmin < Hmax) interval wrap-around

        # Liste des noms des sliders dans le même ordre
        slider_names = ["H min", "H max", "S min", "S max", "V min", "V max"]
        
        # Initialisation des sliders avec les valeurs du tableau
        for i, name in enumerate(slider_names):
            cv2.setTrackbarPos(name, "Sliders", red[i])



    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Récupère les valeurs des sliders
        h_min = cv2.getTrackbarPos("H min", "Sliders")
        h_max = cv2.getTrackbarPos("H max", "Sliders")
        s_min = cv2.getTrackbarPos("S min", "Sliders")
        s_max = cv2.getTrackbarPos("S max", "Sliders")
        v_min = cv2.getTrackbarPos("V min", "Sliders")
        v_max = cv2.getTrackbarPos("V max", "Sliders")

        # Conversion en HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Cas normal : H_min <= H_max
        if h_min <= h_max:
            lower = np.array([h_min, s_min, v_min])
            upper = np.array([h_max, s_max, v_max])
            mask = cv2.inRange(hsv, lower, upper)

        # Cas wrap-around : H_min > H_max (ex. 160 → 20)
        else:
            lower1 = np.array([h_min, s_min, v_min])
            upper1 = np.array([179, s_max, v_max])
            lower2 = np.array([0, s_min, v_min])
            upper2 = np.array([h_max, s_max, v_max])
            mask1 = cv2.inRange(hsv, lower1, upper1)
            mask2 = cv2.inRange(hsv, lower2, upper2)
            mask = cv2.bitwise_or(mask1, mask2)

        # Trouver les contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:  # filtre pour ignorer les petits bruits
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, f"Area: {area}", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)



        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Affichage
        cv2.imshow("Camera", frame)
        cv2.imshow("Mask", mask)
        cv2.imshow("Detection", result)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            cv2.waitKey(1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

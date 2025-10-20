#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time, threading, json, rospy, cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
try:
    from pyzbar.pyzbar import decode as zbar_decode
    USE_ZBAR = True
except Exception:
    USE_ZBAR = False

BRIDGE = CvBridge()
SCAN_TIMEOUT = rospy.get_param('~scan_timeout', 10.0)
CAM_TOPIC = rospy.get_param('~camera_topic', '/camera/image_raw')

scan_active = False
scan_lock = threading.Lock()
result_pub = None

def cmd_cb(msg):
    global scan_active
    txt = msg.data if hasattr(msg,'data') else str(msg)
    if txt.strip().upper() == 'START':
        with scan_lock:
            scan_active = True
        rospy.loginfo('[QR] START recibido; escaneando %.1fs', SCAN_TIMEOUT)
        threading.Thread(target=deactivate_later, daemon=True).start()

def deactivate_later():
    global scan_active
    t0 = time.time()
    while time.time() - t0 < SCAN_TIMEOUT and scan_active:
        time.sleep(0.05)
    with scan_lock:
        scan_active = False
    rospy.loginfo('[QR] Fin ventana')

def image_cb(msg):
    global scan_active
    if not scan_active:
        return
    try:
        frame = BRIDGE.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except CvBridgeError:
        return

    text = None
    if USE_ZBAR:
        dets = zbar_decode(frame)
        if dets:
            text = dets[0].data.decode('utf-8', errors='ignore').strip()
    else:
        qr = cv2.QRCodeDetector()
        val, pts, _ = qr.detectAndDecode(frame)
        if pts is not None and val:
            text = val.strip()

    if text:
        result_pub.publish(String(data=text))
        rospy.loginfo('[QR] Detectado: %s', text)
        with scan_lock:
            scan_active = False

def main():
    global result_pub
    rospy.init_node('qr_scanner_node', anonymous=True)
    result_pub = rospy.Publisher('/tirgo/qr/result', String, queue_size=1)
    rospy.Subscriber('/tirgo/qr/cmd', String, cmd_cb)
    rospy.Subscriber(CAM_TOPIC, Image, image_cb, queue_size=1, buff_size=2**22)
    rospy.loginfo('[QR] Listo. Cámara: %s', CAM_TOPIC)
    rospy.spin()

if __name__ == '__main__':
    main()

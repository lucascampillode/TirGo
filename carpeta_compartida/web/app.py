#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import threading, time
from flask import Flask, render_template, jsonify
import rospy
from std_msgs.msg import String

rospy.init_node('web_buttons_node', anonymous=True, disable_signals=True)
pub = rospy.Publisher('/tirgo/ui/intent', String, queue_size=10)

def spin_ros():
    while not rospy.is_shutdown():
        time.sleep(0.1)
threading.Thread(target=spin_ros, daemon=True).start()

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

@app.post('/press/<action>')
def press(action):
    if action not in ('consultar','diagnosticar','leer'):
        return jsonify({'ok': False, 'error': 'Acción no válida'}), 400
    pub.publish(String(data=action))
    rospy.loginfo('Botón pulsado: %s', action)
    return jsonify({'ok': True, 'pressed': action})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)

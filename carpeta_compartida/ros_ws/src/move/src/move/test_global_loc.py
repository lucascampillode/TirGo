#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_srvs.srv import Empty

def publish_spin(duration=12.0, switch_interval=6.0,
                 angular_speed=1.0,
                 topic='/mobile_base_controller/cmd_vel',
                 rate_hz=10.0):
    pub = rospy.Publisher(topic, Twist, queue_size=1)
    rospy.sleep(0.1)

    twist = Twist()
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0

    total_steps = max(1, int(duration * rate_hz))
    steps_per_switch = max(1, int(switch_interval * rate_hz))
    rate = rospy.Rate(rate_hz)

    for i in range(total_steps):
        sign = -1 if ((i // steps_per_switch) % 2) == 0 else 1
        twist.angular.z = sign * float(angular_speed)
        pub.publish(twist)
        rate.sleep()

    twist.angular.z = 0.0
    pub.publish(twist)

def make_initial_pose(x, y, qz, qw):
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()

    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = 0.0

    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw

    # covarianza pequeña en x, y y yaw
    msg.pose.covariance = [
        0.25, 0,    0,    0,    0,    0,
        0,    0.25, 0,    0,    0,    0,
        0,    0,    1e6,  0,    0,    0,
        0,    0,    0,    1e6,  0,    0,
        0,    0,    0,    0,    1e6,  0,
        0,    0,    0,    0,    0,    0.07
    ]
    return msg

def main():
    rospy.init_node('amcl_global_and_initial', anonymous=False)

    # 1) global localization
    service_name = '/global_localization'   # si no existe, prueba '/amcl/global_localization'
    rospy.loginfo("Esperando al servicio %s...", service_name)
    rospy.wait_for_service(service_name)

    try:
        global_loc = rospy.ServiceProxy(service_name, Empty)
        rospy.loginfo("Llamando a %s para reinicializar AMCL...", service_name)
        global_loc()
        rospy.loginfo("Servicio llamado correctamente.")
    except rospy.ServiceException as e:
        rospy.logerr("Error llamando a %s: %s", service_name, e)
        return
    
    # 3) initialpose para fijar la pose final conocida
    pub_init = rospy.Publisher('/initialpose',
                               PoseWithCovarianceStamped,
                               queue_size=1,
                               latch=True)
    rospy.sleep(0.5)

    # usa aquí tus coordenadas “buenas”
    x  = 0.47278890013694763
    y  = -1.169088363647461
    qz = 0.32774684904650947
    qw = 0.9447655809459214

    msg = make_initial_pose(x, y, qz, qw)

    rospy.loginfo("Publicando pose inicial conocida...")
    rate = rospy.Rate(5)
    for _ in range(5):
        msg.header.stamp = rospy.Time.now()
        pub_init.publish(msg)
        rate.sleep()

    # 2) movimiento para que AMCL tenga lecturas
    rospy.loginfo("Iniciando giro para ayudar a la localización...")
    publish_spin(duration=12.0,
                 switch_interval=6.0,
                 angular_speed=1.0,
                 topic='/mobile_base_controller/cmd_vel',
                 rate_hz=10.0)

    rospy.loginfo("Secuencia global_localization + initialpose terminada.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

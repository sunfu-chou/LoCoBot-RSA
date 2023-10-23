#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from uwb import UWB

uwb = UWB()

distances = Float64MultiArray()


def timer_callback(e):
    # return
    now = rospy.Time.now()
    validate = uwb.validate()
    if not validate:
        rospy.logwarn_throttle(1, uwb.network_id_str + " UWB not validated")
        uwb.connect()
        if uwb.validate():
            rospy.logwarn(uwb.network_id_str + "UWB has reconnected")

    uwb.range_all()

    distances.data = [0.0 for _ in uwb.ranges_all]

    for i in range(len(uwb.ranges_all)):
        if uwb.ranges_all[i].distance != 0 and uwb.ranges_all[i].distance < 200000:
            distances.data[i] = uwb.ranges_all[i].distance

    distances_pub.publish(distances)

    # print(uwb.network_id, distances.data, validate, end='\r')


if __name__ == "__main__":
    rospy.init_node("uwb_ranging", anonymous=False)

    config_path = rospy.get_param("~config_path")
    port = rospy.get_param("~port", None)
    rate = rospy.get_param("~rate", 10)
    network_id = rospy.get_param("~network_id", "0x6A1B")
    # hex to int
    network_id = int(network_id, 16)
    uwb.connect(port, network_id)
    print(f"uwb.network_id: {uwb.network_id_str}")
    uwb.load_env_config(config_path)
    # uwb.write_env_config()

    distances_pub = rospy.Publisher("distances", Float64MultiArray, queue_size=10)
    localization_timer = rospy.Timer(rospy.Duration(1 / rate), timer_callback)
    rospy.spin()

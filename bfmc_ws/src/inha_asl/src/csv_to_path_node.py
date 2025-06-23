#!/usr/bin/env python3
import csv, rospy, os
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def csv_to_path(file_name, frame="map"):
    path = Path()
    path.header.frame_id = frame
    with open(file_name) as f:
        for row in csv.DictReader(f):          # 헤더: x,y,dotted
            p = PoseStamped()
            p.header.frame_id = frame
            p.pose.position.x = float(row["x"])
            p.pose.position.y = float(row["y"])
            p.pose.orientation.w = 1.0         # Yaw 0 (평면 경로)
            path.poses.append(p)
    return path

def main():
    rospy.init_node("csv_path_publisher")
    csv_file  = rospy.get_param("~csv_file")   # csv 파일 경로
    rate_hz   = rospy.get_param("~rate", 1)    # 1 Hz 재송신
    frame_id  = rospy.get_param("~frame_id", "map")

    if not os.path.isfile(csv_file):
        rospy.logerr("CSV file not found: %s", csv_file)
        return

    pub = rospy.Publisher("/global_path", Path, queue_size=1, latch=True)
    path_msg = csv_to_path(csv_file, frame_id)
    r = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        path_msg.header.stamp = rospy.Time.now()
        pub.publish(path_msg)
        r.sleep()

if __name__ == "__main__":
    main()

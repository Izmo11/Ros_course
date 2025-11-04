#!/usr/bin/python3

import rospy, message_filters, csv, argparse, time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range


class Robotposition:
    def __init__(self, duration) -> None:
        self.st = time.time()
        self.duration = duration
        self.ultra1 = message_filters.Subscriber('/robot1/ultrasonic1', Range)
        self.ultra2 = message_filters.Subscriber('/robot1/ultrasonic2', Range)
        self.ultra3 = message_filters.Subscriber('/robot1/ultrasonic3', Range)
        self.ultra4 = message_filters.Subscriber('/robot1/ultrasonic4', Range)
        self.ultra5 = message_filters.Subscriber('/robot1/ultrasonic5', Range)
        self.ultra6 = message_filters.Subscriber('/robot1/ultrasonic6', Range)
        self.odom = message_filters.Subscriber('/robot1/odom', Odometry)

        self.subs = message_filters.ApproximateTimeSynchronizer(
            [self.ultra1, self.ultra2, self.ultra3, self.ultra4, self.ultra5, self.ultra6, self.odom],
            queue_size=1, 
            slop=0.9, 
            allow_headerless=False
        )

        self.subs.registerCallback(self.sensor_cb)

        self.positions_file = open('robot1_position.csv', mode='w')
        self.positions_writer = csv.writer(self.positions_file, delimiter=',')

    def sensor_cb(self, ultra1:Range, ultra2:Range, ultra3:Range, ultra4:Range, ultra5:Range, ultra6:Range, odom:Odometry):
        row = [odom.header.stamp,
                odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z,
                odom.pose.pose.orientation.w,
                ultra1.range,
                ultra2.range,
                ultra3.range,
                ultra4.range,
                ultra5.range,
                ultra6.range,
                odom.pose.pose.position.x,
                odom.pose.pose.position.y]
        self.positions_writer.writerow(row)
        #print(time.time()-self.st)
        if (self.duration>0 and (time.time()-self.st)>self.duration):
            rospy.signal_shutdown("Timer full")
        #print(row)





if __name__ == '__main__':
    print('Starting script...')
    parser = argparse.ArgumentParser()
    parser.add_argument('-t','--time', type=int, default=-1, help="How many seconds to record.")
    args,_ = parser.parse_known_args()
    node = rospy.init_node('pos_saver_ddd')
    Robotposition(args.time)
    rospy.spin()



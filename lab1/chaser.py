#! /usr/bin/env python3

from math import atan2, sqrt
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn

class Chaser:

    def __init__(self) -> None:
        rospy.init_node('chaser_controller')
        rospy.wait_for_service('/spawn')
        spawn_func = rospy.ServiceProxy('/spawn', Spawn)
        spawn_func(4.0, 4.0, 0.0, 'chaser')
        self.current_chaser_pose = Pose(4.0, 4.0, 0.0, None, None)
        self.current_victim_pose = Pose(5.5, 5.5, 0.0, None, None)

        self.speed = float(rospy.get_param('chaser_speed'))

        self.velocity_publisher = rospy.Publisher('/chaser/cmd_vel', Twist, queue_size=10)
        self.chaser_pose_subscriber = rospy.Subscriber('/chaser/pose', Pose, self.update_pose)
        self.victim_pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_victim_pose)

        self.rate = rospy.Rate(20)
        self.distance_tolerance = 1

    def start_chase(self):
        while (not rospy.is_shutdown()):
            self.chase()
            self.rate.sleep()

    def euclidean_distance(self, victim_pose):
        return sqrt(pow((victim_pose.x - self.current_chaser_pose.x), 2) + pow((victim_pose.y - self.current_chaser_pose.y), 2))

    def angular_velocity(self, victim_pose, constant=5):
        return constant * (self.steering_angle(victim_pose) - self.current_chaser_pose.theta)
   
    def linear_velocity(self, victim_pose):
        return self.speed * self.euclidean_distance(victim_pose)

    def steering_angle(self, victim_pose):
        return atan2(victim_pose.y - self.current_chaser_pose.y, victim_pose.x - self.current_chaser_pose.x)

    def chase(self):
        victim_pose = self.current_victim_pose
        vel_msg = Twist()

        if self.euclidean_distance(victim_pose) >= self.distance_tolerance:
            vel_msg.linear.x = self.linear_velocity(victim_pose)
            vel_msg.angular.z = self.angular_velocity(victim_pose)
            self.velocity_publisher.publish(vel_msg)
        else:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)

    def update_pose(self, data):
        self.current_chaser_pose = data
        self.current_chaser_pose.x = round(self.current_chaser_pose.x, 4)
        self.current_chaser_pose.y = round(self.current_chaser_pose.y, 4)

    def update_victim_pose(self, data):
        self.current_victim_pose = data
        self.current_victim_pose.x = round(self.current_victim_pose.x, 4)
        self.current_victim_pose.y = round(self.current_victim_pose.y, 4)

if __name__ == '__main__':
    chaser = Chaser()
    chaser.start_chase()

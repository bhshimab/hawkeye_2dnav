#!/usr/bin/env python
# Adapted from https://github.com/FiorellaSibona/turtlebot3_nav/blob/devel/catkin_ws/src/simple_navigation_goals/scripts/move_base_seq
import rospy
import math
import csv

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry


class MoveBaseSeq():

    def __init__(self):

	# Open the CSV file that holds the inspection route data
	self.read_from_csv()

	# Initialize node (may need to delete/rework)
        rospy.init_node('move_base_sequence')

        # Define lists to be loaded with data from file
        self.pose_seq = list()	# Pose sequence (for navigation)
	self.name_seq = list()	# Names corresponding to each pose
	self.cam_seq = list()	# Camera orientation corresponding at each pose
        self.goal_cnt = 0
	
	# Create the pose sequence
	i = 0
        for point in self.locations:
	    pos_x = self.locations[i][1]
	    pos_y = self.locations[i][2]
	    pos_z = 0
	    quat_x = 0
	    quat_y = 0
	    quat_z = self.locations[i][6]
	    quat_w = self.locations[i][7]

	    # Append pose with data type pose
            self.pose_seq.append(Pose(Point(pos_x,pos_y,pos_z),Quaternion(quat_x,quat_y,quat_z,quat_w)))
	    i+=1

        #rospy.loginfo(str(self.pose_seq))
        #Create action client
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        #wait = self.client.wait_for_server(rospy.Duration(5.0))
        wait = self.client.wait_for_server()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
	self.write_to_csv()
	# Start sending goals
        self.movebase_client()
	print self.position_x

    def read_from_csv(self):
	"""
	Open CSV file containing list of locations

	CURRENT CSV INDEX FORMAT (Pos = position, Quat = quaternion, Cam = camera)
        -------------------------------------------------------------------------
        0    | 1     2     3     | 4      5      6      7      | 8       9
	Name | Pos_X Pos_Y Pos_Z | Quat_X Quat_Y Quat_Z Quat_W | Cam_pan Cam_tilt
        -------------------------------------------------------------------------
	"""
	
	# TODO: Make the filepath more general
        with open('/home/bhshimab/catkin_ws/src/simple_navigation_goals/goals/testgoal1.csv') as file_to_read:
            csvreader = csv.reader(file_to_read, quoting=csv.QUOTE_NONNUMERIC)
        
            # Create 2D list that contains locations.
    	    # Each row is a location, each column is specified above.
            self.locations = []
            for row in csvreader:
                self.locations.append(row)

    def write_to_csv(self):
	with open('/home/bhshimab/catkin_ws/src/simple_navigation_goals/goals/testwrite.csv', 'w') as file_to_write:
            writer = csv.writer(file_to_write, quoting=csv.QUOTE_NONNUMERIC)

	    for row in self.locations:
    	        # write the header
                writer.writerow(row)

    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        #rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
        rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")

    def done_cb(self, status, result):
        self.goal_cnt += 1
    # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached") 
            if self.goal_cnt< len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb) 
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                return

        if status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

    def get_pose(self, position):
        self.position_x = position.pose.pose.position.x
	
    def movebase_client(self):
    #for pose in pose_seq:   
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() 
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)

	rospy.Subscriber('/odom', Odometry, self.get_pose)

        rospy.spin()

if __name__ == '__main__':
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")

#!/usr/bin/env python

import rospy
import roslaunch
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalID
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

import tf

process_generate_running = True

class ProcessListener(roslaunch.pmon.ProcessListener):
    global process_generate_running

    def process_died(self, name, exit_code):
        global process_generate_running
        process_generate_running = False
        rospy.logwarn("%s died with code %s", name, exit_code)


def init_launch(launchfile, process_listener):
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(
        uuid,
        [launchfile],
        process_listeners=[process_listener],
    )
    return launch

def goal_callback(data):
  #global goal_topic
  global pub
  #global pub_cancel
  #global launch

  print('explored!')
  #launch.shutdown()

  #pub_cancel.publish(GoalID())
  rospy.sleep(2)
  sggoal = PoseStamped()
  sggoal.header.frame_id = 'odom'
  sggoal.header.stamp = rospy.Time.now()

  sggoal.pose.position.x = 0.0
  sggoal.pose.orientation.w = 1.0

  pub.publish(sggoal)

  #rospy.loginfo("Car explored all map. Go back to the start line.")



if __name__ == '__main__': 
  try:	
    
    rospy.init_node('final_project_node')
    odom=None
    #canceled_topic = rospy.get_param('~canceled_topic', '/move_base/cancel') 
    #goal_topic = rospy.get_param('~goal_topic', '/move_base_simple/goal')
    rospy.Subscriber('target_point', PointStamped, goal_callback, queue_size=1)
    #rospy.Subscriber('/odom', Odometry,goal_callback_odom, queue_size=1)
    pub_cancel = rospy.Publisher('/move_base/cancel', GoalID, latch=True, queue_size=1)
    pub = rospy.Publisher('/move_base_simple/goal', Marker, queue_size=1)

    rospy.loginfo("Node 'final_project_node' started.\nListening to %s, publishing to %s.", "/move_base/cancel", '/move_base_simple/goal')
    #launch_file="/opt/ros/kinetic/share/explore_lite/launch/explore.launch"
    #launch = init_launch(launch_file, ProcessListener())
    #launch.start()

    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass

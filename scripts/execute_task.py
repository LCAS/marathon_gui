#!/usr/bin/env python

import rospy
import actionlib
from marathon_gui.msg import ExecuteTaskAction, ExecuteTaskGoal
import marathon_gui.marathon_utils as utils
from marathon_gui.srv import CreatePageService
from std_srvs.srv import Empty

class ExecuteTask():
    "A calss to reconfigure the velocity of the DWAPlannerROS."

    def __init__(self, name):
        rospy.loginfo("Starting %s", name)
        self._action_name = name
        create_page_srv_name = '/marathon_web_interfaces/create_page'
        show_default_page_srv_name = '/marathon_web_interfaces/show_default'
        self.create_page_srv = rospy.ServiceProxy(create_page_srv_name, CreatePageService)
        self.show_default_page_srv = rospy.ServiceProxy(show_default_page_srv_name, Empty)
        self.speak = utils.Speak()
        rospy.loginfo("Creating action server.")
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            ExecuteTaskAction,
            self.executeCallback,
            auto_start=False
        )
        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")

    def executeCallback(self, goal):
        if goal.task == 'info':
            rospy.loginfo("Executing %s action" % goal.task)
            self.create_page_srv('nhm-info.html', goal.text)
            self.speak.speak(goal.text)
            rospy.sleep(30.) # Not nice but necessary since mary reports success before pulse has played the sounds.
            self.show_default_page_srv()
        else:
            rospy.loginfo("Unknown action: %s" % goal.task)

        self._as.set_succeeded()


if __name__ == '__main__':
    rospy.init_node("execute_marathon_task")
    ex = ExecuteTask(rospy.get_name())
    rospy.spin()

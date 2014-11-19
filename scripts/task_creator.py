#!/usr/bin/env python

import rospy
import actionlib
from marathon_gui.srv import MapButtonService, MapButtonServiceResponse
from marathon_gui.msg import ExecuteTaskAction, ExecuteTaskGoal

class TaskCreatorServer(object):
    """docstring for TaskCreatorServer"""
    def __init__(self, name):
        self.name = name
        rospy.loginfo("Creating: %s" % name)
        s = rospy.Service('~create_task', MapButtonService, self.create_task)
        # ExecuteTask client
        rospy.loginfo("Creating execute_marathon_task client")
        self.exClient = actionlib.SimpleActionClient(
            'execute_maraton_task',
            ExecuteTaskAction
        )
        self.exClient.wait_for_server()
        rospy.loginfo("...done")
        rospy.loginfo("Ready to create tasks.")

    def create_task(self, req):
        rospy.loginfo("Request to create task: %s" % req.identifier)
        goal = ExecuteTaskGoal()
        goal.task = 'info'
        goal.text = 'test'
        self.exClient.send_goal_and_wait(goal)
        return MapButtonServiceResponse()


if __name__ == "__main__":
    rospy.init_node('task_creator_server')
    tcs = TaskCreatorServer(rospy.get_name())
    rospy.spin()

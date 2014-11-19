#! /usr/bin/env python

import roslib
import rospy

from marathon_gui.page_utils import PageUtils
import marathon_gui.marathon_utils as utils
from marathon_gui.services import TaskDemander
import actionlib
from marathon_gui.srv import MapButtonService, MapButtonServiceResponse, ShowPageService
from marathon_gui.msg import ExecuteTaskAction, ExecuteTaskGoal

from strands_executive_msgs.msg import Task
from strands_executive_msgs import task_utils
import strands_webserver.client_utils
import strands_webserver.page_utils
from std_srvs.srv import Empty
import std_msgs
from mongodb_store.message_store import MessageStoreProxy
from strands_navigation_msgs.msg import TopologicalNode
import json
import pprint
from random import randint

class WebInterface():
    """docstring for WebInterface"""
    def __init__(self, name):
        self.name = name
	    # display some content
    	display_no = rospy.get_param("~display", 0)
        show_default_page_srv_name = '/marathon_web_interfaces/show_default'
        show_page_srv_name = '/marathon_web_interfaces/show_page'
        self.show_page_srv = rospy.ServiceProxy(show_page_srv_name, ShowPageService)

    	if display_no == 0:
	    	rospy.loginfo('writing to all displays')
    	else:
	    	rospy.loginfo('writing to display: %s', display_no)

        behaviour = rospy.get_param("~behaviour", "")
        if behaviour == "":
            rospy.logfatal("Missing parameters.")
            rospy.logfatal("Please run with _behaviour:=<behaviour_name>")
            return

        self.config = self.loadConfig(behaviour)
        self.buttonmappings = self.config['buttonmappings']
        self.move_speeches = self.config['speeches']['moving']
        self.tweets_action = self.config['tweets']['action']
        pprint.pprint(self.config)

        s = rospy.Service('~create_task', MapButtonService, self.create_task)
        # ExecuteTask client - only used to make sure that the action server is
        # there.
        rospy.loginfo("Creating execute_marathon_task client")
        self.exClient = actionlib.SimpleActionClient(
            'execute_marathon_task',
            ExecuteTaskAction
        )
        self.exClient.wait_for_server()
        rospy.loginfo("...done")
        self.speak = utils.Speak()
        rospy.loginfo("Creating PageUtils")
        self.pu = PageUtils(default_page='nhm-map.html', display_no=display_no)
        rospy.loginfo("...done")
        rospy.loginfo("Creating TaskDemander")
        self.td = TaskDemander()
        rospy.loginfo("...done")
        rospy.loginfo("Ready to create tasks.")

        # Setup -- must be done before other interface calls
    	# serves pages relative to marathon_gui/www -- this is important as there as some javascript files there
        show_default_page_srv = rospy.ServiceProxy(show_default_page_srv_name, Empty)
        show_default_page_srv()

    def loadConfig(self, data_set):
        msg_store = MessageStoreProxy(collection="hri_behaviours")
        query_meta = {}
        query_meta["collection"] = data_set
        if len(msg_store.query(std_msgs.msg.String._type, {}, query_meta)) == 0:
            rospy.logerr("Desired dialogue options '"+data_set+"' not in datacentre.")
            raise Exception("Can't find data in datacentre.")
        else:
            message = msg_store.query(std_msgs.msg.String._type, {}, query_meta)
            return json.loads(message[0][0].data)

    def create_task(self, req):
        rospy.loginfo("Request to create task: %s" % req.identifier)
        data = json.loads(req.identifier)

        self.show_page_srv('nhm-moving.html')
        self.speak.speak(self.move_speeches[randint(0, len(self.move_speeches)-1)])

        task = Task()
        action = self.buttonmappings[data['waypoint']]
        waypoint = action['waypoint']
        task.start_node_id = waypoint
        task.action = 'execute_marathon_task'
        task.max_duration = rospy.Duration(10*60)
        task_utils.add_string_argument(task, data['action'])
        task_utils.add_string_argument(task, action[data['action']]['page'])
        task_utils.add_string_argument(task, action[data['action']]['text'])
        task_utils.add_string_argument(task, self.tweets_action[randint(0, len(self.tweets_action)-1)])
        self.td.demand_task(task)
        return MapButtonServiceResponse()


if __name__ == '__main__':
    rospy.init_node("marathon_web_interfaces")
    wi = WebInterface(rospy.get_name())
    rospy.spin()

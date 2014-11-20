#! /usr/bin/env python

import roslib
import rospy

from marathon_gui.page_utils import PageUtils
import marathon_gui.marathon_utils as utils
from marathon_gui.services import TaskDemander
import actionlib
from marathon_gui.srv import ShowPageService
from strands_executive_msgs.msg import Task
from strands_executive_msgs import task_utils
from std_srvs.srv import Empty
from std_msgs.msg import String
from strands_navigation_msgs.srv import PauseResumeNav

class CardInterface():
    """docstring for WebInterface"""
    def __init__(self, name):
        self.name = name
	    # display some content
    	display_no = rospy.get_param("~display", 0)
        show_default_page_srv_name = '/marathon_web_interfaces/show_default'
        show_page_srv_name = '/marathon_web_interfaces/show_page'
        paus_nav_srv_name = '/monitored_navigation/pause_nav'
        self.show_page_srv = rospy.ServiceProxy(show_page_srv_name, ShowPageService)
        self.pause_nav_srv = rospy.ServiceProxy(paus_nav_srv_name, PauseResumeNav)

    	if display_no == 0:
	    	rospy.loginfo('writing to all displays')
    	else:
	    	rospy.loginfo('writing to display: %s', display_no)

        self.speak = utils.Speak()
        self.photo = utils.Photo()
        rospy.loginfo("Creating PageUtils")
        self.pu = PageUtils(default_page='nhm-map.html', display_no=display_no)
        rospy.loginfo("...done")
        rospy.loginfo("Creating TaskDemander")
        self.td = TaskDemander()
        rospy.loginfo("...done")
        rospy.loginfo("Ready to create tasks.")

        # Setup -- must be done before other interface calls
    	# serves pages relative to marathon_gui/www -- this is important as there as some javascript files there
        self.show_default_page_srv = rospy.ServiceProxy(show_default_page_srv_name, Empty)
        self.sub = rospy.Subscriber("/socialCardReader/commands", String, self.create_task)

    def create_task(self, req):
        if req.data == 'UNKNOWN' or req.data == 'PHOTO':
            return
        rospy.loginfo("Request to create task: %s" % req.data)
        self.photo.photo()
        self.speak.speak(req.data)
        if req.data == 'DOCK':
            demanded_wait = Task(action='wait_action', max_duration=rospy.Duration(30*60), start_node_id='ChargingPoint')
            self.td.demand_task(demanded_wait)
        elif req.data == 'WAIT':
            try:
                  s = self.pause_nav_srv(True)
            except rospy.ServiceException as exc:
                  rospy.logwarn("Failed to call PauseResumeNav service. Only possible when robot is navigating.")
        elif req.data == 'PATROL':
            try:
                  s = self.pause_nav_srv(False)
            except rospy.ServiceException as exc:
                  rospy.logwarn("Failed to call PauseResumeNav service. Only possible when robot is navigating.")


if __name__ == '__main__':
    rospy.init_node("marathon_card_interfaces")
    wi = CardInterface(rospy.get_name())
    rospy.spin()

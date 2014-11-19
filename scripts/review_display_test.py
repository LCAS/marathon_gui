#! /usr/bin/env python

import roslib
import rospy

import y1_interfaces.page_utils
from y1_interfaces.services import TaskDemander

from strands_executive_msgs.msg import Task, TaskEvent
import strands_webserver.client_utils
import strands_webserver.page_utils
from std_srvs.srv import Empty
from mongodb_store.message_store import MessageStoreProxy
from strands_navigation_msgs.msg import TopologicalNode

from task_executor import task_query


if __name__ == '__main__':
	rospy.init_node("y1_web_interfaces")

	# display some content
	display_no = rospy.get_param("~display", 0)	

	if display_no == 0:
		rospy.loginfo('writing to all displays)')
	else:
		rospy.loginfo('writing to display: %s', display_no)

	prefix='stats'

	# Setup -- must be done before other y1_interfaces calls
	# serves pages relative to y1_interfaces/www -- this is important as there as some javascript files there
	strands_webserver.client_utils.set_http_root(roslib.packages.get_pkg_dir('y1_interfaces') + '/www', prefix=prefix)

	
	page = 'index.html'

	right_div = '<div id="top_right_div"></div><div id="bottom_right_div"></div>'
	script = y1_interfaces.page_utils.get_schedule_display("#bottom_right_div")
	script += y1_interfaces.page_utils.get_task_event_display("#top_right_div")
	script += y1_interfaces.page_utils.get_demo_stats_display("#left_div")
	script += y1_interfaces.page_utils.get_map_display("bottom_left")
	script += y1_interfaces.page_utils.get_video_display("bottom_right")
	
	y1_interfaces.page_utils.generate_interface_page(page, right=right_div, script=script)	

	strands_webserver.client_utils.display_relative_page(display_no, page, prefix=prefix)
	rospy.spin()

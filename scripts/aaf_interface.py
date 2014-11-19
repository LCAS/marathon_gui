#! /usr/bin/env python

import roslib
import rospy

import y1_interfaces.page_utils
from y1_interfaces.services import TaskDemander

from strands_executive_msgs.msg import Task
import strands_webserver.client_utils
import strands_webserver.page_utils
from std_srvs.srv import Empty
from mongodb_store.message_store import MessageStoreProxy
from strands_navigation_msgs.msg import TopologicalNode



if __name__ == '__main__':
	rospy.init_node("y1_web_interfaces")

	# display some content
	display_no = rospy.get_param("~display", 0)	

	if display_no == 0:
		rospy.loginfo('writing to all displays)')
	else:
		rospy.loginfo('writing to display: %s', display_no)

	# Setup -- must be done before other y1_interfaces calls
	# serves pages relative to y1_interfaces/www -- this is important as there as some javascript files there
	strands_webserver.client_utils.set_http_root(roslib.packages.get_pkg_dir('y1_interfaces') + '/www')

	page = 'info.html'

	# left_html = '<button type="button">Click Me!</button>'
	left_html = '<img src="strands-logo.png">'

	right_html = '<p>Guten Tag! Wollen Sie mehr Informationen zu mir und dem Projekt? Dr&uuml;cken Sie <a href="strands-aaf-info1.html">hier</a>!</p>'
	y1_interfaces.page_utils.generate_interface_page(page, left=left_html, right=right_html)

	strands_webserver.client_utils.display_relative_page(display_no, page)
	
	master = rospy.get_master()

	while not rospy.is_shutdown():

		# wait for the service to go away
		while not rospy.is_shutdown():			
			rospy.sleep(1)

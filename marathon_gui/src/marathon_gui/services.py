import rospy
import re

from strands_executive_msgs import task_utils
from strands_executive_msgs.msg import Task, ExecutionStatus
from strands_executive_msgs.srv import DemandTask, SetExecutionStatus
from marathon_gui.srv import IsDefaultService, ShowPageService
from std_srvs.srv import Empty


class TaskDemander(object):
    """ Create a services that will propose a the given task on demand """
    def __init__(self, with_monitor=True):
        super(TaskDemander, self).__init__()

        demand_task_service = '/task_executor/demand_task'
        set_exe_stat_srv_name = '/task_executor/set_execution_status'
        is_default_srv_name = '/marathon_web_interfaces/is_default_page'
        default_page_srv_name = '/marathon_web_interfaces/show_default'
        show_page_srv_name = '/marathon_web_interfaces/show_page'
        current_schedule_topic = '/current_schedule'
        rospy.loginfo("Demander: Waiting for task_executor service...")
        rospy.wait_for_service(demand_task_service)
        rospy.wait_for_service(set_exe_stat_srv_name)
        rospy.loginfo("Demander: Done")
        self.demand_task_srv = rospy.ServiceProxy(demand_task_service, DemandTask)
        set_execution_status = rospy.ServiceProxy(set_exe_stat_srv_name, SetExecutionStatus)
        set_execution_status(True)
        rospy.loginfo("Demander: Waiting for marathon_web_interfaces services...")
        rospy.wait_for_service(is_default_srv_name)
        rospy.wait_for_service(default_page_srv_name)
        rospy.wait_for_service(show_page_srv_name)
        rospy.loginfo("Demander: Done")
        self.is_default_srv = rospy.ServiceProxy(is_default_srv_name, IsDefaultService)
        self.default_srv = rospy.ServiceProxy(default_page_srv_name, Empty)
        self.show_srv = rospy.ServiceProxy(show_page_srv_name, ShowPageService)
        if with_monitor:
            self.sub = rospy.Subscriber(
                current_schedule_topic,
                ExecutionStatus,
                callback=self.schedule_monitor,
                queue_size=1
            )

    def demand_task(self, task):
        """ Demand a task and catch any exceptions """
        rospy.loginfo('Demanding task of type %s' % task.action)
    	try:
    		self.demand_task_srv(task)
    	except Exception, e:
    		rospy.logwarn(e)

    def schedule_monitor(self, schedule):
        """docstring for schedule_monitor"""
        rospy.logdebug("schedule_monitor triggered")
        if len(schedule.execution_queue) > 0:
            if schedule.execution_queue[0].action == 'ptu_pan_tilt_metric_map'\
                or schedule.execution_queue[0].action == 'strands_image_tweets'\
                or schedule.execution_queue[0].action == 'move_mongodb_entries':
                rospy.loginfo("Schedule Monitor: Found important task is active. Will show currently working page.")
                self.show_srv('nhm-patrol.html')
                rospy.sleep(60)
                return
        if len(schedule.execution_queue) == 0 and self.is_default_srv().default:
            rospy.sleep(60)
            return
        if not schedule.currently_executing and not self.is_default_srv().default\
                or len(schedule.execution_queue) == 0 and not self.is_default_srv().default:
            self.default_srv()
            rospy.loginfo("Schedule Monitor: No tasks active. Will show map for interaction.")
            rospy.sleep(60)
            return
        if schedule.execution_queue[0].action == '' and not self.is_default_srv().default:
            self.default_srv()
            rospy.loginfo("Schedule Monitor: Current task has no action. Assumed to be a simple patrol. Map will be shown.")
            rospy.sleep(60)
            return

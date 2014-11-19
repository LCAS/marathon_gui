#!/usr/bin/env python

import rospy

from strands_executive_msgs import task_utils
from strands_executive_msgs.msg import Task
from strands_executive_msgs.srv import AddTasks, SetExecutionStatus
from std_srvs.srv import Empty



def get_services():
    # get services necessary to do the jon
    add_tasks_srv_name = '/task_executor/add_tasks'
    set_exe_stat_srv_name = '/task_executor/set_execution_status'
    clear_schedule_srv_name = '/task_executor/clear_schedule'
    rospy.loginfo("Waiting for task_executor service...")
    rospy.wait_for_service(add_tasks_srv_name)
    rospy.wait_for_service(set_exe_stat_srv_name)
    rospy.wait_for_service(clear_schedule_srv_name)
    rospy.loginfo("Done")        
    add_tasks_srv = rospy.ServiceProxy(add_tasks_srv_name, AddTasks)
    set_execution_status = rospy.ServiceProxy(set_exe_stat_srv_name, SetExecutionStatus)
    clear_schedule = rospy.ServiceProxy(clear_schedule_srv_name, Empty)
    return add_tasks_srv, set_execution_status, clear_schedule

def create_wait_task(waypoint, wait_duration, start, end):
    max_duration = rospy.Duration(wait_duration)
    wait_task = Task(action='wait_action',start_node_id=waypoint, end_node_id=waypoint, max_duration=max_duration)
    wait_task.start_after = start
    wait_task.end_before = end
    task_utils.add_time_argument(wait_task, rospy.Time())
    task_utils.add_duration_argument(wait_task, max_duration)
    return wait_task

if __name__ == '__main__':
    rospy.init_node("immediate_patrol")

    # get services to call into execution framework
    add_tasks, set_execution_status, clear_schedule = get_services()

    # this removes scheduled tasks and preempts currently excuting tasks
    clear_schedule()

    # some waypoints to visit
    waypoints = [
        'WayPoint2',
        'WayPoint4',
        'WayPoint8',
        'WayPoint11',
        'WayPoint13',
        'WayPoint14',
        'WayPoint16',
        'WayPoint9',
        'WayPoint12',
        'WayPoint3',
    ]

    # turn them into wait tasks
    tasks = []
    start = rospy.get_rostime()
    end = rospy.get_rostime() + rospy.Duration(2 * 60 * 60)
    for wp in waypoints:        
        tasks.append(create_wait_task(wp, 20, start, end))
        start += rospy.Duration(1)
        end += rospy.Duration(1)

    # register task with the scheduler
    task_ids = add_tasks(tasks)

    # Set the task executor is running (probably is by now...)
    set_execution_status(True)

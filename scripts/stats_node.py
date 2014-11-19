#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from strands_executive_msgs.msg import Task, TaskEvent
from datetime import datetime, timedelta
from task_executor import task_routine, task_query
import argparse
from y1_interfaces.msg import DemoStats
import time
from strands_navigation_msgs.msg import NavStatistics, TopologicalNode
import math

def autonomy_duration(start, task_event_store, stats):

    successes = task_query.query_tasks(task_event_store, 
                    start_date=start,
                    event=[TaskEvent.TASK_SUCCEEDED]
                    )

    fails = task_query.query_tasks(task_event_store, 
                    start_date=start,
                    event=[TaskEvent.TASK_FAILED]
                    )

    preempts = task_query.query_tasks(task_event_store, 
                    start_date=start,
                    event=[TaskEvent.TASK_PREEMPTED]
                    )


    results = task_query.query_tasks(task_event_store, 
                    start_date=start,
                    event=[TaskEvent.TASK_STARTED, TaskEvent.TASK_FINISHED, TaskEvent.NAVIGATION_SUCCEEDED]
                    )

    duration = timedelta()
    charge_wait_duration = timedelta()

    started_task_event = TaskEvent()
    count = 0
    dubious = []
    charge_wait_count = 0
    unstarted_count = 0
    start_count = 0
    day_durations = {}



    for i in range(0, len(results)):

        if i > 0:
            previous = results[i-1][0]
        
        task_event = results[i][0]
        
        if i < len(results) - 1:
            next = results[i+1][0]

        start = False
        end = False

        if task_event.event == TaskEvent.TASK_STARTED or task_event.task.task_id != previous.task.task_id:
            start = True
        elif (task_event.event == TaskEvent.TASK_FINISHED or task_event.task.task_id != next.task.task_id or (task_event.task.action == '' and task_event.event == TaskEvent.NAVIGATION_SUCCEEDED)) and started_task_event.task.task_id != 0:
            end = True

        if start:
            started_task_event = task_event
            start_count += 1
        elif end:

            # if we're closing the previous event
            if task_event.task.task_id == started_task_event.task.task_id:
        
                end_time = datetime.utcfromtimestamp(task_event.time.to_sec())
                task_duration = end_time - datetime.utcfromtimestamp(started_task_event.time.to_sec())

                if task_event.task.action == 'wait_action' and task_event.task.start_node_id == 'ChargingPoint':
                    charge_wait_duration += task_duration
                    charge_wait_count += 1
                else:
                                
                    task_date = end_time.date()

                    if task_date in day_durations:
                        day_durations[task_date].append(task_duration)
                    else:
                        day_durations[task_date] = [task_duration]

                    duration += task_duration
                    count += 1

                # make sure we don't look for another end to this task
                started_task_event = TaskEvent()

    
 



    # time of the first task
    stats.autonomy_start = results[0][0].time

    # time of the last task
    stats.autonomy_end = results[-1][0].time



    # how long has been spent autonomous doing things
    stats.autonomy_duration = rospy.Time(duration.total_seconds())

    stats.task_success_count = len(successes)
    stats.task_fail_count = len(fails)
    stats.task_preempt_count = len(preempts)

    # print 'Starts: %s' % start_count
    # print 'Ends: %s (%s + %s)' % (count + charge_wait_count, count, charge_wait_count)

    # print 'S/F/P: %s/%s/%s (%s)' % (len(successes), len(fails), len(preempts), len(successes) + len(fails) + len(preempts)) 


    # print 'Tasks Completed: %s' % count        
    # print 'Task Duration: %s' % duration

    # print 'Charge Waits Finished: %s' % charge_wait_count        
    # print 'Charge Waits Duration: %s' % charge_wait_duration


def nav_stats(start, nav_stats_store, top_map_msg_store, stats):

    meta_query = {}
    meta_query["inserted_at"] = {"$gte": start} 

    nav_stats = nav_stats_store.query(NavStatistics._type, meta_query=meta_query)

    # sort by timestamps
    nav_stats.sort(key=lambda x: (x[1]['inserted_at']))

    if len(nav_stats) == 0:
        print 'No stats found'
    else:    

        top_map_name = nav_stats[0][0].topological_map
      

        nodes = top_map_msg_store.query(TopologicalNode._type, {"pointset" : top_map_name})

        if len(nodes) == 0:
            print 'No nodes found for top map %s' % top_map_name
        else:            

            node_positions = {}
            for node, meta in nodes:
                node_positions[node.name] = node.pose.position

            distance = 0
            duration_secs = 0
            for nav_stat, meta in nav_stats:
                duration_secs +=  nav_stat.operation_time

                if nav_stat.origin != 'Unknown' and nav_stat.final_node != 'Unknown':

                    distance += math.sqrt((node_positions[nav_stat.origin].x - node_positions[nav_stat.final_node].x)**2 + (node_positions[nav_stat.origin].y - node_positions[nav_stat.final_node].y)**2)

            stats.distance_travelled = distance
            stats.time_travelled = rospy.Duration(duration_secs)


if __name__ == '__main__':

    rospy.init_node("review_stats_node")

    
    parser = argparse.ArgumentParser(description='Publishes stats for other nodes to display')
    
    parser.add_argument('start', metavar='S', type=task_query.mkdatetime, nargs='?', default=datetime.utcfromtimestamp(0),
                   help='start datetime of stats, defaults to no start. Formatted "d/m/y H:M" e.g. "06/07/14 06:38"')
    
    print [rospy.get_param("~start", "")]
    args = parser.parse_args(args= [rospy.get_param("~start", "")])

	

    task_event_store = MessageStoreProxy(collection='task_events')
    nav_stat_msg_store = MessageStoreProxy(collection='message_store')
    top_map_msg_store = MessageStoreProxy(collection='topological_maps')
     
    stats_publisher = rospy.Publisher('demo_stats', DemoStats)


    r = rospy.Rate(1)

    while not rospy.is_shutdown():
        try:

            stats = DemoStats()
            autonomy_duration(args.start, task_event_store, stats)
            nav_stats(args.start, nav_stat_msg_store, top_map_msg_store, stats)
            stats_publisher.publish(stats)
            r.sleep()

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


        

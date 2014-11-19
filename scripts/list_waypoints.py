#! /usr/bin/env python

import rospy
import sys

from mongodb_store.message_store import MessageStoreProxy
from strands_navigation_msgs.msg import TopologicalNode
from bson.objectid import ObjectId

def load_nodes(msg_store, map_name): 
    query_meta = {} 
    query_meta["pointset"] = map_name 
    return msg_store.query(TopologicalNode._type, {}, query_meta)


if __name__ == '__main__':

    rospy.init_node("list_waypoints")

    try: 
        msg_store = MessageStoreProxy(collection='topological_maps') 
        top_map = rospy.get_param('topological_map_name')
        rospy.loginfo("Using topological map: %s", top_map)
        nodes = load_nodes(msg_store, top_map)
        

        for node, meta in nodes:

            print node.name

    except KeyError:
        rospy.logfatal("set ros parameter topological_map_name with the name of the topological map")




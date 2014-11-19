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

    rospy.init_node("set_alias")

    try: 
        msg_store = MessageStoreProxy(collection='topological_maps') 
        top_map = rospy.get_param('topological_map_name')
        rospy.loginfo("Using topological map: %s", top_map)
        node_name = sys.argv[1]
        alias = sys.argv[2]
        rospy.loginfo('Adding alias "%s" to %s', alias, node_name)
        nodes = load_nodes(msg_store, top_map)
        

        for node, meta in nodes:

            if node.name == node_name:
                
                meta["alias"] = alias
                msg_store.update(node, meta, message_query = {'_id': ObjectId(meta["_id"])}, meta_query = {},  upsert = False)            
                break

    except KeyError:
        rospy.logfatal("set ros parameter topological_map_name with the name of the topological map")




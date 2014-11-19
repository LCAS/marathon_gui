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

    rospy.init_node("set_save_points")

    try: 
        msg_store = MessageStoreProxy(collection='topological_maps') 
        top_map = rospy.get_param('topological_map_name')
        rospy.loginfo("Using topological map: %s", top_map)
        safe_nodes = sys.argv[1:]
        rospy.loginfo("Setting these nodes as safe: %s", safe_nodes)
        nodes = load_nodes(msg_store, top_map)
        # rospy.loginfo("Get nodes: %s", nodes)

        for node, meta in nodes:

            if node.name in safe_nodes:
                
                if "safe_point" in meta and meta["safe_point"]:
                    print "already safe"
                else:
                    print "make %s safe" % node.name
                    meta["safe_point"] = True
                    msg_store.update(node, meta, message_query = {'_id': ObjectId(meta["_id"])}, meta_query = {},  upsert = False)
                

            

    except KeyError:
        rospy.logfatal("set ros parameter topological_map_name with the name of the topological map")




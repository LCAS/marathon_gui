#!/usr/bin/env python

import rospy
import roslib
import robblog
import robblog.utils

from datetime import *

if __name__ == '__main__':
    rospy.init_node("robblog")

    server_host = rospy.get_param("~host_ip", "127.0.0.1")
    server_port = str(rospy.get_param("~port", 4040))

    # where are the blog files going to be put
    blog_path = roslib.packages.get_pkg_dir('y1_interfaces') + '/content'
    
    # initialise blog
    robblog.utils.init_blog(blog_path)

    # and start serving content at this place
    proc = robblog.utils.serve(blog_path, server_host, server_port)

    try: 
        converter = robblog.utils.EntryConverter(blog_path=blog_path, collection='robblog')
        
        while not rospy.is_shutdown():
            converter.convert(convert_all=False)
            rospy.sleep(1)

    except Exception, e:
                rospy.logfatal(e)
    finally:
        proc.terminate()


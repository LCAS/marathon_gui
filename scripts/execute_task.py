#!/usr/bin/env python

import rospy
import actionlib
from marathon_gui.msg import ExecuteTaskAction, ExecuteTaskGoal
import marathon_gui.marathon_utils as utils
from marathon_gui.srv import CreatePageService, ShowPageService
from std_srvs.srv import Empty
from std_msgs.msg import String
from strands_tweets.msg import SendTweetAction, SendTweetGoal
from image_branding.msg import ImageBrandingAction, ImageBrandingGoal
from sensor_msgs.msg import Image
from strands_navigation_msgs.srv import PauseResumeNav

class ExecuteTask():
    "A calss to reconfigure the velocity of the DWAPlannerROS."

    def __init__(self, name):
        rospy.loginfo("Starting %s", name)
        self._action_name = name
        create_page_srv_name = '/marathon_web_interfaces/create_page'
        show_default_page_srv_name = '/marathon_web_interfaces/show_default'
        show_page_srv_name = '/marathon_web_interfaces/show_page'
        self.create_page_srv = rospy.ServiceProxy(create_page_srv_name, CreatePageService)
        self.show_default_page_srv = rospy.ServiceProxy(show_default_page_srv_name, Empty)
        self.show_page_srv = rospy.ServiceProxy(show_page_srv_name, ShowPageService)
        self.tweet_string = ''
        ##########################################################
        ## The following needs to be uncommented when run on robot
        #self.ptu = utils.PTU()
        #self.head = utils.Head()
        #self.photo = utils.Photo()
        #self.gaze = utils.Gaze()
        #rospy.loginfo("Create twitter client")
        #self.twitterClient = actionlib.SimpleActionClient(
            #'strands_tweets',
            #SendTweetAction
        #)
        #self.twitterClient.wait_for_server()
        #rospy.loginfo("...done")
        #rospy.loginfo("Create branding client")
        #self.brandingClient = actionlib.SimpleActionClient(
            #'image_branding',
            #ImageBrandingAction
        #)
        #self.brandingClient.wait_for_server()
        #rospy.loginfo("...done")
        ##########################################################
        self.speak = utils.Speak()
        paus_nav_srv_name = '/monitored_navigation/pause_nav'
        self.pause_nav_srv = rospy.ServiceProxy(paus_nav_srv_name, PauseResumeNav)
        rospy.loginfo("Creating action server.")
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            ExecuteTaskAction,
            self.executeCallback,
            auto_start=False
        )
        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")
        self.twitter_pub = rospy.Publisher("/marathon_web_interfaces/twitter/message", String, latch=True)

    def executeCallback(self, goal):
        rospy.loginfo("Executing %s action" % goal.task)
        if goal.task == 'info':
            ##########################################################
            ## The following needs to be uncommented when run on robot
            #self.ptu.turnPTU(-180)
            #self.gaze.people()
            ##########################################################
            self.create_page_srv(goal.page, goal.text)
            self.speak.speak(goal.text)
        elif goal.task == 'twitter':
            self.pause_resume_nav(True)
            self.tweet_string = goal.tweet
            ##########################################################
            ## The following needs to be uncommented when run on robot
            #self.ptu.turnPTU(0)
            #self.head.resetHead()
            #self.gaze.people()
            #self.photo.photo()
            #self.sub = rospy.Subscriber("/head_xtion/rgb/image_color", Image, self.imageCallback)
            ##########################################################
            self.twitter_pub.publish(self.tweet_string)
            self.show_page_srv(goal.page)
            self.speak.speak(goal.text)
            self.pause_resume_nav(False)
        else:
            rospy.loginfo("Unknown action: %s" % goal.task)
            self._as.set_aborted()

        rospy.sleep(30.) # Not nice but necessary since mary reports success before pulse has played the sounds.
        self.show_default_page_srv()
        self._as.set_succeeded()

    def imageCallback(self, message):
        brandgoal = ImageBrandingGoal()
        brandgoal.photo = message
        self.brandingClient.send_goal_and_wait(brandgoal)
        br_ph = self.brandingClient.get_result()
        tweetgoal = strands_tweets.msg.SendTweetGoal()
        text = self.tweet_string
        print "tweeting %s" % text
        tweetgoal.text = text
        tweetgoal.with_photo = True
        tweetgoal.photo = br_ph.branded_image
        self.twitterClient.send_goal_and_wait(tweetgoal)
        self.sub.unregister()

    def pause_resume_nav(self, pause):
            try:
                  s = self.pause_nav_srv(pause)
            except rospy.ServiceException as exc:
                  rospy.logwarn("Failed to call PauseResumeNav service. Only possible when robot is navigating.")


if __name__ == '__main__':
    rospy.init_node("execute_marathon_task")
    ex = ExecuteTask(rospy.get_name())
    rospy.spin()

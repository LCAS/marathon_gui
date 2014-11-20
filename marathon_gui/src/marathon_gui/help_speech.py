#! /usr/bin/env python

import rospy

from monitored_navigation.ui_helper import UIHelper

import actionlib
from mary_tts.msg import maryttsAction, maryttsGoal

    
class HelpSpeech(UIHelper):

    def __init__(self):

        self.speaker=actionlib.SimpleActionClient('/speak', maryttsAction)
        got_server=self.speaker.wait_for_server(rospy.Duration(1))
        while not got_server:
            rospy.loginfo("Help via speech is waiting for marytts action...")
            got_server=self.speaker.wait_for_server(rospy.Duration(1))
            if rospy.is_shutdown():
                return
        
        rospy.loginfo("Help via speech got marytts action") 
        
        self.nav_help_speech='I am having problems moving. Please let me pass.'
        self.bumper_help_speech='My bumper is being pressed. Please release it so I can move on!'
        self.magnetic_help_speech="I am too close to these stairs, and afraid to move. Please call one of my handlers"
        self.help_failed_speech='Something is still wrong. Please let me pass through.'
        self.help_finished_speech='Thank you! I will be on my way.'
        
        self.was_helped=False
        
        UIHelper.__init__(self)
        
        rospy.loginfo("Help via speech initialized")
 

    def ask_help(self, failed_component, interaction_service, n_fails):
        if failed_component=='navigation':
            self.call_speech(self.nav_help_speech)
        elif failed_component=='bumper':
            self.call_speech(self.bumper_help_speech)
        elif failed_component=='magnetic_strip':
            self.call_speech(self.magnetic_help_speech)
        
    def being_helped(self, failed_component, interaction_service, n_fails):
        self.was_helped=True
        return
        
    def help_finished(self, failed_component, interaction_service, n_fails):
        if self.was_helped:
            self.call_speech(self.help_finished_speech)
        self.was_helped=False
        
    def help_failed(self, failed_component, interaction_service, n_fails):
        self.call_speech(self.help_failed_speech)
    
   
    def call_speech(self, text):
        self.speaker.send_goal(maryttsGoal(text=text))
        #self.speaker.wait_for_result()

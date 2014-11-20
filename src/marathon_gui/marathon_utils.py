#!/usr/bin/env python

import rospy
import roslib
import actionlib
import mary_tts.msg
import flir_pantilt_d46.msg
from sensor_msgs.msg import JointState
import strands_interaction_behaviours.msg
from sound_play.libsoundplay import SoundClient
from scitos_msgs.msg import HeadLightState
import strands_gazing.msg
from std_msgs.msg import String
from fake_camera_effects.msg import CameraEffectsAction, CameraEffectsActionGoal


class Gaze():
    def __init__(self):
        self.people_closest_topic = '/upper_body_detector/closest_bounding_box_centre'
        # Gaze client
        rospy.loginfo("Creating gaze client")
        self.gazeClient = actionlib.SimpleActionClient(
            'gaze_at_pose',
            strands_gazing.msg.GazeAtPoseAction
        )
        self.gazeClient.wait_for_server()
        rospy.loginfo("...done")

    def people(self):
        goal = strands_gazing.msg.GazeAtPoseGoal
        goal.topic_name = self.people_closest_topic
        goal.runtime_sec = 0
        self.gazeClient.send_goal(goal)

    def preempt(self):
        self.gazeClient.cancel_all_goals()


class Photo():
    def __init__(self):
        # Camera client
        rospy.loginfo("Creating fake camera client")
        self.cameraClient = actionlib.SimpleActionClient(
            'camera_effects',
            CameraEffectsAction
        )
        self.cameraClient.wait_for_server()
        rospy.loginfo("...done")

    def photo(self):
        goal = CameraEffectsActionGoal()
        self.cameraClient.send_goal_and_wait(goal)

class Idle():
    def __init__(self):
        # BehaviourSwitch client
        rospy.loginfo("Creating behaviour_switch client")
        self.bsClient = actionlib.SimpleActionClient(
            'behaviour_switch',
            strands_interaction_behaviours.msg.BehaviourSwitchAction
        )
        self.bsClient.wait_for_server()
        rospy.loginfo("...done")

    def idle(self, look, speak):
        idle_goal = strands_interaction_behaviours.msg.BehaviourSwitchGoal()
        idle_goal.runtime_seconds = 0
        idle_goal.look = look
        idle_goal.speak = speak
        self.bsClient.send_goal(idle_goal)

    def preempt(self):
        self.bsClient.cancel_all_goals()


class Speak():
    def __init__(self):
        # Speak client
        rospy.loginfo("Creating speech client")
        self.speakClient = actionlib.SimpleActionClient(
            'speak',
            mary_tts.msg.maryttsAction
        )
        self.speakClient.wait_for_server()
        rospy.loginfo("...done")

    def speak(self, text):
        speakgoal = mary_tts.msg.maryttsGoal()
        speakgoal.text = text
        self.speakClient.send_goal_and_wait(speakgoal)


class PTU():
    def __init__(self):
        # PTU client
        rospy.loginfo("Creating PTU client")
        self.ptuClient = actionlib.SimpleActionClient(
            'SetPTUState',
            flir_pantilt_d46.msg.PtuGotoAction
        )
        self.ptuClient.wait_for_server()
        rospy.loginfo("...done")

    def turnPTU(self, pan):
        goal = flir_pantilt_d46.msg.PtuGotoGoal()
        goal.pan = pan
        goal.tilt = 10
        goal.pan_vel = 60
        goal.tilt_vel = 60
        self.ptuClient.send_goal(goal)


class Head():
    def __init__(self):
        self.pub = rospy.Publisher('/head/commanded_state', JointState)

    def resetHead(self):
        self.head_command = JointState()
        self.head_command.name = ["HeadPan", "HeadTilt", "EyesTilt", "EyesPan", "EyeLids"]
        self.head_command.position = [0, 0, 0, 0, 100]
        self.pub.publish(self.head_command)

    def turnHead(self):
        self.head_command = JointState()
        self.head_command.name = ["HeadPan", "HeadTilt", "EyesTilt", "EyesPan", "EyeLids"]
        self.head_command.position = [180, 0, 0, 0, 100]
        self.pub.publish(self.head_command)

    def closeEyes(self):
        self.head_command = JointState()
        self.head_command.name = ["EyeLids"]
        self.head_command.position = [0]
        self.pub.publish(self.head_command)

    def openEyes(self):
        self.head_command = JointState()
        self.head_command.name = ["EyeLids"]
        self.head_command.position = [100]
        self.pub.publish(self.head_command)

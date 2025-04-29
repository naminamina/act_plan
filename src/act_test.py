#!/usr/bin/env python3
# coding: utf-8import os
import sys
import rospy
import roslib
import requests
from happymimi_navigation.srv import NaviLocation, NaviCoord
from happymimi_voice_msgs.srv import TTS, YesNo, ActionPlan,SetStrRequest
from happymimi_msgs.srv import StrTrg, StrTrgRequest,SetStr
from std_msgs.msg import Bool, Float64, String

import time
import socketio

sio = socketio.Client()


base_path = roslib.packages.get_pkg_dir('happymimi_teleop') + '/src/'
sys.path.insert(0, base_path)
from base_control import BaseControl



MIMIBASE_URL = os.getenv('MIMIBASE_URL')
MIMIBASE_URL_GET = f"{MIMIBASE_URL}/get_command"

def get_command():
    res = requests.get(MIMIBASE_URL_GET, timeout=5)
    print(MIMIBASE_URL_GET)
    data = res.json()
    return data


def main():

    data = get_command()
    commands = data["commands"]

    for cmd in commands:
        task = cmd["command"]
        args = cmd["arguments"]
        print(task)
        print(args)




class TaskFunction():
    def __init__(self):
        self.navi = rospy.ServiceProxy("/navi_location_server",NaviLocation)
        self.tts_srv = rospy.ServiceProxy('/tts', StrTrg)
        self.yesno = rospy.ServiceProxy('/yes_no', YesNo)
        self.stt_srv = rospy.ServiceProxy('/whisper_stt', SetStr)
        self.arm_srv = rospy.ServiceProxy("/servo/arm", StrTrg)
        self.head_pub = rospy.Publisher("/servo/head", Float64, queue_size=10)

        self.tts_text = None
        self.ans = None   
        self.base_control = BaseControl()


        self.tts_srv = rospy.ServiceProxy('/distil_whisper_stt', SetStr)



    def Navigation(self, place):
        rospy.loginfo("Navigating to: %s", place)
        try:
            self.navi(place)
            rospy.loginfo("Navigation to %s completed", place)
        except Exception as e:
            rospy.logerr("Navigation failed: %s", str(e))


    def Text2Speech(self,text):
        rospy.loginfo("Text to Speech: %s", text)
        try:
            self.tts_srv(text)
            rospy.loginfo("Text to Speech completed")
        except Exception as e:
            rospy.logerr("Text to Speech failed: %s", str(e))


    def ArmPose(self, pose):
        rospy.loginfo("Setting Arm Pose: %s", pose)
        try:
            self.arm_srv(pose)
            rospy.loginfo("Arm Pose completed")
        except Exception as e:
            rospy.logerr("Arm Pose failed: %s", str(e))

    def HeadDown(self):
        rospy.loginfo("Head Down")
        try:    
            self.head_pub.publish(-30.0)
            rospy.loginfo("Head Down completed")
        except Exception as e:
            rospy.logerr("Head Down failed: %s", str(e))

    def HeadUp(self):
        rospy.loginfo("Head Up")
        try:
            self.head_pub.publish(0.0)
            rospy.loginfo("Head Up completed")
        except Exception as e:
            rospy.logerr("Head Up failed: %s", str(e))
        
    def TranslateDest(self, distance):
        rospy.loginfo("Translating Distance: %f", distance)
        try:
            self.base_control.translateDist(distance, 0.2)
            rospy.loginfo("Translation completed")
        except Exception as e:
            rospy.logerr("Translation failed: %s", str(e))


    def RoateAngle(self, angle):
        rospy.loginfo("Rotating Angle: %f", angle)
        try:
            self.base_control.rotateAngle(angle, 0, 0.5)
            rospy.loginfo("Rotation completed")
        except Exception as e:
            rospy.logerr("Rotation failed: %s", str(e))
    
    # def Stt(self): 
    #     ans = self.whisper().result
    #     self.tts_text = ans
    
    
    def Stt_Yesno(self, question):
        rospy.loginfo("STT Yes/No: %s", question)
        try:    
            rospy.loginfo("STT Yes/No: %s", question)
            self.Text2Speech(question)
            self.ans = self.stt_srv(SetStrRequest()).result
            rospy.loginfo("STT Yes/No completed, result: %s", self.ans)
        except Exception as e:
            rospy.logerr("STT Yes/No failed: %s", str(e))
        

    def Stt(self):
        rospy.loginfo("STT")
        try:
            self.ans = self.tts_srv(SetStrRequest()).result
            rospy.loginfo("STT completed, result: %s", self.ans)
        except Exception as e:      
            rospy.logerr("STT failed: %s", str(e))


task_function = TaskFunction()




def main():
    task_function.Navigation("kitchen")
    task_function.Text2Speech("Hello, I am a robot.")
    task_function.ArmPose("Carry")
    task_function.HeadDown()
    task_function.HeadUp()
    task_function.TranslateDest(1.0)
    task_function.RoateAngle(90)
    task_function.Stt()
    task_function.Stt_Yesno("Do you want to continue?")

if __name__ == '__main__':
    main()
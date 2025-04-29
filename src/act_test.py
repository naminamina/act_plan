#!/usr/bin/env python3

import os
import sys
import rospy
import roslib
import requests
from happymimi_navigation.srv import NaviLocation, NaviCoord
from happymimi_voice_msgs.srv import TTS, YesNo, ActionPlan
from happymimi_msgs.srv import StrTrg, StrTrgRequest,SetStr,SetStrRequest
from std_msgs.msg import Bool, Float64, String
import socketio

sio = socketio.Client()

base_path = roslib.packages.get_pkg_dir('happymimi_teleop') + '/src/'
sys.path.insert(0, base_path)

from base_control import BaseControl

MIMIBASE_URL = os.getenv('MIMIBASE_URL')
MIMIBASE_URL_GET = f"{MIMIBASE_URL}/get_command"


from happymimi_voice_msgs.srv import TextToSpeech


class TaskFunction():
    def __init__(self):
        self.navi = rospy.ServiceProxy("/navi_location_server",NaviLocation)
        self.tts_srv = rospy.ServiceProxy('/mimic1/tts/service', TextToSpeech)

        self.yesno = rospy.ServiceProxy('/yes_no', YesNo)
        self.stt_srv = rospy.ServiceProxy('/whisper_stt', SetStr)
        self.arm_srv = rospy.ServiceProxy("/servo/arm", StrTrg)
        self.head_pub = rospy.Publisher("/servo/head", Float64, queue_size=1)


        self.tts_text = None
        self.ans = None   
        self.base_control = BaseControl()


    def Navigation(self, place):
        rospy.loginfo("Navigating to: %s", place)
        try:
            rospy.sleep(2)
            
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

        self.head_pub.publish(-20.0)

        try:    
            # self.head_pub.publish(-20.0)
            rospy.loginfo("Head Down completed")
        except Exception as e:
            rospy.logerr("Head Down failed: %s", str(e))

    def HeadUp(self):
        rospy.loginfo("Head Up")
        try:
            self.head_pub.publish(10.0)
            rospy.loginfo("Head Up completed")
        except Exception as e:
            rospy.logerr("Head Up failed: %s", str(e))

    def HeadSet(self):
        rospy.loginfo("Head Set")
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
     
    def SttYesno(self, question):
        rospy.loginfo("STT Yes/No: %s", question)
        for i in range(3):
            try: 
                   
                rospy.loginfo("STT Yes/No: %s", question)
                self.Text2Speech(question)
                self.ans = self.stt_srv(SetStrRequest()).result
                self.Text2Speech(f"Do you say {self.ans}, please answer yes or no")
                if self.yesno.result():
                    rospy.loginfo("STT Yes/No completed, result: %s", self.ans)
                    break
            except Exception as e:
                rospy.logerr("STT Yes/No failed: %s", str(e))
            


task_function = TaskFunction()



def get_command():
    res = requests.get(MIMIBASE_URL_GET, timeout=5)
    print(MIMIBASE_URL_GET)
    data = res.json()
    return data


def exe_command(task , args):
    if task == "MOVE":
        task_function.Navigating(args)
        return
    
    if task == "TTS":
        task_function.Text2Speech(args)
        return

    if task == "ARMPOSE":
        task_function.ArmPose(args)
        return

    if task == "HEADDOWN":
        task_function.HeadDown(args)
        return

    if task == "HEADUP":
        task_function.HeadUp(args)
        return

    if task == "HEASDSET":
        task_function.HeadSet(args)
        return

    if task == "TRANSLATEDIST":
        task_function.TranslateDest(args)
        return

    if task == "ROATEANGLE":
        task_function.RoateAngle(args)
        return

    if task == "STTYESNO":
        task_function.SttYesno(args)
        return


def main():
    rospy.init_node('act_test')  
    data = get_command()
    commands = data["commands"]

    for cmd in commands:
        task = cmd["command"]
        args = cmd["arguments"]
        print(f"task:{task},args:{args}")
        


# def main():
    # rospy.init_node('act_test')  
    # task_function.Navigation("bed_room")
    # task_function.Text2Speech("Hello, I am a robot.")
    # task_function.ArmPose("carry")
    # task_function.HeadDown()
    # task_function.HeadUp()
    # task_function.TranslateDest(1.0)
    # task_function.RoateAngle(90)
    # # task_function.Stt()
    # task_function.SttYesno("what is your name?")

if __name__ == '__main__':
    main()
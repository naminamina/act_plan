#!/usr/bin/env python3
# coding: utf-8import os
import sys
import rospy
import roslib
import requests
from std_msgs.msg import String
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
MIMIBASE_WS = MIMIBASE_URL.replace("https", "wss") + "/ws"
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
        print(f"Navigaton args:{place}")
        self.navi(place)

    def Text2Speech(self,text):
        print(f"Text2Speech args:{text}")

        self.tts_srv(text)

    def ArmPose(self, pose):
        self.arm_srv(pose)

    def HeadDown(self):
        self.head_pub.publish(-30.0)

    def HeadUp(self):
        self.head_pub.publish(0.0)
        
    def TranslateDest(self, distance):
        self.base_control.translateDist(distance, 0.2)

    def RoateAngle(self, angle):
        self.base_control.rotateAngle(angle, 0, 0.5)

    def Stt(self): 
        ans = self.whisper().result
        self.tts_text = ans
    
    
    def Stt_Yesno(self, question):
        for i in range(3):
            self.Text2Speech(question)
            self.ans = self.stt_srv(SetStrRequest()).result
            self.Text2Speech("Is your answer " + self.ans + " correct?")
            if self.yesno().result:
                return

    def Stt(self):
        ans = self.tts_srv(SetStrRequest()).result

task_function = TaskFunction()


def on_message(ws, message):
    print(message)
    data = json.loads(message)
    print(data)
    try:
        data = json.loads(message)
        task = data.get("command")
        args = data.get("arguments")

        print(f"data:{data},task:{task},args:{args}")
        if str(task) == "Move":
            print("task : Navigation")
            task_function.Navigation(args)

        elif task == "Text2Speech":
            print("task : Text2Speech")
            task_function.Text2Speech(args)
        else:
            print("no task")
    except Exception as e:
        print(f"error:{e}")


def on_open(ws):
    print("Connected")


def on_error(ws, error):
    print(f"Error: {error}")


def on_close(ws, close_status_code, close_msg):
    print("Closed")

def main():
    print("connect")
    ws = websocket.WebSocketApp(
        MIMIBASE_WS,
        on_open=on_open,
        on_message=on_message,
        on_error=on_error,
        on_close=on_close
    )
    ws.run_forever()

if __name__ == '__main__':
    main()
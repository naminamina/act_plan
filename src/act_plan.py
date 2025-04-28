#!/usr/bin/env python3
import os
import rospy
import requests
from std_msgs.msg import String
from happymimi_navigation.srv import NaviLocation, NaviCoord
from happymimi_voice_msgs.srv import TTS, YesNo, ActionPlan
from happymimi_msgs.srv import SetStr
import time
import socketio

sio = socketio.Client()



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


        self.tts_text = None   

    def Navigation(self, place):
        self.navi(place)

    def Text2Speech(self,text):
        self.tts_srv(text)

    # def Stt_Yesno(self):
    #     for i in range(3):
    #         ans = self.yes_no().result

    #         if ans:
    #             return

    #         se

    def Stt(self):
        ans = self.tts_srv(SetStrRequest()).result

task_function = TaskFunction()


@sio.on('command')
def on_command(data):
    print(f"[SocketIO] Received command: {data}")
    task = data['command']
    args = data['arguments']
    
    if task == "Navigation":
        task_function.Navigation(args)
    elif task == "Text2Speech":
        task_function.Text2Speech(args)
    else:
        print(f"Unknown task: {task}")

def main():
    sio.connect(MIMIBASE_URL)
    print("hello connected")
    sio.wait()

if __name__ == '__main__':
    main()
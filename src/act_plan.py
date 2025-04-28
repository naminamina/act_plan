#!/usr/bin/env python3
import os
import rospy
import requests
from happymimi_msgs.srv import  StrTrg
from happymimi_navigation.srv import NaviLocation, NaviCoord
from happymimi_voice_msgs.srv import TTS, YesNo, ActionPlan
from happymimi_msgs.srv import SetStr
import time
# import socketio
import websocket
# sio = socketio.Client()
import json

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


        self.tts_text = None   

    def Navigation(self, place):
        print(f"Navigaton args:{place}")
        self.navi(place)

    def Text2Speech(self,text):
        print(f"Text2Speech args:{text}")

        self.tts_srv(text)



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
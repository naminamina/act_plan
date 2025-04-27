#!/usr/bin/env python3
import os
import rospy
import requests
from std_msgs.msg import String


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
    def __init

if __name__ == '__main__':
    main()
#!/usr/bin/env python
import os
import rospkg
import roslib, rospy
from datetime import datetime
from snc_sensors_publisher.msg import SnCSensorsMsg
from std_msgs.msg import Bool

queue = []
status_queue = []
chair_on_spot = True
use_ble = False

def init():
    global use_ble, chair_on_spot
    rospy.init_node('snc_events_wrapper')
    topic = rospy.get_param("~events_topic", "/snc_sensors/events")
    tv_chair_topic = rospy.get_param("~tv_chair_topic", "room_status_publisher/tv_chair")
    use_ble = rospy.get_param("~use_ble", False)
    if use_ble:
        chair_on_spot = False
        rospy.Subscriber(tv_chair_topic, Bool, tv_chair_callback)
    rospy.Subscriber(topic, SnCSensorsMsg, eventCallback)
    while not rospy.is_shutdown():
        rospy.spin()

def tv_chair_callback(msg):
    global chair_on_spot
    chair_on_spot = msg.data


def eventCallback(msg):
    global chair_on_spot, queue, status_queue

    for event in msg.sensors:
        dt = datetime.now()

        if event.name == 'No activity':
            if not ('No activity' in queue) or not (event.status in status_queue):
                rospack = rospkg.RosPack()
                filename = 'official_log_presence_'+datetime.today().strftime("%d-%m-%Y")+'_'+dt.strftime("%H%M%S%f")+'.csv'
                logs_path = rospack.get_path('snc_events_wrapper') + '/logs/' + filename
                queue.append(event.name)
                status_queue.append(event.status)
                with open(logs_path,'ab+') as f:
                    f.write("No presence in the room timestamp\n")
                    f.write(event.status+'\n')
        elif event.name == 'Going out the room':
            if not ('Going out the room' in queue) or not (event.status in status_queue):
                rospack = rospkg.RosPack()
                filename = 'official_log_going_'+datetime.today().strftime("%d-%m-%Y")+'_'+dt.strftime("%H%M%S%f")+'.csv'
                logs_path = rospack.get_path('snc_events_wrapper') + '/logs/' + filename
                queue.append(event.name)
                status_queue.append(event.status)
                with open(logs_path,'ab+') as f:
                    f.write("Going out of the room timestamp\n")
                    f.write(event.status+'\n')
        elif event.name == 'Going into the room':
            if not ('Going into the room' in queue) or not (event.status in status_queue):
                rospack = rospkg.RosPack()
                filename = 'official_log_coming_'+datetime.today().strftime("%d-%m-%Y")+'_'+dt.strftime("%H%M%S%f")+'.csv'
                logs_path = rospack.get_path('snc_events_wrapper') + '/logs/' + filename
                queue.append(event.name)
                status_queue.append(event.status)
                with open(logs_path,'ab+') as f:
                    f.write("Coming in the room timestamp\n")
                    f.write(event.status+'\n')
        elif ('Watching TV' in event.name):
            if chair_on_spot:
                if not (('Watching TV on chair' in queue or 'Watching TV on sofa' in queue)) or not (event.status in status_queue):
                    rospack = rospkg.RosPack()
                    filename = 'official_log_tv_'+datetime.today().strftime("%d-%m-%Y")+'_'+dt.strftime("%H%M%S%f")+'.csv'
                    logs_path = rospack.get_path('snc_events_wrapper') + '/logs/' + filename
                    queue.append(event.name)
                    status_queue.append(event.status)
                    with open(logs_path,'ab+') as f:
                        f.write("Watching TV timestamp\n")
                        f.write(event.status+'\n')
        elif event.name == 'Finish cooking':
            if not ('Finish cooking' in queue) or not (event.status in status_queue):
                rospack = rospkg.RosPack()
                filename = 'official_log_cooking_'+datetime.today().strftime("%d-%m-%Y")+'_'+dt.strftime("%H%M%S%f")+'.csv'
                logs_path = rospack.get_path('snc_events_wrapper') + '/logs/' + filename
                queue.append(event.name)
                status_queue.append(event.status)
                with open(logs_path,'ab+') as f:
                    f.write("Finished cooking timestamp\n")
                    f.write(event.status+'\n')

        if len(queue) > 10:
            del queue[0]
            del status_queue[0]

if __name__ == '__main__':
    init()

#!/usr/bin/env python
import os
import rospkg
import roslib, rospy
from datetime import datetime
from snc_sensors_publisher.msg import SnCSensorsMsg

queue = []

def init():
    rospy.init_node('snc_events_wrapper')
    topic = rospy.get_param("~events_topic", "/snc_sensors/events")
    rospy.Subscriber(topic, SnCSensorsMsg, eventCallback)
    while not rospy.is_shutdown():
        rospy.spin()

def eventCallback(msg):
    dt = datetime.now()

    for event in msg.sensors:

        if event.name == 'No activity in room':
            if not ('No activity in room' in queue):
                rospack = rospkg.RosPack()
                filename = 'official_log_presence_'+datetime.today().strftime("%d-%m-%Y")+'_'+dt.strftime("%H%M%S")+'.csv'
                logs_path = rospack.get_path('snc_events_wrapper') + '/logs/' + filename
                with open(logs_path,'ab+') as f:
                    f.write("No presence in the room timestamp\n")
                    f.write(event.status+'\n')
        elif event.name == 'Goint out the room':
            if not ('Goint out the room' in queue):
                rospack = rospkg.RosPack()
                filename = 'official_log_going_'+datetime.today().strftime("%d-%m-%Y")+'_'+dt.strftime("%H%M%S")+'.csv'
                logs_path = rospack.get_path('snc_events_wrapper') + '/logs/' + filename
                with open(logs_path,'ab+') as f:
                    f.write("Going out of the room timestamp\n")
                    f.write(event.status+'\n')
        elif event.name == 'Coming in the room':
            if not ('Coming in the room' in queue):
                rospack = rospkg.RosPack()
                filename = 'official_log_coming_'+datetime.today().strftime("%d-%m-%Y")+'_'+dt.strftime("%H%M%S")+'.csv'
                logs_path = rospack.get_path('snc_events_wrapper') + '/logs/' + filename
                with open(logs_path,'ab+') as f:
                    f.write("Coming in the room timestamp\n")
                    f.write(event.status+'\n')
        elif 'Watching TV' in event.name:
            if not ('Watching TV on chair' in queue or 'Watching TV on sofa' in queue):
                rospack = rospkg.RosPack()
                filename = 'official_log_tv_'+datetime.today().strftime("%d-%m-%Y")+'_'+dt.strftime("%H%M%S")+'.csv'
                logs_path = rospack.get_path('snc_events_wrapper') + '/logs/' + filename
                with open(logs_path,'ab+') as f:
                    f.write("Watching TV timestamp\n")
                    f.write(event.status+'\n')
        elif event.name == 'Finish cooking':
            if not ('Finish cooking' in queue):
                rospack = rospkg.RosPack()
                filename = 'official_log_cooking_'+datetime.today().strftime("%d-%m-%Y")+'_'+dt.strftime("%H%M%S")+'.csv'
                logs_path = rospack.get_path('snc_events_wrapper') + '/logs/' + filename
                with open(logs_path,'ab+') as f:
                    f.write("Finished cooking timestamp\n")
                    f.write(event.status+'\n')

        queue.append(event.name)

        if len(queue) > 5:
            del queue[0]

if __name__ == '__main__':
    init()

#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import os, sys

"""
Get the database and publish each line on the topic.
"""
def alma_publish_db():
    sys.stderr.write('IN ALMA PUBLISH DB')
    rospy.init_node("alma_db") # creating a noew called 'alma_db'
    db_pub = rospy.Publisher("alma_node_cmd", String, queue_size=100) # this node is publishing to alma_node_cmd topic, giving messages as strings
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Step once in the logic engine
        cmd_string = 'sr.'
        sys.stdout.write(cmd_string)
        sys.stdout.flush()    

        # Probably need some code here to flush out the response from alma
        
        
        # Ask for the database
        cmd_string = 'sdb.'
        sys.stdout.write(cmd_string)
        sys.stdout.flush()    

        # Get the response, send each line to the topic
        reply = raw_input()
        sys.stdin.flush()
        a = reply.split(':')
        while (not (a[0] == 'alma')):
            db_pub.publish(reply)
            reply = raw_input()
            sys.stdin.flush()
            a = reply.split(':')
        rate.sleep()
        

def main():
    
    alma_publish_db()
    rospy.spin()


if __name__ == '__main__':
    main()

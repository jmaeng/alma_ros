#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import os, sys

# ALMA_BIN="/home/justin/catkin_ws/src/rosalma/scripts/alma"
ALMA_BIN="/home/jyna/Alfred/Alma/alma"

"""
spawn a child process/program, connect my stdin/stdout to child process's
stdout/stdin--my reads and writes map to output and input streams of the
spawned program; much like tying together streams with subprocess module;
"""

def spawn(prog, *args):                       # pass progname, cmdline args
    stdinFd  = sys.stdin.fileno()             # get descriptors for streams
    stdoutFd = sys.stdout.fileno()            # normally stdin=0, stdout=1

    parentStdin, childStdout  = os.pipe()     # make two IPC pipe channels
    childStdin,  parentStdout = os.pipe()     # pipe returns (inputfd, outoutfd)
    pid = os.fork()                           # make a copy of this process
    if pid:
        os.close(childStdout)                 # in parent process after fork:
        os.close(childStdin)                  # close child ends in parent
        os.dup2(parentStdin,  stdinFd)        # my sys.stdin copy  = pipe1[0]
        os.dup2(parentStdout, stdoutFd)       # my sys.stdout copy = pipe2[1]
    else:
        os.close(parentStdin)                 # in child process after fork:
        os.close(parentStdout)                # close parent ends in child
        os.dup2(childStdin,  stdinFd)         # my sys.stdin copy  = pipe2[0]
        os.dup2(childStdout, stdoutFd)        # my sys.stdout copy = pipe1[1]
        args = (prog,) + args
        os.execvp(prog, args)                 # new program in this process
        assert False, 'execvp failed!'        # os.exec call never returns here



"""
Read commands on the topic and send them to alma. 
"""        
def alma_cmd_callback(data):
    cmd_string = data.data
    sys.stdout.write(cmd_string)
    sys.stdout.flush()
    sys.stderr.write('****SENT ' + cmd_string)
    reply = raw_input()
    sys.stdin.flush()
    sys.stderr.write('****GOT:  ' + reply + 'FROM ALMA.')

    a = reply.split(':')
    while (not (a[0] == 'alma')):
        reply = raw_input()
        sys.stdin.flush()
        sys.stderr.write('****GOT:  ' + reply + 'FROM ALMA.')
        a = reply.split(':')

    
    
"""
Get the database and publish each line on the topic.
"""
def alma_publish_db():
    sys.stderr.write('IN ALMA PUBLISH DB')
    db_pub = rospy.Publisher("alma_db", String, queue_size=100)
    rospy.init_node("alma_db") # error is coming from here
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
    # Start alma; my stdin will be alma's stdout and vice versa
    mypid = os.getpid()
    spawn(ALMA_BIN, 'run', 'false', 'debug', '0', '/tmp/alma-debug')
    
    # Listen to alma_node_cmd topic for commands.  Right now they'll just be strings; we'll probably
    # want more structure long term.
    rospy.init_node('alma_node')
    rospy.Subscriber("alma_node_cmd", String, alma_cmd_callback)
    #rospy.spin()
    # 
    alma_publish_db()
    rospy.spin()


if __name__ == '__main__':
    main()    

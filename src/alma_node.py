#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from alma_ros_pkg.msg import AlmaFmla
from alma_ros_pkg.msg import AlmaDB

import os, sys
import tty, termios
import time
import re

# Change this line to point to your alma binary
#ALMA_BIN="/home/jyna/Alfred/Alma/alma"
ALMA_BIN="/home/justin/catkin_ws/src/rosalma/scripts/alma"
DEBUG=False


io = None

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
Class to handle a line of input from alma; along with a boolean indicating whether it's the prompt
"""

class alma_line:
    def __init__(self, line, is_cmd_prompt = False):
        if is_cmd_prompt:
            self.line = ''
            self.is_prompt = True
        else:
            self.line = line
            self.is_prompt = False

"""
Class to handle alma subprocess communication
"""
class alma_io:
    def __init__(self, alma_bin):
        # Start alma; my stdin will be alma's stdout and vice versa
        self.mypid = os.getpid()
        spawn(alma_bin, 'run', 'false', 'debug', '0', '/tmp/alma-debug')
        self.locked = False
        self.at_prompt = False

        
    # Keep reading until we get a prompt
    def wait_for_prompt(self):
        if not self.at_prompt:  
            line = self.read_line()
            while not line.is_prompt:   line = self.read_line()
            self.at_prompt = True
            
    # writes a command out to alma to execute.
    def write(self, command):
        while self.locked:  time.sleep(0.1)
        self.wait_for_prompt()
        self.locked = True
        if DEBUG:
            sys.stderr.write("Sending " + command + "\n")
        sys.stdout.write(command + '\n')
        sys.stdout.flush()
        self.locked = False
        self.at_prompt = False

    
    def getch(self):
        a = sys.stdin.read(1)
        return a

    # Read a line
    def read_line(self):
        while self.locked:
            if DEBUG: sys.stderr.write('read locked! ')
            time.sleep(0.1)
        self.locked = True
        num_ch = 0
        line = []
        ch = self.getch()
        while ch != '\n':
            line += ch
            num_ch += 1
            if (num_ch == 5) and (line == ['a', 'l', 'm', 'a', ':']):
                self.locked = False
                if DEBUG: sys.stderr.write("Read line " + ''.join(line) + " (==prompt) from alma.\n")
                self.at_prompt = True
                return alma_line('', True)
            ch = self.getch()
        self.locked = False
        if DEBUG: sys.stderr.write("Read line: " + ''.join(line) + " from alma.\n")
        self.at_prompt = False
        return alma_line(''.join(line)  , False)
        
                
"""
Read commands on the command topic and send them to alma. 
"""        
def alma_cmd_callback(data):
    global io
    cmd_string = data.data
    io.write(cmd_string)



"""
Periodically advance the time step and publish the new database
"""
def alma_publish_db():
    global io

    # We'll publish once every 4 seconds
    db_pub = rospy.Publisher("alma_db", AlmaDB, queue_size=100)
    rate = rospy.Rate(0.25)

    while not rospy.is_shutdown():
        db_list = []   # All the raw lines of the database
    
        # Step once in the logic engine
        if DEBUG: sys.stderr.write('STEPPING')
        io.write('sr.')
        if DEBUG: sys.stderr.write('SENT SR')
        
        # Ask for the database
        io.write('sdb.')
        if DEBUG: sys.stderr.write('SENT SDB')
        
        # Add the lines to db_list and process the list (ignoring some lines)
        line = io.read_line()
        while not line.is_prompt:
            if (line.line.find("idling") == -1) and (line.line.find(":") != -1) :
            	db_list.append(line.line);
            line = io.read_line()
        db_pub.publish(list_to_msg(db_list));
        rate.sleep()
        
"""
Take the list of database lines, convert it to an array of AlmaFmla messages.
"""
def list_to_msg(db_list):
    msg_dict = {}

    # Process list; every formula goes in.  It will be trusted unless otherwise noted
    parse = re.compile(".*distrusted\((\\d+),(\\d+)\)")
    for line in db_list:
        linep = line.split(':')
        code = int(linep[0])
        fmla = linep[1]
        
        trusted = True
        # Is this a distrust formula?  If so, mark the distrusted formula as distrusted.
        # Note that we cannot assume the distrusted formula is already in the message
        # dictionary.
        m = parse.match(fmla)
        if m:
            referent = int(m.group(1))
            timed = int(m.group(2)) # Not used right now, but that info is there
            if msg_dict.has_key(referent):
                msg_dict[referent][1] = False
            else:
                msg_dict[referent] = ["", False]

        # In any case, add the formula and mark it trusted *unless* we've already marked it
        if msg_dict.has_key(code):
            msg_dict[code][0] = fmla
        else:
            msg_dict[code] = [fmla, True]

    # Now iterate through the dictionary and add each entry to the list.
    db = AlmaDB()
    for code, val in msg_dict.iteritems():
        alma_msg = AlmaFmla()
        alma_msg.code = code
        alma_msg.fmla = msg_dict[code][0]
        alma_msg.trusted = msg_dict[code][1]
        db.entries.append(alma_msg)
    return db

def main():
    global io
    # Listen to alma_node_cmd topic for commands.  Right now they'll
    # just be strings; we'll probably want more structure long term.
    rospy.init_node('alma_node')
    io = alma_io(ALMA_BIN)
    # this node called 'alma_node' subscribes to the ros topic called
    # 'alma_node_cmd' and invokes with the message as the first arg.
    rospy.Subscriber("alma_node_cmd", String, alma_cmd_callback)
    
    alma_publish_db()
    rospy.spin()


if __name__ == '__main__':
    main()    


#TODO
#Structure the subscriber to take messages with two parts:  
# i.  The kind of thing we want to do (add a formula or delete would be the two logical places to start), 
# ii.  The actual formula

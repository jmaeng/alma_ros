#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import os, sys
import tty, termios
import time

ALMA_BIN="/home/justin/catkin_ws/src/rosalma/scripts/alma"
DEBUG=False
#ALMA_BIN="/home/jyna/Alfred/Alma/alma"

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


class alma_line:
    def __init__(self, line, is_cmd_prompt = False):
        if is_cmd_prompt:
            self.line = ''
            self.is_prompt = True
        else:
            self.line = line
            self.is_prompt = False

class alma_io:
    def __init__(self, alma_bin):
        # Start alma; my stdin will be alma's stdout and vice versa
        self.mypid = os.getpid()
        spawn(alma_bin, 'run', 'false', 'debug', '0', '/tmp/alma-debug')
        self.locked = False
        self.at_prompt = False

        

    def wait_for_prompt(self):
        if not self.at_prompt:  
            line = self.read_line()
            while not line.is_prompt:   line = self.read_line()
            self.at_prompt = True

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
        
    # def getch(self):
    #     """getch() -> key character
    
    #     Read a single keypress from stdin and return the resulting
    #     character.  Nothing is echoed to the console. This call will
    #     block if a keypress is not already available, but will not wait
    #     for Enter to be pressed. 

    #     If the pressed key was a modifier key, nothing will be detected; if
    #     it were a special function key, it may return the first character of
    #     of an escape sequence, leaving additional characters in the buffer.
        
    #     Copied from
    #     http://code.activestate.com/recipes/577977-get-single-keypress/
    #     """
    #     fd = sys.stdin.fileno()
    #     old_settings = termios.tcgetattr(fd)
    #     try:
    #         tty.setraw(fd)
    #         ch = sys.stdin.read(1)
    #     finally:
    #         termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    #     return ch

    def getch(self):
        a = sys.stdin.read(1)
        return a

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
Read commands on the topic and send them to alma. 
"""        
def alma_cmd_callback(data):
    global io
    cmd_string = data.data
    io.write(cmd_string)



"""
Get the database and publish each line on the topic.
"""
def alma_publish_db():
    global io
    
    sys.stderr.write('IN ALMA PUBLISH DB')
    db_pub = rospy.Publisher("alma_db", String, queue_size=100)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        # Step once in the logic engine
        if DEBUG: sys.stderr.write('STEPPING')
        io.write('sr.')
        if DEBUG: sys.stderr.write('SENT SR')
        
        # Ask for the database
        io.write('sdb.')
        if DEBUG: sys.stderr.write('SENT SDB')

        # Get the response, send each line to the topic
        line = io.read_line()
        if DEBUG: sys.stderr.write('Got line: ' + line.line)
        while not line.is_prompt:
            db_pub.publish(line.line)
            if DEBUG: sys.stderr.write('Sent line: ' + line.line)
            line = io.read_line()
        rate.sleep()


def main():
    global io
    # Listen to alma_node_cmd topic for commands.  Right now they'll just be strings; we'll probably
    # want more structure long term.
    rospy.init_node('alma_node')
    io = alma_io(ALMA_BIN)
    rospy.Subscriber("alma_node_cmd", String, alma_cmd_callback) # this node called 'alma_node' subscribes to the ros topic called 'alma_node_cmd' and invokes with the message as the first arg.
    alma_publish_db()
    rospy.spin()


if __name__ == '__main__':
    main()    

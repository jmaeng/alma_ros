# alma_ros

Be aware that the ALMA_BIN var in alma_node.py has been changed to point to where alma is on my own local comp. Please remember to change it before testing this code on your computer.


How to run for our demo purposes

1) Change the directory path of where alma is on your specific local computer in the alma_node.py file
      1.1) You should have cloned the Alma git repository into your local computer already -- remember use the swi branch only
2) Source your ros workspace by : source ~/MyWorkspace/devel/setup.bash
    2.1) You can add this line to your bashrc file to avoid always writing this command
3) Make alma_node.py executable by doing this command: chmod +x alma_node.py
4) If you want to run alma do this command: ./alma run false keyboard true alfile demo/time1.pl debug 0 /tmp/alma-debug

5) Launch the ros launch file to avoid extra commands of running alma_node.py by : roslaunch <package name> alma_launch.launch
    5.1)Remember to change your package name if it not the same as mine (which is alma_ros_pkg) 
    5.2) This command will run alma_node.py
    
6) To give the alma node some commands here are some examples:
    6.1) To publish to our rostopic alma_node_cmd do any of the following commands with example alma commands as input
            rostopic pub -1 /alma_node_cmd std_msgs/String -- 'af(q).'
            rostopic pub -1 /alma_node_cmd std_msgs/String -- 'af(if(p,q)).'
            rostopic pub -1 /alma_node_cmd std_msgs/String -- 'af(p).'
            rostopic pub -1 /alma_node_cmd std_msgs/String -- 'af(p).'
    6.2) To see what is being published in alma_db rostopic do the following command
            rostopic echo /alma_db

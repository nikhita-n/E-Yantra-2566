<h3>Task 1</h3>
<br>
<br>
Copy the packages pkg_ros_iot_bridge and pkg_task1 in your catkin_ws/src
<br>
and run these commands in catkin_ws:

`$catkin build` and `$source devel/setup.bash`


if the files aren't executable, run this in both the packages:
<br>
`$chmod +x *` 
<br>
<br>
You might also need to install python 2.7 requests 
<br>and paho mqtt client libraries if not already installed.
<br>
<br>To see the output,<br>
Run the launch file this way:<br>
`$roslaunch pkg_task1 task1.launch`<br><br>
Two windows will open: Hive MQTT Client page and an excel sheet<br>
In the  HiveMQ MQTT Client window, subscribe to<br>
`eyrc/rSnNRsNn/ros_to_iot` to get messages from the `node_iot_action_client_turtle`<br>
From HiveMQ MQTT Client send the message `start` on MQTT Topic `eyrc/rSnNRsNn/iot_to_ros`<br>
The turtle will then start tracing a hexagon!<br>
You can see the coordinates getting updated on the excel sheet and under messages in Hive MQTT Client<br>
<br>
Our Task 1 output video URL(unlisted):<br>

https://youtu.be/dkiJjR7uH-U
<br>

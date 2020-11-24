<h3>Task 2</h3>
<br>
<br>
Copy the 3 packages in your catkin_ws/src
<br>
and run these commands in your catkin workspace directory:
<br>
`$catkin build` and `$source devel/setup.bash`
<br>
if the files aren't executable, run this in the package script folders:
<br>
`$chmod +x *` 
<br>
<br>
In short, You need the three packages:<br>
`pkg_moveit_ur5_1 ` , `pkg_task2` and `vb_simulation_pkgs`
<br>

<br>
To see the output, run this:<br>
`$roslaunch pkg_task2 task2.launch`
<br>
<br>
To run the main python node, run this:<br>
`$rosrun pkg_task2 node_t2_ur5_1_pick_place.py`
<br>

<br>
Our Task 2 output video URL(unlisted):<br>
https://youtu.be/gwIhEvKGKQA<br>

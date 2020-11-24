#!/usr/bin/env python

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import actionlib
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import Header
from std_msgs.msg import Bool
# from std_srvs.srv import Empty

from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import vacuumGripperRequest
from pkg_vb_sim.srv import vacuumGripperResponse

from pkg_vb_sim.msg import LogicalCameraImage

def gripper_status(msg):
    if msg.data:
        return True
        # print('gripper status = {}'.format(msg.data))

def gripper_on():
    # Wait till the srv is available
    rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
        # Use this handle just like a normal function and call it
        resp = turn_on(True)
        return resp
    except rospy.ServiceException, e:
        print ("Service call failed: %s" %(e,))

def gripper_off():
    rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')
    try:
        turn_off = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
        resp = turn_off(False)
        return resp
    except rospy.ServiceException, e:
        print ("Service call failed: %s" %(e,))

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
    
    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('node_t2_ur5_1_pick_place', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "ur5_1_planning_group"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=30)
    rr= rospy.Rate(3)
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    # See the /home/somayaji/catkin_ws/src/vb_simulation_pkgs/pkg_vb_sim/config/config_vacuum_gripper.yaml
    # for more details on vacuum gripper
    # vacuum_gripper_model_name: "ur5_1"
    # vacuum_gripper_link_name: "vacuum_gripper_link"
    # eef_link = "vacuum_gripper_link"

    print("============ End effector link: %s" % eef_link)
    eef_link = "vacuum_gripper_link"
    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("============ Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state")
    print(robot.get_current_state())
    touch_links = robot.get_link_names(group="ur5_1_planning_group")
    print("============ Touch links:",touch_links)
    print("")

    # Misc variables
    self.box_name = 'package$1'
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def go_to_joint_state(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = 0
    joint_goal[5] = 0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()
    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal1(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()

    pose_goal.orientation.x =  0.027
    pose_goal.orientation.y =  0.001
    pose_goal.orientation.z = -0.027
    pose_goal.orientation.w = 0.999
    pose_goal.position.x = 0.817312194815
    pose_goal.position.y = 0.108
    pose_goal.position.z = 0.944

    move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def go_to_pose_goal2(self):
    move_group = self.move_group
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal = geometry_msgs.msg.Pose()

    pose_goal.orientation.x = 0.027
    pose_goal.orientation.y = 0.000
    pose_goal.orientation.z = -0.027
    pose_goal.orientation.w = 0.999

    pose_goal.position.x= 0.00075802653219
    pose_goal.position.y= 0.24582226028
    pose_goal.position.z= 1.91234099423

    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()

    move_group.clear_pose_targets()

    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def go_to_pose_goal3(self):
    move_group = self.move_group

    pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.position.x = -0.414925357653
    # pose_goal.position.y = 0.284932768677
    # pose_goal.position.z = 1.08927849967
    pose_goal.position.x = -0.799584331372
    pose_goal.position.y = -0.101489614447
    pose_goal.position.z = 1.33497401972

    # roll: -0.0845543747753
    # pitch: -0.00808617063412
    # yaw: 3.14125785657
    pose_goal.orientation.x = -0.001
    pose_goal.orientation.y = -0.000
    pose_goal.orientation.z = 1
    pose_goal.orientation.w = 0.027

    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()

    move_group.clear_pose_targets()

    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.

    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ## Ensuring Collision Updates Are Received
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL


  def add_box(self, timeout=6):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    self.box_name = 'package$1'
    box_name = self.box_name
    scene = self.scene


    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene between the fingers:
    box_pose = geometry_msgs.msg.PoseStamped()
    
    # box_pose.header.frame_id = "panda_hand"
    # box_pose.header.frame_id = "/base_link"
    box_pose.header.frame_id = "world"

    # box_pose.pose.orientation.w = 0.0
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.orientation.x = 0.0
    box_pose.pose.orientation.y = 0.0
    box_pose.pose.orientation.z = 0.0
    # box_pose.pose.position.x = 0.036260 
    # box_pose.pose.position.y = 0.306889 
    # box_pose.pose.position.z = 1.965703
    # box_pose.pose.position.x = 0.036260+0.04 
    # box_pose.pose.position.y = 0.306889+0.76
    # box_pose.pose.position.z = 1.965703-0.05
    box_pose.pose.position.x = 0.01 
    box_pose.pose.position.y = 0.45311
    box_pose.pose.position.z = 1.91
    # box_name = "box"
    box_name = 'package$1'
    scene.add_box(box_name, box_pose, size=(0.15, 0.15, 0.15))

    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def attach_box(self, timeout=6):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    ## Attaching Objects to the Robot

    ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
    ## robot be able to touch them without the planning scene reporting the contact as a
    ## collision. By adding link names to the ``touch_links`` array, we are telling the
    ## planning scene to ignore collisions between those links and the box. For the Panda
    ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
    ## you should change this value to the name of your end effector group name.
    # grasping_group = 'hand'
    grasping_group = "ur5_1_planning_group"
    touch_links = robot.get_link_names(group=grasping_group)
    print(touch_links)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

  def detach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link

    ## Detaching Objects from the Robot

    ## We can also detach and remove the object from the planning scene:
    scene.remove_attached_object(eef_link, name=box_name)

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

  def remove_box(self, timeout=4):
    box_name = self.box_name
    scene = self.scene

    scene.remove_world_object(box_name)

    ## **Note:** The object must be detached before we can remove it from the world
    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


def main():
  try:
    
    #begin the tutorial by setting up the moveit_commander
    tutorial = MoveGroupPythonIntefaceTutorial()
    #xecute a movement using a joint state goal
    rospy.sleep(2)


    # tutorial.go_to_pose_goal1()
    # rospy.sleep(8)
    # tutorial.go_to_joint_state()
    #add a box to the planning scene
    # rospy.sleep(4)


    # tutorial.add_box()
    # rospy.sleep(11)
    # # tutorial.attach_box()
    # # rospy.sleep(6)
    # #execute a movement using a pose goal
    # tutorial.go_to_pose_goal2()
    # rospy.sleep(11)
    # gripper_on()
    # rospy.sleep(3)
    # tutorial.attach_box()
    # # rospy.sleep(6)
    # rospy.sleep(15)
    # #execute a movement using a pose goal
    # tutorial.go_to_pose_goal3()
    # rospy.sleep(13)
    # gripper_off()
    # rospy.sleep(3)
    # tutorial.detach_box()
    # rospy.sleep(3)
    # tutorial.remove_box()
    # rospy.sleep(2)

    tutorial.go_to_pose_goal2()
    rospy.sleep(1)
    tutorial.add_box()
    tutorial.attach_box()
    gripper_on()

    tutorial.go_to_pose_goal3()
    rospy.sleep(1)
    tutorial.detach_box()
    tutorial.remove_box()
    gripper_off()

    
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

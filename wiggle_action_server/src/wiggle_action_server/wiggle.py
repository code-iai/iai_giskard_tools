#!/usr/bin/env python
import pylab as plt
from mpl_toolkits.mplot3d import Axes3D

import actionlib
import rospy
import numpy as np
from copy import deepcopy

from actionlib_msgs.msg._GoalStatus import GoalStatus
from actionlib_msgs.msg._GoalStatusArray import GoalStatusArray
from geometry_msgs.msg._Point import Point
from geometry_msgs.msg._PoseStamped import PoseStamped
from geometry_msgs.msg._Quaternion import Quaternion
from giskard_msgs.msg._ArmCommand import ArmCommand
from giskard_msgs.msg._SemanticFloat64 import SemanticFloat64
from giskard_msgs.msg._WholeBodyCommand import WholeBodyCommand
from std_msgs.msg._Header import Header
from tf.transformations import quaternion_about_axis, quaternion_multiply
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from wiggle_msgs.msg._WiggleAction import WiggleAction
from wiggle_msgs.msg._WiggleFeedback import WiggleFeedback
from wiggle_msgs.msg._WiggleGoal import WiggleGoal


class WIGGLE(object):
    def __init__(self, save=True,
                 cycle_time=.5,
                 steps=20):
        self.steps = steps
        self.steps_size = 2 * np.pi / self.steps
        self.change_cycle_time(cycle_time)
        self.is_giskard_active = False

        self.giskard_goals = rospy.Publisher('/whole_body_controller/goal', WholeBodyCommand, queue_size=10)
        # self.giskard_client = actionlib.SimpleActionClient('controller_action_server/move', WholeBodyAction)
        # self.giskard_client.wait_for_server()
        self.pose_pub = rospy.Publisher('/test', PoseStamped, queue_size=10)
        self.wiggle_counter = 0.
        self.tfBuffer = Buffer()
        self.tf = TransformListener(self.tfBuffer)
        self.save = save

        self.wiggle_action_server = actionlib.SimpleActionServer('wiggle_wiggle_wiggle', WiggleAction,
                                                                 execute_cb=self.wiggle_cb, auto_start=False)
        self.wiggle_action_server.start()
        self.giskard_status = rospy.Subscriber('/controller_action_server/move/status', GoalStatusArray,
                                               self.giskard_status_cb, queue_size=10)
        rospy.sleep(1.)

    def giskard_status_cb(self, data):
        if len(data.status_list) > 0:
            self.is_giskard_active = data.status_list[-1].status == GoalStatus.ACTIVE

    def change_cycle_time(self, new_cycle_time):
        self.rate = int(self.steps / new_cycle_time)

    def transformPose(self, target_frame, pose):
        transform = self.tfBuffer.lookup_transform(target_frame,
                                                   pose.header.frame_id,  # source frame
                                                   rospy.Time(0),  # get the tf at first available time
                                                   rospy.Duration(1.0))
        new_pose = do_transform_pose(pose, transform)
        return new_pose

    def wiggle_cb(self, goal):
        # check for invalid goals
        if goal.arm != WiggleGoal.LEFT_ARM and goal.arm != WiggleGoal.RIGHT_ARM:
            rospy.logerr('arm goal should be {} or {}'.format(WiggleGoal.LEFT_ARM, WiggleGoal.RIGHT_ARM))
            return self.wiggle_action_server.set_aborted(
                text='arm goal should be {} or {}'.format(WiggleGoal.LEFT_ARM, WiggleGoal.RIGHT_ARM))
        if np.linalg.norm(np.array([goal.goal_pose.pose.orientation.x,
                                    goal.goal_pose.pose.orientation.y,
                                    goal.goal_pose.pose.orientation.z,
                                    goal.goal_pose.pose.orientation.w])) < 0.99:
            rospy.logerr('invalid orientation in goal position')
            return self.wiggle_action_server.set_aborted(
                text='invalid orientation in goal position')

        rospy.loginfo('wiggle wiggle wiggle')
        movement_duration = goal.timeout.data.to_sec()
        wiggle_per_sec = goal.cycle_time
        number_of_wiggles = int(movement_duration / wiggle_per_sec)
        number_of_wiggle_points = 10 
        radius_x = goal.upperbound_x
        radius_y = goal.upperbound_y
        hz = number_of_wiggle_points / wiggle_per_sec

        r = rospy.Rate(hz)

        tcp_goal_pose = self.transformPose('left_gripper_tool_frame', goal.goal_pose)
        spiral = self.create_spiral(goal_height=tcp_goal_pose.pose.position.z,
                                    radius_x=radius_x,
                                    radius_y=radius_y,
                                    number_of_points=number_of_wiggle_points,
                                    number_of_wiggles=number_of_wiggles)
        header = Header()
        header.frame_id = 'left_gripper_tool_frame'
        pose_list = self.np_array_to_pose_stampeds(spiral, Quaternion(0, 0, 0, 1), header)
        pose_list = [deepcopy(self.transformPose('base_footprint', pose)) for pose in pose_list]

        end_pose = self.transformPose('base_footprint', goal.goal_pose)

        for i, pose in enumerate(pose_list):
            if self.check_for_stop():
                return
            self.pub_wiggle(pose, goal.arm)
            wiggle_feedback = WiggleFeedback()
            wiggle_feedback.progress = .9 * (i + 1) / float(len(pose_list))
            self.wiggle_action_server.publish_feedback(wiggle_feedback)
            r.sleep()

        # move back to real goal
        time_to_get_to_endpose = rospy.Duration(1.0)
        deadline = rospy.get_rostime() + time_to_get_to_endpose
        while rospy.get_rostime() < deadline:
            if self.check_for_stop():
                return
            self.pub_wiggle(end_pose, goal.arm)

        result = WiggleFeedback()
        result.progress = 1.
        self.wiggle_action_server.publish_feedback(result)
        self.wiggle_action_server.set_succeeded(result)

    def check_for_stop(self):
        if self.wiggle_action_server.is_preempt_requested():
            self.wiggle_action_server.set_preempted(WiggleFeedback(), 'goal canceled')
            return True
        if self.is_giskard_active:
            self.wiggle_action_server.set_aborted(WiggleFeedback(),
                                                  'goal canceled because giskard action server is doing stuff')
            return True
        return False

    def pub_wiggle(self, goal_pose, arm):
        if self.save:
            self.wiggle_wiggle_wiggle(goal_pose, arm)
        self.pose_pub.publish(goal_pose)

    def goal_pose_to_cmd(self, goal_pose, arm):
        moving_arm_cmd = ArmCommand()
        moving_arm_cmd.type = ArmCommand.CARTESIAN_GOAL
        moving_arm_cmd.goal_pose = goal_pose
        asdf = SemanticFloat64()
        asdf.semantics = 'l_rot_error'
        asdf.value = 0.0174
        moving_arm_cmd.convergence_thresholds.append(asdf)
        asdf = SemanticFloat64()
        asdf.semantics = 'l_trans_error'
        asdf.value = 0.001
        moving_arm_cmd.convergence_thresholds.append(asdf)

        nonmoving_arm_cmd = ArmCommand()
        right_goal = PoseStamped()
        right_goal.header.frame_id = 'right_gripper_tool_frame'
        right_goal.pose.orientation.w = 1
        nonmoving_arm_cmd.type = ArmCommand.CARTESIAN_GOAL
        nonmoving_arm_cmd.goal_pose = self.transformPose('base_footprint', right_goal)

        cmd = WholeBodyCommand()
        cmd.type = WholeBodyCommand.STANDARD_CONTROLLER
        if arm == WiggleGoal.LEFT_ARM:
            cmd.left_ee = moving_arm_cmd
            cmd.right_ee = nonmoving_arm_cmd
        else:
            cmd.left_ee = nonmoving_arm_cmd
            cmd.right_ee = moving_arm_cmd
        return cmd

    def wiggle_wiggle_wiggle(self, goal_pose, arm):
        if arm == WiggleGoal.LEFT_ARM or arm == WiggleGoal.RIGHT_ARM:
            cmd = self.goal_pose_to_cmd(goal_pose, arm)
            self.giskard_goals.publish(cmd)
        else:
            rospy.logerr('arm goal should be {} or {}'.format(WiggleGoal.LEFT_ARM, WiggleGoal.RIGHT_ARM))

    def create_spiral(self, goal_height=2., radius_x=1., radius_y=1., number_of_points=10, number_of_wiggles=10):
        wiggle_height = goal_height / number_of_wiggles
        wiggles = []
        wiggle_template = self.spiral_round(radius_x, radius_y, wiggle_height, number_of_points)
        for i in range(number_of_wiggles):
            wiggles.append(wiggle_template + np.array([0, 0, i * wiggle_height]))
        spiral = np.concatenate(wiggles)
        return spiral

    def spiral_round(self, radius_x, radius_y, height, number_of_points):
        points = []
        tau = 2 * np.pi
        for i, r in enumerate(np.arange(0, tau, tau / number_of_points)):
            points.append(
                np.array([np.sin(r) * radius_x, np.cos(r) * radius_y, i / float(number_of_points) * float(height)]))
        return np.array(points)

    def np_array_to_pose_stampeds(self, nparray, orientation, header):
        pose_list = []
        for p in nparray:
            pose = PoseStamped()
            pose.pose.position = Point(*p)
            pose.pose.orientation = orientation
            pose.header = header
            pose_list.append(pose)
        return pose_list


if __name__ == '__main__':
    rospy.init_node('wiggle_action_server', anonymous=True)
    w = WIGGLE()
    print('wiggle action server is running.')
    # goal = WiggleGoal()
    # goal.arm = WiggleGoal.LEFT_ARM
    # goal.goal_pose.header.frame_id = 'left_gripper_tool_frame'
    # goal.goal_pose.pose.orientation.w = 1
    # goal.goal_pose.pose.position.z = -0.1
    # w.wiggle_cb(goal)

    rospy.spin()
    # radius = 2
    # p = w.spiral_round(radius, -0.2, 20)
    # p = w.create_spiral()

    # fig = plt.figure()
    # ax = fig.gca(projection='3d')
    # ax.plot(p[:,0], p[:,1], p[:,2], label='parametric curve')
    # ax.legend()

    # plt.show()

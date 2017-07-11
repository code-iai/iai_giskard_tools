#!/usr/bin/env python
import actionlib
import rospy
import numpy as np
from copy import deepcopy
from geometry_msgs.msg._PoseStamped import PoseStamped
from geometry_msgs.msg._Quaternion import Quaternion
from giskard_msgs.msg._ArmCommand import ArmCommand
from giskard_msgs.msg._SemanticFloat64 import SemanticFloat64
from giskard_msgs.msg._WholeBodyCommand import WholeBodyCommand
from tf.transformations import quaternion_about_axis, quaternion_multiply
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from wiggle_msgs.msg._WiggleAction import WiggleAction
from wiggle_msgs.msg._WiggleFeedback import WiggleFeedback
from wiggle_msgs.msg._WiggleGoal import WiggleGoal


class WIGGLE(object):
    def __init__(self, save=True,
                 cycle_time=1.,
                 steps=20,
                 linear_multiplier=.2,
                 angular_multiplier=.25):
        self.cycle_time = cycle_time
        self.steps = steps
        self.linear_multiplier = linear_multiplier
        self.angular_multiplier = angular_multiplier
        self.giskard_goals = rospy.Publisher('/whole_body_controller/goal', WholeBodyCommand, queue_size=10)
        # self.giskard_client = actionlib.SimpleActionClient('controller_action_server/move', WholeBodyAction)
        # self.giskard_client.wait_for_server()
        self.pose_pub = rospy.Publisher('/test', PoseStamped, queue_size=10)
        self.steps_size = 2 * np.pi / self.steps
        self.rate = int(self.steps / self.cycle_time)
        self.wiggle_counter = 0.
        self.tfBuffer = Buffer()
        self.tf = TransformListener(self.tfBuffer)
        self.save = save

        self.wiggle_action_server = actionlib.SimpleActionServer('wiggle_wiggle_wiggle', WiggleAction,
                                                                 execute_cb=self.wiggle_cb, auto_start=False)
        self.wiggle_action_server.start()
        rospy.sleep(1.)

    def transformPose(self, target_frame, pose):
        transform = self.tfBuffer.lookup_transform(target_frame,
                                                   pose.header.frame_id,  # source frame
                                                   rospy.Time(0),  # get the tf at first available time
                                                   rospy.Duration(1.0))
        new_pose = do_transform_pose(pose, transform)
        return new_pose

    def wiggle_cb(self, goal):
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
        real_duration = rospy.Duration(1.0) + goal.timeout.data
        r = rospy.Rate(self.rate)
        # create one wiggle cycle
        pose_list = []
        tcp_goal_pose = self.transformPose('left_gripper_tool_frame', goal.goal_pose)
        for i in range(int(self.steps)):
            wiggle_goal_pose = deepcopy(tcp_goal_pose)
            wiggle_goal_pose = self.add_wiggle(wiggle_goal_pose, goal.wiggle_type)
            pose_list.append(deepcopy(self.transformPose('base_footprint', wiggle_goal_pose)))
        end_pose = self.transformPose('base_footprint', goal.goal_pose)

        # repeat wiggle cycle until time is over
        start_time = rospy.get_rostime()
        end_time = start_time + goal.timeout.data
        i = 0
        while rospy.get_rostime() < end_time:
            i = i % self.steps
            self.pub_wiggle(pose_list[i], goal.arm)
            self.wiggle_action_server.publish_feedback(self.cal_feedback(start_time, real_duration))
            r.sleep()
            i += 1

        # move back to real goal
        while rospy.get_rostime() < start_time + real_duration:
            self.pub_wiggle(end_pose, goal.arm)
            self.wiggle_action_server.publish_feedback(self.cal_feedback(start_time, real_duration))
        result = WiggleFeedback()
        result.progress = 1.
        self.wiggle_action_server.set_succeeded(result)

    def cal_feedback(self, start_time, duration):
        feedback = WiggleFeedback()
        feedback.progress = min((rospy.get_rostime() - start_time) / duration, 1.)
        return feedback

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

    # def post_wiggle(self, goal_pose, arm):
    #     cmd = self.goal_pose_to_cmd(goal_pose, arm)
    #     goal = WholeBodyGoal()
    #     goal.command = cmd
    #     self.giskard_client.send_goal(goal)

    def wiggle_wiggle_wiggle(self, goal_pose, arm):
        if arm == WiggleGoal.LEFT_ARM or arm == WiggleGoal.RIGHT_ARM:
            cmd = self.goal_pose_to_cmd(goal_pose, arm)
            self.giskard_goals.publish(cmd)
        else:
            rospy.logerr('arm goal should be {} or {}'.format(WiggleGoal.LEFT_ARM, WiggleGoal.RIGHT_ARM))

    def add_wiggle(self, goal, wiggle_type):
        wiggle_pose = deepcopy(goal)
        self.wiggle_counter += 1.
        if self.wiggle_counter >= self.max_i():
            self.wiggle_counter = 0
        if wiggle_type & WiggleGoal.X > 0:
            wiggle_pose.pose.position.x += self.linear_wiggle(self.wiggle_counter)
        if wiggle_type & WiggleGoal.Y > 0:
            wiggle_pose.pose.position.y += self.linear_wiggle(self.wiggle_counter, offset=np.pi / 2)
        if wiggle_type & WiggleGoal.ANGLE > 0:
            wiggle_pose.pose.orientation = Quaternion(*(quaternion_multiply(
                [goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w],
                self.angular_wiggle(self.wiggle_counter))))
        return wiggle_pose

    def max_i(self):
        return 2 * np.pi / self.steps_size

    def linear_wiggle(self, i, offset=0):
        return np.sin(i * self.steps_size + offset) * self.linear_multiplier

    def angular_wiggle(self, i, offset=0):
        return quaternion_about_axis(-np.sin(i * self.steps_size + offset) * self.angular_multiplier, (0, 0, -1))


if __name__ == '__main__':
    rospy.init_node('wiggle_action_server', anonymous=True)
    w = WIGGLE()
    print('wiggle action server is running.')
    rospy.spin()
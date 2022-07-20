#! /usr/bin/env python3

import rospy

import actionlib

from control_msgs.msg import GripperCommandAction, GripperCommandGoal, GripperCommandResult, GripperCommandFeedback
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input, Robotiq2FGripper_robot_output


class GripperActionServer(object):
    # create messages that are used to publish feedback/result
    _status = Robotiq2FGripper_robot_input()
    _feedback = GripperCommandFeedback()
    _result = GripperCommandResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, GripperCommandAction, execute_cb=self.execute_cb,
                                                auto_start=False)
        self._as.start()
        self.command_pub = rospy.Publisher("/gripper_control/command", Robotiq2FGripper_robot_output, queue_size=5)
        self.status_listener = rospy.Subscriber("/gripper_control/status", Robotiq2FGripper_robot_input, queue_size=5,
                                                callback=self.update_status)

    def update_status(self, msg: Robotiq2FGripper_robot_input):
        self._status = msg

    def execute_cb(self, goal: GripperCommandGoal):

        # helper variables
        r = rospy.Rate(10)
        success = True

        # publish info to the console for the user
        rospy.loginfo('%s: Moving gripper to %f with effort %f' % (
        self._action_name, goal.command.position, goal.command.max_effort))
        command = Robotiq2FGripper_robot_output()
        command.rPR = int(goal.command.position * 255)
        command.rSP = 100
        command.rFR = int(goal.command.max_effort * 255)
        self.command_pub.publish(command)

        # TODO(Jack, 6/21): Find a better way around the sleep to check when the gripper has actually recieved the command
        rospy.sleep(.3)

        # start executing the action
        while not rospy.is_shutdown():
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            elif self._status.gOBJ != 0:  # The gripper either detected an obstacle or is finished moving
                self._feedback.reached_goal == True
                break
            # publish the feedback
            self._feedback.position = self._status.gPO
            self._as.publish_feedback(self._feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()

        if success:
            self._result.position = self._feedback.position
            self._result.reached_goal = False
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('gripper_action_server')
    server = GripperActionServer(rospy.get_name())
    rospy.spin()

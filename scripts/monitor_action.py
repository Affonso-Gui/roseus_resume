from action_interrupt.msg import InterruptActionStatus
from action_interrupt.srv import InterruptController
from action_interrupt.srv import InterruptControllerResult
from actionlib_msgs.msg import GoalID
import argparse
from control_msgs.msg import FollowJointTrajectoryAction
import rospy


LOG_LEVELS = {
    "debug": rospy.DEBUG,
    "info": rospy.INFO,
    "warn": rospy.WARN,
    "error": rospy.ERROR,
    "fatal": rospy.FATAL,
}


class ActionStatus(object):
    def __init__(self, name, goal_msg):
        self.status = InterruptActionStatus(name=name,
                                            id=goal_msg.goal_id.id,
                                            goal=goal_msg.goal)

    def update(self, feedback_msg):
        self.status.feedback = feedback_msg.feedback


class MonitorController(object):
    def __init__(self, ns):
        if ns.endswith('/'):
            ns = ns[:-1]

        self.active_command = None
        self.ns = ns

        inst = FollowJointTrajectoryAction()

        self.goal_sub = rospy.Subscriber(
            ns + '/goal', type(inst.action_goal), self._goal_cb)
        self.feedback_sub = rospy.Subscriber(
            ns + '/feedback', type(inst.action_feedback), self._feedback_cb)
        self.result_sub = rospy.Subscriber(
            ns + '/result', type(inst.action_result), self._result_cb)
        self.cancel_pub = rospy.Publisher(
            ns + '/cancel', GoalID, queue_size=100)
        rospy.loginfo("Ready to take orders from {}".format(ns))

    def _goal_cb(self, msg):
        self.active_command = ActionStatus(self.ns, msg)

    def _feedback_cb(self, msg):
        if self.active_command.status.id == msg.status.goal_id.id:
            self.active_command.update(msg)

    def _result_cb(self, msg):
        if self.active_command.status.id == msg.status.goal_id.id:
            self.active_command = None

    def interrupt(self, msg=None):
        if self.active_command:
            status = self.active_command.status
            self.cancel_pub.publish(GoalID())
            rospy.logdebug("Interrupted Action:\n%s" % status.id)
            rospy.logdebug("Interrupted Time: %s" %
                           status.feedback.actual.time_from_start.to_sec())
            return status


class MonitorAllControllers(object):
    def __init__(self, ns, *controllers):
        self.controllers = [MonitorController(x) for x in controllers]

        if ns.endswith('/'):
            ns = ns[:-1]

        self.interrupt_service = rospy.Service(
            ns + '/interrupt', InterruptController, self.interrupt_all)

    def interrupt_all(self, req):
        res = InterruptControllerResult()
        for c in self.controllers:
            res.status.append(c.interrupt())


def main():
    p = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--controller", "-c", action='append',
                   help="motor controllers")
    p.add_argument("--namespace", "-ns", default='fullbody_controller',
                   help="interrupt service namespace")
    p.add_argument("--log-level", "-l", choices=LOG_LEVELS.keys(),
                   default="debug", help="Log level")
    args = p.parse_args()

    log_level = LOG_LEVELS[args.log_level]
    rospy.init_node("interrupt_server", anonymous=True, log_level=log_level)
    MonitorAllControllers(args.namespace, *args.controller)

    rospy.spin()


if __name__ == '__main__':
    main()

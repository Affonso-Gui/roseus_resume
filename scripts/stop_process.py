import argparse
from jsk_gui_msgs.msg import VoiceMessage
import os
import rospy
import signal
from std_msgs.msg import Empty


class StopSubscriber(object):
    def __init__(self, pid):
        self.pid = pid
        self.sound_sub = rospy.Subscriber('Tablet/voice', VoiceMessage,
                                          self.callback)
        self.stop_sub = rospy.Subscriber('stop', Empty, self.stop)
        self.resume_sub = rospy.Subscriber('resume', Empty, self.resume)

    def stop(self, msg=None):
        os.kill(self.pid, signal.SIGINT)

    def resume(self, msg=None):
        os.kill(self.pid, signal.SIGCONT)

    def callback(self, msg):
        for text in msg.texts:
            if 'stop' in text:
                self.stop()
                return
            if 'continue' in text:
                self.resume()
                return

    def spin(self):
        while True:
            try:
                os.kill(self.pid, 0)
            except OSError:
                rospy.logwarn("...Closing")
                exit()


def main():
    p = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("pid", type=int, help="process id")
    args = p.parse_args()

    rospy.init_node("stopper" + str(args.pid))

    StopInstance = StopSubscriber(args.pid)
    rospy.loginfo("Monitoring pid %d..." % args.pid)
    StopInstance.spin()


if __name__ == '__main__':
    main()

#!/usr/bin/env python

import threading
import rospy
import time

class MpexClient(object):
    def __init__(self):
        self.action_handlers = {}
        self.sub = rospy.Subscriber('start/PddlGroundAction', PddlGroundAction, self._trigger_action)
        self.pub_accept = rospy.Publisher('accept', Accept, queue_size=0)
        self.pub_finish = rospy.Publisher('finish', Finish, queue_size=0)

    def add_listener(self, action, callback):
        self.action_handlers[action] = callback

    def remove_listener(self, action):
        self.action_handlers.pop(action, None)

    def _trigger_action(self, action):
        # Check that the action is for this agent
        # and this agent has registered for this action

        if action.name in self.action_handlers:
            # TODO: pass in whatever parameters need to be passed in

            msg = Accept()
            msg.ID = action.ID
            msg.accepted = 1
            self.pub_accept.publish(msg)

            thread = MpexCallbackThread(self.action_handlers[action.name], self.pub_finish, action=action.name, data=action)
            thread.start()
        else:
            # some sort of error passed back

            msg = Accept()
            msg.ID = action.ID
            msg.accepted = 0
            self.pub_accept.publish(msg)

            pass
        return

    def run(self):
        rospy.spin()


class MpexCallbackThread(threading.Thread):
    def __init__(self, fn, pub, action=None, data=None):
        self.fn = fn
        self.action = action
        self.data = data
        self.pub = pub
        super(MpexCallbackThread, self).__init__()

    def run(self):
        self.fn(self.data.parameters)
        if self.action is not None:
            self.report_finished()

    def report_finished(self):
        # report back via ROS that we finished the action
        msg = Finish()
        msg.ID = self.data.ID
        msg.finished = 1
        self.pub.publish(msg)
        pass

if __name__ == '__main__':


    def func(data):
        print "yay"
        time.sleep(5)
        print "hello"

    def func10(data):
        print "yay10"
        time.sleep(10)
        print "hello10"

    robot = MpexClient('robot')

    robot.add_listener('print', func)
    robot.add_listener('print2', func)
    robot.add_listener('print10', func10)

    print "robot has been initialized"
    robot.run()

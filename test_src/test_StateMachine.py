#! /usr/bin/env python
import rospy
from smach import State, StateMachine
from time import sleep

class TeleOp(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        sleep(3)
        return 'success'

class AutoOp(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        sleep(3)
        return 'success'

if __name__ == '__main__':
    states = StateMachine(outcomes=['success2'])
    rospy.init_node('test_state')
    with states:
        StateMachine.add('Tele', TeleOp(), transitions={'success':'Auto'})
        StateMachine.add('Auto', AutoOp(), transitions={'success':'Tele'})

    states.execute()
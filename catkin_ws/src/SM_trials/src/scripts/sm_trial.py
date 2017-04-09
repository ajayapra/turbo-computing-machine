#!/usr/bin/env python

import rospy
import smach
import smach_ros

class Foo(smach.State):
	def __init__(self):
		smach.State.__init__(	self,
								outcomes=['outcome1','outcome2'],
								input_keys = 'foo_counter_in',
								output_keys = 'foo_counter_out')
		self.counter = 0

	def execute(self, userdata):
		rospy.loginfo('Executing state FOO: %s',self.counter)
		if self.counter < 3:
			self.counter +=1
			return 'outcome1'
		else:
			return 'outcome2'

class Bar(smach.State):
	def __init__(self):
		smach.State.__init__(	self,
								outcomes=['outcome1'],
								input_keys = 'bar_counter_in')

	def execute(self, userdata):
		rospy.loginfo('Executing State BAR')
		rospy.logino('Counter = %f', userdata.bar_counter_in)
		return 'outcome1'

def main():
	rospy.init_node('State_Machine')

	sm = smach.StateMachine(outcomes=['outcome4'])
	with sm:
		smach.StateMachine.add('FOO', Foo(), transitions={'outcome1':'BAR', 'outcome2':'outcome4'}, remapping={'foo_counter_in':'sm_counter', 'foo_counter_out':'sm_counter'})
		smach.StateMachine.add('BAR',Bar(), transitions={'outcome1':'FOO'}, remapping={'bar_counter_in':'sm_counter'})
		outcome = sm.execute()
if __name__=='__main__':
	main()

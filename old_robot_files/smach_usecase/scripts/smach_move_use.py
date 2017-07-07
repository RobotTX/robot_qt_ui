#!/usr/bin/env python

import rospy
from smach import State, StateMachine, Concurrence, Container, UserData
from smach_ros import MonitorState, ServiceState, SimpleActionState, IntrospectionServer
from std_msgs.msg import Float32
import actionlib
from actionlib import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from tf.transformations import quaternion_from_euler
from battery.srv import *
from random import randrange

class PickWaypoint(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded'], input_keys=['waypoints'], output_keys=['waypoint_out'])

    def execute(self, userdata):
        waypoint_out = randrange(len(userdata.waypoints))

        userdata.waypoint_out = waypoint_out

        rospy.loginfo("Going to waypoint " + str(waypoint_out))

        return 'succeeded'

class Nav2Waypoint(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                       input_keys=['waypoints', 'waypoint_in'])

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # Wait up to 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move_base action server")

        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'

    def execute(self, userdata):
        self.goal = userdata.waypoints[userdata.waypoint_in]

        # Send the goal pose to the MoveBaseAction server
        self.move_base.send_goal(self.goal)

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        # Allow 1 minute to get there
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(60))

        # If we don't get there in time, abort the goal
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
            return 'aborted'
        else:
            # We made it!
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
            return 'succeeded'

class Stop(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        pass

    def execute(self, userdata):
        rospy.loginfo("Shutting down the state machine")
        return 'succeeded'

class Patrol():
    def __init__(self):
        rospy.init_node('patrol_smach_concurrence', anonymous=False)

        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)

        # Track success rate of getting to the goal locations
        self.n_succeeded = 0
        self.n_aborted = 0
        self.n_preempted = 0

	self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
	
	self.docking_station_pose = (Pose(Point(-0.5, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)))

	self.waypoints = list()
	nav_goal = MoveBaseGoal()
        nav_goal.target_pose.header.frame_id = 'map'
        nav_goal.target_pose.pose.position.x = 0.7
        nav_goal.target_pose.pose.position.y = 0.0
        nav_goal.target_pose.pose.position.z = 0.0
        nav_goal.target_pose.pose.orientation.w = 1.0
        self.waypoints.append(nav_goal)

        nav_goal = MoveBaseGoal()
        nav_goal.target_pose.header.frame_id = 'map'
        nav_goal.target_pose.pose.position.x = 2.51611566544
        nav_goal.target_pose.pose.position.y = 0.100562250018
        nav_goal.target_pose.pose.position.z = 0.0
        nav_goal.target_pose.pose.orientation.w = 1.0
        self.waypoints.append(nav_goal)

        nav_goal = MoveBaseGoal()
        nav_goal.target_pose.header.frame_id = 'map'
        nav_goal.target_pose.pose.position.x = 2.94330668449
        nav_goal.target_pose.pose.position.y = -0.77200148106
        nav_goal.target_pose.pose.position.z = 0.0
        nav_goal.target_pose.pose.orientation.z = -0.585479230542
        nav_goal.target_pose.pose.orientation.w = 0.81068740622
        self.waypoints.append(nav_goal)

        nav_goal = MoveBaseGoal()
        nav_goal.target_pose.header.frame_id = 'map'
        nav_goal.target_pose.pose.position.x = 2.94330668449
        nav_goal.target_pose.pose.position.y = -1.67200148106
        nav_goal.target_pose.pose.position.z = 0.0
        nav_goal.target_pose.pose.orientation.z = -0.585479230542
        nav_goal.target_pose.pose.orientation.w = 0.81068740622
        self.waypoints.append(nav_goal)

        # A variable to hold the last/current navigation goal
        self.last_nav_state = None

        # A flag to indicate whether or not we are rechargin
        self.recharging = False

        # A list to hold then navigation goa
        nav_states = list()

        # Create a MoveBaseAction state for the docking station
        nav_goal = MoveBaseGoal()
        nav_goal.target_pose.header.frame_id = 'map'
        nav_goal.target_pose.pose = self.docking_station_pose
        nav_docking_station = SimpleActionState('move_base', MoveBaseAction, goal=nav_goal, result_cb=self.move_base_result_cb,
			exec_timeout=rospy.Duration(20.0),
                        server_wait_timeout=rospy.Duration(10.0) )

        # Initialize the navigation state machine
        self.sm_nav = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

        # Add the nav states to the state machine with the appropriate transitions
	self.sm_nav.userdata.waypoints = self.waypoints
        with self.sm_nav:
		StateMachine.add('PICK_WAYPOINT', PickWaypoint(),
                             transitions={'succeeded':'NAV_WAYPOINT'},
                             remapping={'waypoint_out':'patrol_waypoint'})

        	StateMachine.add('NAV_WAYPOINT', Nav2Waypoint(),
                             transitions={'succeeded':'PICK_WAYPOINT',
                                          'aborted':'PICK_WAYPOINT',},
                             remapping={'waypoint_in':'patrol_waypoint'})
        # Register a callback function to fire on state transitions within the sm_nav state machine
        self.sm_nav.register_transition_cb(self.nav_transition_cb, cb_args=[])

        # Initialize the recharge state machine
        self.sm_recharge = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

        with self.sm_recharge:
            StateMachine.add('NAV_DOCKING_STATION', nav_docking_station, transitions={'succeeded':'RECHARGE_BATTERY'})
            StateMachine.add('RECHARGE_BATTERY', ServiceState('battery/set_battery', SetBattery, 100, response_cb=self.recharge_cb),
                             transitions={'succeeded':''})

        # Create the nav_patrol state machine using a Concurrence container
        self.nav_patrol = Concurrence(outcomes=['succeeded', 'recharge', 'stop'],
                                        default_outcome='succeeded',
                                        child_termination_cb=self.concurrence_child_termination_cb,
                                        outcome_cb=self.concurrence_outcome_cb)

        # Add the sm_nav machine and a battery MonitorState to the nav_patrol machine             
        with self.nav_patrol:
           Concurrence.add('SM_NAV', self.sm_nav)
           Concurrence.add('MONITOR_BATTERY', MonitorState("battery/battery_level", Float32, self.battery_cb))

        # Create the top level state machine
        self.sm_top = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

        # Add nav_patrol, sm_recharge and a Stop() machine to sm_top
        with self.sm_top:
            StateMachine.add('PATROL', self.nav_patrol, transitions={'succeeded':'PATROL', 'recharge':'RECHARGE', 'stop':'STOP'})
            StateMachine.add('RECHARGE', self.sm_recharge, transitions={'succeeded':'PATROL'})
            StateMachine.add('STOP', Stop(), transitions={'succeeded':''})

        # Create and start the SMACH introspection server
        intro_server = IntrospectionServer('patrol', self.sm_top, '/SM_ROOT')
        intro_server.start()

        # Execute the state machine
        sm_outcome = self.sm_top.execute()

        rospy.loginfo('State Machine Outcome: ' + str(sm_outcome))

        intro_server.stop()

    def nav_transition_cb(self, userdata, active_states, *cb_args):
        self.last_nav_state = ['NAV_WAYPOINT']

    # Gets called when ANY child state terminates
    def concurrence_child_termination_cb(self, outcome_map):
        # If the current navigation task has succeeded, return True
        if outcome_map['SM_NAV'] == 'succeeded':
            return True
        # If the MonitorState state returns False (invalid), store the current nav goal and recharge
        if outcome_map['MONITOR_BATTERY'] == 'invalid':
            rospy.loginfo("LOW BATTERY! NEED TO RECHARGE...")
            if self.last_nav_state is not None:
                self.sm_nav.set_initial_state(self.last_nav_state, UserData())
            return True
        else:
            return False

    # Gets called when ALL child states are terminated
    def concurrence_outcome_cb(self, outcome_map):
        # If the battery is below threshold, return the 'recharge' outcome
        if outcome_map['MONITOR_BATTERY'] == 'invalid':
            return 'recharge'
        # Otherwise, if the last nav goal succeeded, return 'succeeded' or 'stop'
        elif outcome_map['SM_NAV'] == 'succeeded':
            #self.patrol_count += 1
            #rospy.loginfo("FINISHED PATROL LOOP: " + str(self.patrol_count))
            # If we have not completed all our patrols, start again at the beginning
            #if self.n_patrols == -1 or self.patrol_count < self.n_patrols:
            self.sm_nav.set_initial_state(['NAV_WAYPOINT'], UserData())
            return 'succeeded'
            # Otherwise, we are finished patrolling so return 'stop'
            #else:
            #  	self.sm_nav.set_initial_state(['NAV_STATE_0'], UserData())
            #    return 'succeeded'
        # Recharge if all else fails
        else:
            return 'recharge'

    def battery_cb(self, userdata, msg):
        if msg.data < 30.0:
            self.recharging = True
            return False
        else:
            self.recharging = False
            return True

    def recharge_cb(self, userdata, response):
        return 'succeeded'

    def move_base_result_cb(self, userdata, status, result):
	print status,userdata
        if not self.recharging:
            if status == actionlib.GoalStatus.SUCCEEDED:
                self.n_succeeded += 1
            elif status == actionlib.GoalStatus.ABORTED:
                self.n_aborted += 1
            elif status == actionlib.GoalStatus.PREEMPTED:
                self.n_preempted += 1

            try:
                rospy.loginfo("Success rate: " + str(100.0 * self.n_succeeded / (self.n_succeeded + self.n_aborted  + self.n_preempted)))
            except:
                pass

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")

        self.sm_nav.request_preempt()

        self.cmd_vel_pub.publish(Twist())

        rospy.sleep(1)

if __name__ == '__main__':
    try:
        Patrol()
    except rospy.ROSInterruptException:
        rospy.loginfo("SMACH test finished.")

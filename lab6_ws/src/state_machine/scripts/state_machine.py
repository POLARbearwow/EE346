import smach
import rospy
import smach_ros
from approach_cylinder import  Approachlinder
from navigate_to_point import NavigateToPoint
import smach.state

def main():
    rospy.init_node('smach_navigation_with_move_base')

    # 创建状态机
    #状态的 outcomes（用于transitions到下一个state） 和状态机的 outcomes（整个结果） 不同
    sm = smach.StateMachine(outcomes=['all_tasks_completed', 'aborted'])

    with sm:
        smach.StateMachine.add('to_pointA',
                            NavigateToPoint(), #point name / position
                            transitions={'succeded':'to_cylinderA', 'failure':'aborted'}
                            )
        
        smach.StateMachine.add('to_cylinderA',
                            Approachlinder(),
                            transitions={'succeded':'to_pointB', 'aborted':'aborted'}
                            )

        smach.StateMachine.add('to_pointB',
                            NavigateToPoint(), #point name / position
                            transitions={'succeded':'to_cylinderB', 'failure':'aborted'}
                            )
        
        smach.StateMachine.add('to_cylinderB',
                            Approachlinder(),
                            transitions={'succeded':'to_pointC', 'aborted':'aborted'}
                            )

        smach.StateMachine.add('to_pointC',
                            NavigateToPoint(), #point name / position
                            transitions={'succeded':'to_cylinderC', 'failure':'aborted'}
                            )

        smach.StateMachine.add('to_cylinderC',
                            Approachlinder(),
                            transitions={'succeded':'all_tasks_completed', 'aborted':'aborted'}
                            )
    
    sis = smach_ros.IntrospectionServer('ns', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()
    rospy.loginfo(f"State Machine finished with outcome: {outcome}")

    rospy.spin()
    sis.stop()

    #rosrun smach_viewer smach_viewer.py

if __name__ == '__main__':
    main()
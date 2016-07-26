#!/usr/bin/env python


# AMQP modules
import pika

# ROS modules
import rospy



if __name__ == '__main__':

    try:
        argv = rospy.myargv(argv=sys.argv[1:])
        # print "arguments: {0}".format(argv)
        rc = RobotController(*argv)
        # print("rc final state: {0}".format(rc.fsm.current))

        while not rospy.is_shutdown():
            rc.rate.sleep()

    except rospy.ROSInterruptException:
        pass

    print('######## master_relay exiting ########')
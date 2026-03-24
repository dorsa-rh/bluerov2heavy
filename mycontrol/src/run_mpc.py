#!/usr/bin/env python3
from a_rovmanager import configure_logging, Autopilot, ExtendedAutopilot
from pymavlink import mavutil
import output_pub
import time
from rov_mpc_controller import mpc_loop
import rospy
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from nav_msgs.msg import Odometry


thruster_output=[0]*8
#thruster_output=0
'''def thruster_listener(msg):
    """Example function to listen to thruster outputs from the vehicle."""
    thruster_output=msg.data
    stamp = msg.header.stamp.to_sec()
    rospy.loginfo("Thruster output received at time %.2f: %f", stamp, thruster_output)
'''
cbprint=0
def tr1_cb(msg):
    thruster_output[0]=msg.data
    stamp = msg.header.stamp.to_sec()
    if cbprint:
        rospy.loginfo("Thruster_1 output received at time %.2f: %f", stamp, thruster_output[0])
def tr2_cb(msg):
    thruster_output[1]=msg.data
    stamp = msg.header.stamp.to_sec()
    if cbprint:
        rospy.loginfo("Thruster_2 output received at time %.2f: %f", stamp, thruster_output[1])
def tr3_cb(msg):
    thruster_output[2]=msg.data
    stamp = msg.header.stamp.to_sec()
    if cbprint:
        rospy.loginfo("Thruster_3 output received at time %.2f: %f", stamp, thruster_output[2])
def tr4_cb(msg):
    thruster_output[3]=msg.data
    stamp = msg.header.stamp.to_sec()
    if cbprint:
        rospy.loginfo("Thruster_4 output received at time %.2f: %f", stamp, thruster_output[3])
def tr5_cb(msg):
    thruster_output[4]=msg.data
    stamp = msg.header.stamp.to_sec()
    if cbprint:
        rospy.loginfo("Thruster_5 output received at time %.2f: %f", stamp, thruster_output[4])
def tr6_cb(msg):
    thruster_output[5]=msg.data
    stamp = msg.header.stamp.to_sec()
    if cbprint:
        rospy.loginfo("Thruster_6 output received at time %.2f: %f", stamp, thruster_output[5])
def tr7_cb(msg):
    thruster_output[6]=msg.data
    stamp = msg.header.stamp.to_sec()
    if cbprint:
        rospy.loginfo("Thruster_7 output received at time %.2f: %f", stamp, thruster_output[6])
def tr8_cb(msg):
    thruster_output[7]=msg.data
    stamp = msg.header.stamp.to_sec()
    if cbprint:
        rospy.loginfo("Thruster_8 output received at time %.2f: %f", stamp, thruster_output[7])
tf_cb=[tr1_cb,tr2_cb,tr3_cb,tr4_cb,tr5_cb,tr6_cb,tr7_cb,tr8_cb]


if __name__ == '__main__':
    # running as a script
    # set up some useful commandline arguments
    from argparse import ArgumentParser

    parser = ArgumentParser()
    parser.add_argument('-l', '--log', default='note',
                        help=('Lowest importance level to log '
                              '(debug/note/info/warning/error/critical)'))
    parser.add_argument('-f', '--filename', default='',
                        help=('File to log to - '
                              'prints to console if not specified'))
    parser.add_argument('-d', '--datefmt', default='%H:%M:%S',
                        help='datetime format for logging')
    parser.add_argument('-m', '--mavlink', default='udpin:0.0.0.0:14550',
                        help='MAVLink connection specifier')
    parser.add_argument('-r', '--request', action='store_true',
                        help='Connection requires messages to be requested')
    #parser.add_argument('-t', '--use-topic-thrusters', action='store_true',
    #                    help='Use /bluerov2heavy/thrusters/*/input topics as outputs')
    args, _ = parser.parse_known_args()
    client = args.mavlink.startswith('udpin')

    configure_logging(args.log.upper(), args.filename, args.datefmt)

    # run/test the desired functionality
    print('If script gets stuck, press CTRL+C to disarm and quit\n')

    #control_mode = 'topic'  # choose between 'rc' , 'servo' and 'topic' control modes
    control_mode = 'servo'
    # initialize ROS node and thruster subscriber
    try:
        rospy.init_node('run_mpc_thruster_listener', anonymous=True, disable_signals=True)
    except Exception:
        # node may already be initialized; ignore
        pass
    #rospy.init_node("mav_odom_pub")
    odom_pub=rospy.Publisher(
        "/bluerov2heavy/pose_gt",
        Odometry,
        queue_size=10
    )
    #thruster_sub = ThrusterSubscriber(num_thrusters=8)
    topic_outputs=[0]*8  # initialize outputs list
    outputs=[0]*6  # initialize control outputs list
    with ExtendedAutopilot(args.mavlink, client=client) as sub, \
        sub.per_thruster_control(mode='RCPassThru' if control_mode == 'rc' else 'Servo'):

        #sub.set_mode('manual')
        sub.get_thruster_outputs()
        # make sure all motion is set to stationary before arming
        sub.clear_motion()
        sub.get_thruster_outputs()
        sub.arm()
        start = time.time()
        #sub.publish()
        rate = rospy.Rate(50) #control frequency



        while(rospy.is_shutdown() == False):
            loopstart_time = time.time()
            sub.get_thruster_outputs()
            #sub.get_odometry_msg()
            #odom_pub.publish(sub.get_odometry_msg())
            if control_mode == 'topic':
                # read thruster topic inputs
                odom_pub.publish(sub.get_odometry_msg())
                for i in range(8):
                    topic =f"/bluerov2heavy/thrusters/{i}/input"
                    #thruster_listener=tr{i+1}_cb()
                    rospy.Subscriber(topic, FloatStamped, tf_cb[i],queue_size=1)
                    topic_outputs=thruster_output
            else:
            # Call MPC controller function
                outputs = mpc_loop(sub, 0, 0.1, 10)
                print('Control outputs: {}'.format(outputs))

            if control_mode == 'servo':
                servo_outputs = output_pub.demixer2servo(outputs)
                print('Servo outputs: {}'.format(servo_outputs))
                output_pub.servo_control_outputs(sub, servo_outputs)
            elif control_mode == 'rc':  # 'rc' control
                pwm_values = output_pub.output2pwm(outputs)
                print('PWM values: {}'.format(pwm_values))
                output_pub.rc_control_outputs(sub, pwm_values)
            else:  # 'topic' control
                pwm_values = output_pub.topic2pwm(topic_outputs)
                print('Topic thruster outputs: {}'.format(topic_outputs))
                output_pub.servo_control_outputs(sub, pwm_values)
            rate.sleep()
            '''if loopstart_time+0.02<time.time():
                time.sleep(loopstart_time+0.02-time.time())  # 50 Hz loop rate'''
            if time.time() - start > 5:  # run for 30 seconds
                time.sleep(0.1)
                break
            
    #sub.disarm()
    #sub.disconnect()
    # disarm and wait for a bit
    time.sleep(5)
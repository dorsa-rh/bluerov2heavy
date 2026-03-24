#!/usr/bin/env python3

from __future__ import division
import time
import weakref
from numbers import Number
from functools import partial
from itertools import repeat, chain
from contextlib import contextmanager
from mavactive import mavactive, mavutil, mavlink

#from bridge




import json
import math
import re
import rospy
import sys
#import time

#from bridge import Bridge

try:
    from pubs import Pubs
    from subs import Subs
    from video import Video
except:
    from bluerov.pubs import Pubs
    from bluerov.subs import Subs
    from bluerov.video import Video

# convert opencv image to ros image msg
from cv_bridge import CvBridge

# msgs type
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import UInt16






# logging setup
import logging

logging.NOTE = logging.INFO - 5
logging.addLevelName(logging.NOTE, 'NOTE')
class NoteLogger(logging.getLoggerClass()):
    def note(self, msg, *args, **kwargs):
        if self.isEnabledFor(logging.NOTE):
            self._log(logging.NOTE, msg, args, **kwargs)

logging.setLoggerClass(NoteLogger)
logging.note = partial(NoteLogger.note, logging.getLogger())


class Autopilot:
    """ An ArduSub autopilot connection manager. """
    def __init__(self, *args, client=True, thrusters=8, **kwargs):
        self.thrusters = thrusters
        self.master = mavutil.mavlink_connection(*args, **kwargs)






        if client:
            logging.debug('connecting to MAVLink client...')
            self.master.wait_heartbeat()
        else:
            logging.debug('connecting to MAVLink server...')
            self._server_wait_conn()

        logging.info('MAVLink connection successful')

        # send regular heartbeats, as a ground control station
        self.heart = mavactive(self.master, mavlink.MAV_TYPE_GCS)
        self._finalizer = weakref.finalize(self, self.cleanup)

        # convenience
        self.mav        = self.master.mav
        self.recv_match = self.master.recv_match
        self.target     = (self.master.target_system,
                           self.master.target_component)

        logging.debug('checking for autopilot...')
        #self._wait_autopilot_heartbeat()
        logging.info('connection to autopilot confirmed')

                #new
        self.data = {}
        self.pub = Pubs()
        self.sub = Subs()
        self.ROV_name = 'BlueRov2'
        self.model_base_link = '/base_link'

        #self.video = Video()
        #self.video_bridge = CvBridge()

        self.pub_topics = [
            [
                self._create_battery_msg,
                '/battery',
                BatteryState,
                1
            ],
            [
                self._create_camera_msg,
                '/camera/image_raw',
                Image,
                1
            ],
            [
                self._create_ROV_state,
                '/state',
                String,
                1
            ],
            [
                self._create_imu_msg,
                '/imu/data',
                Imu,
                1
            ],
            [
                self._create_odometry_msg,
                '/bluerov2heavy/pose_gt',
                Odometry,
                1
            ],
        ]

        self.sub_topics= [
            [
                self._setpoint_velocity_cmd_vel_callback,
                '/setpoint_velocity/cmd_vel',
                TwistStamped,
                1
            ],
            [
                self._set_servo_callback,
                '/servo{}/set_pwm',
                UInt16,
                1,
                [1, 2, 3, 4, 5, 6, 7, 8]
            ],
            [
                self._set_rc_channel_callback,
                '/rc_channel{}/set_pwm',
                UInt16,
                1,
                [1, 2, 3, 4, 5, 6, 7, 8]
            ],
            [
                self._set_mode_callback,
                '/mode/set',
                String,
                1
            ],
            [
                self._arm_callback,
                '/arm',
                Bool,
                1
            ],
        ]

        self.mavlink_msg_available = {}

        for _, topic, msg, queue in self.pub_topics:
            self.mavlink_msg_available[topic] = 0
            self._pub_subscribe_topic(topic, msg, queue)

        for topic in self.sub_topics:
            if len(topic) <= 4:
                callback, topic_name, msg, queue = topic
                self._sub_subscribe_topic(topic_name, msg, queue, callback)
            else:
                callback, topic_name, msg, queue, arg = topic
                for name in arg:
                    self._sub_subscribe_topic(topic_name.format(name), msg, queue, callback)
        #above are new




    def _server_wait_conn(self):
        while 'waiting for server to respond':
            self.master.mav.ping_send(int(time.time() * 1e6), 0, 0, 0)
            if self.master.recv_match():
                break
            time.sleep(0.5)

    def _wait_autopilot_heartbeat(self):
        ensure_autopilot_heartbeat = (
            f'HEARTBEAT.get_srcSystem() == {self.master.target_system} and '
            f'HEARTBEAT.get_srcComponent() == {mavlink.MAV_COMP_ID_AUTOPILOT1}'
        )
        self.recv_match(type='HEARTBEAT', condition=ensure_autopilot_heartbeat,
                        blocking=True)

    def read_param(self, name: str, index: int=-1, timeout: float=None):
        self.mav.param_request_read_send(
            *self.target,
            name.encode('utf8'),
            index
        )
        logging.note(f'read_param({name=}, {index=}, {timeout=})')
        return self.recv_match(type='PARAM_VALUE', blocking=True,
                               timeout=timeout)

    def set_param(self, name: str, value: float, type: int=0,
                  timeout: float=1, retries: int=3):
        name = name.encode('utf8')
        self.mav.param_set_send(
            *self.target,
            name, value, type
        )
        logging.info(f'set_param({name=}, {value=}, {type=})')

        while not (msg := self.recv_match(type='PARAM_VALUE', blocking=True,
                                          timeout=timeout)) and retries > 0:
            retries -= 1
            logging.debug(f'param set timed out after {timeout}s, retrying...')
            self.mav.param_set_send(
                *self.target,
                name, value, type
            )
        return msg

    def __enter__(self):
        ''' Send regular heartbeats while using in a context manager. '''
        logging.info('__enter__ -> reviving heart (if required)')
        self.heart.revive()
        return self

    def __exit__(self, *args):
        ''' Automatically disarm and stop heartbeats on error/context-close. '''
        logging.info('__exit__ -> disarming, and stopping heart')
        self.cleanup(disconnect=False)

    def arm(self):
        self.master.arducopter_arm()
        logging.debug('arm requested, waiting...')
        self.master.motors_armed_wait()
        logging.info('Motors armed!')

    def disarm(self):
        self.master.arducopter_disarm()
        logging.debug('disarm requested, waiting...')
        self.master.motors_disarmed_wait()
        logging.info('Motors disarmed')

    def set_mode(self, mode):
        ''' Sets autopilot 'mode', by name or id number. '''
        if isinstance(mode, str):
            for attempt in range(5):
                if (mapping := self.master.mode_mapping()):
                    break # success - mapping available
                self._wait_autopilot_heartbeat()
                logging.info('mode mapping not available, retrying...')
            else:
                logging.warning('mode change failed - no mapping available!')
                return
            mode_id = mapping[mode.upper()]
        else:
            mode_id = mode
        self.master.set_mode(mode_id)
        logging.debug(f'setting {mode=} ({mode_id=}), waiting...')

        while 'mode change not confirmed':
            ack_msg = self.recv_match(type='COMMAND_ACK', blocking=True)
            # check if acknowledged MAV_CMD_DO_SET_MODE or SET_MODE (old)
            if ack_msg.command in (176, 11):
                if ack_msg.result == 0:
                    logging.info(f'{mode=}, change successful')
                else:
                    result = mavlink.enums['MAV_RESULT'][ack_msg.result]
                    logging.warning('mode change failed!\n\t%s: %s',
                                    result.name, result.description)
                break

    def set_servo(self, servo, pwm):
        ''' Set a single servo output pwm value.

        'servo' can only be outputs that aren't assigned as motors, so is
          generally used for lights/camera etc.

        When in a per_thruster_control context in Servo mode, also allows
          controlling individual thrusters.

        '''
        logging.info(f'set_servo({servo=}, {pwm=})')
        self.master.set_servo(servo, pwm)

    def send_rc(self, rcin1=65535, rcin2=65535, rcin3=65535, rcin4=65535,
                rcin5=65535, rcin6=65535, rcin7=65535, rcin8=65535,
                rcin9=65535, rcin10=65535, rcin11=65535, rcin12=65535,
                rcin13=65535, rcin14=65535, rcin15=65535, rcin16=65535,
                rcin17=65535, rcin18=65535, *, # keyword-only from here
                pitch=None, roll=None, throttle=None, yaw=None, forward=None,
                lateral=None, camera_pan=None, camera_tilt=None, lights1=None,
                lights2=None, video_switch=None):
        ''' Sets all 18 rc channels as specified.

        Values should be between 1100-1900, or left as 65535 to ignore.

        Can specify values:
            positionally,
            or with rcinX (X=1-18),
            or with default RC Input channel mapping names
              -> see https://ardusub.com/developers/rc-input-and-output.html

        It's possible to mix and match specifier types (although generally
          not recommended). Default channel mapping names override positional
          or rcinX specifiers.

        '''
        rc_channel_values = (
            pitch        or rcin1,
            roll         or rcin2,
            throttle     or rcin3,
            yaw          or rcin4,
            forward      or rcin5,
            lateral      or rcin6,
            camera_pan   or rcin7,
            camera_tilt  or rcin8,
            lights1      or rcin9,
            lights2      or rcin10,
            video_switch or rcin11,
            rcin12, rcin13, rcin14, rcin15, rcin16, rcin17, rcin18
        )
        logging.info(f'send_rc')
        logging.debug(rc_channel_values)
        self.mav.rc_channels_override_send(
            *self.target,
            *rc_channel_values
        )

    def clear_motion(self, stopped_pwm=1500):
        ''' Sets all 6 motion direction RC inputs to 'stopped_pwm'. '''
        logging.info('clear_motion')
        self.send_rc(*[stopped_pwm]*6)

    def get_thruster_outputs(self):
        ''' Returns (and notes) the first 'self.thrusters' servo PWM values.

        Offset by 1500 to make it clear how each thruster is active.

        '''
        logging.debug('get_thruster_outputs')
        servo_outputs = self.recv_match(type='SERVO_OUTPUT_RAW',
                                        blocking=True).to_dict()
        thruster_outputs = [servo_outputs[f'servo{i+1}_raw'] - 1500
                            for i in range(self.thrusters)]
        logging.note(f'{thruster_outputs=}')
        return thruster_outputs

    def status_loop(self, duration, delay=0.05):
        ''' Loop for 'duration', with 'delay' between iterations. [s]

        Useful for debugging.

        '''
        start = time.time()
        while time.time() - start < duration:
            self.get_thruster_outputs()
            time.sleep(delay)

    def command_long_send(self, command, confirmation=0, param1=0, param2=0,
                          param3=0, param4=0, param5=0, param6=0, param7=0):
        self.mav.command_long_send(
            *self.target,
            getattr(mavlink, f'MAV_CMD_{command}', command),
            confirmation,
            param1, param2, param3, param4, param5, param6, param7
        )

    def set_message_interval(self, message, interval, response_target=0):
        ''' Set message interval, as per MAV_CMD_SET_MESSAGE_INTERVAL.

        Required to get specific messages if not auto-streamed by connection.

        'message' is the name or id of the message
        'interval' is the desired interval [us]

        '''
        logging.info(f'set_message_interval({message=}, {interval=})')
        self.command_long_send(
            'SET_MESSAGE_INTERVAL',
            param1 = getattr(mavlink, f'MAVLINK_MSG_ID_{message}', message),
            param2 = interval, param7 = response_target)

    def set_bulk_message_intervals(self, messages, interval=None):
        ''' Set several message intervals at once. '''
        if interval is None:
            if isinstance(messages, dict):
                iterator = messages.items()
            else:
                # assume iterator of pairs
                iterator = messages
        elif isinstance(interval, Number):
            iterator = zip(messages, repeat(interval))
        else:
            iterator = zip(messages, interval)

        for message, interval in iterator:
            self.set_message_interval(message, interval)

    def disconnect(self):
        ''' End the MAVLink connection. '''
        logging.info('disconnect -> closing MAVLink connection')
        self.master.close()

    def cleanup(self, *, disconnect=True):
        ''' Attempt to disarm, then stop sending heartbeats.
            Optionally disconnect (true by default).
        '''
        try:
            self.disarm()
        except OSError:
            pass
        self.heart.kill()
        
        if disconnect:
            self.disconnect()

    @classmethod
    def connect_to_client(cls, ip='0.0.0.0', port=14550, *args, **kwargs):
        ''' Default for topside connection to Blue Robotics Companion/BlueOS. '''
        logging.note(f'connect_to_client({ip=}, {port=}, {args=}, {kwargs=})')
        return cls(f'udpin:{ip}:{port}', *args, **kwargs)

    @classmethod
    def connect_to_server(cls, ip='blueos.local', port=14550, *args, **kwargs):
        ''' Less network configuration required, but seemingly less robust. '''
        logging.note(f'connect_to_server({ip=}, {port=}, {args=}, {kwargs=})')
        return cls(f'udpout:{ip}:{port}', *args, **kwargs, client=False)






    #more ros callback and publish_functions are here:
    def get_data(self):
        """ Return data

        Returns:
            TYPE: Dict
        """
        return self.data

    def get_all_msgs(self):
        """ Return all mavlink messages

        Returns:
            TYPE: dict
        """
        msgs = []
        while True:
            msg = self.master.recv_match()
            if msg != None:
                msgs.append(msg)
            else:
                break
        return msgs

    def update(self):
        """ Update data dict
        """
        # Get all messages
        msgs = self.get_all_msgs()
        # Update dict
        for msg in msgs:
            self.data[msg.get_type()] = msg.to_dict()

    def print_data(self):
        """ Debug function, print data dict
        """
        print(self.data)


    def decode_mode(self, base_mode, custom_mode):
        """ Decode mode from heartbeat
            http://mavlink.org/messages/common#heartbeat

        Args:
            base_mode (TYPE): System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
            custom_mode (TYPE): A bitfield for use for autopilot-specific flags.

        Returns:
            [str, bool]: Type mode string, arm state
        """
        flight_mode = ""

        mode_list = [
            [mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, 'MANUAL'],
            [mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED, 'STABILIZE'],
            [mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED, 'GUIDED'],
            [mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED, 'AUTO'],
            [mavutil.mavlink.MAV_MODE_FLAG_TEST_ENABLED, 'TEST']
        ]

        if base_mode == 0:
            flight_mode = "PreFlight"
        elif base_mode & mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
            flight_mode = mavutil.mode_mapping_sub[custom_mode]
        else:
            for mode_value, mode_name in mode_list:
                if base_mode & mode_value:
                    flight_mode = mode_name

        arm = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

        return flight_mode, arm

    def set_guided_mode(self):
        """ Set guided mode
        """
        #https://github.com/ArduPilot/pymavlink/pull/128
        params = [mavutil.mavlink.MAV_MODE_GUIDED, 0, 0, 0, 0, 0, 0]
        self.command_long_send(mavutil.mavlink.MAV_CMD_DO_SET_MODE, params)


    def set_position_target_local_ned(self, param=[]):
        """ Create a SET_POSITION_TARGET_LOCAL_NED message
            http://mavlink.org/messages/common#SET_POSITION_TARGET_LOCAL_NED

        Args:
            param (list, optional): param1, param2, ..., param11
        """
        if len(param) != 11:
            print('SET_POISITION_TARGET_GLOBAL_INT need 11 params')

        # Set mask
        mask = 0b0000000111111111
        for i, value in enumerate(param):
            if value is not None:
                mask -= 1<<i
            else:
                param[i] = 0.0

        #http://mavlink.org/messages/common#SET_POSITION_TARGET_GLOBAL_INT
        self.master.mav.set_position_target_local_ned_send(
            0,                                              # system time in milliseconds
            self.master.target_system,                        # target system
            self.master.target_component,                     # target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,            # frame
            mask,                                           # mask
            param[0], param[1], param[2],                   # position x,y,z
            param[3], param[4], param[5],                   # velocity x,y,z
            param[6], param[7], param[8],                   # accel x,y,z
            param[9], param[10])                            # yaw, yaw rate

    def set_attitude_target(self, param=[]):
        """ Create a SET_ATTITUDE_TARGET message
            http://mavlink.org/messages/common#SET_ATTITUDE_TARGET

        Args:
            param (list, optional): param1, param2, ..., param7
        """
        if len(param) != 8:
            print('SET_ATTITUDE_TARGET need 8 params')

        # Set mask
        mask = 0b11111111
        for i, value in enumerate(param[4:-1]):
            if value is not None:
                mask -= 1<<i
            else:
                param[i+3] = 0.0

        if param[7] is not None:
            mask += 1<<6
        else:
            param[7] = 0.0

        q = param[:4]

        if q != [None, None, None, None]:
            mask += 1<<7
        else:
            q = [1.0, 0.0, 0.0, 0.0]

        self.master.mav.set_attitude_target_send(0,   # system time in milliseconds
            self.master.target_system,                # target system
            self.master.target_component,             # target component
            mask,                                   # mask
            q,                                      # quaternion attitude
            param[4],                               # body roll rate
            param[5],                               # body pitch rate
            param[6],                               # body yaw rate
            param[7])                               # thrust


    
    
    
    @staticmethod
    def _callback_from_topic(topic):
        """ Create callback function name

        Args:
            topic (str): Topic name

        Returns:
            str: callback name
        """
        return topic.replace('/', '_') + '_callback'

    def _pub_subscribe_topic(self, topic, msg, queue_size=1):
        """ Subscribe to a topic using the publisher

        Args:
            topic (str): Topic name
            msg (TYPE): ROS message type
            queue_size (int, optional): Queue size
        """
        self.pub.subscribe_topic(self.ROV_name + topic, msg, queue_size)

    def _sub_subscribe_topic(self, topic, msg, queue_size=1, callback=None):
        """ Subscribe to a topic using the subscriber

        Args:
            topic (str): Topic name
            msg (TYPE): ROS message type
            queue_size (int, optional): Queue size
            callback (None, optional): Callback function
        """
        self.sub.subscribe_topic(self.ROV_name + topic, msg, queue_size, callback)

    def _set_servo_callback(self, msg, topic):
        """ Set servo from topic

        Args:
            msg (TYPE): ROS message
            topic (TYPE): Topic name

        Returns:
            None: Description
        """
        paths = topic.split('/')
        servo_id = None
        for path in paths:
            if 'servo' in path:
                servo_id = int(re.search('[0-9]', path).group(0)) + 1
                # Found valid id !
                break
        else:
            # No valid id
            return

        self.set_servo(servo_id, msg.data)

    def _set_rc_channel_callback(self, msg, topic):
        """ Set RC channel from topic

        Args:
            msg (TYPE): ROS message
            topic (TYPE): Topic name

        Returns:
            TYPE: Description
        """
        paths = topic.split('/')
        channel_id = None
        for path in paths:
            if 'rc_channel' in path:
                channel_id = int(re.search('[0-9]', path).group(0))  - 1
                # Found valid id !
                break
        else:
            # No valid id
            return

        self.send_rc(channel_id, msg.data)

    def _set_mode_callback(self, msg, _):
        """ Set ROV mode from topic

        Args:
            msg (TYPE): Topic message
            _ (TYPE): Description
        """
        self.set_mode(msg.data)

    def _arm_callback(self, msg, _):
        """ Set arm state from topic

        Args:
            msg (TYPE): ROS message
            _ (TYPE): Description
        """
        self.arm(msg.data)

    def _setpoint_velocity_cmd_vel_callback(self, msg, _):
        """ Set angular and linear velocity from topic

        Args:
            msg (TYPE): ROS message
            _ (TYPE): Description
        """
        #http://mavlink.org/messages/common#SET_POSITION_TARGET_GLOBAL_INT
        params = [
            None,
            None,
            None,
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
            None,
            None,
            None,
            None,
            None,
            ]
        self.set_position_target_local_ned(params)

        #http://mavlink.org/messages/common#SET_ATTITUDE_TARGET
        params = [
            None,
            None,
            None,
            None,
            msg.twist.angular.x,
            msg.twist.angular.y,
            msg.twist.angular.z,
            None,
            ]
        self.set_attitude_target(params)

    def _create_header(self, msg):
        """ Create ROS message header

        Args:
            msg (ROS message): ROS message with header
        """
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.model_base_link

    def _create_odometry_msg(self):
        """ Create odometry message from ROV information

        Raises:
            Exception: No data to create the message
        """

        # Check if data is available
        if 'LOCAL_POSITION_NED' not in self.get_data():
            raise Exception('no LOCAL_POSITION_NED data')

        if 'ATTITUDE' not in self.get_data():
            raise Exception('no ATTITUDE data')

        #TODO: Create class to deal with BlueRov state
        msg = Odometry()

        self._create_header(msg)

        #http://mavlink.org/messages/common#LOCAL_POSITION_NED
        local_position_data = self.get_data()['LOCAL_POSITION_NED']
        xyz_data = [local_position_data[i]  for i in ['x', 'y', 'z']]
        vxyz_data = [local_position_data[i]  for i in ['vx', 'vy', 'z']]
        msg.pose.pose.position.x = xyz_data[0]
        msg.pose.pose.position.y = xyz_data[1]
        msg.pose.pose.position.z = xyz_data[2]
        msg.twist.twist.linear.x = vxyz_data[0]/100
        msg.twist.twist.linear.y = vxyz_data[1]/100
        msg.twist.twist.linear.z = vxyz_data[2]/100

        #http://mavlink.org/messages/common#ATTITUDE
        attitude_data = self.get_data()['ATTITUDE']
        orientation = [attitude_data[i] for i in ['roll', 'pitch', 'yaw']]
        orientation_speed = [attitude_data[i] for i in ['rollspeed', 'pitchspeed', 'yawspeed']]

        #https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_Angles_to_Quaternion_Conversion
        cy = math.cos(orientation[2] * 0.5)
        sy = math.sin(orientation[2] * 0.5)
        cr = math.cos(orientation[0] * 0.5)
        sr = math.sin(orientation[0] * 0.5)
        cp = math.cos(orientation[1] * 0.5)
        sp = math.sin(orientation[1] * 0.5)

        msg.pose.pose.orientation.w = cy * cr * cp + sy * sr * sp
        msg.pose.pose.orientation.x = cy * sr * cp - sy * cr * sp
        msg.pose.pose.orientation.y = cy * cr * sp + sy * sr * cp
        msg.pose.pose.orientation.z = sy * cr * cp - cy * sr * sp
        msg.twist.twist.angular.x = orientation_speed[0]
        msg.twist.twist.angular.y = orientation_speed[1]
        msg.twist.twist.angular.z = orientation_speed[2]

        self.pub.set_data('/odometry', msg)

    def _create_imu_msg(self):
        """ Create imu message from ROV data

        Raises:
            Exception: No data available
        """

        # Check if data is available
        if 'ATTITUDE' not in self.get_data():
            raise Exception('no ATTITUDE data')

        #TODO: move all msgs creating to msg
        msg = Imu()

        self._create_header(msg)

        #http://mavlink.org/messages/common#SCALED_IMU
        imu_data = None
        for i in ['', '2', '3']:
            try:
                imu_data = self.get_data()['SCALED_IMU{}'.format(i)]
                break
            except Exception as e:
                pass

        if imu_data is None:
            raise Exception('no SCALED_IMUX data')

        acc_data = [imu_data['{}acc'.format(i)]  for i in ['x', 'y', 'z']]
        gyr_data = [imu_data['{}gyro'.format(i)] for i in ['x', 'y', 'z']]
        mag_data = [imu_data['{}mag'.format(i)]  for i in ['x', 'y', 'z']]

        #http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
        msg.linear_acceleration.x = acc_data[0]/100
        msg.linear_acceleration.y = acc_data[1]/100
        msg.linear_acceleration.z = acc_data[2]/100
        msg.linear_acceleration_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]

        msg.angular_velocity.x = gyr_data[0]/1000
        msg.angular_velocity.y = gyr_data[1]/1000
        msg.angular_velocity.z = gyr_data[2]/1000
        msg.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]

        #http://mavlink.org/messages/common#ATTITUDE
        attitude_data = self.get_data()['ATTITUDE']
        orientation = [attitude_data[i] for i in ['roll', 'pitch', 'yaw']]

        #https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_Angles_to_Quaternion_Conversion
        cy = math.cos(orientation[2] * 0.5)
        sy = math.sin(orientation[2] * 0.5)
        cr = math.cos(orientation[0] * 0.5)
        sr = math.sin(orientation[0] * 0.5)
        cp = math.cos(orientation[1] * 0.5)
        sp = math.sin(orientation[1] * 0.5)

        msg.orientation.w = cy * cr * cp + sy * sr * sp
        msg.orientation.x = cy * sr * cp - sy * cr * sp
        msg.orientation.y = cy * cr * sp + sy * sr * cp
        msg.orientation.z = sy * cr * cp - cy * sr * sp

        msg.orientation_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.pub.set_data('/imu/data', msg)

    def _create_battery_msg(self):
        """ Create battery message from ROV data

        Raises:
            Exception: No data available
        """

        # Check if data is available
        if 'SYS_STATUS' not in self.get_data():
            raise Exception('no SYS_STATUS data')

        if 'BATTERY_STATUS' not in self.get_data():
            raise Exception('no BATTERY_STATUS data')

        bat = BatteryState()
        self._create_header(bat)

        #http://docs.ros.org/jade/api/sensor_msgs/html/msg/BatteryState.html
        bat.voltage = self.get_data()['SYS_STATUS']['voltage_battery']/1000
        bat.current = self.get_data()['SYS_STATUS']['current_battery']/100
        bat.percentage = self.get_data()['BATTERY_STATUS']['battery_remaining']/100
        self.pub.set_data('/battery', bat)

    def _create_camera_msg(self):
        if not self.video.frame_available():
            return
        frame = self.video.frame()
        image_msg = Image()
        self._create_header(image_msg)
        height, width, channels = frame.shape
        image_msg.width = width
        image_msg.height = height
        image_msg.encoding = 'bgr8'
        image_msg.data = frame
        msg = self.video_bridge.cv2_to_imgmsg(frame, "bgr8")
        self._create_header(msg)
        msg.step = int(msg.step)
        self.pub.set_data('/camera/image_raw', msg)

    def _create_ROV_state(self):
        """ Create ROV state message from ROV data

        Raises:
            Exception: No data available
        """

        # Check if data is available
        if 'SERVO_OUTPUT_RAW' not in self.get_data():
            raise Exception('no SERVO_OUTPUT_RAW data')

        if 'HEARTBEAT' not in self.get_data():
            raise Exception('no HEARTBEAT data')

        servo_output_raw_msg = self.get_data()['SERVO_OUTPUT_RAW']
        servo_output_raw = [servo_output_raw_msg['servo{}_raw'.format(i+1)] for i in range(8)]
        motor_throttle = [servo_output_raw[i] - 1500 for i in range(6)]
        # 1100 -> -1 and 2000 -> 1
        for throttle in motor_throttle:
            if throttle < 0:
                throttle = throttle/400
            else:
                throttle = throttle/500

        light_on = (servo_output_raw[6] - 1100) / 8
        #need to check
        camera_angle = servo_output_raw[7] - 1500

        # Create angle from pwm
        camera_angle = -45*camera_angle/400

        base_mode = self.get_data()['HEARTBEAT']['base_mode']
        custom_mode = self.get_data()['HEARTBEAT']['custom_mode']

        mode, arm = self.decode_mode(base_mode, custom_mode)

        state = {
            'motor': motor_throttle,
            'light': light_on,
            'camera_angle': camera_angle,
            'mode': mode,
            'arm': arm
        }

        string = String()
        string.data = str(json.dumps(state, ensure_ascii=False))

        self.pub.set_data('/state', string)

    def publish(self):
        """ Publish the data in ROS topics
        """
        self.update()
        for sender, topic, _, _ in self.pub_topics:
            try:
                if time.time() - self.mavlink_msg_available[topic] > 1:
                    sender()
            except Exception as e:
                self.mavlink_msg_available[topic] = time.time()
                print(e)
    def get_odometry_msg(self):
        msg=Odometry()
        msg.header.stamp=rospy.Time.now()
        msg.header.frame_id="map"
        msg.child_frame_id="base_link"
        glo_pos=self.master.recv_match(type='GLOBAL_POSITION_INT',blocking=False)
        #if glo_pos:
            #print(f"globle_ned:{glo_pos.lat:.6f},{glo_pos.lon:.6f},{glo_pos.alt:.6f}")
            #print(f"globle_ned:{glo_pos.x},{glo_pos.y},{glo_pos.z}")
        attitude = self.master.recv_match(type='ATTITUDE',blocking=True,timeout=0.1)
        #print(f"attitude:{attitude.roll},{attitude.pitch},{attitude.yaw}")
        

        if glo_pos==None:
            msg.pose.pose.position.x=0
            msg.pose.pose.position.y=0
            msg.pose.pose.position.z=0
            msg.twist.twist.linear.x=0
            msg.twist.twist.linear.y=0
            msg.twist.twist.linear.z=0
        else:
            msg.pose.pose.position.x=glo_pos.lat
            msg.pose.pose.position.y=glo_pos.lon
            msg.pose.pose.position.z=-glo_pos.alt
            msg.twist.twist.linear.x=glo_pos.vy
            msg.twist.twist.linear.y=glo_pos.vx
            msg.twist.twist.linear.z=-glo_pos.vz
        '''q=attitude.q
        msg.pose.pose.orientation.w=q[0]
        msg.pose.pose.orientation.x=q[2]
        msg.pose.pose.orientation.y=q[1]
        msg.pose.pose.orientation.z=-q[3]'''
        orientation = [attitude.roll,attitude.pitch,attitude.yaw]
        cy = math.cos(orientation[2] * 0.5)
        sy = math.sin(orientation[2] * 0.5)
        cr = math.cos(orientation[0] * 0.5)
        sr = math.sin(orientation[0] * 0.5)
        cp = math.cos(orientation[1] * 0.5)
        sp = math.sin(orientation[1] * 0.5)
        msg.pose.pose.orientation.w = cy * cr * cp + sy * sr * sp
        msg.pose.pose.orientation.x = cy * sr * cp - sy * cr * sp
        msg.pose.pose.orientation.y = cy * cr * sp + sy * sr * cp
        msg.pose.pose.orientation.z = sy * cr * cp - cy * sr * sp
        msg.twist.twist.angular.x=attitude.rollspeed
        msg.twist.twist.angular.y=attitude.pitchspeed
        msg.twist.twist.angular.z=attitude.yawspeed
        return msg

class ExtendedAutopilot(Autopilot):
    ''' An Autopilot with extended control functionality. '''
    SERVO_OUTPUT_FUNCTIONS = {
        'Disabled': 0,
        'RCPassThru': 1,
        **{f'Motor{i}': param_value for i, param_value in
           enumerate(chain(range(33, 33+8), range(82, 82+4)), start=1)},
        **{f'RCIN{i}': param_value for i, param_value in
           enumerate(range(51, 51+16), start=1)},
    }

    @contextmanager
    def per_thruster_control(self, mode='Servo', stopped_pwm=1500,
                             backup_timeout: float=None,
                             set_timeout: float=1, set_retries: int=3):
        ''' Allow thrusters to be controlled individually, within a context.

        WARNING: while in context, thruster actions no longer require arming,
          and failsafes are no longer engaged!

        'mode' can be 'Servo' (default) or 'RCPassThru' to control thrusters
          either using self.set_servo, or using self.send_rc.

        WARNING: if the vehicle is powered down while in RCPassThru mode the
          thrusters may become active WITHOUT ARMING on next power up, from
          joystick input. It is more powerful than Servo mode because multiple
          thrusters can be controlled in a single function call, but use with
          significant caution.

        For vehicles with more than 6 thrusters RCPassThru mode is not
          recommended, because it doubles-up on existing RC Input mappings
          (e.g. Camera Pan and Tilt are mapped to inputs 7 and 8). That issue
          can be worked around if necessary by mapping individual outputs to
          only otherwise unused RCINx inputs (generally 1-6, 12-18), but that's
          not currently supported/implemented by this function.
        Another alternative is remapping the input channels with PARAM_MAP_RC,
          although that can cause some difficult debugging issues when
          comparing behavior to BR documentation/examples, which assumes the
          default mappings.

        '''
        logging.info('Engaging per-thruster control!')
        logging.warning('Thruster actions no longer require arming!')
        logging.warning('Failsafes no longer engaged!')

        if mode == 'RCPassThru':
            output_function = self.SERVO_OUTPUT_FUNCTIONS[mode]
            logging.critical('RCPassThru mode engaged:\n'
                ' -> THRUSTERS RESPOND DIRECTLY TO RC/JOYSTICK INPUTS')
        elif mode == 'Servo':
            output_function = self.SERVO_OUTPUT_FUNCTIONS['Disabled']
            logging.info('Servo mode engaged:\n'
                         ' -> control thrusters with self.set_servo')
        else:
            raise ValueError(f'Invalid per-thruster control {mode=}')

        self._backup_servo_functions(backup_timeout)
        try:
            # try to make sure thrusters are stopped before changeover
            self._stop_thrusters(mode, stopped_pwm)
            # set first n SERVO outputs to passthrough first n RC inputs
            for n in range(1, self.thrusters + 1):
                self.set_param(f'SERVO{n}_FUNCTION', output_function,
                               timeout=set_timeout, retries=set_retries)
            # make sure thrusters are actually stopped
            self._stop_thrusters(mode, stopped_pwm)

            yield self
        finally:
            # make sure to stop thrusters before changing back
            self._stop_thrusters(mode, stopped_pwm)
            self._restore_servo_functions(set_timeout, set_retries)

        logging.info('per-thruster control successfully exited')
        logging.warning('Arming and failsafe requirements re-engaged')

    def _stop_thrusters(self, mode, stopped_pwm):
        if mode == 'RCPassThru':
            self.send_rc(*[stopped_pwm]*self.thrusters)
        else:
            for servo in range(1, self.thrusters + 1):
                self.set_servo(servo, stopped_pwm)

    def _backup_servo_functions(self, timeout: float=None):
        logging.note('backing up servo thruster functions')
        self._old_servo_functions = {}
        for n in range(1, self.thrusters + 1):
            param = f'SERVO{n}_FUNCTION'
            self._old_servo_functions[param] = self.read_param(param)

    def _restore_servo_functions(self, timeout: float=1, retries: int=3):
        for param, backup in self._old_servo_functions.items():
            self.set_param(param, backup.param_value, backup.param_type,
                           timeout, retries)
        logging.note('servo functions restored')

def configure_logging(level, filename, datefmt):
    # configure logging, based on defaults/user input
    log_level = getattr(logging, level)
    logging.basicConfig(filename=filename, level=log_level, datefmt=datefmt,
                        format=('%(levelname)s: %(asctime)s.%(msecs)03d - '
                                '%(message)s'))

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
    args=parser.parse_args()
    client = args.mavlink.startswith('udpin')

    configure_logging(args.log.upper(), args.filename, args.datefmt)

    # run/test the desired functionality
    print('If script gets stuck, press CTRL+C to disarm and quit\n')

    with Autopilot(args.mavlink, client=client) as sub:
        if args.request:
            # request SERVO_OUTPUT_RAW at 5Hz
            sub.set_message_interval('SERVO_OUTPUT_RAW', 2e5)

        sub.set_mode('manual')
        sub.get_thruster_outputs()
        # make sure all motion is set to stationary before arming
        sub.clear_motion()
        sub.get_thruster_outputs()
        sub.arm()
        # set some roll
        sub.send_rc(roll=1700)
        sub.status_loop(1)
        # add a bit of throttle
        sub.send_rc(throttle=1600)
        sub.status_loop(1.5)

    # disarm and wait for a bit
    time.sleep(5)

    # re-enter
    with sub:
        logging.info('RE-ENTERING')
        sub.set_mode('stabilize')
        sub.clear_motion()
        sub.arm()
        # set yaw velocity and some forward motion at the same time
        sub.send_rc(yaw=1800, forward=1600)
        sub.status_loop(2)

    # disconnect, and wait for a bit
    sub.disconnect()
    time.sleep(5)

    # make a new connection, and engage per-thruster control mode
    with ExtendedAutopilot(args.mavlink, client=client) as e_sub, \
         e_sub.per_thruster_control():
        # display initial thruster output values
        e_sub.get_thruster_outputs()
        # set some thruster values
        e_sub.set_servo(1, 1300)
        e_sub.status_loop(1)
        e_sub.set_servo(3, 1400)
        e_sub.set_servo(4, 1550)
        e_sub.status_loop(1.5)
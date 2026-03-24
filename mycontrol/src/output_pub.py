#this node pubishes control inputs to ros and directly sets servo to bluerov2heavy
#!/usr/bin/env python3

#import rc_override_example
from pymavlink import mavutil

max_servo_change = 32 #for 50hz control, 0.5 second from 1100->1900pwm
last_output=[1500]*8
def output_slew(output):
    global max_servo_change
    global last_output

    for i in range(8):
        output[i]=max(last_output[i]-max_servo_change, min(last_output[i]+max_servo_change, output[i]))
        last_output[i] = output[i]

    return output


#mapping control output to pwm values
def output2pwm(u):
    #the pwm value range is 1100-1900
    output=[1500,1500,1500,1500,1500,1500] #default pwm value
    c_p = 400 #coefficient for pitch
    c_r = 400 #coefficient for roll
    c_t = 400 #coefficient for throttle
    c_y = 400 #coefficient for yaw
    c_f = 400 #coefficient for forward
    c_l = 400 #coefficient for lateral
    output[0]=int(1500+c_p*u[0])  # pitch
    output[1]=int(1500+c_r*u[1])  # roll
    output[2]=int(1500+c_t*u[2])  # throttle
    output[3]=int(1500+c_y*u[3])  # yaw
    output[4]=int(1500+c_f*u[4])  # forward
    output[5]=int(1500+c_l*u[5])  # lateral
    output=[max(1100, min(1900, val)) for val in output] #saturate the pwm values to be in [1100,1900]
    return output


def rc_control_outputs(sub, output):
    sub.send_rc(1, output[0])  # set channel pitch
    sub.send_rc(2, output[1])  # set channel roll
    sub.send_rc(3, output[2])  # set channel throttle
    sub.send_rc(4, output[3])  # set channel yaw
    sub.send_rc(5, output[4])  # set channel forward
    sub.send_rc(6, output[5])  # set channel lateral


def demixer2servo(outputs):
    # Convert 6 control outputs to 8 servo outputs
    # Placeholder demixing logic; replace with actual demixing as needed
    # Note that the mixer here is just a simple example and may not reflect the actual thruster configuration
    servo_outputs = [1500]*8
    servo_outputs[0] = outputs[0] + outputs[1]   # Motor 1
    servo_outputs[1] = outputs[0] - outputs[1]   # Motor 2
    servo_outputs[2] = outputs[2] + outputs[3]   # Motor 3
    servo_outputs[3] = outputs[2] - outputs[3]   # Motor 4
    servo_outputs[4] = outputs[4]                # Motor 5
    servo_outputs[5] = outputs[5]                # Motor 6
    servo_outputs[6] = outputs[4]                # Motor 7
    servo_outputs[7] = outputs[5]                # Motor 8
    return servo_outputs


def servo_control_outputs(sub, servo_outputs):
    servo_outputs = [int(1500 + 400*so) for so in servo_outputs]
    servo_outputs = output_slew(servo_outputs)
    servo_outputs = [max(1100, min(1900, val)) for val in servo_outputs] #saturate the pwm values to be in [1100,1900]
    sub.set_servo(1, servo_outputs[0])  # set motor 1
    sub.set_servo(2, servo_outputs[1])  # set motor 2
    sub.set_servo(3, servo_outputs[2])  # set motor 3
    sub.set_servo(4, servo_outputs[3])  # set motor 4
    sub.set_servo(5, servo_outputs[4])  # set motor 5
    sub.set_servo(6, servo_outputs[5])  # set motor 6
    sub.set_servo(7, servo_outputs[6])  # set motor 7
    sub.set_servo(8, servo_outputs[7])  # set motor 8

def topic2pwm(outputs):
    pwm_values = [int(1500 + 400*o) for o in outputs]
    pwm_values = [max(1100, min(1900, val)) for val in pwm_values] #saturate the pwm values to be in [1100,1900]
    return pwm_values
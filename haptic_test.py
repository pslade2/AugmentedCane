#!/usr/bin/env python3

# example calling:
# python3 haptic_test.py mode_num mode_cnt subj
# mode_num = [0,1,2] for (vibration, motor, audio)
# mode_cnt = 1 ... for trial number with that mode
# subj is subj number "SXX"
# ex:  python3 haptic_test.py 2 1 SXX
# yellow wire connects to outside pin in header

#TODO: actual controls setup
#save csv file at the end with time stamp
#or, ask for subject number / name to save with

import numpy as np
import time
from datetime import datetime
import datetime as dt
import serial
import struct
import signal
import sys
import random
import pandas as pd
import os
from enum import Enum
import cane_functions as cf
import vlc

import RPi.GPIO as IO
import board
import busio
import adafruit_drv2605
#0x48 - address of the adc
import adafruit_ads1x15.ads1015 as ADS #adc
from adafruit_ads1x15.analog_in import AnalogIn
import adafruit_tca9548a #mux


#folder containing audio. names must be like "-90.mp3"
audio_folder = "turn_audio_files/"
audio_mode_file = audio_folder + "audio_mode.mp3"

#controls proportion constant
proportion_constant = 1
integral_constant = 0.001 #0.001 to 0.01 is reasonable
steering_interval = 30 #max seconds user has to reach the goal
write_data_interval = 0.01 #max seconds between write current user position
button_hold_thres = 0.5 #in ms
deadband_degrees = 1 #n degrees on either side of goal to give deadband

############# pins and hardware settings #############

deadband_min_l = 0.15 #proportion of range that the driver produces no vibration.
deadband_min_r = 0.19 #proportion of range that the driver produces no vibration.
full_rotation_adc = 9856 #adc output for full 360 degrees of pot rotation

motor_gain = 0.43 # for angles, 0.035 for based on distance
motor_max = 70 # max speed (PWM)
motor_min = 30 # min speed where motion occurs w/ friction (PWM)
degree_clip = 90 #motor reaches max value at this angle (0-180)
motor_command = 1

#bcm numbering system!
push_button_pin = 18
m_pin_R = 12
m_pin_L = 13
pwm_freq = 100

######################### Devices ##############################
p_L = None #will be initialized when we make motor
p_R = None
i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1015(i2c) #adc
chan = AnalogIn(ads, ADS.P0)
tca = adafruit_tca9548a.TCA9548A(i2c) #mux
drv_l = adafruit_drv2605.DRV2605(tca[0])
drv_r = adafruit_drv2605.DRV2605(tca[1])

# DEFINING BIT CONSTANTS
_DRV2605_REG_MODE = 0x01
_DRV2605_REG_RTPIN = 0x02
_DRV2605_DATA_FORMAT_RTP = 0x1D


###########################################################
#############     STATE MACHINE VARIABLES       ###########

#device operation mode. 0 for vibration, 1 for motor, 2 for audio.
mode = 0
start_button_held = datetime.now() #initial timestamp for holding down the button
end_mode_select = False
waiting = False


#########################################################################
###################### HARDWARE MANIPULATION METHODS ####################
#########################################################################

################    Setup Methods #############3

#sets the register mode to real time
#configures input data format for this as well
def setup_piezo(drv):
    drv._write_u8(_DRV2605_REG_MODE, 0x05)  #activate real time playback
        #mask to set only 3rd bit
    mask = 0b00001000
    data_format = drv._read_u8(_DRV2605_DATA_FORMAT_RTP) & ~mask
    drv._write_u8(_DRV2605_DATA_FORMAT_RTP, data_format)

#basic setup for push button
def setup_pushbutton():
    IO.setup(push_button_pin, IO.IN, pull_up_down=IO.PUD_DOWN)

#do motor setup stuff
def setup_motor():
    global p_L
    global p_R
    IO.setup(m_pin_L,IO.OUT)
    IO.setup(m_pin_R,IO.OUT)
    p_L = IO.PWM(m_pin_L,pwm_freq)
    p_R = IO.PWM(m_pin_R,pwm_freq)
    p_L.start(0)
    p_R.start(0)


def cleanup():
    IO.cleanup()
    idle_piezo(drv_l)
    idle_piezo(drv_r)
    if p_L is not None and p_R is not None:
        cf.update_motors(0, p_L, p_R)

########  controlling the devices #########

#read raw value of the pot and translate into radians
def read_angle():
    #print(chan.value)
    raw = chan.value % full_rotation_adc
    return ((raw / full_rotation_adc) * 360) - 180

#sets vibration level based on input value
#clips input to range between 0 and 1.
def set_vibration(drv, strength, deadband_min=0):
    strength = max(0, min(1.0, strength))
    strength = deadband_min + (1 - deadband_min)*strength
    strength = int(strength * 127)
    drv._write_u8(_DRV2605_REG_RTPIN, strength)

#set motor speed based on error. maps error
#range to the range of motor min to motor max.
#directly uses error angle (-180 to 180), since
#update_motors handles actually setting the motor values
def set_ground_motor(error):
    strength = 0
    if abs(error) > deadband_degrees: #only update outside the deadband
        str_range = motor_max - motor_min
        #if error is negative, make the offset negative
        m_min_flipped = motor_min
        if error < 0:
            m_min_flipped = -motor_min
        error_clipped = np.clip((error / degree_clip), -1.0, 1.0)
        strength = (error_clipped * str_range) + m_min_flipped
    cf.update_motors(strength, p_L, p_R)

#set vibration level to 0
def stop_piezo(drv):
    drv._write_u8(_DRV2605_REG_RTPIN, 0x00)

#When not being used, call this to idle the device.
#Use stop when you want higher-speed start/stop behavior.
def idle_piezo(drv):
    drv._write_u8(_DRV2605_REG_MODE, 0x00)

#pulse piezo for 0.1s
def pulse_piezo(drv):
    set_vibration(drv, 100)
    time.sleep(0.1)
    stop_piezo(drv)

def pulse_motor():
    cf.update_motors(50, p_L, p_R)
    time.sleep(0.1)
    cf.update_motors(0, p_L, p_R)

#when called, set waiting to false
def wait_callback(channel):
    global waiting
    print("button pushed.")
    waiting = False
    #prevent button bounce - wait for button to finish before starting
    #time.sleep(0.1)

#simple blocking code to just wait for a button press
def wait_for_press():
    print("Press button to proceed.")
    global waiting
    waiting = True
    IO.add_event_detect(push_button_pin, IO.FALLING, callback=wait_callback)
    while waiting:
        pass
    IO.remove_event_detect(push_button_pin)


#################   basic data methods ########################33

#choose an randomized series of angles between -90 and 90,
#in 10 degree intervals
def sample_goals():
    goal_range = list(range(-90, 100, 10))
    goal_range.remove(0)
    random.shuffle(goal_range)
    return goal_range

#save df as a csv. naming convention is hard-coded currently
def save_file(df, mode, mode_num, subject):
    filename = "mode " + str(mode) + " " + str(mode_num) + " trial " + subject[1:] + str(datetime.now())[7:13].replace(":", " ")
    filename = filename.replace(" ", "_")
    curr_path = os.getcwd()
    subj_folder = curr_path + "/haptic_data/" + subject
    try:
        os.mkdir(subj_folder)
    except:
        print("Subject folder already exists")
    #curr_path = os.path.split(os.path.abspath(__file__))[0]
    df.to_csv(subj_folder + "/" + filename + ".csv")

#convert mode number to mode names
def mode_str(mode):
    if mode == 0:
        return "vibration"
    elif mode == 1:
        return "motor"
    elif mode == 2:
        return "audio"

#used for button presses in mode select stage.
#Rising edge: just start counting the time
#Falling edge: quit if long time elapsed since rising edge,
# else update the mode
def change_mode(channel):
    global start_button_held
    #falling
    if IO.input(push_button_pin) == IO.LOW:
        start_button_held = datetime.now()
    #rising edge
    else:
        global end_mode_select
        time_elapsed = datetime.now() - start_button_held
        if time_elapsed.total_seconds() > button_hold_thres:
            end_mode_select = True
        else:
            global mode
            mode = (mode + 1) % 3
            print("Mode: " + mode_str(mode))

#select controls output, setup devices
def set_devices():
    global mode
    if mode == 0:
        print("Vibration mode.")
        setup_piezo(drv_r)
        setup_piezo(drv_l)
        pulse_piezo(drv_r)
        pulse_piezo(drv_l)
    elif mode == 1:
        print("Ground motor mode.")
        setup_motor()
        pulse_motor()
    elif mode == 2:
        print("Audio mode.")
        p = vlc.MediaPlayer(audio_mode_file)
        p.play()


############ TIMER CLASS DEFINITON ###########################
# initialize with an interval in millis
# check returns true if timer expired, and resets, so you can
# run it in a loop. Does not interrupt; if you don't check,
# it won't update anything.
class IntervalTimer:
    #specify the interval in SECONDS. it has up to millisecond resolution.
    def __init__(self, interval):
        self.interval = interval
        self.start_time = datetime.now()

    def check(self):
        transpired = (datetime.now() - self.start_time).total_seconds()
        if transpired >= self.interval:
            self.start_time = datetime.now()
            return True
        else:
            return False
##############################################################


#Cycle through modes and print mode, and give feedback through
#haptic/motor/audio, whichever selected. Press  + hold to exit.
def select_mode():
    print("Select device feedback mode. Push to change, push and hold to select.")
    global mode
    print("Mode: " + mode_str(mode))
    IO.add_event_detect(push_button_pin, IO.BOTH, callback=change_mode)
    while not end_mode_select:
        pass
    IO.remove_event_detect(push_button_pin)

#play audio telling user to go to the goal
def play_audio(goal):
    fname = audio_folder + str(goal) + ".mp3"
    p = vlc.MediaPlayer(fname)
    p.play()
    return p

#computes the angle and applies steering controls.
#currently hard coded to steer on the vibration motors but
#will change this to depend on the mode.
def steer(curr, goal, integral=0.0):
    global mode
    #comment out the integral addition if not wanted
    error = goal - curr
    error_w_integral = error# + integral
    if mode == 0:
        strength = error_w_integral / 180.0
        if abs(error) < deadband_degrees:
            stop_piezo(drv_r)
            stop_piezo(drv_l)
            return error
        elif error_w_integral < 0:
            set_vibration(drv_l, -strength, deadband_min_l)
            stop_piezo(drv_r)
        else:
            set_vibration(drv_r, strength, deadband_min_r)
            stop_piezo(drv_l)
    elif mode == 1:
        set_ground_motor(error_w_integral)
    return error

#control user to goal until they press button, or until 15 seconds finishes
#set timer_exit to false if you want to force wait for button press
def controls(goal, timer_exit=True):
    print("Start controls.")
    global waiting
    waiting = True
    path = []
    integral = 0.0 #integral term if we want to include

    #set timers and push button
    IO.add_event_detect(push_button_pin, IO.FALLING, callback=wait_callback)
    write_timer = IntervalTimer(write_data_interval)
    end_timer = IntervalTimer(steering_interval)

    #in audio mode, add audio output here.
    player = None
    if mode == 2:
        player = play_audio(goal)
        #time.sleep(0.01)
        #while player.get_state() == vlc.State.Playing:
        #    pass
        time.sleep(2.1)
    else:
        time.sleep(0.5)

    # for this try and see if the error starts out correctly
    start_offset = read_angle()
    cnt = 0
    while True:
        #read the current position
        curr = read_angle()
        if goal == 0:
            temp_goal = goal
        else:
            temp_goal = goal + start_offset
        error = steer(curr, temp_goal, integral) # add start_offset to goal?
        integral += error * integral_constant
        if write_timer.check():
            cnt += 1
            path.append(curr)
            if goal == 0 and cnt%50==0:
                print("Reset: " + str(np.round(curr,1)))
            elif cnt%50==0:
                print(np.round(curr,1), np.round(error,1))
            else:
                pass
        #use AND so end-timer only ends loop  if timer_exit is enabled
        if waiting == False or (end_timer.check() and timer_exit):
            break

    if player is not None:
        player.stop()
        #player gets stuck if try to init after stopping.
        #interrupt will delay the next instruction, so use sleep.
        time.sleep(0.5)
    else:
        time.sleep(0.5)
    IO.remove_event_detect(push_button_pin)
    print("End controls.")
    return path

def default_to_vibration():
    global mode
    mode = 0

#main
def main(mode_input=0, mode_num = 1, subject = 'SXX'):
    global mode
    mode = mode_input

    cols = ['goal', 'stream']
    trials = pd.DataFrame(columns=cols)

    setup_pushbutton()
    #select_mode()
    set_devices()
    wait_for_press()
    #now iterate through list of goals and control each one.
    for goal in sample_goals(): #[40, -30]: #
        controls(0, timer_exit=False)
        print()
        print()
        print("Goal: " + str(goal))
        path = controls(goal)
        new_row = pd.Series([goal, path], index=cols)
        trials = trials.append(new_row, ignore_index=True)
    print(trials)
    save_file(trials, mode, mode_num, subject)

    #cleanup stuff
    cleanup()


########################## TEST AND OTHER METHODS ####################

def test_timer():
    timer1 = IntervalTimer(1.5)
    count = 0
    while True:
        if timer1.check():
            print("Go!")
            count += 1
        if count >= 3:
            return True

def test_push_button():
    setup_pushbutton()
    message = input("Press enter to quit\n\n")
    IO.cleanup()

def test_piezos():
    setup_piezo(drv_l)
    setup_piezo(drv_r)
    strength = 0.15
    while strength <= 0.2:
        print(strength)
        set_vibration(drv_l, strength, deadband_min=0)
        set_vibration(drv_r, strength, deadband_min=0)
        time.sleep(1)
        strength += 0.01
    idle_piezo(drv_l)
    idle_piezo(drv_r)

def test_pot():
    for i in range(10000):
            time.sleep(0.01)
            print(chan.value)
    IO.cleanup()

def test_pot_angle():
    for i in range(0, 1000):
        curr = read_angle()
        print(curr)
        time.sleep(0.1)

def test_audio():
    for goal in [-90, 80, 0, 60]:
        play_audio(goal)
        time.sleep(5)

#if __name__ == "__main__":
    #try:
mode = int(sys.argv[1])
mode_num = int(sys.argv[2])
subj = str(sys.argv[3])
main(mode, mode_num, subj)
    #except Exception as e:
    #    print(e)
    #    cleanup()
### TODO add a function to see if actually closer to further point than current waypoint and skip to that one.


import time
import board
import busio
import adafruit_gps
import serial
import signal
import numpy as np
import board
import busio
import adafruit_fxos8700
import adafruit_fxas21002c
import RPi.GPIO as IO

# stops the motors  called when ctrl-c is hit
def exit_function(signal, frame):
    print('Exiting...')
    p_L.stop()
    p_R.stop()
    # lidar.stop()
    # lidar.stop_motor()
    time.sleep(2.0)
    # lidar.disconnect()
    time.sleep(2.0)
    sys.exit(0)
# intialize exit function
signal.signal(signal.SIGINT, exit_function)

uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=10)
gps = adafruit_gps.GPS(uart, debug=False)     # Use UART/pyserial
# Turn on the basic GGA and RMC info (what you typically want)
gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
gps.send_command(b'PMTK220,1000') # Set update rate to once a second.
# Main loop runs forever printing the location, etc. every second.
last_print = time.monotonic()

# initialize the gps settings
d2r = 3.14159/180.0 # degrees to radians
r2d = 1.0/d2r # radians to degrees
initialized = False
main_rate = 1.0 # rate in hz
dec_angle = 0.0 # declination angle https://www.pveducation.org/pvcdrom/properties-of-sunlight/declination-angle
waypoint_dist_tolerance = 5 # tolerance in meter to waypoint; once within this tolerance, we'll advance to the nect waypoint
heading_tolerance = 15 # tolerance +/- (in degrees) within which we don't attempt to turn to intercept targetHeading
# waypoints lat/long in degree decimal
waypoint_list = [[37.426995365893596,-122.17254054883],[37.42721666379759,-122.1724720452093],[37.42757960628986, -122.17234587409479]]
number_waypoints = len(waypoint_list)
waypnt_cnt = 0
cur_waypoint = waypoint_list[waypnt_cnt]
reached_last_waypoint = False
# GPS heading is 0 deg at N, with  0 --> 360 going clockwise from 12 to 12

target_heading = 0.0
current_heading = 0.0
head_vec = np.zeros(5)
deadband = 6 # within +/- degrees for compass to turn off

# setup the imu settings
i2c = busio.I2C(board.SCL, board.SDA)
fxos = adafruit_fxos8700.FXOS8700(i2c)
time.sleep(0.5)
offset_vec = np.array([-31.85, -9.2, 534.15])
scale_vec = np.array([0.9871977, 0.97198879, 1.04360902])
#fxas = adafruit_fxas21002c.FXAS21002C(i2c)

# setup the motor specs
IO.setwarnings(False)
IO.setmode (IO.BCM)
m_pin_R = 12
m_pin_L = 13
pwm_freq = 100
IO.setup(m_pin_L,IO.OUT)
IO.setup(m_pin_R,IO.OUT)
p_L = IO.PWM(m_pin_L,pwm_freq)
p_R = IO.PWM(m_pin_R,pwm_freq)
p_L.start(0)
p_R.start(0)

######## FUNCTIONS
def distanceToWaypoint(cur_point, cur_waypoint):
    currentLat = cur_point[0]
    currentLong = cur_point[1]
    targetLat = cur_waypoint[0]
    targetLong = cur_waypoint[1]
    delta = d2r*(currentLong - targetLong)
    sdlong = np.sin(delta)
    cdlong = np.cos(delta)
    lat1 = d2r*(currentLat)
    lat2 = d2r*(targetLat)
    slat1 = np.sin(lat1)
    clat1 = np.cos(lat1)
    slat2 = np.sin(lat2)
    clat2 = np.cos(lat2)
    delta = (clat1 * slat2) - (slat1 * clat2 * cdlong)
    delta = delta**2
    delta += (clat2 * sdlong)**2
    delta = delta**0.5
    denom = (slat1 * slat2) + (clat1 * clat2 * cdlong)
    delta = np.arctan2(delta, denom)
    distanceToTarget =  delta * 6372795
    return distanceToTarget

# get course in degrees (North = 0, West = 270) from current pos to targetLat
# because earth is not a sphere may be off slightly
def courseToWaypoint(cur_point, cur_waypoint):
    currentLat = cur_point[0]
    currentLong = cur_point[1]
    targetLat = cur_waypoint[0]
    targetLong = cur_waypoint[1]
    dlon = d2r*(targetLong - currentLong)
    cLat = d2r*(currentLat)
    tLat = d2r*(targetLat)
    a1 = np.sin(dlon) * np.cos(tLat)
    a2 = np.sin(cLat) * np.cos(tLat) * np.cos(dlon)
    a2 = np.cos(cLat) * np.sin(tLat) - a2
    a2 = np.arctan2(a1, a2)
    if (a2 < 0.0):
        a2 += 2*3.14159 # add 2 pi to make it positive
    targetHeading = r2d*(a2)
    return targetHeading

# return the heading of the person in degrees
def readCompass(dec_angle, offset_vec, scale_vec):
    mag_input = np.array([fxos.magnetometer[0],fxos.magnetometer[1],fxos.magnetometer[2]])
    mag_corr = (mag_input-offset_vec)*scale_vec
    heading = r2d*np.arctan2(-mag_corr[2],  mag_corr[1]) # need to negative sign to go 0-->360 CW
    heading += dec_angle
    if heading < 0.0:
        heading += 360.0
    if heading > 360.0:
        heading -= 360.0
    return int(heading)

# turn the person until they are within the threshold
def calcDesiredTurn(c_head, t_head, deadband):
    head_err = t_head - c_head
    if abs(head_err) < deadband or abs(head_err) > 360.0-deadband: # close enough, turn off motor
        m_cmnd = 0
    else:
        if abs(head_err) < 180:
            if head_err > 0:
                m_cmnd = 70 # turn right
            else:
                m_cmnd = -70
        else:
            if head_err > 180: # target is 360, cur is 10 --> turn left
                m_cmnd = -70
            else:
                m_cmnd = 70
    return m_cmnd

def update_motors(motor_command, p_L, p_R):
    if motor_command > 0: # turn right
        p_L.ChangeDutyCycle(0)
        p_R.ChangeDutyCycle(motor_command)
    elif motor_command < 0:
        p_R.ChangeDutyCycle(0)
        p_L.ChangeDutyCycle(-motor_command)
    else:
        p_L.ChangeDutyCycle(0)
        p_R.ChangeDutyCycle(0)

def initialization_print(): # print once to get info
    print('Fix timestamp: {}/{}/{} {:02}:{:02}:{:02}'.format(
        gps.timestamp_utc.tm_mon,   # Grab parts of the time from the
        gps.timestamp_utc.tm_mday,  # struct_time object that holds
        gps.timestamp_utc.tm_year,  # the fix time.  Note you might
        gps.timestamp_utc.tm_hour,  # not get all data like year, day,
        gps.timestamp_utc.tm_min,   # month!
        gps.timestamp_utc.tm_sec))
    print('Latitude: {0:.6f} degrees'.format(gps.latitude))
    print('Longitude: {0:.6f} degrees'.format(gps.longitude))
    print('Fix quality: {}'.format(gps.fix_quality))
    if gps.satellites is not None:
        print('# satellites: {}'.format(gps.satellites))
    if gps.altitude_m is not None:
        print('Altitude: {} meters'.format(gps.altitude_m))
    if gps.speed_knots is not None:
        print('Speed: {} knots'.format(gps.speed_knots))
    if gps.track_angle_deg is not None:
        print('Track angle: {} degrees'.format(gps.track_angle_deg))
    if gps.horizontal_dilution is not None:
        print('Horizontal dilution: {}'.format(gps.horizontal_dilution))
    if gps.height_geoid is not None:
        print('Height geo ID: {} meters'.format(gps.height_geoid))

#####################


while not reached_last_waypoint:
    gps.update() # call fast

    # update the imu heading
    current_heading = readCompass(dec_angle, offset_vec, scale_vec) # compute current heading
    head_vec[:-1] = head_vec[1:]
    head_vec[-1] = current_heading
    # update the motor commands frequently based on heading
    motor_command = calcDesiredTurn(np.mean(head_vec), target_heading, deadband)
    update_motors(motor_command, p_L, p_R)
    # run gps waypoint updates at slower rate
    current = time.monotonic() # update timer to check rate
    if current - last_print >= main_rate:
        last_print = current
        if not initialized: # store the initializing info
            if not gps.has_fix:
                print('Waiting for fix...')
                continue
            initialization_print() # print starting info
            day_cnt = (gps.timestamp_utc.tm_mon - 1)*30 +  gps.timestamp_utc.tm_mday
            dec_angle = -23.45*np.cos(2*3.14159/365*(day_cnt+10))
            print("Day cnt: ", day_cnt, "  Declination angle (deg): ", dec_angle)
            initialized = True
        else: # main loop to run the code
            if not gps.has_fix:
                print('Lost fix...')
                continue
            # update distance
            cur_pt = [gps.latitude, gps.longitude]
            dist_to_target = distanceToWaypoint(cur_pt, cur_waypoint)
            # check to see if we have reached the current waypoint
            if (dist_to_target <= waypoint_dist_tolerance):
                waypnt_cnt += 1
                if waypnt_cnt == number_waypoints: # done
                    reached_last_waypoint = True # exit code
                    print("Reached last waypoint!")
                    break
                cur_waypoint = waypoint_list[waypnt_cnt]
                dist_to_target = distanceToWaypoint(cur_pt, cur_waypoint)
            # update the course to the next waypoint
            target_heading = courseToWaypoint(cur_pt, cur_waypoint)

            # print status update
            print(fxos.magnetometer)
            print("Waypnt: ", waypnt_cnt, " Distance (m): ", dist_to_target, " Target heading: ", target_heading, " Cur heading: ", current_heading)

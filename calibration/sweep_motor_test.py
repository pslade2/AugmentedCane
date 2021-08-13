import RPi.GPIO as IO
import time
import numpy as np

IO.setwarnings(False)
IO.setmode (IO.BCM)         #we are programming the GPIO by BCM pin numbers. (PIN35 as ‘GPIO19’)
m_pin_R = 12
m_pin_L = 13
pwm_freq = 100

IO.setup(m_pin_L,IO.OUT)           # initialize GPIO19 as an output.
p_L = IO.PWM(m_pin_L,pwm_freq)          #GPIO19 as PWM output, with 100Hz frequency
p_L.start(0)                              #generate PWM signal with 0% duty cycle
IO.setup(m_pin_R,IO.OUT)           # initialize GPIO19 as an output.
p_R = IO.PWM(m_pin_R,pwm_freq)          #GPIO19 as PWM output, with 100Hz frequency
p_R.start(0)                              #generate PWM signal with 0% duty cycle

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

# pass in offset (minimum control val), change in motor speed from offset (duty cycle %), and period (s) of the sine oscillation for the motor
def sweep_sine(m_avg, m_amp, bias, m_per, p_L, p_R):
    cur_time = time.time()
    sine_m_cmnd = m_amp*np.sin(2*3.14159*cur_time/m_per)
    sine_m_cmnd += np.sign(sine_m_cmnd)*m_avg # add offset to get rid of deadband
    update_motors(sine_m_cmnd, p_L, p_R)

m_avg = 25
m_amp = 25
m_per = 1.8
m_bias = 0.0

for i in range(1500):
    sweep_sine(m_avg, m_amp, m_bias, m_per, p_L, p_R)
    time.sleep(0.1)

p_L.stop()
p_R.stop()

import RPi.GPIO as IO
import time
IO.setwarnings(False)
IO.setmode (IO.BCM)         #we are programming the GPIO by BCM pin numbers. (PIN35 as ‘GPIO19’)
m_pin_R = 12
m_pin_L = 13
pwm_freq = 100

IO.setup(m_pin_L,IO.OUT)           # initialize GPIO19 as an output.
p_L = IO.PWM(m_pin_L,pwm_freq)          #GPIO19 as PWM output, with 100Hz frequency
p_L.start(0)                              #generate PWM signal with 0% duty cycle

for i in range(100):
    p_L.ChangeDutyCycle(i)
    print(i)
    time.sleep(0.2)

p_L.stop()

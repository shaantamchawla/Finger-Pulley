from machine import ADC, Pin, PWM
import time

button = Pin(16, Pin.IN, Pin.PULL_UP) ## setup pull-up to enable interrupt request on push button. This allows an override to disengage pawl during flexion.
fsr = machine.ADC(26) ## FSR variable resistance sensor - ADC0 on pico
emg = machine.ADC(27) ## EMG sensor - ADC1 on pico
stepper = machine.PWM(Pin(0)) # Motor setup
stepper.freq(50) ## 50 Hz cycles — 1 cycle = 20 ms

window_length = 10 ## Moving average filter for processing force signal
force_threshold = 23000 ## Needs to be tuned for EMG sensing

ENGAGED = 0
DISENGAGED = 1

DISENGAGE_TIME = 3.0 ## Needs to be tuned. Value = how long pawl disengages during flexion, before re-engaging.

state = None
button_press = False

# motor datasheet: http://www.ee.ic.ac.uk/pcheung/teaching/DE1_EE/stores/sg90_datasheet.pdf
# pico pinout: https://www.raspberrypi-spy.co.uk/wp-content/uploads/2021/01/raspberry_pi_pico_pinout.png
# fsr datasheet: https://learn.sparkfun.com/tutorials/force-sensitive-resistor-hookup-guide?_ga=2.97262495.1479833929.1651187327-1561052132.1648762346

## Class definition for a streaming moving average filter - should play around with diff window sizes & see what works best
class MovingAverage:
    def __init__(self, window_length):
        self.window_length = window_length
        self.total = 0
        self.values = []

    def process(self, value):
        self.values.append(value)
        self.total += value
        
        if len(self.values) > self.window_length:
            self.total -= self.values.pop(0)
        
        return float(self.total) / float(len(self.values))

## Offline debugging function to ensure Pico is alive
def blink_led():
    led = Pin(25, Pin.OUT)
    led.toggle()

## Takes in an initialized MovingAverage object, produces next value in the filtered signal
def read_fsr(moving_average):
    fsr_raw = fsr.read_u16() # Read raw signal
    filtered = moving_average.process(fsr_raw) # Apply moving average to filter
    time.sleep(0.05)
    print(filtered)
    
    return filtered

## Takes in an initialized MovingAverage object, produces next value in the filtered signal
def read_emg(moving_average):
    emg_raw = emg.read_u16() # Read raw signal
    filtered = moving_average.process(emg_raw) # Apply moving average to filter
    time.sleep(0.05)
    print(filtered)
    
    return filtered

## Gets called when we have an interrupt request on the push button i.e. it is pressed
def callback(irq):
    global button_press
    
    print("pressed")
    blink_led()
    button_press = True
    motor_disengage_state()
        
def drive_motor(degrees): ## takes in desired degree setpoint, outputs, 16-bit 'write' duty command to motor
    MIN_DUTY = 1800 # 0 deg
    MAX_DUTY = 7200 # 180 deg
    duty_cycle_1_deg = (MAX_DUTY - MIN_DUTY) / 180
    
    duty_cycle = int(duty_cycle_1_deg * degrees) + MIN_DUTY
    
    if (duty_cycle > MAX_DUTY):
        duty_cycle = MAX_DUTY
        
    if (duty_cycle < MIN_DUTY):
        duty_cycle = MIN_DUTY
    
    stepper.duty_u16(duty_cycle)
    
def motor_disengage_state(): ## move pawl out of the way of ratchet gear for *DISENGAGE_TIME* seconds, to allow cable to re-wind
    drive_motor(152)
    time.sleep(DISENGAGE_TIME)
    motor_engage_state()
    
def motor_engage_state(): ## keep pawl in place as ratchet turns & cable unwinds
    drive_motor(118)
    #time.sleep(2.5)
    
def motor_debug(): ## Debug function to move pawl back & forth with motor
    state = str(input("Enter a state: "))
    
    if state == 'e':
        motor_engage_state()
        
    elif state == 'd':
        motor_disengage_state()
    
def bootup(): ## to know Pico is alive
    for i in range(10):
        blink_led()
        time.sleep(0.2)
    
## Need to revise for final demo
def main():
    bootup()
    ## Default state for pawl is engaged
    state = ENGAGED
    global button_press
    motor_engage_state()
    
    button.irq(trigger=Pin.IRQ_FALLING, handler=callback) ## button interrupt handler — if button is ever pressed, disengage immediately
    FSR_moving_average = MovingAverage(window_length)
    EMG_moving_average = MovingAverage(window_length)
    
    while True:
        if button_press == True:
            EMG_moving_average = MovingAverage(window_length)
            button_press = False
            
        value = read_emg(EMG_moving_average)
        #value = read_fsr(FSR_moving_average)
        if value > force_threshold:
            if state == ENGAGED:
                state = DISENGAGED
                print('DISENGAGE')
                motor_disengage_state()
                EMG_moving_average = MovingAverage(window_length)
            
            else: ## return disengaged to engaged
                state = ENGAGED  ## already open, don't act on high signal
                time.sleep(5)
                EMG_moving_average = MovingAverage(window_length)
        
        else:
            motor_engage_state()
        
        #print(value)
        time.sleep(0.01)
    
main()
import time
from dataclasses import dataclass

import sys
sys.path.append("/home/pi/.local/lib/python3.7/site-packages")

import wiringpi
import vcgencmd

IDLE = 0
SILENT = 256
QUIET = 512
MODERATE = 768
TURBO = 1024

IDLE_TEMP_MAX = 52
SILENT_TEMP_MAX = 60
QUIET_TEMP_MAX = 64
MODERATE_TEMP_MAX = 67

INHALE_PIN = 12
EXHALE_PIN = 13

wiringpi.wiringPiSetupGpio()
wiringpi.pinMode(INHALE_PIN, wiringpi.PWM_OUTPUT)
wiringpi.pinMode(EXHALE_PIN, wiringpi.PWM_OUTPUT)

# How long to sleep for between polling of state.
POLL_INTERVAL = 6

# The lowest (unboosted) clocks the Pi can be.
BASE_ARM_CLOCK = 600
UNINTERESTING_ARM_CEILING = 700
BASE_CORE_CLOCK = 200
UNINTERESTING_CORE_CEILING = 250

# How much higher the clock can be from the base and
# still be considered unboosted.
CLOCK_TOLERANCE = 100

DATA_HISTORY = 60 / POLL_INTERVAL

@dataclass
class PollData:
    temp: float
    boosted: bool
    historically_boosted_max_temp: float
    poll_edge_trigger: bool
    set_pin: int

vcgm = vcgencmd.Vcgencmd()

time.sleep(5)

def is_boosted():
    arm_clock = vcgm.measure_clock("arm") / 1000000
    core_clock = vcgm.measure_clock("core") / 1000000

    if arm_clock > UNINTERESTING_ARM_CEILING + CLOCK_TOLERANCE:
        return True
    elif core_clock > UNINTERESTING_CORE_CEILING + CLOCK_TOLERANCE:
        return True

    return False

def historically_boosted(data_history):
    for datum in data_history:
        if datum.boosted:
            return True

    return False

def historical_temp(data_history):
    was_boosted = historically_boosted(data_history)

    max_temp = 0

    for datum in data_history:
        if datum.temp > max_temp:
            max_temp = datum.temp
        if was_boosted and datum.historically_boosted_max_temp > max_temp:
            max_temp = datum.historically_boosted_max_temp

    return max_temp

def historical_poll_edge_trigger(data_history):
    for datum in data_history:
        if datum.poll_edge_trigger:
            return True

    return False

datapoints = []

counter = 1
while True:
    cur_boosted = is_boosted()
    cur_temp = vcgm.measure_temp()

    counter += 1
    if counter % int(60 / POLL_INTERVAL) == 0:
        counter = 1

        # If it seems that we have tripped a throttling bit, just
        # activate max fans and exit. This shouldn't happen, so
        # do what we can to protect the chip and stop being smart.
        throttle_data = int(vcgm.get_throttled()['raw_data'], 16)
        if throttle_data != 0:
            wiringpi.pwmWrite(INHALE_PIN, TURBO)
            wiringpi.pwmWrite(EXHALE_PIN, TURBO)

            exit(1)

    poll_sleep = POLL_INTERVAL

    poll_edge_trigger = False
    if len(datapoints) > 0 and cur_temp - datapoints[-1].temp >= 3:
        poll_edge_trigger = True
    
    datapoints.append(PollData(temp=cur_temp, boosted=cur_boosted, historically_boosted_max_temp=0, poll_edge_trigger=poll_edge_trigger, set_pin=-1))

    data_history_lim = DATA_HISTORY

    if historical_poll_edge_trigger(datapoints):
        poll_sleep = 1
        data_history_lim = DATA_HISTORY * POLL_INTERVAL

    if len(datapoints) > data_history_lim:
        del datapoints[:int(-1 * data_history_lim)]

    highest_historical_temp = historical_temp(datapoints)

    was_boosted = historically_boosted(datapoints)

    if was_boosted:
        datapoints[-1].historically_boosted_max_temp = highest_historical_temp

    if highest_historical_temp < IDLE_TEMP_MAX:
        if (len(datapoints) > 1 and datapoints[-2].set_pin != IDLE) or len(datapoints) < 2:
            wiringpi.pwmWrite(INHALE_PIN, IDLE)
            wiringpi.pwmWrite(EXHALE_PIN, IDLE)
            datapoints[-1].set_pin = IDLE
        else:
            datapoints[-1].set_pin = IDLE
    elif highest_historical_temp < SILENT_TEMP_MAX:
        if (len(datapoints) > 1 and datapoints[-2].set_pin != SILENT) or len(datapoints) < 2:
            wiringpi.pwmWrite(INHALE_PIN, SILENT)
            wiringpi.pwmWrite(EXHALE_PIN, SILENT)
            datapoints[-1].set_pin = SILENT
        else:
            datapoints[-1].set_pin = SILENT
    elif highest_historical_temp < QUIET_TEMP_MAX:
        if (len(datapoints) > 1 and datapoints[-2].set_pin != QUIET) or len(datapoints) < 2:
            wiringpi.pwmWrite(INHALE_PIN, QUIET)
            wiringpi.pwmWrite(EXHALE_PIN, QUIET)
            datapoints[-1].set_pin = QUIET
        else:
            datapoints[-1].set_pin = QUIET
    elif highest_historical_temp < MODERATE_TEMP_MAX:
        if (len(datapoints) > 1 and datapoints[-2].set_pin != MODERATE) or len(datapoints) < 2:
            wiringpi.pwmWrite(INHALE_PIN, MODERATE)
            wiringpi.pwmWrite(EXHALE_PIN, MODERATE)
            datapoints[-1].set_pin = MODERATE
        else:
            datapoints[-1].set_pin = MODERATE
    else:
        if (len(datapoints) > 1 and datapoints[-2].set_pin != TURBO) or len(datapoints) < 2:
            wiringpi.pwmWrite(INHALE_PIN, TURBO)
            wiringpi.pwmWrite(EXHALE_PIN, TURBO)
            datapoints[-1].set_pin = TURBO
        else:
            datapoints[-1].set_pin = TURBO

    time.sleep(poll_sleep)

exit(0)


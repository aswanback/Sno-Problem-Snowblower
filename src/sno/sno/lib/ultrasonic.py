from sno.lib.arduino import digitalRead, digitalWrite, pinMode, getPin, micros, delay, delayMicroseconds, IOType, IOValue
# from arduino import pinMode, digitalWrite, getPin, micros, delayMicroseconds

class UltrasonicSensor:
    NO_ECHO = 0
    PING_OVERHEAD = 5 # ping overhead time in uS
    US_ROUNDTRIP_CM = 57
    US_ROUNDTRIP_IN = 146
    TRIGGER_WIDTH = 12
    NO_ECHO = 0
    MAX_SENSOR_DELAY = 5800
    MAX_SENSOR_DISTANCE = 500 
    ECHO_TIMER_FREQ = 24
    PING_MEDIAN_DELAY = 30000

    def __init__(self, trigger_pin: int, echo_pin: int, max_cm_distance: int) -> None:
        self._triggerPin = getPin(trigger_pin)
        self._echoPin = getPin(echo_pin)
        self._one_pin_mode = (trigger_pin == echo_pin)  # Automatic one pin mode detection per sensor.
        self.set_max_distance(max_cm_distance)  # Call function to set the max sensor distance.
        pinMode(self._triggerPin, IOType.OUTPUT)  # Set trigger pin to output.
        pinMode(self._echoPin, IOType.INPUT)  # Set echo pin to input.
        digitalWrite(self._triggerPin, IOValue.LOW.value)  # Trigger pin should already be low, but set to low to make sure.

    def ping(self, max_cm_distance:int = -1) -> int:
        if max_cm_distance > 0:
            self.set_max_distance(max_cm_distance)  # Call function to set a new max sensor distance.
            
        if not self.ping_trigger():
            print('ping trigger 0')
            return self.NO_ECHO  # Trigger a ping, if it returns false, return self.NO_ECHO to the calling function.
        
        while digitalRead(self._echoPin):
            if micros() > self._max_time:
                return self.NO_ECHO  # Stop the loop and return self.NO_ECHO (false) if we're beyond the set maximum distance.
        
        # Calculate ping time, include overhead.
        return micros() - (self._max_time - self._maxEchoTime) - self.PING_OVERHEAD
    
    def ping_cm(self, max_cm_distance):
        echoTime = self.ping(max_cm_distance) # Calls the ping method and returns with the ping echo distance in uS.
        if not self.US_ROUNDTRIP_IN:
            return echoTime //self.US_ROUNDTRIP_CM # Call the ping method and returns the distance in centimeters (no rounding).
        else:
            return self.NewPingConvert(echoTime,self.US_ROUNDTRIP_CM) # Convert uS to centimeters.

    def ping_in(self,max_cm_distance):
        echoTime = self.ping(max_cm_distance) # Calls the ping method and returns with the ping echo distance in uS.
        if not self.US_ROUNDTRIP_IN:
            return echoTime //self.US_ROUNDTRIP_IN # Call the ping method and returns the distance in inches (no rounding).
        else:
            return self.NewPingConvert(echoTime,self.US_ROUNDTRIP_IN) # Convert uS to inches.
    
    def ping_median(self, it, max_cm_distance):
        uS = [self.NO_ECHO] * it
        last = 0
        j = 0
        i = 0
        t = 0
        if max_cm_distance > 0:
            self.set_max_distance(max_cm_distance)

        while i < it:
            t = micros()          # Start ping timestamp.
            last = self.ping()         # Send ping.

            if last != self.NO_ECHO: # Ping in range, include as part of median.
                if i > 0:       # Don't start sort till second ping.
                    for j in range(i, 0, -1): # Insertion sort loop.
                        if uS[j - 1] < last:
                            uS[j] = uS[j - 1]  # Shift ping array to correct position for sort insertion.
                        else:
                            break
                else:
                    j = 0  # First ping is sort starting point.
                    
                uS[j] = last  # Add last ping to array in sorted position.
                i += 1  # Move to next ping.
            else:
                it -= 1  # Ping out of range, skip and don't include as part of median.

            if i < it and micros() - t < self.PING_MEDIAN_DELAY:
                delay((self.PING_MEDIAN_DELAY + t - micros()) >> 10) # Millisecond delay between pings.
            
        return uS[it >> 1] # Return the ping distance median.
    def NewPingConvert(self, echoTime, conversionFactor):
        return max((echoTime + conversionFactor // 2) // conversionFactor, int(bool(echoTime)))

    def ping_trigger(self):
        if self._one_pin_mode:
            pinMode(self._triggerPin, IOType.OUTPUT)
        
        digitalWrite(self._triggerPin, IOValue.HIGH.value)
        delayMicroseconds(self.TRIGGER_WIDTH)
        digitalWrite(self._triggerPin, IOValue.LOW.value)
        
        if self._one_pin_mode:
            pinMode(self._triggerPin, IOType.INPUT)

        if digitalRead(self._echoPin):
            return False
        
        self._max_time = micros() + self._maxEchoTime + self.MAX_SENSOR_DELAY
        while not digitalRead(self._echoPin):
            if micros() > self._max_time:
                print('timeout')
                return False
                        
        self._max_time = micros() + self._maxEchoTime
        return True

    def set_max_distance(self, max_cm_distance):
        if self.US_ROUNDTRIP_IN == False:
            max_echo_time = min(max_cm_distance + 1, self.MAX_SENSOR_DISTANCE + 1) *self.US_ROUNDTRIP_CM
        else:
            max_echo_time = min(max_cm_distance, self.MAX_SENSOR_DISTANCE) *self.US_ROUNDTRIP_CM + (self.US_ROUNDTRIP_CM // 2)
        self._maxEchoTime = max_echo_time

def read_distance(trig, echo):
    digitalWrite(trig, True)
    time.sleep(0.00001)
    digitalWrite(trig, False)

    start_time = time.time_ns()
    stop_time = time.time_ns()
    timeout_ns = 300*1e6 # ms * 1e6 = ns
    timeout_start = time.time_ns()
    while digitalRead(echo) == 0:
        curr = time.time_ns()
        if curr > timeout_start + timeout_ns:
            print('timeout 1')
            break
        start_time = curr
    while digitalRead(echo) == 1:
        curr = time.time_ns()
        if curr > timeout_start + timeout_ns:
            print('timeout 2')
            break
        stop_time = curr
    
    time_elapsed = stop_time - start_time
    time_elapsed /= 1e6
    return (time_elapsed * 34300) / 2



if __name__ == '__main__':
    from config import *
    import time
    from arduino import pinMode, IOType, digitalWrite, getPin, micros, delayMicroseconds
    # us1 = UltrasonicSensor(US_TRIG_1, US_ECHO_1, 100)
    # # us2 = UltrasonicSensor(US_TRIG_2, US_ECHO_2, 40)
    # while True:
    #     print(us1.ping())
    #     delay(100)
    trig, echo = getPin(US_TRIG_1), getPin(US_ECHO_1)
    pinMode(trig, IOType.OUTPUT)
    pinMode(echo, IOType.INPUT)
    while True:
        distance = read_distance(trig, echo)
        print("Distance:", distance, "cm")
        time.sleep(1)

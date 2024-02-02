# RaspberryPi Coprocessors
Our robot uses two RaspberryPi Coprocessors for vision processing, each running 
[PhotonVision](https://photonvision.org/).

# LED Controller
One Pi is also running [NTLEDController.py](NTLEDController.py), which generates 
PWM pulses to control a strip of LED lights at the frequency commanded by the 
RoboRio through NetworkTables. This is done to allow the LEDs to be controlled 
while the robot is disabled. See instructions [here](https://robotpy.readthedocs.io/en/stable/install/pyntcore.html)
on installing pyntcore on the Pi.

# REV Color Sensor
The Pi on the arm is running two color sensors.
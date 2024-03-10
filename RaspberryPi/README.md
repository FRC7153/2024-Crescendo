# RaspberryPi Coprocessors
Our robot uses two RaspberryPi Coprocessors for vision processing, each running 
[PhotonVision](https://photonvision.org/).

## Installing Python3 and Dependencies
Our Pi's run Python programs in the background to read sensors and push the data to NetworkTables.
To install Python3 and pyntcore:
1. In the web configuration for PhotonVision, ensure 'Manage Device Networking' is ON and 'Ip Assignment
Mode' is set to 'Static'
2. SSH to raspberry pi (`ssh pi@hostname.local`) while the pi is connected to the internet
3. Install Python3: `sudo apt-get install python3`
4. Install pip3: `sudo apt-get install python3-pip`
5. Install NetworkTables: `sudo pip3 install pyntcore --break-system-packages` (may take a while)*
6. Install PiGPIO: `sudo pip3 install pigpio --break-system-packages`

To install I2C-related software:
1. Enable I2C: `sudo rasp-config` -> Interface Options -> I2C -> Yes
2. Install tools: `sudo apt-get install -y i2c-tools`
3. Reboot
4. Ensure device is connected: `i2cdetect -y 1`

*There may be some errors related to publishing arrays to NT on certain installs of pyntcore. Check
ChiefDelphi before if this occurs.

## Background Python3 Execution
(TODO)

## PrimaryPi.local
(TODO)

## SecondaryPi.local
The SecondaryPi is located on the robot's arm. It runs (TODO cameras here) and two REV Color Sensor V3, outputting their data to NetworkTables. This Pi is running [NTSecondarySensors.py](NTSecondarySensors.py) in the background
(TODO wiring diagram)
The configuration backup for this Pi can be found at [SecondaryPiConfig.zip](SecondaryPiConfig.zip)
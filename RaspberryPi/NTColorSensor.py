## Runs in the background, publishing the inputs from two REV Color Sensors

# NOTE params
kNOTE_MIN_HSV = [0.0, 0.79, 0.05]
kNOTE_MAX_HSV = [0.07, 1.0, 0.35]
kNOTE_MIN_PROX = 0.05

# Imports
from ntcore import NetworkTableInstance
import pigpio
import time
import colorsys

# Init Pi
pi = pigpio.pi()

# Color Sensor
class RevColorSensor:
    def __init__(self, port):
        """
        Constructs a new Rev Color Sensor on i2c port specified
        """
        self._i2c = pi.i2c_open(port, 0x52)

        if not self.checkDevice():
            print("Unknown device connected to i2c!")
            return
    
        self.initializeDevice()
        self.hasReset()

    def checkDevice(self) -> bool:
        """
        Self-checks that this is the correct sensor
        """
        id = pi.i2c_read_byte_data(self._i2c, 0x06) # Part ID register
        return (id == 0xC2)
    
    def initializeDevice(self):
        '''
        Initialize the sensor
        '''
        self._write8Bit(
            0x00, # Main control register
            0x04 | # RGB Mode
            0x02 | # Light sensor enable
            0x01 # Proximity sensor enable
        )

        self._write8Bit(
            0x03, # Proximity sensor rate register
            0x18 | # 11 bit proximity resolution
            0x05 # 100ms proximity rate
        )

        self._write8Bit(
            0x02, # Proximity Sensor Pulses Register
            32
        )

    def hasReset(self) -> bool:
        """
        Indicates if device has reset. This flag is self-clearing.
        """
        raw = pi.i2c_read_byte_data(
            self._i2c,
            0x07 # Main status register
        )
        return (raw & 0x20) != 0
    
    def getHSV(self) -> list[int]:
        """
        Returns the current value as a HSV
        """
        # Get raw colors
        # calibrated against white surface
        color = [
            min(self._read20Bit(0x13) / 15000, 1.0), # Red register
            min(self._read20Bit(0x0D) / 30000, 1.0), # Green register
            min(self._read20Bit(0x10) / 15000, 1.0)  # Blue register
        ]

        # Convert RGB -> HSV
        color = colorsys.rgb_to_hsv(color[0], color[1], color [2])

        return color

    def getProximity(self) -> float:
        """
        Returns the proximity
        """
        return self._read11Bit(0x08) / 2047 # Proximity data register

    # I2C functions
    def _read11Bit(self, reg) -> int:
        _, raw = pi.i2c_read_i2c_block_data(self._i2c, reg, 2)
        return ((raw[0] & 0xFF) | ((raw[1] & 0xFF) << 8)) & 0x7FF

    def _read20Bit(self, reg) -> int:
        _, raw = pi.i2c_read_i2c_block_data(self._i2c, reg, 3)
        return ((raw[0] & 0xFF) | ((raw[1] & 0xFF) << 8) | 
            ((raw[2] & 0xFF) << 16)) & 0x03FFFF

    def _write8Bit(self, reg, data):
        pi.i2c_write_byte_data(self._i2c, reg, data)

# Init color sensors
colorSensor = RevColorSensor(1)

# Init network tables
ntTableInst = NetworkTableInstance.create()
ntTableInst.setServerTeam(7153)
ntTableInst.startClient4("SecondaryPiSensors")
ntTable = ntTableInst.getTable("SecondaryPiSensors")

sensorHSVOut = ntTable.getDoubleArrayTopic("SensorHSV").publish()
sensorProxOut = ntTable.getDoubleTopic("SensorProx").publish()
sensorTargetOut = ntTable.getBooleanTopic("Target").publish()

# Check if sees note
def seesNote(hsv, prox):
    return (hsv[0] >= kNOTE_MIN_HSV[0] and  hsv[0] <= kNOTE_MAX_HSV[0] and
            hsv[1] >= kNOTE_MIN_HSV[1] and hsv[1] <= kNOTE_MAX_HSV[1] and
            hsv[2] >= kNOTE_MIN_HSV[2] and hsv[2] <= kNOTE_MAX_HSV[2] and
            prox >= kNOTE_MIN_PROX)

# Main loop
print("Running main loop... (v1.0.0)")

while True:
    # Left sensor
    hsv = colorSensor.getHSV()
    prox = colorSensor.getProximity()

    sensorTargetOut.set(seesNote(hsv, prox))
    sensorHSVOut.set(hsv)
    sensorProxOut.set(prox)

    # Pause
    time.sleep(0.05)
from micropython import const  # NOQA
import struct
import time
import imu_sensor_framework  # NOQA

_ONE_G = const(9.807)

_VERSION_REG = const(0x00)
_REVISION_REG = const(0x01)
_CTRL1_REG = const(0x02)  # serial interface (address auto-increment, endianness)
_ACCEL_SETTING_REG = const(0x03)  # CTRL2: accelerometer range + rate
_GYRO_SETTING_REG = const(0x04)  # CTRL3: gyroscope range + rate
_CTRL5_REG = const(0x06)  # accel / gyro low-pass filters
_CTRL6_REG = const(0x07)  # AttitudeEngine ODR / motion on demand
_ENABLE_REG = const(0x08)  # CTRL7: sensor enables

_TIME_REG = const(0x30)
_TEMP_REG = const(0x33)
_ACCEL_REG = const(0x35)
_GYRO_REG = const(0x3B)

_RESET_REG = const(0x60)
_RESET_CMD = const(0xB0)


def _encode_setting(rnge, rate):
    return ((rnge & 0x7) << 4) | (rate & 0xF)


def _int16(value):
    return value - 0x10000 if value & 0x8000 else value


ACCEL_RANGE_2 = const(0)  # +/- 2g
ACCEL_RANGE_4 = const(1)  # +/- 4g
ACCEL_RANGE_8 = const(2)  # +/- 8g (default value)
ACCEL_RANGE_16 = const(3)  # +/- 16g

GYRO_RANGE_16 = const(0)  # +/- 16 deg/s
GYRO_RANGE_32 = const(1)  # +/- 32 deg/s
GYRO_RANGE_64 = const(2)  # +/- 64 deg/s
GYRO_RANGE_128 = const(3)  # +/- 128 deg/s
GYRO_RANGE_256 = const(4)  # +/- 256 deg/s
GYRO_RANGE_512 = const(5)  # +/- 512 deg/s
GYRO_RANGE_1024 = const(6)  # +/- 1024 deg/s
GYRO_RANGE_2048 = const(7)  # +/- 2048 deg/s (default value)

# Rates are nominal. With accelerometer and gyroscope both enabled the chip
# derives them from the gyro drive frequency, so e.g. code 6 is ~112 Hz, not
# exactly 125 Hz. Codes 0-2 only exist in this 6DOF mode.
ACCEL_RATE_8000_HZ = const(0)
ACCEL_RATE_4000_HZ = const(1)
ACCEL_RATE_2000_HZ = const(2)
ACCEL_RATE_1000_HZ = const(3)
ACCEL_RATE_500_HZ = const(4)
ACCEL_RATE_250_HZ = const(5)
ACCEL_RATE_125_HZ = const(6)  # (default value)
ACCEL_RATE_62_HZ = const(7)
ACCEL_RATE_31_HZ = const(8)
ACCEL_RATE_LP_128_HZ = const(12)  # low-power modes need the gyroscope disabled
ACCEL_RATE_LP_21_HZ = const(13)
ACCEL_RATE_LP_11_HZ = const(14)
ACCEL_RATE_LP_3_HZ = const(15)

GYRO_RATE_8000_HZ = const(0)
GYRO_RATE_4000_HZ = const(1)
GYRO_RATE_2000_HZ = const(2)
GYRO_RATE_1000_HZ = const(3)
GYRO_RATE_500_HZ = const(4)
GYRO_RATE_250_HZ = const(5)
GYRO_RATE_125_HZ = const(6)  # (default value)
GYRO_RATE_62_HZ = const(7)
GYRO_RATE_31_HZ = const(8)

I2C_ADDR = 0x6B
BITS = 8


class QMI8658C(imu_sensor_framework.IMUSensorFramework):
    """QMI8658C 6-axis IMU (accelerometer + gyroscope, no magnetometer).

    roll/pitch/yaw come from 6DOF fusion. Yaw has no absolute reference, so it
    starts at 0 and drifts slowly. Call calibrate_gyro() once while the board
    is held still to remove the gyroscope bias.

    flip=True rotates the sensor frame 180 degrees about X (negates Y and Z)
    so a board lying flat with the chip's Z axis pointing down reads roll ~0.
    """

    ACCEL_ONE_G = _ONE_G

    def _read_into(self, reg, n):
        """Read n bytes starting at reg into self._rx_buf.

        lvgl_micropython's i2c.Device.read_mem has differed between versions:
        read_mem(reg, nbytes) -> bytes, or read_mem(reg, buf) filling buf.
        Try the first form, fall back to the second, and remember which works.
        """
        if self._read_style == 0:
            try:
                data = self._device.read_mem(reg, n)
                self._rx_buf[:n] = data
                return
            except TypeError:
                self._read_style = 1

        self._device.read_mem(reg, self._rx_mv[:n])

    def _read_reg(self, reg):
        self._read_into(reg, 1)
        return self._rx_buf[0]

    def _write_reg(self, reg, data):
        self._tx_buf[0] = data
        self._device.write_mem(reg, self._tx_buf)

    def __init__(self, device, accel_range=ACCEL_RANGE_8, accel_rate=ACCEL_RATE_125_HZ,
                 gyro_range=GYRO_RANGE_2048, gyro_rate=GYRO_RATE_125_HZ,
                 delay_between_samples=100, flip=False):

        super().__init__(device, 0.0, delay_between_samples)
        self._device = device
        self._read_style = 0  # see _read_into
        self._tx_buf = bytearray(1)
        self._rx_buf = bytearray(6)
        self._rx_mv = memoryview(self._rx_buf)
        self._flip = flip

        if self._read_reg(_VERSION_REG) != 0x05:
            raise RuntimeError("Failed to find QMI8658C")

        self._accel_range = accel_range
        self._accel_rate = accel_rate
        self._gyro_range = gyro_range
        self._gyro_rate = gyro_rate
        self._update_scales()

        self._write_reg(_RESET_REG, _RESET_CMD)  # soft reset
        time.sleep_ms(200)

        self._write_reg(_CTRL1_REG, 0x40)  # address auto-increment on, little-endian reads
        self._write_reg(_ACCEL_SETTING_REG, _encode_setting(self._accel_range, self._accel_rate))
        self._write_reg(_GYRO_SETTING_REG, _encode_setting(self._gyro_range, self._gyro_rate))
        self._write_reg(_CTRL5_REG, 0x00)  # accel and gyro low-pass filters off
        self._write_reg(_CTRL6_REG, 0x00)  # AttitudeEngine and motion on demand off
        self._write_reg(_ENABLE_REG, 0x03)  # enable accel and gyro
        time.sleep_ms(300)  # gyroscope start-up + filter settling

    def _update_scales(self):
        self._accel_scale = _ONE_G / (1 << (14 - self._accel_range))  # m/s^2 per LSB
        self._gyro_scale = 1.0 / (1 << (11 - self._gyro_range))  # deg/s per LSB

    @property
    def accel_range(self):
        return self._accel_range

    @accel_range.setter
    def accel_range(self, value):
        self._accel_range = value
        self._update_scales()
        self._write_reg(_ACCEL_SETTING_REG, _encode_setting(value, self._accel_rate))

    @property
    def accel_rate(self):
        return self._accel_rate

    @accel_rate.setter
    def accel_rate(self, value):
        self._accel_rate = value
        self._write_reg(_ACCEL_SETTING_REG, _encode_setting(self._accel_range, value))

    @property
    def gyro_range(self):
        return self._gyro_range

    @gyro_range.setter
    def gyro_range(self, value):
        self._gyro_range = value
        self._update_scales()
        self._write_reg(_GYRO_SETTING_REG, _encode_setting(value, self._gyro_rate))

    @property
    def gyro_rate(self):
        return self._gyro_rate

    @gyro_rate.setter
    def gyro_rate(self, value):
        self._gyro_rate = value
        self._write_reg(_GYRO_SETTING_REG, _encode_setting(self._gyro_range, value))

    @property
    def timestamp(self) -> int:
        self._read_into(_TIME_REG, 3)
        return self._rx_buf[0] + (self._rx_buf[1] << 8) + (self._rx_buf[2] << 16)

    @property
    def temperature(self) -> float:
        """Chip temperature in degrees C (signed 16 bit, 1/256 C per LSB)"""
        self._read_into(_TEMP_REG, 2)
        return _int16(self._rx_buf[0] | (self._rx_buf[1] << 8)) / 256.0

    def _get_accelerometer(self):
        self._read_into(_ACCEL_REG, 6)
        x, y, z = struct.unpack_from('<hhh', self._rx_buf)

        s = self._accel_scale
        if self._flip:
            return x * s, -y * s, -z * s
        return x * s, y * s, z * s

    def _get_gyrometer(self):
        self._read_into(_GYRO_REG, 6)
        x, y, z = struct.unpack_from('<hhh', self._rx_buf)

        s = self._gyro_scale
        if self._flip:
            return x * s, -y * s, -z * s
        return x * s, y * s, z * s

    def _get_magnetometer(self):  # NOQA
        return None

import math
import time

try:
    import fusion  # only needed for sensors that have a magnetometer (9DOF)
except ImportError:
    fusion = None


class Fusion6DOF:
    """Accelerometer + gyroscope fusion (Mahony-style complementary filter).

    No magnetometer is needed. Roll and pitch are referenced to gravity and
    are stable. Yaw has no absolute reference: it starts at 0 and drifts
    slowly with gyroscope bias.

    accel: any consistent unit, gated against `one_g` (default m/s^2)
    gyro:  degrees per second
    """

    def __init__(self, gain=1.0, one_g=9.80665):
        self.gain = gain  # higher = tilt settles faster, lower = less jitter
        self._one_g = one_g
        self._q0 = 1.0
        self._q1 = 0.0
        self._q2 = 0.0
        self._q3 = 0.0
        self._seeded = False

    def reset(self):
        self._q0 = 1.0
        self._q1 = self._q2 = self._q3 = 0.0
        self._seeded = False

    def _seed(self, ax, ay, az):
        # start from the gravity-derived roll/pitch so there is no settling time
        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        self._q0 = cr * cp
        self._q1 = sr * cp
        self._q2 = cr * sp
        self._q3 = -sr * sp
        self._seeded = True

    def angles(self):
        """Returns (roll, pitch, yaw) in degrees."""
        q0, q1, q2, q3 = self._q0, self._q1, self._q2, self._q3
        roll = math.atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2))
        sinp = 2.0 * (q0 * q2 - q3 * q1)
        if sinp > 1.0:
            sinp = 1.0
        elif sinp < -1.0:
            sinp = -1.0
        pitch = math.asin(sinp)
        yaw = math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3))
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    def update(self, accel, gyro, dt):
        ax, ay, az = accel
        norm = math.sqrt(ax * ax + ay * ay + az * az)

        if not self._seeded:
            if norm > 0.0:
                self._seed(ax, ay, az)
            return self.angles()

        if dt <= 0.0:
            return self.angles()

        gx = math.radians(gyro[0])
        gy = math.radians(gyro[1])
        gz = math.radians(gyro[2])

        q0, q1, q2, q3 = self._q0, self._q1, self._q2, self._q3

        # Only trust the accelerometer as a gravity reference when the board
        # is not accelerating much (magnitude close to 1 g).
        if 0.8 * self._one_g < norm < 1.2 * self._one_g:
            ax /= norm
            ay /= norm
            az /= norm

            # estimated direction of "up" in the body frame
            vx = 2.0 * (q1 * q3 - q0 * q2)
            vy = 2.0 * (q0 * q1 + q2 * q3)
            vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3

            # error = cross(measured, estimated), fed back into the gyro rates
            gx += self.gain * (ay * vz - az * vy)
            gy += self.gain * (az * vx - ax * vz)
            gz += self.gain * (ax * vy - ay * vx)

        # integrate: q += 0.5 * q (x) (0, gx, gy, gz) * dt
        h = 0.5 * dt
        q0n = q0 + h * (-q1 * gx - q2 * gy - q3 * gz)
        q1n = q1 + h * (q0 * gx + q2 * gz - q3 * gy)
        q2n = q2 + h * (q0 * gy - q1 * gz + q3 * gx)
        q3n = q3 + h * (q0 * gz + q1 * gy - q2 * gx)

        n = math.sqrt(q0n * q0n + q1n * q1n + q2n * q2n + q3n * q3n)
        self._q0 = q0n / n
        self._q1 = q1n / n
        self._q2 = q2n / n
        self._q3 = q3n / n

        return self.angles()


class IMUSensorFramework:
    # delay between samples is microsecond resolution. 1000 microseconds = 1 millisecond

    # Accelerometer value that corresponds to 1 g, in whatever unit the
    # subclass returns. Used by the 6DOF filter to ignore samples taken while
    # the device is accelerating. Override in a subclass that returns g.
    ACCEL_ONE_G = 9.80665

    def __init__(self, device, declination_adjustment=0.0, delay_between_samples=100):
        self._device = device
        self._declination = declination_adjustment
        self._fusion = None  # 9DOF filter, created on demand if a magnetometer exists
        self._fusion6 = Fusion6DOF(one_g=self.ACCEL_ONE_G)
        self._delay_between_samples = delay_between_samples
        self._gyro_bias = (0.0, 0.0, 0.0)
        self._last_us = None
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0

    @property
    def roll(self):
        return self._roll

    @property
    def pitch(self):
        return self._pitch

    @property
    def yaw(self):
        return self._yaw

    @property
    def fusion_gain(self):
        """6DOF tilt-correction gain (default 1.0)."""
        return self._fusion6.gain

    @fusion_gain.setter
    def fusion_gain(self, value):
        self._fusion6.gain = value

    def _get_gyrometer(self):
        raise NotImplementedError

    def _get_accelerometer(self):
        raise NotImplementedError

    def _get_magnetometer(self):
        raise NotImplementedError

    def _get_fusion9(self):
        if self._fusion is None:
            if fusion is None:
                raise RuntimeError("the 'fusion' module is required for 9DOF fusion")
            self._fusion = fusion.Fusion(declination=self._declination)
        return self._fusion

    def calibrate_gyro(self, sample_count=200, delay_ms=10):
        """Average the gyroscope while the device is held perfectly still.

        Call once after power-up. delay_ms should be at least one sensor
        sample period (10 ms suits rates down to about 100 Hz).
        """
        sx = sy = sz = 0.0
        for _ in range(sample_count):
            gx, gy, gz = self._get_gyrometer()
            sx += gx
            sy += gy
            sz += gz
            time.sleep_ms(delay_ms)

        self._gyro_bias = (sx / sample_count, sy / sample_count, sz / sample_count)
        self._fusion6.reset()
        self._last_us = None
        return self._gyro_bias

    def calibrate(self, sample_count=5):
        # Magnetometer calibration. Skipped if no magnetometer is available
        # (use calibrate_gyro() for 6DOF sensors).
        try:
            if self._get_magnetometer() is None:
                return False  # no magnetometer -> nothing to calibrate
        except NotImplementedError:
            return False
        ts = [time.ticks_us()]
        count = [0]

        def _stop_func():
            if count[0] == sample_count:
                return True

            now_ts = time.ticks_us()
            diff = time.ticks_diff(now_ts, ts[0])
            remaining = self._delay_between_samples - diff
            if remaining > 0:
                time.sleep_us(remaining)

            ts[0] = time.ticks_us()
            count[0] += 1
            return False

        self._get_fusion9().calibrate(self._get_magnetometer, _stop_func)

    def read(self):
        accel = self._get_accelerometer()
        gyro = self._get_gyrometer()

        try:
            mag = self._get_magnetometer()
        except NotImplementedError:
            mag = None

        if mag is None:
            # 6DOF: accelerometer + gyroscope only
            now = time.ticks_us()
            if self._last_us is None:
                dt = 0.0
            else:
                dt = time.ticks_diff(now, self._last_us) / 1000000.0
                if dt > 0.5:  # long stall (e.g. a blocked UI task): don't integrate a huge step
                    dt = 0.5
            self._last_us = now

            bx, by, bz = self._gyro_bias
            gyro = (gyro[0] - bx, gyro[1] - by, gyro[2] - bz)
            self._roll, self._pitch, self._yaw = self._fusion6.update(accel, gyro, dt)
        else:
            self._roll, self._pitch, self._yaw = self._get_fusion9().update(accel, gyro, mag)

        return self._roll, self._pitch, self._yaw

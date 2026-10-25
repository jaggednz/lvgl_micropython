"""Driver for the TCA6408 / TCA6408A 8-bit I2C GPIO expander (lvgl_micropython).

Pins P0..P7 are numbered 0..7. Each pin is an input (for buttons, other
interrupt sources) or an output. The chip's INT output is open-drain and
active-low: it goes low when any input differs from the last value read from
the Input Port register, and reading that register releases it.

Notes
- The TCA6408A has NO internal pull-ups. Buttons that short a pin to ground
  need an external pull-up (usually already on the board).
- Call poll() regularly (every 10-30 ms is plenty), or hand the expander to
  lvgl_keypad() and LVGL's input-device timer will poll it for you.
- Only input pins generate events. Output pins never do.
"""
import time
from micropython import const

I2C_ADDR = const(0x20)  # 0x21 when the ADDR pin is high
BITS = const(8)

_REG_INPUT = const(0x00)
_REG_OUTPUT = const(0x01)
_REG_POLARITY = const(0x02)
_REG_CONFIG = const(0x03)  # 1 = input, 0 = output

_keep = []  # keeps LVGL callbacks alive


class TCA6408:
    def __init__(self, device, int_pin=None, inputs=0xFF, active_low=0xFF,
                 outputs=0x00, debounce_ms=20):
        """
        device       lvgl_micropython i2c.I2C.Device (needs read_mem / write_mem)
        int_pin      GPIO number or machine.Pin wired to the expander's INT, or
                     None to read the chip on every poll() (no interrupt wiring)
        inputs       bitmask of pins used as inputs; the rest become outputs
        active_low   bitmask of inputs that read LOW when active (button to
                     ground). The chip's polarity register is used to invert
                     them, so "pressed" is always 1 in this driver.
        outputs      initial output levels (bitmask), applied before the
                     output pins are switched on, so they never glitch
        debounce_ms  a pin must be steady this long before an event fires
        """
        self._device = device
        self._tx = bytearray(1)
        self._rx = bytearray(1)
        self._rx_mv = memoryview(self._rx)
        self._read_style = 0  # see _read_reg

        self._inputs = inputs & 0xFF
        self._out = outputs & 0xFF
        self._debounce_ms = debounce_ms

        self._press_cb = [None] * 8
        self._release_cb = [None] * 8
        self._change_cb = None

        self._flag = False  # set from the INT interrupt
        self._pending = False  # a change was seen and is waiting out the debounce time
        self._changed_at = 0
        self._int = None

        self._write_reg(_REG_POLARITY, active_low & 0xFF)
        self._write_reg(_REG_OUTPUT, self._out)
        self._write_reg(_REG_CONFIG, self._inputs)

        # first read also releases any INT that was already pending
        self._raw = self._stable = self._read_reg(_REG_INPUT) & self._inputs

        if int_pin is not None:
            if isinstance(int_pin, int):
                from machine import Pin
                int_pin = Pin(int_pin, Pin.IN, Pin.PULL_UP)
            self._int = int_pin
            self._irq_cb = self._irq  # keep one bound-method object alive
            self._int.irq(trigger=self._int.IRQ_FALLING, handler=self._irq_cb)

    # ------------------------------------------------------------ bus I/O --
    def _read_reg(self, reg):
        # i2c.Device.read_mem has differed between lvgl_micropython versions:
        # read_mem(reg, nbytes) -> bytes, or read_mem(reg, buf) filling buf.
        if self._read_style == 0:
            try:
                return self._device.read_mem(reg, 1)[0]
            except TypeError:
                self._read_style = 1
        self._device.read_mem(reg, self._rx_mv)
        return self._rx[0]

    def _write_reg(self, reg, value):
        self._tx[0] = value & 0xFF
        self._device.write_mem(reg, self._tx)

    def _irq(self, pin):
        self._flag = True  # interrupt context: only set a flag, no I2C here

    def deinit(self):
        if self._int is not None:
            self._int.irq(handler=None)

    # ------------------------------------------------------------- inputs --
    def on_press(self, pin, callback):
        """callback(pin) runs when `pin` becomes active (debounced)."""
        self._press_cb[pin] = callback

    def on_release(self, pin, callback):
        """callback(pin) runs when `pin` becomes inactive (debounced)."""
        self._release_cb[pin] = callback

    def on_change(self, callback):
        """callback(pin, pressed) runs for every debounced change on any input."""
        self._change_cb = callback

    @property
    def pressed_mask(self):
        """Debounced state of all inputs, bit n = pin n active."""
        return self._stable

    def pressed(self, pin):
        return bool(self._stable & (1 << pin))

    def read_inputs(self):
        """Raw, un-debounced read of all input pins (bit set = active)."""
        value = self._read_reg(_REG_INPUT) & self._inputs
        self._flag = True  # this read released INT; make poll() look again
        return value

    def poll(self):
        """Service the expander. Returns a bitmask of pins whose debounced
        state changed on this call (0 if nothing happened)."""
        if not (self._int is None or self._pending or self._flag or self._int.value() == 0):
            return 0  # INT is idle: nothing to read

        self._flag = False
        now = time.ticks_ms()
        raw = self._read_reg(_REG_INPUT) & self._inputs  # also releases INT

        if raw != self._raw:  # still moving (or just started): restart the debounce timer
            self._raw = raw
            self._changed_at = now
            self._pending = True
            return 0

        if not self._pending or time.ticks_diff(now, self._changed_at) < self._debounce_ms:
            return 0

        self._pending = False
        changed = raw ^ self._stable
        self._stable = raw

        for pin in range(8):
            bit = 1 << pin
            if changed & bit:
                is_pressed = bool(raw & bit)
                cb = self._press_cb[pin] if is_pressed else self._release_cb[pin]
                if cb is not None:
                    cb(pin)
                if self._change_cb is not None:
                    self._change_cb(pin, is_pressed)

        return changed

    # ------------------------------------------------------------ outputs --
    def write(self, pin, value):
        bit = 1 << pin
        if self._inputs & bit:
            raise ValueError("pin %d is configured as an input" % pin)
        if value:
            self._out |= bit
        else:
            self._out &= ~bit & 0xFF
        self._write_reg(_REG_OUTPUT, self._out)

    def toggle(self, pin):
        self.write(pin, not ((self._out >> pin) & 1))

    @property
    def output_mask(self):
        return self._out


def lvgl_keypad(expander, keymap, group=None):
    """Expose expander inputs to LVGL as a keypad input device.

    keymap maps pin number -> LVGL key, e.g.
        {0: lv.KEY.ENTER, 1: lv.KEY.NEXT, 2: lv.KEY.PREV}
    LVGL's input timer calls the read callback, which polls the expander, so
    no separate timer is needed. Pass a group to receive the key events:
        group = lv.group_create(); group.add_obj(widget)
    Returns the lv indev.
    """
    import lvgl as lv  # NOQA

    pins = sorted(keymap)  # lowest pin number wins if several are held
    last_key = [0]

    def read_cb(indev, data):
        expander.poll()
        mask = expander.pressed_mask
        for pin in pins:
            if mask & (1 << pin):
                last_key[0] = keymap[pin]
                data.key = last_key[0]
                data.state = lv.INDEV_STATE.PRESSED
                return
        data.key = last_key[0]
        data.state = lv.INDEV_STATE.RELEASED

    indev = lv.indev_create()
    indev.set_type(lv.INDEV_TYPE.KEYPAD)
    indev.set_read_cb(read_cb)
    if group is not None:
        indev.set_group(group)

    _keep.extend((indev, read_cb))
    return indev
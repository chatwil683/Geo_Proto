import gpiod
import threading
import time

class IndicatorLED:
    """
    Controls a single GPIO LED based on a "state" string.
    Handles blinking internally without blocking ROS callbacks.
    """

    def __init__(self, line_offset: int, chip_name="gpiochip0"):
        self.chip = gpiod.Chip(chip_name)
        self.line = self.chip.get_line(line_offset)
        self.line.request(
            consumer="indicator_led",
            type=gpiod.LINE_REQ_DIR_OUT,
            default_vals=[0],
        )

        self._state = None           # Current logical state
        self._blink_period = 1.0     # Default blink period (sec)
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

        self._last_toggle = time.monotonic()
        self._led_on = False

    def set_state(self, state: str):
        """
        Change LED state. Options:
        - "boot"
        - "disconnected"
        - "idle"
        - "arming"
        - "airborne"
        - "offboard"
        """
        if state == self._state:
            return
        self._state = state

        # Define blink period or solid based on state
        if state in ["boot", "disconnected"]:
            self._blink_period = 1.5
        elif state == "idle":
            self._blink_period = 0.0  # solid ON
            self._led_on = True
            self.line.set_value(1)
        elif state == "arming":
            self._blink_period = 0.3
        elif state == "airborne":
            self._blink_period = 0.0  # solid ON
            self._led_on = True
            self.line.set_value(1)
        elif state == "offboard":
            self._blink_period = 0.8
        else:
            # Unknown state → turn off LED
            self._blink_period = 0.0
            self._led_on = False
            self.line.set_value(0)

        # Reset timer for blink
        self._last_toggle = time.monotonic()

    def _run(self):
        """
        Internal thread: non-blocking blink handler.
        Uses monotonic time (like millis()) to toggle LED.
        """
        while not self._stop_event.is_set():
            now = time.monotonic()
            if self._blink_period > 0.0:
                if now - self._last_toggle >= self._blink_period / 2:
                    self._led_on = not self._led_on
                    self.line.set_value(1 if self._led_on else 0)
                    self._last_toggle = now
            # For solid ON/OFF, do nothing
            time.sleep(0.01)  # very short sleep to reduce CPU usage

    def cleanup(self):
        self._stop_event.set()
        self.line.set_value(0)
        self.line.release()


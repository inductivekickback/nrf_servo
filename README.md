This is a Zephyr RTOS driver for controlling servomotors (servos). It was built from the v2.4.0 tag of the [nRF Connect SDK (NCS)](https://github.com/nrfconnect/sdk-nrf).

<p align="center"><img src="https://user-images.githubusercontent.com/6494431/108953457-10ee0f00-7620-11eb-92ee-487393d81d06.JPG" width="256"></p>

### About servomotors
A typical servomotor has 180 degrees of rotation and accepts a PWM signal with a period of 20ms where:
 - A pulse of 1.0ms (5% duty cycle) is 0 degrees
 - A pulse of 1.5ms (7.5% duty cycle) is 90 degrees
 - A pulse of 2.0ms (10% duty cycle) is 180 degrees

However, these pulse widths are not standardized so it's not uncommon to find servos where, for example, 0 degrees corresponds to a pulse of 0.5ms and 180 degrees is 2.5ms. If a pulse width is used that is outside of the range of a given servo then the servo's motor can stall, drawing an undesirable amount of current.

### About the driver
Nordic's [PWM peripheral](https://infocenter.nordicsemi.com/index.jsp?topic=%2Fps_nrf52840%2Fpwm.html&cp=4_0_0_5_16) works well with servos. However, mapping individual servos to channels of particular instances of PWM peripherals is a chore. This driver automatically assigns PWM channels to each servo device in the DT so the application doesn't have to. Of course PWM peripherals might be needed for other purposes in the application so access to specific instances can be denied via the CONFIG_NRF_SERVO_ALLOW_PWMX settings.

Each servo device in the DT specifies a pin, initial value (in the range of [0, 100]), a minimum pulse width, and a maximum pulse width. The driver then maps the values in the range [0, 100] to the specified range of pulse widths.

### Using the driver
This is an example DT entry in the project's local overlay (e.g. "nrf52dk_nrf52832.overlay"):
```
servo0: nrf-servo0 {
    compatible = "nrf-servo";
    pin = <27>;
    init_value = <50>;
    min_pulse_us = <1000>;
    max_pulse_us = <2000>;
    status = "okay";
};
```
Then add the following to **prj.conf**:
```
CONFIG_NRF_SERVO=y
```
The init value will be written to the device at startup. New values can be written a particular servo:
```
ret = servo_write(dev, value);
```
or read:
```
ret = servo_read(dev, &value);
```

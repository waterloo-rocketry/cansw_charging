# CAN Software Template

This repository serves as a template to base future CAN codebases upon.

## File Layout

`canlib/` is a git submodule pointing to [canlib](https://github.com/waterloo-rocketry/canlib), the team's common CAN library.

`nbproject/` is where MPLAB X (the IDE we use for PIC microcontrollers) stores its configuration.

`Makefile` is also maintained by MPLAB X, which uses make internally.

`device_config.c`/`.h` sets the configuration bits for the microcontroller and has the code to initialize the external oscillator. We mostly leave this alone.

`main.c` is the entrypoint and contains both initialization code and the main loop, along with interrupt handlers. In this template it simply sets up CAN, blinks an LED, and responds to the `MSG_LEDS_ON` and `MSG_LEDS_OFF` CAN commands.

`platform.c`/`.h` defines the hardware-specific functionality like GPIO pinouts and peripheral initialization code. In this template it just sets up control for 3 LEDs.

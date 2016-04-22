# Quadcopter---Dimensional-Testing-Code
.ino Files for Teensy 3.2 for testing YPR, XYZ control.  XYZ is positional control, outputting setpoints into YPR controller inputs.  YPR is control over yaw, pitch, and roll.

## Motivation

We designed and had built a custom test rig to be able to test our built from scratch PID control algorithms.  These files contain many of the test code files we developed from scratch.

## Installation

The custom libraries are included.

## Tests

Most of the specific directions are commented at the top of the .ino files.  For communication with the quad either a Bluetooth link can be used or direct serial link via a USB cable connected to the Teensy, just comment one of the top two definitions to select the one you're using.

## Contributors

Bradley Danielson, Rudolf Hulse



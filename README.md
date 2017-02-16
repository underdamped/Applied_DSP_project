Applied DSP project
-------------------
Teensy and MATLAB code by
Javier Lombillo
2017-02-15

CMSIS DSP library and header files by ARM.

This code is part of a CET4190C project to implement a hardware DSP FIR filter
using the Teensy 3.6 dev board (ARM Cortex-M4 core) and the ARM CMSIS DSP library.

The system essentially employs a client-server model:

Using MATLAB on a (*little-endian*) PC, a user calls filterTeensy() to transfer
audio data or such over USB serial to the Teensy, which in turn filters the data
and transfers it back to MATLAB.

Typical usage example (in MATLAB):

```javascript
% get audio data
[y, Fs] = audioread('filename.wav');

% open serial port (change COM11 to an appropriate value)
s = serial('COM11');
fopen(s);

% filter the data
out = filterTeensy(s,y)
```

The filter response is defined by the coefficient array 'coeff', located in
the Applied_DSP_project.ino source file.  Currently it's a 128th order BPF
with corner frequencies of 3 kHz and 4.5 kHz, setup for data sampled at 48 kHz.

The Teensy code was written in the Arduino IDE; to get the CMSIS stuff working
in Arduino, arm_math.h was edited a bit.


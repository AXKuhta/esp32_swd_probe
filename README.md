## ESP32 SWD Probe and Ampere Meter

Flash an RP2040 board over SWD:

```sh
openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 200" -c "program firmware.elf verify reset exit"
```

Observe current consumption:

```sh
$ tio -m INLCRNL /dev/ttyACM0 
[16:08:00.312] tio v2.5
[16:08:00.313] Press ctrl-t q to quit
[16:08:00.313] Connected

ADC PIN 1 data:
   Avg raw value = 1258
   Avg millivolts (BROKEN) value = 125
   Avg millivolts (FIX) value = 230
   ohms = 33.315580
     mA = 99
```

Verify current readings by substituting the RP2040 board with a 100 ohm resistor.

Wiring diagram:

![Screenshot from 2024-08-16 16-03-44](https://github.com/user-attachments/assets/161636f5-040a-4424-894d-7225e5e95f60)

Measuring with a 2.5 ohm resistor will cause a drop in voltage for the part being measured. Be sure to double check the datasheet of whatever it is you want to measure for minimum operating voltage, or experiment.

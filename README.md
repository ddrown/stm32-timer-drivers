* devicetree.patch - contains the device tree changes to have a PPS on PA8 (tim1 ch1), and uses tim5 as a kernel clocksource using a 24MHz signal on PH11
* pps-pin33.patch - another example device tree change for PPS on PC7 (tim8 ch2) as well as i2c5 enabled
* clocksource-stm32 - provides a kernel clocksource (provides the wall clock time)
* pps-stm32 - hardware pps timestamp capture driver

both drivers are built on top of the stm32-timers mfd driver

The clocksource driver accepts arguments from device tree:
* clock-source = <2>; // use channel 2, clock-source=0 means use 209MHz internal clock
* clock-frequency = <24000000>; // needed for external channels

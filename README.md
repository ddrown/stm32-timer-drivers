* devicetree.patch - contains the device tree changes to have a PPS on PA8 (tim1 ch1), and uses tim5 as a kernel clocksource
* clocksource-stm32 - provides a kernel clocksource (provides the wall clock time)
* pps-stm32 - hardware pps timestamp capture driver

both drivers are built on top of the stm32-timers mfd driver

# an arduino (ESP32) library for the rplidar
an (Arduino) library for the RPlidar (A1M8 specifically) and the ESP32

handles all 4 communication protocols layed out by the Slamtec protocol documentation:
- standard scan (2kHz)
- express legacy (4kHz)
- express extended (8kHz)
- express dense (8kHz)

Only tested on the A1M8 on firmware version 1.29 (see source files for getLidarConf() results)
I couldn't find any libraries for RPlidars that actually support all data types.
I bought an A1M8 for a university self-driving racing project, and i'm using it to detect cones (for SLAM and stuff).

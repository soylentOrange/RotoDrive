# RotoDrive
 
Web app / µROS controlled and esp32-powered rotary drive thingy using MKS42D stepper.

## Why do I need it?

This repository is part of the EU funded research project [HospiBot](https://hospibot.eu/).
It's intended as a test application for controlling a linear drive using µROS.

## How do I use it?

### LED Colour Codes

| State | LED | Description | 
| :--- | :--- | :--- |
| WAITING_WIFI | solid white | waiting for connection to WiFi | 
| WAITING_CAPTIVE | quickly flashing white | WiFi not found / timeout while connecting - Safeboot active |
| SAFEBOOT | slowly flashing white | Safeboot active |
| INITIALIZING | quickly flashing green | Motor calibration |
| ERROR | quickly flashing red | Error has occurred |
| HOMING | breathing blue | driving towards home position | 
| IDLE | solid green | waiting for input |
| DRIVING | breathing green| motor is running |
| NONE | off | default, shouldn't happen|

### Required Hardware

### Web logging

Even though there is no button for it, if you open rdrive.local/weblog, you'll see a logging window. 

### Flashing Firmware

The easiest way to get the firmware on the board is using [esptool.py](https://github.com/espressif/esptool). After [installing](https://docs.espressif.com/projects/esptool/en/latest/esp32/installation.html#installation) esptool.py and downloading the factory firmware image, upload it to the board (using the usb port of your board) with: `esptool.py write_flash 0x0 ADJUST_TO_YOUR_PATH/firmware.factory.bin`. 

Otherwise, you'll need to have [PlatformIO](https://platformio.org/platformio-ide) (and [esptool.py](https://github.com/espressif/esptool)).

For minifying the html parts, you also need to have [Node.js](https://nodejs.org/).
Install development dependencies: 
- [html-minifier-terser](https://github.com/terser/html-minifier-terser)
  - `npm install html-minifier-terser -g`
- [clean-css](https://github.com/clean-css/clean-css)
  - `npm install clean-css-cli -g`

Then clone the repository, open it in [Visual Studio Code](https://code.visualstudio.com/), possibly adjust the `platformio.ini` to your likings and build it for your board. After a (successful) build, you can flash it using esptool (a hint on the command line is given after building).  

## Acknowledgements

<!-- * The TMC2209 Stepper Driver is controlled using the [TMC2209](https://github.com/janelia-arduino/TMC2209) library. 

* For generating the steps, the [FastAccelStepper](https://github.com/gin66/FastAccelStepper) library is used. -->

* The favicon was prepared according (loosely) the guide [How to Favicon in 2025](https://evilmartians.com/chronicles/how-to-favicon-in-2021-six-files-that-fit-most-needs). 

* Creating svgs with Inkscape leaves a lot of clutter in the file, [svgo](https://github.com/svg/svgo) helps.

* The Toast notifications are made with [Toastify](https://github.com/apvarun/toastify-js).

* The status indicators are made with [status-indicator](https://github.com/tnhu/status-indicator).

* Splashcreens for iOS are auto-generated with [iosPWASplash](https://github.com/avadhesh18/iosPWASplash).

* The formula for generating the breathing LED was found in a post at [ThingPulse](https://thingpulse.com/breathing-leds-cracking-the-algorithm-behind-our-breathing-pattern).

## Funding Note

This work is part of the **Hospibot** project, which is funded by [Interreg Deutschland-Danmark](https://www.interreg.eu/) ![Interreg-Logo_bilingual_RGB](assets/doc/funding_note.png)
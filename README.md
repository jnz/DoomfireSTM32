DoomfireSTM32 ("flamebuffer")
=============================

An implementation of the Playstation 1 Doom fire effect on an STM32F429 microcontroller:

![gif](doc/gyro.gif?raw=1)

Read how the Doom fire was done in an article by Fabien Sanglard:

 * https://fabiensanglard.net/doom_fire_psx/

Overall this is pretty useless, but you can use it as a desktop ornament (if you
have a spare STM32429DISC1 discovery kit lying around) 
Press the blue user button to stop the fire temporarily.

Features:

    * Fire is reacting to movement (reading gyroscope data)
    * Full TFT resolution (measly 320x240 though) but running at stable 60 Hz
    * CPU is idle for a good part of the 60 Hz frametime and put into low-power mode
    * ChromART (DMA2D) is used to speed up the drawing

Installation
------------

Just flash `Release/DoomfireSTM32.elf` to the STM32F429DISC1 discovery board via
USB, e.g. with the STM32 ST-LINK utility or the STM32CubeProgrammer.

Development
-----------

The STM32CubeIDE workspace (Eclipse) project files are included.
The main code is in `Core/Src/main.c` plus a big load of boilerplate code.


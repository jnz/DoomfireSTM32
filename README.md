DoomfireSTM32 ("flamebuffer")
=============================

An implementation of the Playstation 1 Doom fire effect on an STM32F429 microcontroller:

![gif](doc/example.gif?raw=1)

Overall this is pretty useless, but you can use it as a desktop ornament (if you
have a spare STM32429DISC1 discovery kit lying around) 

Features:
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


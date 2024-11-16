Materials
=========
* A Gamecube controller

* An N64

* An Arduino with an AtMega328 running at 16MHz.

* Gamecube controller extension cables.

* N64 extension cable 

* 1K ohm resistor, as a pull-up resistor.

Quick Setup
===========

Hooking the N64 to the Arduino
------------------------------
The N64 controller cable had 3 wires: ground, +3.3V, and data. The pin-out is shown in Figure 1 (left).

1. +3.3V (red) - connect to 3.3v

2. Data (white) - connect to Arduino digital I/O 8

3. GND (black) - connect to Arduino ground

The wire colors may vary, check that the wire colors match the pin-out before you connect them up.

.. figure:: https://github.com/brownan/Gamecube-N64-Controller/raw/master/connections.png
    :alt: Gamecube and N64 controller connections

    Figure 1: Pin numbers for an N64 plug (left) and a Gamecube socket (right).
    Credit to this diagram goes to the `Cube64 project`_.

Hooking up the Gamecube controller to the Arduino
-------------------------------------------------
The GC controller cable had 5 wires: +5V, +3.3V, data, 2x ground. The pin-out is shown in Figure 1 (right).

1. +5V (White) - connect to Arduino +5V supply

2. Data (Red ) - connect to Arduino digital I/O 2

3. GND (yellow) - connect to Arduino ground

4. GND (Black) - connect to Arduino ground

5. N/C () - No wire

6. +3.3V (Green) - connect to Arduino +3.3V supply

Same as N64 cable, the wire colors may vary, check that the wire colors match the pin-out before you connect them up.



Attaching the Pull-up Resistor
------------------------------
The protocol requires the data line to be idle-high, so **attach the 1K
resistor between digital I/O 2 and the 3.3V supply**. This will keep the line
at 3.3V unless the Arduino or the controller pulls it down to ground.


Configuration
=============
To edit the button mapping, edit `gamecube.ino` from line 172.

Default mapping  is:

| N64  | GC |
| ------------- | ------------- |
|A|A|
|B|B|
|CLEFT|Y, CLEFT
|CRIGHT|X, CRIGHT
|CUP|CUP|
|CDOWN|CDOWN|
|DLEFT|DLEFT|
|DRIGHT|DRIGHT|
|DUP|DUP|
|DDOWN|DDOWN|
|START|START|
|Z|L,Z|
|L|UNMAPPED|
|R|R|


This is done with Bitwise operations, here's an example of how I mapped GC-L to N64-L:
Original Button Mapping:

    GameCube (GC) controller data is stored in two bytes, data1 and data2.
    The N64 controller expects the button states in a different format.

Button Bits in gc_status:

    gc_status.data1 contains: 0, 0, 0, start, y, x, b, a
    gc_status.data2 contains: 1, L, R, Z, Dup, Ddown, Dright, Dleft

Mapping Explanation:
L Button:

    The L button is bit 6 in gc_status.data2 (counting from the right, starting at 0).
    We need to map this to the appropriate bit in the N64's button buffer.

Bitwise Operations:

    Bitwise >> 4 shifts the bits of gc_status.data2 to the right by 4 positions.

Why >> 4 for L?

    Locate L Button: gc_status.data2 & 0x40
        0x40 in binary is 01000000, so this masks out all bits except for bit 6.
    Shift to Correct Position: >> 4
        Shifting the masked result (01000000) right by 4 bits results in 00000100.

Updated Mapping in n64_buffer:

    n64_buffer[1] should contain: 0, 0, L, R, Cup, Cdown, Cleft, Cright
    Mapping the L button from gc_status.data2 to n64_buffer[1] is done as:

``n64_buffer[1] |= (gc_status.data2 & 0x40) >> 4; // L -> L``


X and Y button mappings
-----------------------
Since the X and Y buttons don't exist on the N64, one has some freedom in
mapping these buttons. I like to map them to C-down and C-left respectively for
games like Starfox where those buttons are more significant. For something like
Perfect Dark where C-left and C-right strafe, I map X and Y to those instead.

You can go to around line 235 in ``gamecube.pde`` to configure the mapping. Try
uncommenting the mapping for X -> Cdown and comment out the line for X ->
Cright if you'd prefer that mapping.

Analog Stick Curve
------------------
On some games, such as Perfect Dark, the control stick feels a bit weird. That
is, it feels too sensitive, like there's not enough difference between fully
tilted and slightly tilted. (or maybe it was not sensitive enough, I forget)

To help with this, I apply a curve mapping inputs on the GC controller to
outputs on the N64 "controller" in a non-linear fashion.

.. figure:: https://github.com/brownan/Gamecube-N64-Controller/raw/master/curve.png
    :alt: Analog Stick curve graph

    Figure 2: A graph showing a linear mapping of inputs to outputs (red) and a
    cubic mapping from inputs to outputs (green). Inputs (from the GC
    controller) are along the X axis, while outputs (to the N64) are on the Y
    axis.

To turn this off, head to line 279 in ``gamecube.pde`` and change the 0 to a 1.
In my experience, this curve helps in some games, but hurts in others.

Method
======

Hardware Setup
--------------
The gamecube connection has 6 wires: 2 ground, a 3.3V rail, a 5V rail for rumble, a data line, and an unused line. The data line goes into digital I/O 2.

The N64 has 3 wires: 3.3V power supply, data, and ground. If you want rumble to work, you will need to supply power to the arduino, if not, this can run on 3.3v driect from the N64. The data plugs into digital I/O 8 and ground goes to ground.

Pull-up Resistor
----------------
The line to the controller is idle-high at 3.3V and is brought low to signal a bit. This means we can't use the Arduino's built-in pull-up resistors to signal, since they operate at 5V. The solution I found works is to bridge the Arduino's 3.3V supply and digital I/O pin 2 with a 1K ohm resistor. This keeps the line high at 3.3V when the pin is in input mode, and can be lowered by setting the pin to output a 0. Thus forming the signaling mechanism.

Signaling
---------
The protocol is simple, it uses low pulses of either 1μs or 3μs to indicate a 1 bit or 0 bit respectively. Bits come in every 4μs, so a 1 bit is 1μs low followed by 3μs high.

This microsecond timing is no problem for the AtMega328, but it does cut it kind of close. At 16MHz I get exactly 16 clock cycles per microsecond. Which is for the most part plenty, but one code path where the loops iterate on a byte boundary with a 1μs budget takes exactly 16 cycles.

Coding
------
I coded the entire signaling routine (sending and receiving) in C, and then analyzed the assembly output, calculated the number of cycles each branch took using the `AVR Instruction Set manual`_, and added in the necessary number of "nop" instructions. Then re-compiled and tested.

.. _AVR Instruction Set manual: http://www.atmel.com/dyn/resources/prod_documents/doc0856.pdf

After some trial and error, I was successfully sending and receiving commands from a gamecube controller. The N64 was easy after that, since they used the same encoding.

Resources
=========
* The `Cube64 Project`_
* `Gamecube Controller Protocol information`_
* `Nintendo 64 Controller Protocol information`_
* `N64/Gamecube to USB adapter Project`_ had some code that was useful as a reference
* `N64 to GameCube conversion project`_ (not sure why anyone would want to go in this direction)

.. _Cube64 Project: http://cia.vc/stats/project/navi-misc/cube64
.. _Gamecube Controller Protocol information: http://www.int03.co.uk/crema/hardware/gamecube/gc-control.htm
.. _Nintendo 64 Controller Protocol information: http://www.mixdown.ca/n64dev/
.. _N64/Gamecube to USB adapter Project: http://www.raphnet.net/electronique/gc_n64_usb/index_en.php
.. _N64 to GameCube conversion project: http://www.raphnet.net/electronique/x2wii/index_en.php

See Also
========
Since this project hasn't been updated in a while, check out NicoHood's `Nintendo`_ project, and the related `HID`_ project
for a more polished library for connecting gamecube controllers to the Arduino and to the computer.

.. _Nintendo: https://github.com/NicoHood/Nintendo
.. _HID: https://github.com/NicoHood/HID


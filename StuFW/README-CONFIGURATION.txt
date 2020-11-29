This is only a primer on configuration.

As StuFW firmware is derived from MK4duo and MK4duo used a special mechanism
for configuration it used an online configurator and that produced a file
called Configuration_Overall.h.

This file has been retained in StuFW as a very quick manner to join in an
unique file all the configuration options.

Original Configuration_Overall.h produced by MK4duo SHOULD NOT be used as
a replacement of StuFW Configuration_Overall.h.

It maybe could be used as a trace to make a working Configuration_Overall.h.

Configuration is done in various files named Configuration_xxxxx.h, a list is:

- Configuration_Basic.h
- Configuration_Cartesian.h
- Configuration_Core.h
- Configuration_Motor_Driver.h
- Configuration_Temperature.h
- Configuration_Feature.h

All the setting in these files could be placed in a proper
Configuration_Overall.h.

So there are many ways to configure StuFW:
- create and empty Configuration_Overall.h and then modify the options in
  Configuration_xxxx.h files listed above.
- Create a proper Configuration_Overall.h that include all the relevant
  #define needed to configure your printer copying and pasting the not
  commented out options present in these configuration files.
- Use an existing Configuration_Overall.h files precompiled by someone
  containing proper settings. These files will be present in future in
  the main repository or may be supplied by developers of 3d printer project.

Some efforts have been made and are planned to be made to clarify better the
many options in these files.

There is another file not cited above:

- Configuration_Pins.h

In this file you could override most of the pin settings done by the
motherboard specific file, overriding in example the HB pin to use an external
MOSFET.

The stock Configuration_Pins.h contains many values set as ORIG_xxxx that
should be overriden to make the proper pin assigned to another function, These
ORIG_xxxx definition are done in the motherboard specific file.

In future there will be a better explanation for this behaviour.

If needed use the Issues in the GithHub page to ask for an help.

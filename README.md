# ignition
Ignition focused on older Motorcycle Twins

This is some code that runs on D1 MINI ESP8266 board to perform advance timing.
Works on my 1981 Honda CM400T where I arrange it to pickup the two pulse coil input signals,
one ADV at 45deg BTDC, other at start/idle F around 14deg BTDC.  Between around 2K to 5K RPM
it adjusts the output inbetween these two extremes.

You can find more about my effort over on this forum I have a thread on learning about ignition,
the more recent focused on breadboard IO and prototype efforts.
https://www.vintagehondatwins.com/forums/index.php?threads/making-spark-cave-man-with-a-smart-phone.5865/
In later posts you can see pics of my rough schematics and prototypes.

The code is a bit messy, strange in many regards, hard to understand, I hope to improve that over time.
I hope to make something that could be applied to other motorcycles, with an emphasis on
older bikes that have points and mechanical advance.

I currently am running it with stock AC-CDI where I bypass it's analog advance circuit.
I have also used it with a common inexpensive AC-CDI bypassing my stock one.  A common DC-CDI
could also be used I presume.

I hope to get the code organized better(so I can understand it),
make it work with TCI(points, smart COP/NOP, 12v ignitors), this requires a long dwell time to be worked in.
Also I would like to build in some decent trace logs so when there are problems you can get
a trace log that can tell you what is going on.  Like a digital logic analyzer recording of IO changes would
be nice.

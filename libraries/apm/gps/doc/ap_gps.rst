-----------
apm::ap_gps
-----------

GPS library, Forked from `ArduPilot.AP_GPS`_ .

-----------------------------
Changes from ArduPilot.AP_GPS
-----------------------------

* Use `Generic Programming`_  . Code is placed in the apm namespace. 
  Lower case is used except for macros ( All Caps), template arguments and archetypes (CamelCase). 
* Aggressively eliminate coupling to other libraries.
* Multiple instances of a physical GPS are are not supported in 1 class instance. 
  Use an array of GPS if this is required

.. _`Generic Programming`: https://en.wikipedia.org/wiki/Generic_programming

.. _`ArduPilot.AP_GPS`: https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_GPS









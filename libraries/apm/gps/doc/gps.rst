-----------
apm::ap_gps
-----------

GPS library, Forked from `ArduPilot.AP_GPS`_ .

-----------------------------
Changes from ArduPilot.AP_GPS
-----------------------------

* The _`ArduPilot.AP_GPS` code has been moved to the libraries/apm namespace. 
* Use `Generic Programming`_ .
* Use lower case ed except for macros ( All Caps), template arguments and archetypes (CamelCase). 
* Aggressively eliminate coupling, so the gps library can be used standalone.
* One physical GPS in 1 class instance. (Use an array of GPS if more than one GPS is used)
* Use Quan_ library types to represent Physical quantities.

.. _`Generic Programming`: https://en.wikipedia.org/wiki/Generic_programming
.. _`ArduPilot.AP_GPS`: https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_GPS
.. _Quan: http://www.zoomworks.org/quan-trunk/quan_matters/doc/index.html

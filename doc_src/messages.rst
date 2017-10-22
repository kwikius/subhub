----------------
Sub Hub messages
----------------

-----
matol
-----

* update servo positions every 1/50th sec each 
  
   - used ids 0 - 32 for servos

-----
letom
-----
* rc input channels

   - send num channels
   - send each channel Total 16 every 1/50th sec

* airspeed

   - send every 1/100th sec
  
* mag

   - send every 1/50th sec

* gps

   - send every 1/10th sec

------
split
------
   
* port

   - servos      <-
   - mag         ->
   - rcrx input  ->
   - airspeed    ->

* stbd

   - servos      <-
   - gps         ->


--------------
message format
--------------

* Wrap messages using `COBS protocol`_ 
* Messages provide status
  - Default message status "nofeedback". for example ESC and servos dont provide a way to tell if they are connected
  -

----------
Message ID
----------

Every message in the system has a unique name  At boot message names are mapped to ID's

Q: what happens if a node goes down? 
A: On reboot  it must broadcast all its names to get ID's. 

every message has a type. unless an array it's type is it's name.
domain is represented by prefixing with a '.'
leaf arrays are represented by [ names ] where names: name | names , name

sample system

     .Fusepod
        .OSD_FC
            .vtx     
            .cam     
            .RCPWMO [pitch,yaw] type=rcpwmo
            
        .PortWing
            [asi]
            [mag]
            .RCPWMI[1 to 256] type=rcpwmi

            .frskyout
            .RCPWMO[1,2] type=rcpwmo
            
        .StbdWing
            .Srvo[1,2] type=rcpwmo
            .gps

        .TailBoom
            .RCPWMO [pitch,yaw] type=rcpwmo
      

.. _`COBS protocol` : https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing



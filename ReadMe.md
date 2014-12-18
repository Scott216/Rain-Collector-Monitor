Rain Collector

Outside board is panStamp plugged into a PCB which monitors temp of two heating pads, temp inside rain collector and temp below rain collector.  It also monitors pulses from rain collector and closes a relay for every pulse for the Davis Weather station.  Data is wirelessly sent to the panStamp on the inside.  The outside unit also turns on the heaters to melt the snow.  If outside temp is below freezing, heaters will come on for a few minutes.  If nothing melts to trip the water paddle, then heaters will turn off for a while.  If water is detected, heaters will stay on until all snow is melted.

The inside panStamp is connected to an Ethernet module which posts the data on Xively.
https://xively.com/feeds/103470/


As of 12/17/14 - the Rx sketch is compiled on Arduino IDE 1.5.8.  Some of the libraries need to be modified so the Ethernet SS pin can be assigned to a pin other then D10, because panStamp uses D10 to communicate with the radio.  The Tx sketch on the other hand is still compiled with Arduino 1.0.5, but needs to be updated to 1.5.8.

See [Arduino forum thread](http://forum.arduino.cc/index.php?topic=217423.msg1962182#msg1962182) and files regarding changing SS pin:





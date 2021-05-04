# Arduino RDM master 
This is an example of how to use the TimoTwo SPI RDM interface in TX mode to implement a wireless RDM controller/master.

This example does not have fault control, and it also does not deal with ACK_TIMER or ACK_OVERFLOW.

## How to use
Connect your Arduino to your TimoTwo's SPI interface, all five signals are required.

The example will automatically start to discover any connected receivers and downstream RDM devices. It will show you the manufacturer label and device model description of the discovered devices.
It will let you trigger a new discover or to identify any of the discovered devices.

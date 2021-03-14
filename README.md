# Signalling with CBUS
This repository contains example sketches that illustrate how the [SignalControl](https://github.com/SvenRosvall/SignalControl)
library can be used with CBUS to create Arduino controlled
signals that react to CBUS events.

# Overview
The examples are based on the examples in SignalControl.
But instead of connecting sensors directly to the Arduino, these examples 
use CBUS events to control the signal.

The signal configuration is set up in the Arduino sketch. 
The event handler defines what to do when events are received.

When an event is taught to the Arduino signal module, an event variable
must be set to define what shall happen when the event is received.

# Examples
All examples are configured with a UK style four aspect signal using
PWM output pins to make the LEDs change slowly.

The switch and LEDs for CBUS setup are connected to the analogue pins to
keep them separate from the signal pins.

All examples have the following pin assignment:

| Pin | Use |
| ---: | :--- |
|  2| CAN Interrupt |
| 10 | CAN CS |
| 11 | CAN SI |
| 12 | CAN SO |
| 13 | CAN SCK |
| 3 | Signal Red light |
| 5 | Signal first Yellow light |
| 6 | Signal Green light |
| 9 | Signal second Yellow light |
| A3 | CBUS Yellow FLiM LED |
| A4 | CBUS Green SLiM LED |
| A5 | CBUS Switch |

Both examples react to track occupancy sensor events to decide if the 
track in advance of the signal is occupied.
The examples also react an event from a trailing point where the signal 
shall show red if the point is set against the track with the signal.

## Signal with Block Sensors
This example uses a BlockDistanceInput object to determine how many
track blocks are unoccupied in advance of the signal. 

ACON events show that a block is occupied and ACOF that it is free again.
Teach these sensor events to the module with event variable values 
1, 2 or 3 to indicate which block in advance of the signal the event 
comes from.

Events from a point position sensor can make the signal change to danger
(red) if the point is set to a route that is not allowed.
An ACON event from the sensor indicates that the route is valid and an
ACOF event that the route is invalid.
The event variable shall be set to 10.

## Signal with Timer
This example uses a DistanceTimer that counts the time since a train passed
a sensor.
This translates to a distance of free tracks counted in number of blocks
which is then controlling the signal aspect.

The timer is triggered by a track sensor, either a block sensor or a spot
sensor.
An ACON event sets the signal to danger and an ACOF event starts the timer.

A point position sensor is also supported as described above.

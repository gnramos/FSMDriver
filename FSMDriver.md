FSMDriver
=========

A driver implementation using a Finite State Machine implementation to control its actions.

Main ideia
--------------------

The main ideia behind this implementation is that different driving situations need different driving behavior. Those driving situations will be called states each of them with it's respective behavior. At the begining we designed 4 states: Curve, StraightLine, OutOfTrack and Stuck besides the Transition function responsible for deciding wich state should be called. However, Curve and StraightLine were states so different that a fifth state was created in order to smooth the transition between them. This is when things start to become hairy, The Transition. With those 5 states the Transition function needed to be as accurate as possible since triggering the wrong state would end in disastrous behavior. A new model with simpler Transition would solve this problem.

The 3 states model was then created with a much simpler transition. Since it is very easy to say if the car is inside the boundaries of the track the transition would be responsible only for telling that. If the car is inside the track the InsideTrack state would be called otherwise the OutOfTrack is triggered, if it is stuck on the track border, the Stuck state is called. Due to the thrilling results of the 3 states model, we decided to focus on this controller and abandon the other one with 5 states.

Finite State Machine
--------------------

A finite state machine can be described as device (or model of a device) that has a finite number of states it can be in at any given time, though it can only be in one state at any moment, and can operate on input to either make transitions from one state to another or to cause an output or action to take place.

The modular architecture
--------------------
In order to facilitate de developing stage of the controller, 5 modules were created:

### DrivingState::get_steer ###

### DrivingState::get_gear ###

### DrivingState::get_accel ###

### DrivingState::get_brake ###

### DrivingState::get_clutch ###

 Which each one had to be implemented in each module with it's own behaviour. Each one returns a value for the respective actuator, and then DrivingState returns a CarControl with these actuatros values.

States
------

The implemented states are:

### InsideTrack ###

Given that the driver is inside the track it will aim to the higher reading rangefinder and will estimate a target speed based on how far the car is from the edge of the track. It is a state used at 3 states controller.

### OutOfTrack ###

Given that driver is outside the boundaries of the track it will try to get back to the track with an appropriate return angle.

### Stuck ###

State used when the car speed is too low, actuators such as steer, accel, gear much be set at values to take it out of this condition. This state is triggred when reverse behavior is desired.

Transitions
-----------

Transitions between states are defined according to the sensor readings and the FSM's model.

FSMDriver3
----------

This implements a FSM with 3 states: InsideTrack, OutOfTrack, and Stuck.

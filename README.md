# matlab_soft_pid
**Bryce M. Henson** 
A reasonably robust & fast software PID controller

This conde provides a convenient PID function for running your own computer in the loop feedback, a feedback freqeuncy of 200Hz is acheivable with modest hardware.
While PID is a very simple algorithm the very large dynamic range of software controllers means that care must be taken to prevent integeral windup of the controller under actuator saturation.

## To Do
- [ ] Build test scripts
  - may need to add in the ability to external tell what the time is.
  - simple heater & disturbance plant model

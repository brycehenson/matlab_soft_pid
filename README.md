# matlab_soft_pid
**Bryce M. Henson** 
A reasonably robust & fast software PID controller  
**Status:** This Code is **not ready for use in other projects**. Testing is **not** implemented.

This code provides a convenient PID function for running your own computer in the loop feedback, a feedback freqeuncy of >200Hz is acheivable with modest hardware.

## Features
- integral windup prevention
  - if the error will increment the integral towards the output limit then the integral action is heavily attenuated (with a sigmoid curve)
  - if it will move the output towards the middle of the range then the increment is used in full
- slew rate limitng by back calcluation of the integral
  - can also specify the maximum output step
- Fast, <1ms execution time
- feed-forward terms
  - control needed to maintain static system
  - impulse needed to move system to different state
- Bumpless Operation
  - bumpless start
  - bumpless change of proportional and integral gains


## To Do
contributors welcome! Drop me an [email](mailto:bryce.m.henson+github.matlab_soft_pid@gmail.com?subject=I%20would%20Like%20to%20Contribute[github][matlab_soft_pid]) .

- [x] Basic Test script
  - ability to tell the function what the time is. (to overide the realtime(ish) nature)
  - simple tank & disturbance plant model
- [ ] convert code to a class instead of a function
- [ ] usage examples
  - real life examples
- [x] add in derivative term
- [x] feed foward terms
  - value veclocity and acceleration compensation
- [ ] Advanced tests
  - bandwith limits for actuator
  - specify arbitraty plant model
  - better disturbances, periodic function or specified drift model
- [ ] Measuring loop gain
      - add in some small function and look how it comes back through the loop
      - psrn/Xcorr would be interesting
      - using threads to keep the speed up?
- [x] Smooth change of gains
  - Bumpless operation
  - back calculate the integral to give the same output as before the change in the prop gain
- [ ] learning controller
  - adaptive feed foward terms based on on the fly fitting

## Resources
- [How to Build a Fixed-Point PI Controller That Just Works](https://www.embeddedrelated.com/showarticle/121.php)
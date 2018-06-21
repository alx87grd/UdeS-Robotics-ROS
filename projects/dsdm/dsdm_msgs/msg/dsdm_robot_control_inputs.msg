# Message of DSDM robot control inputs
Header header
int8 m              # number of actuators
int8 p              # number of discrete mode
float32[] F         # vector of torque/forces
int8 k              # index  of desired mode

#ifndef _HEADERS_H_
#define _HEADERS_H_

#define L1 120 //link 1 length
#define L2 50 //link 2 length
#define dToF 10 //distance from ToF to the end effector
#define dBase 105 //distance from the base to the first joint


typedef enum
{
    INIT,  //initializing system
    SUSPENDED,  //standby while drone is flying landing gear is fixed in default position
    ACTIVE,     //landing gear is moving dynamically
    DESCENDING, //landing gear is calculating the terrain and adjust its height
    TOUCHDOWN, //landing gear is touching the ground, moving dyanmically using IMU
    STANDBY, //landing gear is in standby position after touchdown
} State_LG_t;

#endif
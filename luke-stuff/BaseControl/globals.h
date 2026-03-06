#ifndef GLOBALS_H
#define GLOBALS_H

//////////////////////////////////////////////////////////
#define ROBOT_IS_PROXY //comment-out if the robot is Boxy
//////////////////////////////////////////////////////////

//wifi
const char* WIFI_SSID = "AndroidAPwhat2";
const char* WIFI_PASS = "qqqqqqvv";


//add any class .h files you need
#include "data_structs.h"



//add globals below this point

//hit plates globals
inline bool buttonStates[8] = { false }; // true = pressed

//serial comms globals


volatile bool rxDataFresh = false;


//platform globals


//head movement globals


//comms globals



#endif
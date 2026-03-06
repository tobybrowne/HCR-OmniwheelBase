#ifndef DATA_STRUCTS_H
#define DATA_STRUCTS_H

const int BOARD_COMM_ID = 2;

typedef struct robotJointDataStruct {//to be received
    int id = BOARD_COMM_ID; // must be unique for each sender board

    float X_mm;
    float Z_mm;
    float LIFT_mm;

    float TORSO_TILT_deg;
    float TORSO_YAW_deg;

    float SHOULDER_R_deg;
    float ARM1_R_deg;
    float SPIN_R_deg;
    float ARM2_R_deg;

    float SHOULDER_L_deg;
    float ARM1_L_deg;
    float SPIN_L_deg;
    float ARM2_L_deg;
} robotJointDataStruct;

typedef struct pressurePadsDataStruct { //to be sent out;
    int id = BOARD_COMM_ID; // must be unique for each sender board

    int buttonIndex;
    bool buttonState;
} pressurePadsDataStruct;

typedef struct platformDataStruct { //to be sent out;
    int id = BOARD_COMM_ID; // must be unique for each sender board

    float x = 0;
    float z = 0;
} platformDataStruct;

#endif
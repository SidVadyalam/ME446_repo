#include "math.h"
#include "F28335Serial.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define GRAV        9.81

// These two offsets are only used in the main file user_CRSRobot.c  You just need to create them here and find the correct offset and then these offset will adjust the encoder readings
float offset_Enc2_rad = -0.412; //-0.37;
float offset_Enc3_rad = 0.216; // 0.27;


// Your global varialbes.

long mycount = 0;

#pragma DATA_SECTION(whattoprint, ".my_vars")
float whattoprint = 0.0;

#pragma DATA_SECTION(theta1array, ".my_arrs")
float theta1array[100];

long arrayindex = 0;
int UARTprint = 0;

float x = 0;
float y = 0;
float z = 0;

float L1 = 0;
float L2 = 0;
float L3 = 0;

float theta1dh = 0;
float theta2dh = 0;
float theta3dh = 0;

float theoretical_TM1 = 0;
float theoretical_TM2 = 0;
float theoretical_TM3 = 0;

float printPosXmotor = 0;
float printPosYmotor = 0;
float printPosZmotor = 0;

float printtheta1motor = 0;
float printtheta2motor = 0;
float printtheta3motor = 0;

float printInvTheta1 = 0;
float printInvTheta2 = 0;
float printInvTheta3 = 0;

// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;


void mains_code(void);

//
// Main
//


void main(void)
{
	mains_code();
}




// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {

    x =  0.254*cos(theta1motor)*(cos(theta3motor) + sin(theta2motor));
    y =  0.254*sin(theta1motor)*(cos(theta3motor) + sin(theta2motor));
    z = 0.254*cos(theta2motor) - 0.254*sin(theta3motor) + 0.254;

//    inv
    L1 = 0.254;
    L2 = 0.254;
    L3 = 0.254;

    theta1dh = atan2(y,x);
    theta2dh = PI - acos(((L3*L3) + (L2*L2) - ((z-L1)*(z-L1) + (x*x) + (y*y))) /(2*L3*L2));
    theta3dh = -(atan((-z+L1)/(sqrt(x*x + y*y))) - acos( (-(L3*L3) + (L2*L2) + (z-L1)*(z-L1) + (x*x) + (y*y))/(2*L2*(sqrt( (z-L1)*(z-L1) + (x*x) + (y*y) ))) ));

    theoretical_TM1 = theta1dh;
    theoretical_TM2 = theta2dh + HALFPI;
    theoretical_TM3 = theta3dh + theta2dh;

    *tau1 = 0;
    *tau2 = 0;
    *tau3 = 0;

    //Motor torque limitation(Max: 5 Min: -5)

    // save past states
    if ((mycount%50)==0) {

        theta1array[arrayindex] = theta1motor;

        if (arrayindex >= 99) {
            arrayindex = 0;
        } else {
            arrayindex++;
        }

    }

    if ((mycount%500)==0) {
        UARTprint = 1;
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Blink LED on Control Card
        GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1; // Blink LED on Emergency Stop Box
    }

    printtheta1motor = theta1motor;
    printtheta2motor = theta2motor;
    printtheta3motor = theta3motor;

    printPosXmotor = x;
    printPosYmotor = y;
    printPosZmotor = z;

    printInvTheta1 = theoretical_TM1;
    printInvTheta2 = theoretical_TM2;
    printInvTheta3 = theoretical_TM3;

    Simulink_PlotVar1 = theta1motor;
    Simulink_PlotVar2 = theta2motor;
    Simulink_PlotVar3 = theta3motor;
    Simulink_PlotVar4 = 0;

    mycount++;
}

void printing(void){
    if (whattoprint == 0) {
        serial_printf(&SerialA, "%.2f %.2f,%.2f   ",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI);
        serial_printf(&SerialA, "%.2f %.2f,%.2f   ",printPosXmotor,printPosYmotor,printPosZmotor);
        serial_printf(&SerialA, "%.2f %.2f,%.2f   \n\r",printInvTheta1*180/PI,printInvTheta2*180/PI,printInvTheta3*180/PI);

    } else {
        serial_printf(&SerialA, "Print test   \n\r");
    }
}


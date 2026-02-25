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

float theta_desired = 0;

float Theta1_old = 0;
float Omega1_raw = 0;
float Omega1_old1 = 0;
float Omega1_old2 = 0;
float Omega1 = 0;

float Theta2_old = 0;
float Omega2_raw = 0;
float Omega2_old1 = 0;
float Omega2_old2 = 0;
float Omega2 = 0;

float Theta3_old = 0;
float Omega3_raw = 0;
float Omega3_old1 = 0;
float Omega3_old2 = 0;
float Omega3 = 0;

float error1 = 0;
float error2 = 0;
float error3 = 0;

float error1_old = 0;
float error2_old = 0;
float error3_old = 0;

float Kp1 = 35;
float Kd1 = 2.2;
float Ki1 = 0;
float Kp2 = 35;
float Kd2 = 2.2;
float Ki2 = 0;
float Kp3 = 35;
float Kd3 = 2.2;
float Ki3 = 0;

float Ik1_old = 0;
float Ik1 = 0;
float Ik2_old = 0;
float Ik2 = 0;
float Ik3_old = 0;
float Ik3 = 0;

float windup_parameter = 0;

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
    theta3dh = PI - acos(((L3*L3) + (L2*L2) - ((z-L1)*(z-L1) + (x*x) + (y*y))) /(2*L3*L2));
    theta2dh = (atan((-z+L1)/(sqrt(x*x + y*y))) - acos( (-(L3*L3) + (L2*L2) + (z-L1)*(z-L1) + (x*x) + (y*y))/(2*L2*(sqrt( (z-L1)*(z-L1) + (x*x) + (y*y) ))) ));

    theoretical_TM1 = theta1dh;
    theoretical_TM2 = theta2dh + HALFPI;
    theoretical_TM3 = theta3dh + theta2dh;


    if (mycount % 4000 == 0){
        if(theta_desired < 0.1){
            theta_desired = 0.5;
        } else{
            theta_desired = 0.0;
        }
    }

    Omega1_raw = (theta1motor - Theta1_old)/0.001;
    Omega1 = (Omega1_raw + Omega1_old1 + Omega1_old2)/3.0;
    Theta1_old = theta1motor;
    //order matters here. Why??
    Omega1_old2 = Omega1_old1;
    Omega1_old1 = Omega1;

    Omega2_raw = (theta2motor - Theta2_old)/0.001;
    Omega2 = (Omega2_raw + Omega2_old1 + Omega2_old2)/3.0;
    Theta2_old = theta2motor;
    //order matters here. Why??
    Omega2_old2 = Omega2_old1;
    Omega2_old1 = Omega2;

    Omega3_raw = (theta3motor - Theta3_old)/0.001;
    Omega3 = (Omega3_raw + Omega3_old1 + Omega3_old2)/3.0;
    Theta3_old = theta3motor;
    //order matters here. Why??
    Omega3_old2 = Omega3_old1;
    Omega3_old1 = Omega3;


    error1 = theta_desired - theta1motor;
    error2 = theta_desired - theta2motor;
    error3 = theta_desired - theta3motor;

    Ik1 = Ik1_old + ((error1 - error1_old)/2)*0.001;
    Ik2 = Ik2_old + ((error2 - error2_old)/2)*0.001;
    Ik3 = Ik3_old + ((error3 - error3_old)/2)*0.001;


    error1_old = error1;
    error2_old = error2;
    error3_old = error3;
    Ik1_old = Ik1;
    Ik2_old = Ik2;
    Ik3_old = Ik3;



    *tau1 = Kp1*error1 - Kd2*Omega1 + Ki1*Ik1;
    *tau2 = Kp2*error2 - Kd2*Omega2 + Ki2*Ik2;
    *tau3 = Kp3*error3 - Kd3*Omega3 + Ki3*Ik3;

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
    Simulink_PlotVar4 = theta_desired;

    mycount++;
}

void printing(void){
    if (whattoprint == 0) {
        serial_printf(&SerialA, "%.2f %.2f,%.2f   ",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI);
        serial_printf(&SerialA, "%.2f %.2f,%.2f   ",theta1dh,theta2dh,theta3dh);
        serial_printf(&SerialA, "%.2f %.2f,%.2f   ",printPosXmotor,printPosYmotor,printPosZmotor);
        serial_printf(&SerialA, "%.2f %.2f,%.2f   \n\r",printInvTheta1*180/PI,printInvTheta2*180/PI,printInvTheta3*180/PI);

    } else {
        serial_printf(&SerialA, "Print test   \n\r");
    }
}


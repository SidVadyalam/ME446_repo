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

//float Kp1 = 60;
//float Kd1 = 6;
////float Ki1 = 100;
//float Kp2 = 70;
//float Kd2 = 5;
////float Ki2 = 200;
//float Kp3 = 70;
//float Kd3 = 5;
////float Ki3 = 150;

//float Ik1_old = 0;
//float Ik1 = 0;
//float Ik2_old = 0;
//float Ik2 = 0;
//float Ik3_old = 0;
//float Ik3 = 0;
//
//float thresh_1 = 0.05;
//float thresh_2= 0.05;
//float thresh_3= 0.05;

float tmod = 0;
float theta_dot_desired = 0;
float theta_double_dot_desired = 0;
//
//float J1 = 0.0167;
//float J2 = 0.03;
//float J3 = 0.0128;

float x_fun = 0;
float y_fun = 0;
float z_fun = 0;

//float theta1dh_desired = 0;
//float theta2dh_desired = 0;
//float theta3dh_desired = 0;

float theta1_desired = 0;
float theta2_desired = 0;
float theta3_desired = 0;

float theta1_dot_desired = 0;
float theta1_double_dot_desired = 0;
float theta2_dot_desired = 0;
float theta2_double_dot_desired = 0;
float theta3_dot_desired = 0;
float theta3_double_dot_desired = 0;


// fric comp lab3
float fric_c1 = 0;
float v1pos = 0.15;
float v1neg = 0.235;
float c1pos = 0.3637;
float c1neg = -0.2948;
float slope_between_minimum1 = 3.6; // maybe need to tune

float fric_c2 = 0;
float v2pos = 0.14;
float v2neg = 0.25;
float c2pos = 0.4759;
float c2neg = -0.5031;
float slope_between_minimum2 = 3.6; // maybe need to tune

float fric_c3 = 0;
float v3pos = 0.15;
float v3neg = 0.287;
float c3pos = 0.5339;
float c3neg = -0.5190;
float slope_between_minimum3 = 3.6; // maybe need to tune

//float a_theta_2 = 0;
//float a_theta_3 = 0;
//
//float Kp_outer_2 = 6000;
//float Kd_outer_2 = 250;
//float Kp_outer_3 = 6000;
//float Kd_outer_3 = 250;

//
//float p1 = 0.03;
//float p2 = 0.0128;
//float p3 = 0.0076;
//float p4 = 0.0753;
//float p5 = 0.0298;

//
//float D12 = 0;
//float D21 = 0;
//float C12 = 0;
//float C21 = 0;

float Joint_torque1 = 0;
float Joint_torque2 = 0;
float Joint_torque3 = 0;

//float step = 0;
//float step1 = 0;
//float step3 = 0;


float fric_factor = 0;
//
//int choose_inv_dyn_ctrl = 0;



//typedef struct steptraj_s {
//    long double b[5];
//    long double a[5];
//    long double xk[5];
//    long double yk[5];
//    float qd_old;
//    float qddot_old;
//    int size;
//} steptraj_t;
//
//steptraj_t trajectory = {1.4014129239240780e-08L,5.6056516956963121e-08L,8.4084775435444689e-08L,5.6056516956963121e-08L,1.4014129239240780e-08L,
//                        1.0000000000000000e+00L,-3.9129574678536105e+00L,5.7417135544617519e+00L,-3.7445134885345683e+00L,9.1575762615249467e-01L,
//                        0,0,0,0,0,
//                        0,0,0,0,0,
//                        0,
//                        0,
//                        5};
//
//steptraj_t trajectory2 = {1.4014129239240780e-08L,5.6056516956963121e-08L,8.4084775435444689e-08L,5.6056516956963121e-08L,1.4014129239240780e-08L,
//                        1.0000000000000000e+00L,-3.9129574678536105e+00L,5.7417135544617519e+00L,-3.7445134885345683e+00L,9.1575762615249467e-01L,
//                        0,0,0,0,0,
//                        0,0,0,0,0,
//                        0,
//                        0,
//                        5};
//
//// this function must be called every 1ms.
//void implement_discrete_tf(steptraj_t *traj, float step, float *qd, float *qd_dot, float *qd_ddot) {
//    int i = 0;
//
//    traj->xk[0] = step;
//    traj->yk[0] = traj->b[0]*traj->xk[0];
//    for (i = 1;i<traj->size;i++) {
//        traj->yk[0] = traj->yk[0] + traj->b[i]*traj->xk[i] - traj->a[i]*traj->yk[i];
//    }
//
//    for (i = (traj->size-1);i>0;i--) {
//        traj->xk[i] = traj->xk[i-1];
//        traj->yk[i] = traj->yk[i-1];
//    }
//
//    *qd = traj->yk[0];
//    *qd_dot = (*qd - traj->qd_old)*1000;  //0.001 sample period
//    *qd_ddot = (*qd_dot - traj->qddot_old)*1000;
//
//    traj->qd_old = *qd;
//    traj->qddot_old = *qd_dot;
//}

// to call this function create a variable that steps to the new positions you want to go to, pass this var to step
// pass a reference to your qd variable your qd_dot variable and your qd_double_dot variable
// for example
//  implement_discrete_tf(&trajectory, mystep, &qd, &dot, &ddot);

// to call this function create a variable that steps to the new positions you want to go to, pass this var to step
// pass a reference to your qd variable your qd_dot variable and your qd_double_dot variable
// for example
//  implement_discrete_tf(&trajectory, mystep, &qd, &dot, &ddot);


// Loops Calculation Simplification
//float sintheta2 = 0;
//float costheta3 = 0;


float cosq1 = 0;
float sinq1 = 0;
float cosq2 = 0;
float sinq2 = 0;
float cosq3 = 0;
float sinq3 = 0;
float JT_11 = 0;
float JT_12 = 0;
float JT_13 = 0;
float JT_21 = 0;
float JT_22 = 0;
float JT_23 = 0;
float JT_31 = 0;
float JT_32 = 0;
float JT_33 = 0;
float cosz = 0;
float sinz = 0;
float cosx = 0;
float sinx = 0;
float cosy = 0;
float siny = 0;
float thetaz = 0;
float thetax = 0;
float thetay = 0;
float R11 = 0;
float R12 = 0;
float R13 = 0;
float R21 = 0;
float R22 = 0;
float R23 = 0;
float R31 = 0;
float R32 = 0;
float R33 = 0;
float RT11 = 0;
float RT12 = 0;
float RT13 = 0;
float RT21 = 0;
float RT22 = 0;
float RT23 = 0;
float RT31 = 0;
float RT32 = 0;
float RT33 = 0;


float xd = 0;
float yd = 0;
float zd = 0;
float xd_dot = 0;
float yd_dot = 0;
float zd_dot = 0;


float KPx = 700;
float KDx = 25;
float KPy = 700;
float KDy = 25;
float KPz = 700;
float KDz = 25;


float x_old = 0;
float x_dot_raw = 0;
float x_dot_old1 = 0;
float x_dot_old2 = 0;
float x_dot = 0;


float y_old = 0;
float y_dot_raw = 0;
float y_dot_old1 = 0;
float y_dot_old2 = 0;
float y_dot = 0;


float z_old = 0;
float z_dot_raw = 0;
float z_dot_old1 = 0;
float z_dot_old2 = 0;
float z_dot = 0;

float F_zcmd = 0;
float Joint_torque_FFx = 0;
float Joint_torque_FFy = 0;
float Joint_torque_FFz = 0;

float speed = 0;
float dist = 0;
float time_total = 0;
float curr_time = 0;
float xa = 0;
float xb = 0;
float ya = 0;
float yb = 0;
float za = 0;
float zb = 0;
float tinit = 0;





/* Call helper functions */
void matrixTranspose(float A[][3], float A_trans[][3]);
void matrixVectorMultiplication(float A[][3], float B[][1], float C[][1]);
void matrixMultiplication(float A[][3], float B[][3], float C[][3]);
void rotationMatrix(float theta, float R[3][3], char axis);

/* Define all your matrices as global variables */
// 3x3 matrices for example
float Rzxy[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
float Rzxy_Transpose[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
float JT[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};  // Initialize
float KD[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};    // Initialize
float KP[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};    // Initialize
float KP_R[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};    // Initialize
float KD_R[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};    // Initialize
float KD_R_error_dot[3][1] = {{0}, {0}, {0}};    // Initialize
float KP_R_error[3][1] = {{0}, {0}, {0}};    // Initialize
float sum_gain_terms[3][1] = {{0}, {0}, {0}};
float JT_R[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
float Final_Torques[3][1] = {{0}, {0}, {0}};


float Rx[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};   // Initialize
float Ry[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};   // Initialize
float Rz[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};   // Initialize

// 3x1 vectors for example
float error_1[3][1] = {{0}, {0}, {0}};
float error_dot[3][1] = {{0}, {0}, {0}};    // Initialize





/* Helper functions to make the code concise
This goes outside of main function */

// For matrix-vector multiplication (3x3 matrix multiplied by 3x1 vector)
// This function gets 3x3 matrix (A) and 3x1 vector (B) as arguments then returns their multiplication C = AB
void matrixVectorMultiplication(float A[][3], float B[][1], float C[][1]) {
    // Initialize result to zero
    for(int i=0; i<3; i++) {
        C[i][0] = 0;
    }

    // Perform multiplication
    for(int i=0; i<3; i++) {
        for(int k=0; k<3; k++) {
            C[i][0] += A[i][k]*B[k][0];
        }
    }
}

// For matrix multiplication (3x3 matrix multiplied by 3x3 matrix)
// This function gets 3x3 matrix (A) and 3x3 matrix (B) as arguments then returns their multiplication C = AB
void matrixMultiplication(float A[][3], float B[][3], float C[][3]) {
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            C[i][j] = 0;
            for(int k=0; k<3; k++) {
                C[i][j] += A[i][k]*B[k][j];
            }
        }
    }
}

// For matrix transpose (3x3 matrix)
// This function gets 3x3 matrix (A) as arguments then returns its transpose A_trans
void matrixTranspose(float A[][3], float A_trans[][3]) {
    // Size of A should be 3x3
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            A_trans[i][j] = A[j][i];
        }
    }
}


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


    // Jacobian Transpose
    cosq1 = cos(theta1motor);
    sinq1 = sin(theta1motor);
    cosq2 = cos(theta2motor);
    sinq2 = sin(theta2motor);
    cosq3 = cos(theta3motor);
    sinq3 = sin(theta3motor);
    JT_11 = JT[0][0] = -0.254*sinq1*(cosq3 + sinq2);
    JT_12 = JT[0][1] = 0.254*cosq1*(cosq3 + sinq2);
    JT_13 = JT[0][2] = 0;
    JT_21 = JT[1][0] = 0.254*cosq1*(cosq2 - sinq3);
    JT_22 = JT[1][1] = 0.254*sinq1*(cosq2 - sinq3);
    JT_23 = JT[1][2] = -0.254*(cosq3 + sinq2);
    JT_31 = JT[2][0] = -0.254*cosq1*sinq3;
    JT_32 = JT[2][1] = -0.254*sinq1*sinq3;
    JT_33 = JT[2][2] = -0.254*cosq3;


    // Rotation zxy and its Transpose
    cosz = cos(thetaz);
    sinz = sin(thetaz);
    cosx = cos(thetax);
    sinx = sin(thetax);
    cosy = cos(thetay);


    RT11 = R11 = Rzxy[0][0] = Rzxy_Transpose[0][0] = cosz*cosy-sinz*sinx*siny;
    RT21 = R12 = Rzxy[0][1] = Rzxy_Transpose[1][0] = -sinz*cosx;
    RT31 = R13 = Rzxy[0][2] = Rzxy_Transpose[2][0] = cosz*siny+sinz*sinx*cosy;
    RT12 = R21 = Rzxy[1][0] = Rzxy_Transpose[0][1] = sinz*cosy+cosz*sinx*siny;
    RT22 = R22 = Rzxy[1][1] = Rzxy_Transpose[1][1] = cosz*cosx;
    RT32 = R23 = Rzxy[1][2] = Rzxy_Transpose[2][1] = sinz*siny-cosz*sinx*cosy;
    RT13 = R31 = Rzxy[2][0] = Rzxy_Transpose[0][2] = -cosx*siny;
    RT23 = R32 = Rzxy[2][1] = Rzxy_Transpose[1][2] = sinx;
    RT33 = R33 = Rzxy[2][2] = Rzxy_Transpose[2][2] = cosx*cosy;




    KD[0][0] = KDx;
    KD[1][1] = KDy;
    KD[2][2] = KDz;

    KP[0][0] = KPx;
    KP[1][1] = KPy;
    KP[2][2] = KPz;








    x =  0.254*cos(theta1motor)*(cos(theta3motor) + sin(theta2motor));
    y =  0.254*sin(theta1motor)*(cos(theta3motor) + sin(theta2motor));
    z = 0.254*cos(theta2motor) - 0.254*sin(theta3motor) + 0.254;

////    inv
//    L1 = 0.254;
//    L2 = 0.254;
//    L3 = 0.254;
//
//    theta1dh = atan2(y,x);
//    theta3dh = PI - acos(((L3*L3) + (L2*L2) - ((z-L1)*(z-L1) + (x*x) + (y*y))) /(2*L3*L2));
//    theta2dh = (atan((-z+L1)/(sqrt(x*x + y*y))) - acos( (-(L3*L3) + (L2*L2) + (z-L1)*(z-L1) + (x*x) + (y*y))/(2*L2*(sqrt( (z-L1)*(z-L1) + (x*x) + (y*y) ))) ));
//
//    theoretical_TM1 = theta1dh;
//    theoretical_TM2 = theta2dh + HALFPI;
//    theoretical_TM3 = theta3dh + theta2dh;


    // Compute them trajectories
//
//    // Step Trajectory
//    if (mycount % 4000 == 0){
//        if(step < 0.1){
//            step = PI/6;
//        } else{
//            step = 0.0;
//        }
//    }
//
//implement_discrete_tf(&trajectory, step, &theta_desired, &theta_dot_desired, &theta_double_dot_desired);

    // Trajectory for testing Inv control versus the Feed Forward
    // Step Trajectory
//    if (mycount % 4000 == 0){
//        if(step1 < 0.1){
//            step1 = 0.25;
//            step3 = 0.3;
//        } else{
//            step1 = 0.85;
//            step3 = -0.3;
//        }
//    }


//    // Modified step trajectory for exciting some of the higher order dynamics
//
//    if (mycount%8000 == 4000){
//        step1 = 0.25;
//        step3 = 0.3;
//    }
//    else if (mycount%8000 == 0){
//        step1 = 0.85;
//        step3 = -0.3;
//    }
//
//implement_discrete_tf(&trajectory, step1, &theta1_desired, &theta1_dot_desired, &theta1_double_dot_desired);
//implement_discrete_tf(&trajectory2, step3, &theta3_desired, &theta3_dot_desired, &theta3_double_dot_desired);
//
//


//
//
//// Cubic Trajectory
//    tmod = ((mycount%6000) *0.001);
//    if (mycount%6000 < 1000 ){
//        theta_desired = (PI/2)*(tmod*tmod) - (PI/3)*(tmod)*(tmod)*(tmod);
//        theta_dot_desired = PI*tmod - PI*tmod*tmod;
//        theta_double_dot_desired = PI - 2*PI*tmod;
//    }else if (mycount%6000 < 3000 ){
//        theta_desired = PI/6;
//        theta_dot_desired = 0;
//        theta_double_dot_desired = 0;
//    }else if (mycount%6000 <4000 ){
//        theta_desired = (-13.3333333333)*PI + 12*PI*tmod - (3.5)*PI*tmod*tmod + (PI/3)*tmod*tmod*tmod;
//        theta_dot_desired = 12*PI - (7)*PI*tmod + (PI)*tmod*tmod;
//        theta_double_dot_desired = -7*PI + 2*PI*tmod;
//    }else if (mycount%6000 <6000 ){
//        theta_desired = 0;
//        theta_dot_desired = 0;
//        theta_double_dot_desired = 0;
//    }
//





// Fun trajectory
//    tmod = (mycount*0.001);
//
//    y_fun =  0.1*sin(tmod)*sin(tmod);
//    z_fun =  0.1*(cos(tmod) + sin(tmod)) + 0.25;
//    x_fun = 0.254;
//
//
//    theta1dh_desired = atan2(y_fun,x_fun);
//    theta3dh_desired = PI - acos(((L3*L3) + (L2*L2) - ((z_fun-L1)*(z_fun-L1) + (x_fun*x_fun) + (y_fun*y_fun))) /(2*L3*L2));
//    theta2dh_desired = (atan((-z_fun+L1)/(sqrt(x_fun*x_fun + y_fun*y_fun))) - acos( (-(L3*L3) + (L2*L2) + (z_fun-L1)*(z_fun-L1) + (x_fun*x_fun) + (y_fun*y_fun))/(2*L2*(sqrt( (z_fun-L1)*(z_fun-L1) + (x_fun*x_fun) + (y_fun*y_fun) ))) ));
//
//    theta1_desired = theta1dh_desired;
//    theta2_desired = theta2dh_desired + HALFPI;
//    theta3_desired = theta3dh_desired + theta2dh_desired;

//
//    speed = 0.1;
//    dist = 2*0.254;
//    time_total = dist/speed;
//    xa = 0.254;
//    xb = 0.254;
//    ya = 0.254;
//    yb = -0.254;
//    za = 0.254;
//    zb = 0.254;
//    curr_time = mycount % 1000;
//
//    if((curr_time < 5)){
//        xd = 0 + xa;
//        yd = (yb - ya)*((curr_time - tinit)/time_total) + ya;
//        zd = 0 + za;
//
//    }else{
//        tinit = curr_time;
//        yb *= -1;
//        ya *= -1;
//        xd = 0 + xa;
//        yd = (yb - ya)*((curr_time - tinit)/time_total) + ya;
//        zd = 0 + za;
//    }
//


    xd = 0.254;
    yd = -0.254;
    zd = 0.254;

    xd_dot = 0;
    yd_dot = 0;
    zd_dot = 0;




// Calculate them states

    Omega1_raw = (theta1motor - Theta1_old)/0.001;
    Omega1 = (Omega1_raw + Omega1_old1 + Omega1_old2)/3.0;

    Omega2_raw = (theta2motor - Theta2_old)/0.001;
    Omega2 = (Omega2_raw + Omega2_old1 + Omega2_old2)/3.0;

    Omega3_raw = (theta3motor - Theta3_old)/0.001;
    Omega3 = (Omega3_raw + Omega3_old1 + Omega3_old2)/3.0;


    error1 = theta1_desired - theta1motor;
    error2 = theta1_desired - theta2motor;
    error3 = theta3_desired - theta3motor;


    error_1[0][0] = (xd - x);
    error_1[1][0] = (yd - y);
    error_1[2][0] = (zd - z);

//    Ik1 = Ik1 + ((error1 + error1_old)/2)*0.001;
//    Ik2 = Ik2 + ((error2 + error2_old)/2)*0.001;
//    Ik3 = Ik3 + ((error3 + error3_old)/2)*0.001;


    // Computing end effector velocities

    // This function is called every 1 ms

    // This code should be towards that top of your lab function since it is calculating
    // the angular velocity states.
    x_dot_raw = (x - x_old)/0.001;
    x_dot = (x_dot_raw + x_dot_old1 + x_dot_old2)/3.0;
    // Put the saving of the past states towards the bottom of your lab function
    x_old = x;
    //order matters here. Why??
    x_dot_old2 = x_dot_old1;
    x_dot_old1 = x_dot;



    y_dot_raw = (y - y_old)/0.001;
    y_dot = (y_dot_raw + y_dot_old1 + y_dot_old2)/3.0;
    // Put the saving of the past states towards the bottom of your lab function
    y_old = y;
    //order matters here. Why??
    y_dot_old2 = y_dot_old1;
    y_dot_old1 = y_dot;



    z_dot_raw = (z - z_old)/0.001;
    z_dot = (z_dot_raw + z_dot_old1 + z_dot_old2)/3.0;
    // Put the saving of the past states towards the bottom of your lab function
    z_old = z;
    //order matters here. Why??
    z_dot_old2 = z_dot_old1;
    z_dot_old1 = z_dot;




    error_dot[0][0] = (xd_dot - x_dot);
    error_dot[1][0] = (yd_dot - y_dot);
    error_dot[2][0] = (zd_dot - z_dot);


// Saving Computation time
//    D12 = -p3*sin(theta3motor - theta2motor);
//    D21 = -p3*sin(theta3motor - theta2motor);
//    C12 = -p3*cos(theta3motor - theta2motor) * Omega3;
//    C21 = p3*cos(theta3motor - theta2motor) * Omega2;
//
//
//
//// Outer Loop
//    a_theta_2 = theta1_double_dot_desired + Kp_outer_2 * (error2) + Kd_outer_2 * (theta1_dot_desired - Omega2);
//    a_theta_3 = theta1_double_dot_desired + Kp_outer_3 * (error3) + Kd_outer_3 * (theta1_dot_desired - Omega3);
//
//    sintheta2 = sin(theta2motor);
//    costheta3 = cos(theta3motor);
//

//
//    Joint_torque2 = J2*theta_double_dot_desired + Kp2*error2 + Kd2*(theta_dot_desired - Omega2);
//    Joint_torque3 = J3*theta_double_dot_desired + Kp3*error3 + Kd3*(theta_dot_desired - Omega3);

// Inner Loop
//
//
//    if (choose_inv_dyn_ctrl){
//        Joint_torque2 = p1*a_theta_2 + D12*a_theta_3 + C12*Omega3 - p4*GRAV*sintheta2;
//        Joint_torque3 = p2*a_theta_3 + D21*a_theta_2 + C21*Omega2 - p5*GRAV*costheta3;
//    }else{
//        Joint_torque2 = J2*theta1_double_dot_desired + Kp2*error2 + Kd2*(theta1_dot_desired - Omega2);
//        Joint_torque3 = J3*theta3_double_dot_desired + Kp3*error3 + Kd3*(theta3_dot_desired - Omega3);
//    }
//
//
//
//
//    Joint_torque1 = J1*theta1_double_dot_desired + Kp1*error1 + Kd1*(theta1_dot_desired - Omega1);
//
//


// Friction Comp Trajectory
    if(Omega1 > 0.1){
        fric_c1 = v1pos * Omega1 + c1pos;
    }else if(Omega1 < -0.1){
        fric_c1 = v1neg * Omega1 + c1neg;
    }else{
        fric_c1 = slope_between_minimum1*Omega1;
    }

    if(Omega2 > 0.1){
        fric_c2 = v2pos * Omega2 + c2pos;
    }else if(Omega2 < -0.1){
        fric_c2 = v2neg * Omega2 + c2neg;
    }else{
        fric_c2 = slope_between_minimum2*Omega2;
    }

    if(Omega3 > 0.1){
        fric_c3 = v3pos * Omega3 + c3pos;
    }else if(Omega3 < -0.1){
        fric_c3 = v3neg * Omega3 + c3neg;
    }else{
        fric_c3 = slope_between_minimum3*Omega3;
    }




//// Calculate our Integral Windup Control
//
//    if(fabs(error1) > thresh_1){
//        Ik1 = 0;
////        Ik1_old = 0;
//    }
//
//    if(fabs(error2) > thresh_2){
//           Ik2= 0;
////           Ik2_old = 0;
//       }
//
//    if(fabs(error3) > thresh_3){
//           Ik3 = 0;
////           Ik3_old = 0;
//       }



//    Joint_torque1 = JT_11 * (KPx*(xd - x) + KDx*(xd_dot - x_dot)) + JT_12 * (KPy*(yd - y) + KDy*(yd_dot - y_dot)) + JT_13 * (KPz*(zd - z) + KDz*(zd_dot - z_dot));
//    Joint_torque2 = JT_21 * (KPx*(xd - x) + KDx*(xd_dot - x_dot)) + JT_22 * (KPy*(yd - y) + KDy*(yd_dot - y_dot)) + JT_23 * (KPz*(zd - z) + KDz*(zd_dot - z_dot));
//    Joint_torque3 = JT_31 * (KPx*(xd - x) + KDx*(xd_dot - x_dot)) + JT_32 * (KPy*(yd - y) + KDy*(yd_dot - y_dot)) + JT_33 * (KPz*(zd - z) + KDz*(zd_dot - z_dot));
//
//    // Feed Forward Force
//
//    Joint_torque_FFx = JT_13*(F_zcmd/6);
//    Joint_torque_FFy = JT_23*(F_zcmd/6);
//    Joint_torque_FFz = JT_33*(F_zcmd/6);





    matrixMultiplication(KP, Rzxy_Transpose, KP_R);
    matrixMultiplication(KD, Rzxy_Transpose, KD_R);
    matrixVectorMultiplication(KP_R, error_1, KP_R_error);
    matrixVectorMultiplication(KD_R, error_dot, KD_R_error_dot);
    sum_gain_terms[0][0] = KD_R_error_dot[0][0] + KP_R_error[0][0];
    sum_gain_terms[1][0] = KD_R_error_dot[1][0] + KP_R_error[1][0];
    sum_gain_terms[2][0] = KD_R_error_dot[2][0] + KP_R_error[2][0];
    matrixMultiplication(JT, Rzxy, JT_R);
    matrixVectorMultiplication(JT_R, sum_gain_terms, Final_Torques);




    *tau1 = Final_Torques[0][0] + fric_factor * fric_c1;
    *tau2 = Final_Torques[1][0] + fric_factor * fric_c2;
    *tau3 = Final_Torques[2][0] + fric_factor * fric_c3;




// Saving Past States

    // Update Errors
    error1_old = error1;
    error2_old = error2;
    error3_old = error3;


    // Update Omegas
    Theta1_old = theta1motor;
    //order matters here. Why??
    Omega1_old2 = Omega1_old1;
    Omega1_old1 = Omega1;

    Theta2_old = theta2motor;
    //order matters here. Why??
    Omega2_old2 = Omega2_old1;
    Omega2_old1 = Omega2;

    Theta3_old = theta3motor;
    //order matters here. Why??
    Omega3_old2 = Omega3_old1;
    Omega3_old1 = Omega3;

    // Update our IK values

//    Ik1_old = Ik1;
//    Ik2_old = Ik2;
//    Ik3_old = Ik3;


//Motor torque limitation(Max: 5 Min: -5)

    if (*tau1 > 5){
        *tau1 = 5;
    }
    else if (*tau1 < -5){
        *tau1 = -5;
    }

    if (*tau2 > 5){
        *tau2 = 5;
    }
    else if (*tau2 < -5){
        *tau2 = -5;
    }


    if (*tau3 > 5){
        *tau3 = 5;
    }
    else if (*tau3 < -5){
        *tau3 = -5;
    }


    // save more past states
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
//
//    Simulink_PlotVar1 = theta1_desired;
//    Simulink_PlotVar2 = theta3_desired;
//    Simulink_PlotVar3 = step1;
//    Simulink_PlotVar4 = step3;


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


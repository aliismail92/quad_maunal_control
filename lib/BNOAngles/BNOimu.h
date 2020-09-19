#ifndef BNOimu_h
#define BNOimu_h
#include <Arduino.h>
#include <Wire.h>
#include "math.h"
#include "SparkFun_BNO080_Arduino_Library.h"

class BNOimu{

    private:

        BNO080 myIMU;
        float theta;
        float phi;
        float phi_f;
        float theta_f;
        float psi_f;
        float theta_a;
        float phi_a;
        float theta_old = 0.;
        float phi_old = 0.;
        float theta_G = 0;
        float phi_G = 0;
        float psi_G = 0;
        float delta_phi;
        float delta_theta;
        float delta_psi;
        float Xm;
        float Ym;
        float psi_m;
        float thetaRad;
        float phiRad;
        float psiRad;
        float x_a ;
        float y_a ;
        float z_a ;
        float x_g;
        float y_g;
        float z_g;
        float x_m ;
        float y_m ;
        float z_m ;  
        float dt;
        float quatI, quatJ, quatK, quatReal;
        unsigned long timer;
        unsigned long millisOld_roll = 0;
        unsigned long millisOld_pitch = 0;
        unsigned long millisOld_yaw = 0;

        float rad2deg = 360/(2*3.14159265);
        float deg2rad = 1/rad2deg;

        

    public:

        BNOimu();
        void setup(boolean vector);
        void read_sensor();
        void read_quat();
        float get_roll_eul();
        float get_pitch_eul();
        float get_yaw_eul();
        float get_roll_quat();
        float get_pitch_quat();
        float get_yaw_quat();
        float get_gyro_roll();
    
    };

#endif
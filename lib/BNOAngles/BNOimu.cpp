
#include <Wire.h>
#include "math.h"
#include "BNOimu.h"
#include "SparkFun_BNO080_Arduino_Library.h"


BNOimu::BNOimu(){

}


void BNOimu::setup(boolean vector){
  
  delay(1);
  Wire.begin();
  myIMU.begin();
  myIMU.calibrateAll();
  Wire.setClock(400000);
  delay(5);

  if (vector == true){
    myIMU.enableAccelerometer(5); 
    myIMU.enableGyro(5); 
    myIMU.enableMagnetometer(5); 

  }else{
    //myIMU.enableAccelerometer(5);
    myIMU.enableRotationVector(5);   
  }

  delay(100);

}

void BNOimu::read_sensor(){

  if (myIMU.dataAvailable() == true)
  {
    x_a = myIMU.getAccelX();
    y_a = myIMU.getAccelY();
    z_a = myIMU.getAccelZ();

    x_g = myIMU.getGyroX();
    y_g = myIMU.getGyroY();
    z_g = myIMU.getGyroZ();

    x_m = myIMU.getMagX();
    y_m = myIMU.getMagY();
    z_m = myIMU.getMagZ();

  }

}

float BNOimu::get_gyro_roll(int filter){
  
  float gyro_roll =  y_g * (1 - filter/100) + gyro_prev * (filter/100);
  gyro_prev = y_g;

  return gyro_roll*rad2deg;
}

float BNOimu::get_z_acc(int filter){
  float z_acc = z_a * (1-filter/100) + z_acc_prev * (filter/100);
  z_acc_prev = z_a;

  return z_acc;
}

/* float BNOimu::get_altitude(int dt){
  float vertical_acc = z_a - 9.81 + (9.81 * sin(thetaRad));
  float altitude = vertical_acc * dt * dt * 0.5 + prev_pos_z; 

}*/
float BNOimu::get_roll_eul(){
    
    timer = millis();
    dt = (timer - millisOld_roll)/1000.;
    millisOld_roll = timer;

    delta_phi = -1 * x_g * dt * rad2deg;
    phi_G = phi_G + delta_phi;
    phi = atan2(z_a, y_a) * rad2deg - 90;
    phi_f = (phi_f + delta_phi)*0.95 + phi*0.05;
    phiRad = phi_f * deg2rad;
    
    return phi_f;
}

float BNOimu::get_pitch_eul(){

    timer = millis();
    dt = (timer - millisOld_pitch)/1000.;
    millisOld_pitch = timer;

    delta_theta = y_g * dt * rad2deg;
    theta_G =theta_G + delta_theta;   
    theta = atan2(z_a, x_a) * rad2deg - 90;
    theta_f = (theta_f + delta_theta) * 0.95 + theta * 0.05;
    thetaRad = theta_f * deg2rad;

    return theta_f;
}


float BNOimu::get_yaw_eul(){
    
    timer = millis();
    dt = (timer - millisOld_yaw)/1000.;
    millisOld_yaw = timer;

    delta_psi = z_g * dt * rad2deg;
    psi_G = psi_G +  delta_psi;
    Xm = x_m * cos(thetaRad)-y_m * sin(phiRad) * sin(thetaRad)+z_m * cos(phiRad) * sin(thetaRad);
    Ym = y_m * cos(phiRad) + z_m * sin(phiRad);
    psi_m = atan2(Ym,Xm) * rad2deg;  
    psi_f = (psi_f -  delta_psi) * 0.98 + psi_m * 0.02;
    psiRad = psi_f * deg2rad;

    return psi_f;

}


void BNOimu::read_quat(){
  
   if (myIMU.dataAvailable() == true){ 
      quatI = myIMU.getQuatI();
      quatJ = myIMU.getQuatJ();
      quatK = myIMU.getQuatK();
      quatReal = myIMU.getQuatReal();
/*       Serial.print(quatI);
      Serial.print(" ");
      Serial.println(quatJ); */
      //z_a = myIMU.getAccelZ();
   }
}

float BNOimu::get_roll_quat(){
    
    double sinr_cosp = 2 * (quatReal * quatI + quatJ * quatK);
    double cosr_cosp = 1 - 2 * (quatI * quatI + quatJ * quatJ);
    float roll = atan2(sinr_cosp, cosr_cosp);
    return -1 * roll * rad2deg;
}

float BNOimu::get_pitch_quat(){
    
    double sinp = 2 * (quatReal * quatJ - quatK * quatI);
    float pitch;
    if (abs(sinp) >= 1)
        pitch = copysign(3.141592 / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);
    
    return pitch * rad2deg;
}

float BNOimu::get_yaw_quat(){

    double siny_cosp = 2 * (quatReal * quatK + quatI * quatJ);
    double cosy_cosp = 1 - 2 * (quatJ * quatJ + quatK * quatK);
    float yaw = atan2(siny_cosp, cosy_cosp);
    return yaw * rad2deg;
}

    
      

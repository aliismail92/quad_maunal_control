#include <Arduino.h>
#include "BNOimu.h"
#include <Wifi.h>
#include <PubSubClient.h>
#include <IBusBM.h>

// MQTT credentials

const char* ssid = "LeBgEnD";
const char* password = "ali12345";
const char* mqtt_server = "192.168.1.14";
int count_wifi = 0;
WiFiClient espClient;
PubSubClient client(espClient);

#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];


boolean start_condition = false;

BNOimu imu;
IBusBM IBus;

 
//Eurler Angles
float roll;
float pitch;
float yaw;
float yaw_rate;
float prev_yaw = 0. , prev_pitch = 0., prev_roll = 0.;
float roll_speed;
float z_acc;
int alt_contl = 0;
int counter = 1000;

//command 
float u[4] = {0, 0, 0, 0};
int base = 0;
int cap = 700; // max is 819

//Remote
float remote_val [8] = {1000, 1500, 1500, 1500, 1000, 1000, 1000, 1000};
float angle_des[3] = {0., 0., 0.}; 
int flight_mode = 1000;


//Gains
float kp[] = {0.45, 0.45, 30, 0};//gains for roll, pitch, yaw, altitude {0.12, 0.10, 0.06, 0};
float kd[] = {0.08, 0.08, 0.01, 0};//{0.07, 0.07, 0.02, 0};
float ki[] = {0.00, 0.00, 0.01, 0};//{0.05, 0.08, 0.02, 0};
float prev_ed[]= {0, 0, 0, 0};
float sum_i[] = {0, 0, 0, 0};

//timers
unsigned long timer;
unsigned long prev_timer = 0;
unsigned long counter_timer = 0;
unsigned long mqtt_timer = 0;
unsigned long online_timer = 0;
unsigned long prev_check_imu_timer = 0;

//Define motor PWM
uint32_t freq = 800;
uint32_t resolution = 12;
uint32_t Motor_fr = 0;
uint32_t Motor_fl = 1;
uint32_t Motor_br = 3;
uint32_t Motor_bl = 4;

const int PWM_fr = 4;
const int PWM_fl = 14;
const int PWM_br = 0;
const int PWM_bl = 12;

//custom function

void get_angles();
void get_angles_quat();
void cal_command(float angle, float angle_des, int gain, int dt);
void motor_command(int base);
void send_command(int com_fr, int com_fl, int com_br, int com_bl);
int mapping_oneShot125(int command);
boolean reconnect();
void setup_wifi();
void callback(char* topic, byte* payload, unsigned int length);
void remote_update();
int check_remote_value(int val);
void angle_remote_ctl();
void direct_remote_ctl();
float mapf(float x, float in_min, float in_max, float out_min, float out_max);
void check_imu();

void setup() {
  
  Serial.begin(115200);

  //PWM Setup
  ledcSetup(Motor_fr, freq, resolution);
  ledcSetup(Motor_fl, freq, resolution);
  ledcSetup(Motor_br, freq, resolution);
  ledcSetup(Motor_bl, freq, resolution);
  ledcAttachPin(PWM_fr, Motor_fr);
  ledcAttachPin(PWM_fl, Motor_fl);  
  ledcAttachPin(PWM_br, Motor_br);
  ledcAttachPin(PWM_bl, Motor_bl); 
  send_command(0, 0, 0, 0);

  //Setup IMU
  imu.setup(false);

  //Remot Setup
  IBus.begin(Serial2,IBUSBM_NOTIMER);

  //MQTT Setup
  //setup_wifi();
  //client.setServer(mqtt_server, 1883);
  //client.setCallback(callback);

  delay(500);
}

void loop() {
  // put your main code here, to run repeatedly:
  timer = millis();
  
  //Get remote control updates
  remote_update();

  if (timer - prev_timer >=5 && start_condition){
    
    if (flight_mode == 1000){
      
      //Optional direct remote control no PID
      direct_remote_ctl(); 
    }

    else{
    
      //Get angle update from IMU
      get_angles_quat();

      //Check if IMU is working
      check_imu();

      //Get Remote Command Angle
      angle_remote_ctl();
      
      //Calculate PID correction for each angle
      cal_command(roll, angle_des[0], 0, 5);
      cal_command(pitch, angle_des[1], 1, 5);
      cal_command(yaw_rate, 0, 2, 5);
    
    Serial.print(" ");
    Serial.print(roll);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.println(" ");
/*     Serial.print(yaw_rate);
    Serial.println(" "); */
/*     Serial.print(angle_des[1]);
    Serial.print(" ");
 */
    }
    
    //Send updated command to the motor
    motor_command(base);

    prev_timer = timer;
  }

  else if (start_condition == false){
    send_command(408, 408, 408, 408);
  }
  if (timer - mqtt_timer >= 1000){
    //if(!client.connected()){
      //reconnect();
    //}
    //else {
     // client.loop();
    //}
    mqtt_timer = timer;
  }

  if (timer-online_timer > 60000000){
    send_command(0, 0, 0, 0);
    start_condition = false;
    online_timer = timer;
  }
  
}

//testing commit
void get_angles(){

  imu.read_sensor();
  roll = imu.get_roll_eul() ;
  pitch =  imu.get_pitch_eul();
  yaw = imu.get_yaw_eul();
  roll_speed = imu.get_gyro_roll(70);
  
/*   Serial.print(roll);
  Serial.print(" ");
  Serial.print(roll_speed);
  Serial.println(" "); */
}
 
void get_angles_quat(){

  imu.read_quat();
  roll = imu.get_roll_quat()-1.43 ;
  pitch =  imu.get_pitch_quat()+0.23;
  yaw = imu.get_yaw_quat()*2*3.14159265;
  yaw_rate = yaw - prev_yaw;
  prev_yaw = yaw;
  //Serial.println("Angles are being calculated");
  //z_acc = imu.get_z_acc(90);
  
  //roll_speed = imu.get_gyro_roll(70);
 /*  Serial.print(roll);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.println(" "); */
}

void cal_command(float angle, float angle_des, int gain, int dt){

  float e_p = angle - angle_des;
  float e_d = (e_p - prev_ed[gain])/(dt ) * 1000;
  prev_ed[gain] = prev_ed[gain] * 0.6 + e_p * 0.4;
  float e_i = sum_i[gain] + e_p;
  e_i = constrain(e_i, -100, 100);
  sum_i[gain] = e_i;

  //float e_d = roll_speed;
  //Serial.println(e_d);

  int u_command = kp[gain] * e_p + kd[gain] * e_d + e_i * ki[gain];

/*   Serial.print(kp[gain] * e_p);
  Serial.print(" ");
  Serial.print(kd[gain] * e_d);
  Serial.print(" ");
  Serial.println(e_i * ki[gain]); */
  u[gain] = constrain(u_command, -70, 70);

}

void motor_command(int base){

  int motor_fr = base + u[0] - u[1] - u[2] + u[3];
  int motor_fl = base - u[0] - u[1] + u[2] + u[3];
  int motor_br = base + u[0] + u[1] + u[2] + u[3];
  int motor_bl = base - u[0] + u[1] - u[2] + u[3];

/*   Serial.print(motor_fr);
  Serial.print(" ");
  Serial.print(motor_fl);
  Serial.print(" ");
  
  Serial.print(motor_br);
  Serial.print(" ");
  Serial.print(motor_bl);
  Serial.println(" ");  */

  send_command(motor_fr, motor_fl, motor_br, motor_bl);
}

void send_command(int com_fr, int com_fl, int com_br, int com_bl){

  ledcWrite(Motor_fr, mapping_oneShot125(com_fr));
  ledcWrite(Motor_fl, mapping_oneShot125(com_fl));
  ledcWrite(Motor_br, mapping_oneShot125(com_br));
  ledcWrite(Motor_bl, mapping_oneShot125(com_bl));
}

int mapping_oneShot125(int command){

  int OS125 = constrain(command, 408, cap);
  return OS125;
}

float alt_estimate (int dt){
  float vel = z_acc * dt;
  float alt = vel *dt;
  return alt;
}


void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED && count_wifi <20) {
    delay(500);
    Serial.print(".");
    count_wifi +=1;
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  //Serial.print("Message arrived [");
  //Serial.print(topic);
  String msgs = "";
  String topic_in = String(topic);
  
  for (int i = 0; i < length; i++) {
     msgs = msgs + String ((char)payload[i]);
  }
  //Serial.print("] ");
  if (topic_in == "stop"){
    send_command(0, 0, 0, 0);
    start_condition = false;
    client.publish("status", "stoping");
  }
  else if (topic_in == "alt"){
    alt_contl = msgs.toInt();
    msgs = "set alt " + msgs;
    Serial.println(msgs);
    char msg_sent[msgs.length()+1];
    msgs.toCharArray(msg_sent, msgs.length()+1);
    client.publish("status", msg_sent);
  }
  else if (topic_in == "height"){
    alt_contl = msgs.toInt();
    msgs = "set alt " + msgs;
    Serial.println(msgs);
    char msg_sent[msgs.length()+1];
    msgs.toCharArray(msg_sent, msgs.length()+1);
    client.publish("status", msg_sent);
  }

  else if (topic_in == "rollKp"){
    kp[0] = msgs.toFloat();
    msgs = "setting KpRoll " + msgs;
    char msg_sent[msgs.length()+1];
    msgs.toCharArray(msg_sent, msgs.length()+1);
    client.publish("status", "kp roll chgd");
  } 

  else if (topic_in == "rollKd"){
    kd[0] = msgs.toFloat();
    msgs = "set KdRoll "+ msgs;
    char msg_sent[msgs.length()+1];
    msgs.toCharArray(msg_sent, msgs.length()+1);
    client.publish("status", msg_sent);
  }  

  else if (topic_in == "pitchKp"){
    kp[1] = msgs.toFloat();
    Serial.println(kp[1]);
    msgs = "setting KpPitch " + msgs;
    char msg_sent[msgs.length()+1];
    msgs.toCharArray(msg_sent, msgs.length()+1);
    client.publish("status", msg_sent);
  } 

  else if (topic_in == "pitchKd"){
    kd[1] = msgs.toFloat();
    Serial.println(kd[1]);
    msgs = "setting kdPitch" + msgs;
    char msg_sent[msgs.length()+1];
    msgs.toCharArray(msg_sent, msgs.length()+1);
    client.publish("status", msg_sent);
  }

  else if (topic_in == "start"){
    start_condition = true;
    client.publish("status", "starting");
  }

  else if (topic_in == "base"){
  base = msgs.toInt();
  msgs = "setting base" + msgs;
  char msg_sent[msgs.length()+1];
  msgs.toCharArray(msg_sent, msgs.length()+1);
  client.publish("status", msg_sent);
}    

}

boolean reconnect() {
    // Create a random client ID
    String clientId = "ESP32_123";
    // Attempt to connect
    if (client.connect(clientId.c_str(), "admin", "admin")) {
      //resubscribe
      client.subscribe("all");
      client.subscribe("rollKd");
      client.subscribe("rollKp");
      client.subscribe("pitchKd");
      client.subscribe("pitchKp");
      client.subscribe("stop");
      client.subscribe("start");
      client.subscribe("base");
    } 

    return client.connected();
}

void remote_update(){
  IBus.loop();
  delayMicroseconds(500);
  remote_val[0] = IBus.readChannel(2); //Thrust
  remote_val[1] = check_remote_value(IBus.readChannel(3));//Yaw
  remote_val[2] = check_remote_value(IBus.readChannel(1));//Pitch
  remote_val[3] = check_remote_value(IBus.readChannel(0));//Roll
  remote_val[4] = IBus.readChannel(6);
  remote_val[5] = IBus.readChannel(4);
  remote_val[6] = IBus.readChannel(5);
  remote_val[7] = IBus.readChannel(8);
  int start_condition_value = check_remote_value(IBus.readChannel(7));//Start State
  flight_mode = IBus.readChannel(9);
  
  if (start_condition_value >= 1500){
    start_condition = true;
  }
  else{ start_condition = false;}

  if (remote_val[7] == 2000){

    if (remote_val[4] == 1000){
      kp[0] =  mapf(remote_val[5], 1000, 2000, 0, 2);
      kp[1] =  mapf(remote_val[5], 1000, 2000, 0, 2);
      ki[0] =  mapf(remote_val[6], 1000, 2000, 0, 0.5);
      ki[1] =  mapf(remote_val[6], 1000, 2000, 0, 0.5);
      Serial.print(kp[0]);
      Serial.print(" ");
      Serial.print(kp[1]);
      Serial.print(" ");
      Serial.print(ki[0]);
      Serial.print(" ");
      Serial.print(ki[1]);
      Serial.println(" ");

    }
    else if (remote_val[4] == 3500){
      ki[0] = mapf(remote_val[5], 1000, 2000, 0, 1);
      ki[1] = mapf(remote_val[6], 1000, 2000, 0, 1);
      Serial.print(ki[0]);
      Serial.print(" ");
      Serial.print(ki[1]);
      Serial.println(" ");  
    
    }
    else{
      kp[2] = mapf(remote_val[5], 1000, 2000, 0, 50);
      kd[2] = mapf(remote_val[6], 1000, 2000, 0, 0.2);
      Serial.print(kp[2]);
      Serial.print(" ");
      Serial.print(kd[2]);
      Serial.println(" "); 
    }
  }
}

int check_remote_value(int val){
  if (abs(val-1500) <= 25){
    return 1500;
  }
  else{
    return val;
  }
}

void angle_remote_ctl(){
  base = map(remote_val[0], 1000, 2000, 408, cap);
  angle_des[0] = mapf(remote_val[3], 1000, 2000, 25, -25);
  angle_des[1] = mapf(remote_val[2], 1000, 2000, 25, -25);
  angle_des[2] = mapf(remote_val[1], 1000, 2000, -12, 12);
}

void direct_remote_ctl(){

  base = map(remote_val[0], 1015, 2000, 408, cap);
  u[0] = map(remote_val[3], 1000, 2000, -100, 100);
  u[1] = map(remote_val[2], 1000, 2000, -100, 100);
  u[2] = map(remote_val[1], 1000, 2000, -100, 100);
/*   Serial.print(u[0]);
  Serial.print(" ");
  Serial.print(u[1]);
  Serial.print(" ");
  
  Serial.println(u[2]); */
  

}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void check_imu(){
  if (timer -  prev_check_imu_timer >= 15){
    if ((pitch - prev_pitch)== 0 || (roll - prev_roll)==0){
      send_command(0, 0, 0, 0);
      start_condition = false;
      prev_timer = timer + 1000000;
    }
    prev_check_imu_timer = timer;
    prev_pitch = pitch;
    prev_roll = roll;
  } 

}
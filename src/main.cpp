#include <Arduino.h>
#include "BNOimu.h"
#include <PSX.h>
#include <Wifi.h>
#include <PubSubClient.h>


// MQTT credentials

const char* ssid = "LeBgEnD";
const char* password = "ali12345";
const char* mqtt_server = "192.168.1.12";
WiFiClient espClient;
PubSubClient client(espClient);

#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];


boolean start_condition = false;


BNOimu imu;
PSX psx;

//Controller Param
PSX::PSXDATA PSXdata;
int PSXerror;

#define dataPin   34  
#define cmdPin    35 
#define attPin    32  
#define clockPin  33  

//Eurler Angles
float roll;
float pitch;
float yaw;
float roll_speed;
int counter = 1000;

//command 
float u[4] = {0, 0, 0, 0};
int base = 0;
int cap = 700; // max is 819

//Gains
float kp[] = {0, 0, 0, 0};//gains for roll, pitch, yaw, altitude
float kd[] = {0.5, 0, 0, 0};
float prev_ed[4]= {0, 0, 0, 0};

//timers
unsigned long timer;
unsigned long prev_timer = 0;
unsigned long counter_timer = 0;
unsigned long mqtt_timer = 0;
unsigned long online_timer = 0;

//Define motor PWM
uint32_t freq = 800;
uint32_t resolution = 12;
uint32_t Motor_fr = 0;
uint32_t Motor_fl = 1;
uint32_t Motor_br = 3;
uint32_t Motor_bl = 4;

const int PWM_fr = 16;
const int PWM_fl = 14;
const int PWM_br = 4;
const int PWM_bl = 12;

//custom function

void get_angles();
void cal_command(float angle, float angle_des, int gain, int dt);
void motor_command(int base);
void send_command(int com_fr, int com_fl, int com_br, int com_bl);
int mapping_oneShot125(int command);
boolean reconnect();
void setup_wifi();
void callback(char* topic, byte* payload, unsigned int length);

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


  //MQTT Setup
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  delay(500);
}

void loop() {
  // put your main code here, to run repeatedly:
  timer = millis();
  

  if (timer - prev_timer >=5 && start_condition){
    //Get angle update from IMU
    get_angles();

    //Calculate PID correction for each angle
    cal_command(roll, 0, 0, 5);
    //cal_command(pitch, 0, 1, 5);

/*     Serial.print(roll);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.println(" "); */
    
    //Send updated command to the motor
    //send_command(base, base, base, base);
    motor_command(base);

    prev_timer = timer;
  }
  if (timer - mqtt_timer >= 1000){
    if(!client.connected()){
      reconnect();
    }
    else {
      client.loop();
    }
    mqtt_timer = timer;
  }

  if (timer-online_timer > 600000){
    send_command(0, 0, 0, 0);
    start_condition = false;
    online_timer = timer;
  }
  
}


void get_angles(){

  imu.read_sensor();
  roll = imu.get_roll_eul() + 3;
  pitch =  imu.get_pitch_eul();
  yaw = imu.get_yaw_eul();
  roll_speed = imu.get_gyro_roll();
}

void cal_command(float angle, float angle_des, int gain, int dt){

  float e_p = angle - angle_des;
  //float e_d = (e_p - prev_ed[gain])/(dt ) * 1000;
  float e_d = roll_speed;
  Serial.print(e_d);
  Serial.print(" ");
  Serial.print(kd[gain] * e_d);
  Serial.print(" ");
  prev_ed[gain] = e_p;

  int u_command = kp[gain] * e_p + kd[gain] * e_d;
  u[gain] = constrain(u_command, -200, 200);

}

void motor_command(int base){

  int motor_fr = base + u[0] - u[1] - u[2] + u[3];
  int motor_fl = base - u[0] - u[1] + u[2] + u[3];
  int motor_br = base + u[0] + u[1] - u[2] + u[3];
  int motor_bl = base - u[0] + u[1] + u[2] + u[3];

  Serial.print(motor_fr);
  Serial.print(" ");
  Serial.print(motor_fl);
  Serial.println(" ");
/*   Serial.print(motor_br);
  Serial.print(" ");
  Serial.print(motor_bl);
  Serial.println(" "); */

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


void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
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
  else if (topic_in == "all"){
    u[3] = msgs.toInt();
    Serial.println(u[3]);
    msgs = "set all " + msgs;
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


// Add imu readings
// Add basic filter
// Add controller input
// Add on/off triggers
// Add hover control
// Add visPy 
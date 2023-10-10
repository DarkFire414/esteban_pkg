#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Vector3Stamped.h>
/*
  https://github.com/RoboticsBrno/ServoESP32
*/
#include <Servo.h>

/*
 * Handler for Display Oled 0.96' I2C V1.0
 */
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define PI 3.14159265
#define LOOPTIME 10

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define PWMA 5
#define PWMB 18

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL)
// On an ESP32              21(SDA), 22(SCL)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x323

#define LED_BUILTIN 2
#define SERVO_PIN 23

void messageCb( const geometry_msgs::Twist& twist_msg);
float gammaSaturation(float gammaAngle);
float rad2deg(float degAngle);

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

ros::NodeHandle nh;

geometry_msgs::Twist twist_msg;
geometry_msgs::Vector3Stamped speed_msg;
ros::Publisher speed_pub("speed", &speed_msg);   

float vlx = 0.0;   // Lienar velocity x
float _gamma = 0.0; // Steering angle
float _gamma_radianes = 0.0; // Steering angle

float gammaSaturationValue = PI/6;
float velocitySaturationValueMax = 0.25;
float velocitySaturationValueMin = 0.14;

Servo myservo;

void messageCb( const geometry_msgs::Twist& twist_msg){
  // Callback
  vlx = twist_msg.linear.x;
  if (vlx < 0.03f) {
    vlx = 0;    
  }
  else{
    vlx=0.2f;
  }
  /*
  else if (vlx > velocitySaturationValueMax) {
    vlx = velocitySaturationValueMax;
  }
  else if (vlx < velocitySaturationValueMin) {
    vlx = velocitySaturationValueMin;
  }
*/
  _gamma = twist_msg.linear.z;

  _gamma = gammaSaturation(_gamma);
  _gamma = rad2deg(_gamma) + 90;
  _gamma = 180-(int)_gamma;  
  myservo.write(_gamma); 
  _gamma_radianes = deg2rad(90 - _gamma);

}

/*
  Set steering angle
  gamma: (radians)
*/
float gammaSaturation(float gammaAngle) {
  // Saturation
  if (gammaAngle > gammaSaturationValue) {
    return gammaSaturationValue;
  }
  else if (gammaAngle < -gammaSaturationValue) {
    return -gammaSaturationValue;
  }
  else {
    return gammaAngle;
  }
}

/*
  Radian to degrees conversion
*/
float rad2deg(float degAngle) {
  return degAngle*(180.0/PI);
}
float deg2rad(float degAngle) {
  return degAngle*(PI/180.0);
}


float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    float run = in_max - in_min;
    if(run == 0){
        log_e("map(): Invalid input range, min == max");
        return -1; // AVR returns -1, SAM returns 0
    }
    float rise = out_max - out_min;
    float delta = x - in_min;
    return (delta * rise) / run + out_min;
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &messageCb);

const byte Encoder_C1 = 15; // Cable amarillo pin 3 digital
const byte Encoder_C2 = 0; // Cable verde al pin 4 digital
const byte Encoder2_C1 = 19; // Cable amarillo pin 3 digital
const byte Encoder2_C2 = 0; // Cable verde al pin 4 digital

byte Encoder_C1Last;
int paso = 0;
boolean direccion;

byte Encoder2_C1Last;
int paso2 = 0;
boolean direccion2;

float rpm = 0;
unsigned long timeold = 0;

float rpm2 = 0;
unsigned long timeold2 = 0;

void setup() {
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(speed_pub);  
  //nh.advertise(pub_range);
  
  delay(2000);
  display.clearDisplay();
  display.display();

  myservo.attach(SERVO_PIN);

  attachInterrupt(digitalPinToInterrupt(Encoder_C1), calculapulso, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Encoder2_C1), calculapulso2, CHANGE);

  //pinMode(PWMA, OUTPUT);
  _gamma = 90;
  myservo.write(90);
}

float currentVelocity = 0;
float currentVelocity2 = 0;
void loop() {
      /*Para por cada vuelta del rotor antes de la caja de engranes, se producen 22 pulsos (o pasos) del encoder.
    El motor tiene una relación de reducción en su caja de engranes de 1:34. Por tanto, se tienen 748 ticks o pulsos
    del encoder por cada revolución del rotor después de la caja de engranes. Por lo que 748/360 = 0.48128...*/

  if (millis() - timeold >= 200) {
    rpm = paso* (1.2371134/(millis() - timeold)) * (60000/360);
    //Serial.print("RPM: ");
    //Serial.println(rpm);
    timeold = millis();
    paso = 0;
  }

  if (millis() - timeold2 >= 200) {
    rpm2 = paso2* (1.2371134/(millis() - timeold2)) * (60000/360);
    //Serial.print("RPM: ");
    //Serial.println(rpm);
    timeold2 = millis();
    paso2 = 0;
  }

  currentVelocity = rpm*2*PI/60.0; // rad/s
  currentVelocity = 0.032*currentVelocity;

  currentVelocity2 = rpm2*2*PI/60.0; // rad/s
  currentVelocity2 = 0.032*currentVelocity2;

  float out = vlx;
  float out2 = vlx;   
if(_gamma_radianes < 0){
    out = vlx;  
    out2 = vlx/(PI/2) *((PI/2)+_gamma_radianes);// velocidad IZQUIERDA
 }  
 else{
   out2 = vlx;
   out = vlx/(PI/2)* ((PI/2)-_gamma_radianes);// velocidad derecha
 }
 out = mapf(out, velocitySaturationValueMin, velocitySaturationValueMax, 191, 255);
 out2 = mapf(out2, velocitySaturationValueMin, velocitySaturationValueMax, 191, 255);

  if (vlx < 0.03) {
    out = 0;
    out2 = 0;
  }
  //analogWrite(PWMA, int((75)*255/100));
  analogWrite(PWMA, (int)out);
  analogWrite(PWMB, (int)out2);

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE); // Draw white text
  /*
   * Pixeles
   * size = 1 -> 12
   * size = 2 -> 16
  */
  display.setTextSize(2);
  display.setCursor(0, 0); // Consider a screen of 128x64 pixels
  display.print("vlxB" + String(currentVelocity2));
  display.setTextSize(2);
  display.setCursor(0, 16); // Consider a screen of 128x64 pixels
  display.print("vlxA" + String(currentVelocity));
  display.setTextSize(2);
  display.setCursor(0, 32); // Consider a screen of 128x64 pixels
  display.print("g=" + String(_gamma));
  display.setCursor(0, 48); // Consider a screen of 128x64 pixels
  display.print("vlx=" + String(vlx));  
  display.display(); // Shows the information! must be at the end of statement


  publishSpeed(LOOPTIME);
  nh.spinOnce();
  delay(1);
}
//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = currentVelocity;    //left wheel speed (in m/s)
  speed_msg.vector.y = currentVelocity2;   //right wheel speed (in m/s)
  speed_msg.vector.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
//  nh.loginfo("Publishing odometry");
}
void calculapulso()
{
  int Lstate = digitalRead(Encoder_C1);
  if ((Encoder_C1Last == LOW) && Lstate == HIGH)
  {
    int val = digitalRead(Encoder_C2);
    if (val == LOW && direccion)
    {
      direccion = false; //Reverse
    }
    else if (val == HIGH && !direccion)
    {
      direccion = true;  //Forward
    }
  }
  Encoder_C1Last = Lstate;

  if (!direccion)  paso++;
  else  paso--;
}

void calculapulso2()
{
  int Lstate = digitalRead(Encoder2_C1);
  if ((Encoder2_C1Last == LOW) && Lstate == HIGH)
  {
    int val = digitalRead(Encoder2_C2);
    if (val == LOW && direccion2)
    {
      direccion2 = false; //Reverse
    }
    else if (val == HIGH && !direccion2)
    {
      direccion2 = true;  //Forward
    }
  }
  Encoder2_C1Last = Lstate;

  if (!direccion2)  paso2++;
  else  paso2--;
}

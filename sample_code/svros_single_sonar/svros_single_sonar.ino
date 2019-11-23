/*
 *Process rosserial messages for a Maxbotix Sonar 
 */
#include <SPI.h>
//#include <ArduinoHardware.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/Range.h>

const int fwd_sonar=0;

char sonarframe_fwd[]="/ultrasound_fwd";

ros::NodeHandle  nh;

sensor_msgs::Range range_msg_fwd;

//sensor_msgs::Range range_msg_arr[3];

void initSonar( sensor_msgs::Range range_msg, char* frameid) {

  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.1;  // fake
  range_msg.min_range = 0.0;
  range_msg.max_range = 6.47;
}

ros::Publisher pub_sonar_fwd( "/ultrasound_fwd", &range_msg_fwd);

void setup()
{
  Serial.begin(9600);
  initSonar(range_msg_fwd,sonarframe_fwd);
 
  nh.initNode();
  nh.advertise(pub_sonar_fwd);
  
}

void loop()
{
  
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32]; 
  float sonarval=0.0;
  int num_sonars=1;
  int num_samples=5;
  
   sonarval=ProcSonar(num_samples);
   
   initSonar(range_msg_fwd,sonarframe_fwd);
  
   range_msg_fwd.range = sonarval;
  
  range_msg_fwd.header.stamp = nh.now();
 
   if ( range_msg_fwd.range < 100) {
   
       pub_sonar_fwd.publish(&range_msg_fwd);
   }
  
   nh.spinOnce();

  //Serial.println(sonarval);
   
  delay(100);
}

int ProcSonar(int num_samples) {
     int raw_val=0;    
        for(int j=0;j <num_samples;j++)  {
                 raw_val +=analogRead(fwd_sonar);
        }  
        return raw_val/num_samples;
        //float inchvalue = (254.0/1024.0) *2.0* (raw_val/5);
        // sonar_range[i]=inchvalue;
         
        delay(50);
}

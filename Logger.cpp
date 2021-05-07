#include <SFE_BMP180.h>
#include <Servo.h>
#include <Wire.h>
#include <EEPROM.h>

//#define softwareParachuteDeploy
#define live 

SFE_BMP180 pressure;

double baseline;
float oldAlt;
double acc;
double logIteration;
double oneG;


int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
boolean set_gyro_angles;
 
long acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc;
 
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
float angle_pitch_output, angle_roll_output;

double temp;
bool calibrated;
bool done = false;

void setup_mpu_6050_registers();
void   read_mpu_6050_data();

struct dataBlock{
  float timestamp;
  float alt;
  float acc;
  float P;
};

void setup()
{
  pinMode(2, OUTPUT);
  
  Serial.begin(9600);
  Wire.begin();
  #ifdef live
    Serial.println("Inilizing Pressure Sensor");
  #endif 
  if (pressure.begin()){
    #ifdef live
      Serial.println("BMP180 init success!");
    #endif  
  }
  else
  {
    #ifdef live
      Serial.println("failed BMP180 init!");
    #endif 
    
   while(1){
      delay(100);
      digitalWrite(2,HIGH);
      delay(100);
      digitalWrite(2,LOW);
    }
  }
}
int writtenBytes = 0;
void loop()
{
  if(digitalRead(4) == LOW && calibrated && !done){
    measureIMU();
    if(acc > 3){
 
      double flightStartTime = millis();
      while(true){
        delay(100); // messen mit 10Hz
        if(writtenBytes >= 1022){
          done = true;
          digitalWrite(2,LOW);
          #ifdef live
            Serial.println("EEPROM Full, stoping data logging");
          #endif  
          break;
        }
        
        float P =  getPressure();
        float alt = pressure.altitude(P,baseline);

        float timestamp = (millis() - flightStartTime) / 1000.00;
        
        measureIMU();

        #ifdef live
          Serial.print("Timestamp: ");
          Serial.print(timestamp);
          Serial.print("S   Alttitude: ");
          Serial.print(alt);
          Serial.print("m   Pressure: ");
          Serial.print(P);  
          Serial.print("hpa   Acceleration: ");
          Serial.println(acc);  
        #endif
    
        #ifdef softwareParachuteDeploy
          if((alt - oldAlt) < 0){
            deployParachute();
          }
        #endif
        
        dataBlock data = {
          timestamp,
          alt,
          acc,
          P
        };
        writtenBytes += 16;
        logData(data);
    }
  }
 }

  if(digitalRead(4) == HIGH   && !calibrated){ //change Pin to jumper cable pin!!!
    calibrate();
    digitalWrite(2, HIGH); //Show that board is calibrated!!!
    calibrated = true;
  }

   while(Serial.available() > 0){
    readData();
    flushSerialBuffer();
   }
  
}

void deployParachute(){
  Servo servo; 
  servo.attach(0); //change pin

  digitalWrite(3, LOW); //change pin

  servo.write(0);
}

void calibrate(){
  #ifdef live
    Serial.println();
    Serial.println("Calibrating Pressure Sensor...");
  #endif
  baseline = getPressure();
  #ifdef live
    Serial.print("Baseline Pressure: ");
    Serial.println(baseline);
    Serial.println("Pressure calibration finished!");
    Serial.println();
    Serial.println("Calibrating IMU...");
  #endif
                                                  
  setup_mpu_6050_registers(); 
  
  calibrateIMU();
  #ifdef live
    Serial.println("IMU calibration finished!");
  #endif
}

void logData(dataBlock data){
  float  addr = (logIteration * sizeof(data))+4;
  EEPROM.put(addr,data);
  logIteration++;
  EEPROM.put(0,logIteration);
}

void readData(){
  Serial.println("Timestamp, Altitude, Acceleration, Pressure"); //Change if dataBlock did
  dataBlock data;
  float addr = 4;
  float readPointer;
  int index = 0;
  EEPROM.get(0,readPointer);
  while(readPointer > index){
    EEPROM.get(addr,data);
    sendData(data);
    addr += sizeof(data);
    index++;
  }
}

void sendData(dataBlock data){  //Change if dataBlock did
  Serial.print(data.timestamp);
  Serial.print(",");
  Serial.print(data.alt);
  Serial.print(",");
  Serial.print(data.acc);
  Serial.print(",");
  Serial.println(data.P);
}

void flushSerialBuffer(){
  while(Serial.available() > 0){
    Serial.read();
  }
}

float getPressure()
{
  char status;
  double T,P,p0,a;

  float c = millis();
  Serial.println(c);
  status = pressure.startTemperature();
    Serial.println(millis());
    Serial.println(millis()-c);
  if (status != 0)
  {

    delay(status);

    status = pressure.getTemperature(T);
    if (status != 0)
    {

      status = pressure.startPressure(3);
      if (status != 0)
      {
  
        delay(status);

        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          return(P);
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
}

void calibrateIMU(){
    for (int cal_int = 0; cal_int < 1000 ; cal_int ++){                  
    read_mpu_6050_data(); 
                                               
    gyro_x_cal += gyro_x;

    oneG +=  sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z)); ; 
                                              
    gyro_y_cal += gyro_y; 
                                            
    gyro_z_cal += gyro_z; 
                                            
    delay(3);                                                          
  }
 

  gyro_x_cal /= 1000;                                                 
  gyro_y_cal /= 1000;                                                 
  gyro_z_cal /= 1000;  
  oneG /= 1000;
}

void measureIMU(){
 
  read_mpu_6050_data();   

  gyro_x -= gyro_x_cal;                                                
  gyro_y -= gyro_y_cal;                                                
  gyro_z -= gyro_z_cal;                                                
         
  
  angle_pitch += gyro_x * 0.0000611;  
                              
  angle_roll += gyro_y * 0.0000611; 
                                     
 
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);
              
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               
  

  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z)); 
  acc = acc_total_vector / oneG;
   
 
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296; 
     
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       
  

  angle_pitch_acc -= 0.0;
                                             
  angle_roll_acc -= 0.0;                                               
 
  if(set_gyro_angles){ 
  
                  
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004; 
  
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        
  }
  else{ 
  
                                                             
    angle_pitch = angle_pitch_acc;
                                 
    angle_roll = angle_roll_acc;
                                     
    set_gyro_angles = true;                                            
  }
  

  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1; 

  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1; 
  }

 
void setup_mpu_6050_registers(){
 
  Wire.beginTransmission(0x68); 
                                     
  Wire.write(0x6B);  
                                              
  Wire.write(0x00);
                                               
  Wire.endTransmission(); 
                                              

  
 
  Wire.beginTransmission(0x68); 
                                  
  Wire.write(0x1C);   
                                                
  Wire.write(0x10); 
                                                 
  Wire.endTransmission(); 

  Wire.beginTransmission(0x68);
                                       
  Wire.write(0x1B);
                                                   
  Wire.write(0x08); 
                                                  
  Wire.endTransmission(); 
                                            
}
 
 
void read_mpu_6050_data(){ 
 
                                         
  Wire.beginTransmission(0x68);  
                                    
  Wire.write(0x3B);
                                                  
  Wire.endTransmission(); 
                                 
  Wire.requestFrom(0x68,14);    
                                      
  while(Wire.available() < 14);
  
  // weil Wire.read nur 1 byte ;(
  acc_x = Wire.read()<<8|Wire.read();                                  
  acc_y = Wire.read()<<8|Wire.read();                                  
  acc_z = Wire.read()<<8|Wire.read();                                  
  temp = Wire.read()<<8|Wire.read();                                   
  gyro_x = Wire.read()<<8|Wire.read();                                 
  gyro_y = Wire.read()<<8|Wire.read();                                 
  gyro_z = Wire.read()<<8|Wire.read();
}

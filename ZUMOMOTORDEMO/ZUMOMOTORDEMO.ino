#include <Wire.h>
#include <ZumoShield.h>

#define hs 200// Hoogste snelheid
#define ds 100// Draaisnelheid

#define itter 70 // iteraties om te calibreren
#define CRB_REG_M_2_5GAUSS 0x60// magnetometer offset
#define CRA_REG_M_220HZ    0x1C // magnetometer frequentie
#define afwijking 5
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);
LSM303 compass;

void setup() {
  LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = { -32767, -32767, -32767};
  unsigned char index;

  Serial.begin(9600);

  // Initiate the Wire library and join the I2C bus as a master
  Wire.begin();

  // Initiate LSM303
  compass.init();

  // Enables accelerometer and magnetometer
  compass.enableDefault();

  compass.writeReg(LSM303::CRB_REG_M, CRB_REG_M_2_5GAUSS); // +/- 2.5 gauss sensitivity to hopefully avoid overflow problems
  compass.writeReg(LSM303::CRA_REG_M, CRA_REG_M_220HZ);    // 220 Hz compass update rate

  button.waitForButton();

  Serial.println("starting calibration");

  // To calibrate the magnetometer, the Zumo spins to find the max/min
  // magnetic vectors. This information is used to correct for offsets
  // in the magnetometer data.
  motors.setLeftSpeed(hs);
  motors.setRightSpeed(-hs);

  for (index = 0; index < itter; index ++)
  {
    // Take a reading of the magnetic vector and store it in compass.m
    compass.read();

    running_min.x = min(running_min.x, compass.m.x);
    running_min.y = min(running_min.y, compass.m.y);

    running_max.x = max(running_max.x, compass.m.x);
    running_max.y = max(running_max.y, compass.m.y);

    Serial.println(index);

    delay(50);
  }

  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);

  Serial.print("max.x   ");
  Serial.print(running_max.x);
  Serial.println();
  Serial.print("max.y   ");
  Serial.print(running_max.y);
  Serial.println();
  Serial.print("min.x   ");
  Serial.print(running_min.x);
  Serial.println();
  Serial.print("min.y   ");
  Serial.print(running_min.y);
  Serial.println();

  // Set calibrated values to compass.m_max and compass.m_min
  compass.m_max.x = running_max.x;
  compass.m_max.y = running_max.y;
  compass.m_min.x = running_min.x;
  compass.m_min.y = running_min.y;

}

void loop() {
  // put your main code here, to run repeatedly:
  button.waitForButton();
  attje(145);
  delay(500);
  dng(0);
  attje(20);
  dng(0);
  attje(145);
  dng(1);
  attje(265);
}

int ans(float afstand, float tijd) {
  float snelheid = afstand / tijd;
  float sped = map(snelheid, 0, 32.95, 0, 200);
  return (sped);

}

int msa(int afstand) {
  int  tijd = abs((afstand / 31.5) * 1000);
  Serial.print(tijd);
  return (tijd);

}

void attje(int afstand) {
  if (afstand > 0) {
    motors.setSpeeds(200, 200);
    delay(msa(afstand));
    motors.setSpeeds(0, 0);
  } else {
    motors.setSpeeds(-200, -200);
    delay(msa(afstand));
    motors.setSpeeds(0, 0);
  }

}

void dng(bool richting) {
  float heading, relatieve_richting;
    int snelheid;
    static float doel_richting = gh();
  int klaar = 1;
   if (richting){
    doel_richting = fmod(gh() + 90,360);
    }else{
      doel_richting = fmod(gh() - 90,360);
      }
  while (1) {
    Serial.println("hahaha");
    
   
    heading = gh();
    Serial.println(heading);
    relatieve_richting = rh(heading, doel_richting);
    Serial.print(relatieve_richting);
    if (abs(relatieve_richting) < afwijking) {
      Serial.print("you dumb bitch");
      motors.setSpeeds(0,0);
      klaar = 0;
      return(0);

    } else
      snelheid = hs * relatieve_richting / 180;

    if (snelheid < 0) {
      snelheid -= ds;
    } else {
      snelheid += ds;

    }
    Serial.println(snelheid);
    motors.setSpeeds(snelheid, -snelheid);
    Serial.println("mofo");


  }
}

template <typename T> float heading(LSM303::vector<T> v) {
  float x_scaled = 2.0 * (float) (v.x - compass.m_min.x) / (compass.m_max.x - compass.m_min.x) - 1.0;
  float y_scaled =  2.0*(float)(v.y - compass.m_min.y) / (compass.m_max.y - compass.m_min.y) - 1.0;
  
  float hoek = atan2(y_scaled,x_scaled) * 180 / M_PI;
  if(hoek < 0){
    hoek+=360;
    return hoek;
    }
}

float rh(float heading_from, float heading_to)
{
  Serial.print("Ding: ");
  Serial.println(heading_from);
  Serial.print("Dong: ");
  Serial.println(heading_to);
  float relative_heading = heading_to - heading_from;
  Serial.print("Dell: ");
  Serial.println(relative_heading);
  // constrain to -180 to 180 degree range
  if (relative_heading > 180)
    relative_heading -= 360;
  if (relative_heading < -180)
    relative_heading += 360;

  return relative_heading;
}

// Average 10 vectors to get a better measurement and help smooth out
// the motors' magnetic interference.
float gh()
{
  LSM303::vector<int32_t> avg = {0, 0, 0};

  for(int i = 0; i < 10; i ++)
  {
    compass.read();
    avg.x += compass.m.x;
    avg.y += compass.m.y;
  }
  avg.x /= 10.0;
  avg.y /= 10.0;

  // avg is the average measure of the magnetic vector.
  return heading(avg);
}

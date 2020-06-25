//#include "PCF8574.h"
#include <Wire.h>
#include <ZumoShield.h>
#include <math.h>
#include <VL53L0X.h>
#define hs 200// Hoogste snelheid
#define ds 25// Draaisnelheid


//PCF8574 extra(0x21);
int sensing = 0;
bool richting = false;
volatile int rijd = 0;
int b = 0;
uint32_t cur_t = 0;
int tijd = 0;
int timing = 0;
int klaar  = 0;
int help;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);
//LSM303 compass;
ZumoBuzzer buzzer;
VL53L0X tof1;
VL53L0X tof2;
bool vlag;
long duration, cm;
float left[2];
float right[2];
int cur_r;
int cur_l;
int max_l = 100;
int max_r = 100;
int p = 0;

void setup() {
  Serial.begin(9600);
  // Serial.print("serial started");

  Serial.print("shit zooi");
  // Initiate the Wire library and join the I2C bus as a master
  Wire.begin();
  //extra.begin();
  delay(100);


  cli();
  noInterrupts();

  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 2;// = (16*10^6) / (1*1024) - 1 (must be <65536) 15624
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);



  interrupts();
  sei();
  LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = { -32767, -32767, -32767};
  unsigned char index;


  //    pinMode(4, OUTPUT);
  //    pinMode(11, OUTPUT);
  //    digitalWrite(11, LOW);
  //    digitalWrite(4, LOW);
  //    digitalWrite(11, HIGH);
  //
  //    tof2.setTimeout(500);
  //
  //    while(!tof2.init()){
  //      Serial.print("Tof 2 failed");
  //      while(1);
  //      }
  //       tof2.setAddress(0x30);
  //
  //    digitalWrite(4, HIGH);
  //
  //    tof1.setTimeout(500);
  //    while(!tof1.init()){
  //      Serial.print("Tof 1 failed");
  //      while(1);
  //      }
  //       // tof1.init();
  //        tof1.setAddress(0x31);


  //  compass.init();
  //
  //  // Enables accelerometer and magnetometer
  //  compass.enableDefault();
  //
  //  compass.writeReg(LSM303::CRB_REG_M, CRB_REG_M_2_5GAUSS); // +/- 2.5 gauss sensitivity to hopefully avoid overflow problems
  //  compass.writeReg(LSM303::CRA_REG_M, CRA_REG_M_220HZ);    // 220 Hz compass update rate
  // Serial.println("wacht op knop");
  // button.waitForButton();

  //  Serial.println("starting calibration");
  //
  //  // To calibrate the magnetometer, the Zumo spins to find the max/min
  //  // magnetic vectors. This information is used to correct for offsets
  //  // in the magnetometer data.
  //  motors.setLeftSpeed(hs);
  //  motors.setRightSpeed(-hs);
  //
  //  for (index = 0; index < itter; index ++)
  //  {
  //    // Take a reading of the magnetic vector and store it in compass.m
  //    compass.read();
  //
  //    running_min.x = min(running_min.x, compass.m.x);
  //    running_min.y = min(running_min.y, compass.m.y);
  //
  //    running_max.x = max(running_max.x, compass.m.x);
  //    running_max.y = max(running_max.y, compass.m.y);
  //
  //    Serial.println(index);
  //
  //    delay(50);
  //  }
  //
  //  motors.setLeftSpeed(0);
  //  motors.setRightSpeed(0);
  //
  //  Serial.print("max.x   ");
  //  Serial.print(running_max.x);
  //  Serial.println();
  //  Serial.print("max.y   ");
  //  Serial.print(running_max.y);
  //  Serial.println();
  //  Serial.print("min.x   ");
  //  Serial.print(running_min.x);
  //  Serial.println();
  //  Serial.print("min.y   ");
  //  Serial.print(running_min.y);
  //  Serial.println();
  //
  //  // Set calibrated values to compass.m_max and compass.m_min
  //  compass.m_max.x = running_max.x;
  //  compass.m_max.y = running_max.y;
  //  compass.m_min.x = running_min.x;
  //  compass.m_min.y = running_min.y;
  //  cur_l = max_l;
  //  cur_r = max_r;

  Serial.println("agv ready");
}

void loop() {

  // put your main code here, to run repeatedly:
  Serial.println("begin loop");

  button.waitForButton();
  Serial.println("knop");
  Serial.println(sense(2));
  klaar = 1;
  richting = 0;
  master();
  master();

  Serial.print("Jobs done");

}



int msa(int afstand) {
  int  timemsa = abs((afstand  * 100) / 6.2 );
  Serial.print(timemsa);
  return (timemsa);

}

void attje(int afstand) {
  Serial.println("HELLEP");
  if (klaar == 1) {
    klaar = 0;
    Serial.println("Ik gooi nu een attje");
    if (afstand > 0) {
      cur_l = max_l;
      cur_r  = max_r;
      motors.setSpeeds(cur_l, cur_r);
      tijd = (msa(afstand));
      Serial.println(tijd);

      timing = 1;
      //motors.setSpeeds(0, 0);
    } else {
      cur_l = -max_l;
      cur_r = - max_r;
      motors.setSpeeds(cur_l, cur_r);
      tijd = (msa(afstand));

      timing = 1;
    }
  } else {
    attje(afstand);
  }
}



void rdte() {
  if (klaar || rijd) {
    //Serial.println("rdte");

    klaar = 0;
    sensing = 1;
    // Serial.println("RIJDEN IS WAAR JIJ LULLO");
    if (rijd == 0) {
      motors.setSpeeds(cur_l, cur_r);
    }
    rijd = 1;
    if (richting == 1) {
      if (right[0] >= 8) {
        rijd = 0;
        motors.setSpeeds(0, 0);
        klaar = 1;
        sensing = 0;


        return;

      } else {
        //boom();

        rdte();
      }
    } else {
      if (left[0] >=  20) {
        rijd = 0;
        motors.setSpeeds(0, 0);
        klaar = 1;
        sensing = 0;
        Serial.println("returning");
        return;

      } else {
        // boom();

        rdte();

      }
    }


  } else {
    buzzer.playFrequency(5000, 1, 15);
    rdte();
    delay(500);
  }



}

//void dng(bool richting) {
//  float heading, relatieve_richting;
//  int snelheid;
//  static float doel_richting = gh();
//  int klaar = 1;
//  if (richting) {
//    doel_richting = (fmod(gh() + 90, 360));
//
//  } else {
//    doel_richting = (fmod(gh() - 90, 360));
//
//  }
//  while (1) {
//    Serial.println("hahaha");
//
//
//    heading = gh();
//    Serial.println(heading);
//    relatieve_richting = rh(heading, doel_richting);
//    Serial.print(abs(relatieve_richting));
//    if (abs(relatieve_richting) < afwijking) {
//      Serial.print("you dumb bitch");
//      motors.setSpeeds(0, 0);
//      klaar = 0;
//      return (0);
//
//    } else
//      snelheid = hs * relatieve_richting / 180;
//
//    if (snelheid < 0) {
//      snelheid -= ds ;
//    } else {
//      snelheid += ds ;
//
//    }
//    //Serial.println(snelheid);
//    motors.setSpeeds(snelheid , -snelheid );
//    Serial.println("mofo");
//
//
//  }
//}
float sense(int sensor) {
  //Serial.print("sensing");
  //Serial.println("Start sense");
  int sens;
  int trig;
  float a, b, c;
  switch (sensor) {
    case (0):

      trig = 2;
      sens = 14;

      break;
    case (1):

      trig = 4;
      sens = 15;

      break;
    case (2):
      trig = 5;
      sens = 16;
      break;
    case (3):
      trig = 11;
      sens = 17;
      break;
  }
  //Serial.println("Pins chosen");
  pinMode(trig, OUTPUT);
  //Serial.println("1");
  pinMode(sens, INPUT);
  //Serial.println("2");
  digitalWrite(trig, HIGH);
  //extra.toggle(trig);
  //Serial.println("3");
  delayMicroseconds(500);
  //Serial.println("4");
  a = analogRead(sens);
  //Serial.println("5");
  digitalWrite(trig, LOW);
  //extra.toggle(trig);
  // Serial.println("6");
  delayMicroseconds(500);
  // Serial.println("6");
  b = analogRead(sens);

  c = a - b;
  //Serial.println("8");
  float dist = (2076 / c) / 2 ;
  //  Serial.println(a);
  //  Serial.println(b);
  //  Serial.println(c);
  //Serial.print("Dist:");
  // Serial.println(c);
  // Serial.println(dist);
  //delay(5000);
  return (dist);
}


//template <typename T> float heading(LSM303::vector<T> v) {
//  float x_scaled = 2.0 * (float) (v.x - compass.m_min.x) / (compass.m_max.x - compass.m_min.x) - 1.0;
//  float y_scaled =  2.0 * (float)(v.y - compass.m_min.y) / (compass.m_max.y - compass.m_min.y) - 1.0;
//
//  float hoek = atan2(y_scaled, x_scaled) * 180 / M_PI;
//  if (hoek < 0) {
//    hoek += 360;
//    return hoek;
//  }
//}

//float rh(float heading_from, float heading_to)
//{
//  // Serial.print("Ding: ");
//  //Serial.println(heading_from);
//  //Serial.print("Dong: ");
//  //Serial.println(heading_to);
//  float relative_heading = heading_to - heading_from;
//  //Serial.print("Dell: ");
//  //Serial.println(relative_heading);
//  // constrain to -180 to 180 degree range
//  if (relative_heading > 180)
//    relative_heading -= 360;
//  if (relative_heading < -180)
//    relative_heading += 360;
//
//  return relative_heading;
//}

// Average 10 vectors to get a better measurement and help smooth out
// the motors' magnetic interference.
//float gh()
//{
//  LSM303::vector<int32_t> avg = {0, 0, 0};
//
//  for (int i = 0; i < 25; i ++)
//  {
//    //compass.read();
//    avg.x += compass.m.x;
//    avg.y += compass.m.y;
//  }
//  avg.x /= 25.0;
//  avg.y /= 25.0;
//
//  // avg is the average measure of the magnetic vector.
//  //Serial.print("AVG:");
//  //Serial.println(heading(avg));
//  return heading(avg);
//}
ISR(TIMER1_COMPA_vect) {

  if ( help == 1) {
    timer();

    help = 0 ;

  } else {
    gek();
    help = 1;
    boom;
  }







}

void gek() {
  p++;
  //Serial.println(p);
  if (p > 200) {


    //Serial.println(tof2.readRangeSingleMillimeters());
    //Serial.print(tof1.readRangeSingleMillimeters());
    // if (tof1.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    // Serial.print(tof2.readRangeSingleMillimeters());
    //buzzer.playFrequency(400, 1, 30);
    // Serial.println("gek");
    //Serial.println(sense(2));

    for (int i = 2; i > 0; i --) {
      left[i] = left[i - 1];
      right[i] = right[i - 1];
      //    Serial.print(i);
      //    Serial.print(": ");
      //    Serial.println(left[i]);
    }


    left[0] = sense(0);

    right[0] = sense(1);
    //Serial.println(sense(2));
    //Serial.println(left[0]);
    // Serial.println(right[0]);
    if (rijd) {
      boom();
      //  Serial.println("rijdenactiveren");
      if (richting == true) {
        if (right[0] != 2) {

          // int afw = (((right[7] + right[6] + right[5] + right[4]) - (right[4] + right[2] + right[1] + right[0])) * 10);
          int afw = (right[1] - right[0]) * 5;
          Serial.print("Dit is de afwijking");
          Serial.println(afw);
          if ( afw > 0) {
            motors.setSpeeds(max_l - afw, max_r);
            // buzzer.playFrequency(800, 1, 100);
          } else {
            motors.setSpeeds(max_l, max_r - abs(afw));
            // buzzer.playFrequency(800, 1, 100);

          }

        }

      } else {
        //int afw = (((left[7] + left[6] + left[5] + left[4]) - (left[3] + left[2] + left[1] + left[0])) * 1);
        int afw = (left[1] - left[0]) * 5;
        //Serial.print("Dit is de afwijking");
        //Serial.println(left[0]);
        if (left[0] != 2) {
          if ( afw > 0) {
            // buzzer.playFrequency(800, 1, 100);
            motors.setSpeeds(max_l, max_r - afw);
          } else {
            //buzzer.playFrequency(1600, 1, 100);
            motors.setSpeeds(max_l - abs(afw), max_r);

          }
        }
      }

    }
    p = 0;
  }

}


void draai(int graden) {
  if (klaar == 1) {
    klaar = 0;
    Serial.println("Draaien");

    float timet =  175;
    float in = 90;
    float t = (((timet / in) * graden) );
    Serial.println(t);
    if (t > 0) {
      motors.setSpeeds(max_l, -max_r);
      tijd = t;

      timing = 1;


    } else {
      motors.setSpeeds(-max_l, max_r);
      tijd = abs(t);

      timing = 1;

    }
  } else {
    Serial.println("draaien niet gelukt");
    draai(graden);
  }
}


void boom() {
  if (sense(2) <= 20) {
    vlag = true;
    max_l = 0;
    max_r = 0;
    delay(500);
  } else if (sense(2) >= 20) {
    vlag = false;

  }
  if (vlag == true) {


    max_l = 0;
    max_r = 0;
    rijd = 0;
    motors.setSpeeds(0, 0);

    buzzer.play("L16 cdegreg4");
    delay(2000);
    rijd = 1;
    max_l = 95;
    max_r = 100;

  }
}

void timer() {
  if (timing == 1) {
    cur_t++;
    Serial.println(cur_t);
    //Serial.println(tijd);
    if (cur_t >= tijd) {
      Serial.println("KLAAR");
      motors.setSpeeds(0, 0);
      klaar = 1;
      timing = 0;

      tijd = 0;
      cur_t = 0;

    }
  }
}

void wtk() {
  while (klaar == 0) {
    Serial.println("WTk");
    delay(100);
  }



}

void rdtb() {
  Serial.println("rdtb");
  if (klaar) {
    Serial.println("rdtb begonnen");
    klaar = 0;
    motors.setSpeeds(max_l, max_r);
    while (klaar == 0) {
      Serial.println(sense(0));
      if (sense(0) < 18) {
        buzzer.playFrequency(8000, 10, 15);
        motors.setSpeeds(0, 0);
        klaar = 1;
      }


    }
  } else {
    rdtb();


  }







}


void oversteek() {
  rdtb();
  rdte();
  wtk();
  rdtb();
  wtk();
  rdte();
  wtk();
  attje(15);
  wtk();
  draai(-90);
  wtk();
  attje(10);
  rdtb();






}


void master() {

  rdte();
  wtk();
  attje(15);
  wtk();
  draai(-90);
  wtk();
  rdtb();
  delay(100);
  wtk();
  oversteek();
  wtk();

  rdte();
  wtk();
  draai(85);
  wtk();
  attje(3);
  wtk();
  draai(85);
  wtk();
  rdtb();
  wtk();
  rdte();





}

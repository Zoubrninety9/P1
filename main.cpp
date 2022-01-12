#include "DigitalIn.h"
#include "PinNames.h"
#include "uop_msb.h"
#include <cstdio>
#include <iostream>
using namespace uop_msb;

// Motion Sensor
MotionSensor motion;

//On board LEDs
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
 
//On board switch
DigitalIn BlueButton(USER_BUTTON);
DigitalIn Button1(BTN1_PIN);
DigitalIn Button2(BTN2_PIN);
DigitalIn Button3(BTN3_PIN);
DigitalIn Button4(BTN4_PIN);

//LCD Display
LCD_16X2_DISPLAY disp;

//Buzzer
Buzzer buzz;

//Traffic Lights
DigitalOut traf1RedLED(TRAF_RED1_PIN,1);
DigitalOut traf1YelLED(TRAF_YEL1_PIN);
DigitalOut traf1GrnLED(TRAF_GRN1_PIN);
DigitalInOut traf2RedLED(TRAF_RED2_PIN, PIN_OUTPUT, OpenDrainNoPull, 0);
DigitalInOut traf2YelLED(TRAF_YEL2_PIN, PIN_OUTPUT, OpenDrainNoPull, 1);
DigitalInOut traf2GrnLED(TRAF_GRN2_PIN, PIN_OUTPUT, OpenDrainNoPull, 1);

//Light Levels
AnalogIn ldr(AN_LDR_PIN);

//LCD Backlight
DigitalOut backLight(LCD_BKL_PIN);

//DIP Switches
DIPSwitches dipSwitches;

//Push Buttons
Buttons button;

//Environmental Sensor
EnvSensor env;

int main()
{
    // UNCOMMENT THIS TO TEST YOUR BOARD
    // UOP_MSB_TEST board;
    // board.test();

    //LCD Backlight ON
    backLight = 1;
    // Initial display
    disp.cls();
    disp.locate(0,0);
    disp.printf("MSB v%d", MSB_VER);    
    disp.locate(1, 0);

    // Interrogate Environmental Sensor Driver
    switch (env.getSensorType())
    {
        case EnvSensor::BMP280:
        disp.printf("BMP280\n");
        break;
        case uop_msb::EnvSensor::SPL06_001:
        disp.printf("SPL06_001\n");
        break;
        default:
        disp.printf("ERROR");
    }

    wait_us(2000000); 
    float PThreshold = 45.0;
    float NThreshold = -45.0;
    float Beta; // Note for Project Mates this should be used for task 5.

    while (true) {

        // TEST MEMS SENSORS
        disp.cls();
        disp.locate(0,0);
        disp.printf("Testing MEMS");

        Timer tmr;
        tmr.start();
        long long tprev;
        tprev = tprev - tmr.elapsed_time().count();

        for (uint16_t n = 0; n<20; n++) {  
            long prevgyrx = 0;
            long prevgyry = 0;
            long prevgyrz = 0;
            float OmegaAverageOfgyrx;
            float OmegaAverageOfgyry;
            float OmegaAverageOfgyrz;
            float previntegratedGyrx = 0;
            float previntegratedGyry = 0;
            float previntegratedGyrz = 0;

            // to get the measurements (angular velocity + acceleration)
            Motion_t acc   = motion.getAcceleration();   
            Motion_t gyr   = motion.getGyro(); 

            //Temperature of sensor
            float tempMems = motion.getTemperatureC();  


            // to get delta T
            long long tNow = tmr.elapsed_time().count();
            long long deltaT = tNow - tprev;
            tprev = tNow;
            //printf("T = %lld\n", deltaT);
            long long deltaTs = (deltaT) * (1e-6) ; // to change from uS to Seconds
            
            // finding the avergae of the measured angular velocities
            float newgyrx = (( gyr.x + prevgyrx )/2); // the initial prev measurements are zeros
            float newgyry = (( gyr.y + prevgyry )/2);
            float newgyrz = (( gyr.z + prevgyrz )/2);

            // Subtracting the mean values from the measurements
            OmegaAverageOfgyrx = (gyr.x - newgyrx) ;
            OmegaAverageOfgyry = (gyr.y - newgyry);
            OmegaAverageOfgyrz = (gyr.z - newgyrz);

            // numerically integratting to avoid the drift
            float integratedgyrx = (OmegaAverageOfgyrx * deltaTs); // dtheta = (w "omega" * dt )
            float newgyrx1 = integratedgyrx + previntegratedGyrx;
            float integratedgyry = (OmegaAverageOfgyry * deltaTs); 
            float newgyry1 = integratedgyry + previntegratedGyry;
            float integratedgyrz = (OmegaAverageOfgyrz * deltaTs); 
            float newgyrz1 = integratedgyrz + previntegratedGyrz;
            // integratedgyr x, y, z are the angles called pitch, roll and yaw, (thata).

            // assigning the angular velocities and the integrated values measured and calculated in this loop into (( prevgyr )) and (( previntegratedGyrx, x, y, z)) so they can be used in the next loop in order to keep the integration process running with less errors.
                 prevgyrx = gyr.x;
                 prevgyry = gyr.y;
                 prevgyrz = gyr.z;

            previntegratedGyrx = integratedgyrx;
            previntegratedGyry = integratedgyry;
            previntegratedGyry = integratedgyrz;

            // if the rotation was over the threshold angles, the MSB will sound the buzzer. 
            if ( newgyrx1 > PThreshold) {
                        buzz.playTone("A", Buzzer::MIDDLE_OCTAVE);
                        wait_us(80000);
                        }

                        if (newgyrx1 < 42.0) {
                        buzz.rest();
                        wait_us(80000);
                        }

                        if ( newgyrx1 < NThreshold) {
                        buzz.playTone("A", Buzzer::MIDDLE_OCTAVE);
                        wait_us(80000);
                        }  

                        if (newgyrx1> -42.0) {
                        buzz.rest();
                        wait_us(80000);
                        }
            if ( newgyry1 > PThreshold) {
                        buzz.playTone("A", Buzzer::MIDDLE_OCTAVE);
                        wait_us(80000);
                        }

                        if (newgyry1 < 42.0) {
                        buzz.rest();
                        wait_us(80000);
                        }

                        if (newgyry1 < NThreshold) {
                        buzz.playTone("A", Buzzer::MIDDLE_OCTAVE);
                        wait_us(80000);
                        }  

                        if (newgyry1 > -42.0) {
                        buzz.rest();
                        wait_us(80000);
                        }
                        
            //Display sensor values
            printf("%8.3f,\t%8.3f,\t%8.3f,\t", acc.x , acc.y, acc.z); 
            printf("\n"); 
            printf("%8.3f,\t%8.3f,\t%8.3f,\t", newgyrx1, newgyrz1, newgyry1);
            printf("\n");      
            printf("%8.3f\n",             tempMems); 
            printf("\n");  


            // if Button 3 "A" is pressed it will increase the threshold angle by 1 degrees. However if Button 3 "C" is pressed it will it by one degrees.

            //**** ( Note for Project Mates , the code below shaould also be useful with task 5 of the project as both the 4th and 5th have the same concepts of requirements ) ****//
            if (Button1 == 1 ) {
                    ++PThreshold;
                    ++NThreshold;
                    disp.cls();
                    disp.locate(0,-1);
                    disp.printf("%8.1f", PThreshold);
                    disp.printf("%8.1f", NThreshold);
                    wait_us(500000);
                    disp.cls();
            }
            if (Button3 == 1 ) {
                    --PThreshold;
                    --NThreshold;
                    disp.cls();
                    disp.locate(0, -1);
                    disp.printf("%8.1f", PThreshold);
                    disp.printf("%8.1f", NThreshold);
                    wait_us(500000);
                    disp.cls();
            }      

            // the code below should be used for task 5.
            if (Button2 == 1 ) {
                    ++Beta;
                    disp.cls();
                    disp.locate(1,-1);
                    disp.printf("%8.1f", Beta);
                    wait_us(500000);
                    disp.cls();
            }
            if (Button4 == 1 ) {
                    --Beta;
                    disp.cls();
                    disp.locate(1, -1);
                    disp.printf("%8.1f", Beta);
                    wait_us(500000);
                    disp.cls();
            }   
            wait_us(1000000); 
        }


    // TEST ENV SENSOR
   /*   disp.cls();
        disp.locate(0,0);
        disp.printf("Testing:");
        disp.locate(1,0);
        disp.printf("%s", (MSB_VER == 2) ? "BMP280" : "SPL06_001");
        for (uint16_t n = 0; n < 20; n++) {
            float temp = env.getTemperature();
            float pres = env.getPressure();
            float hum = env.getHumidity();
            float lux = ldr.read();
            printf("T=%.2f, P=%.2f, H=%.2f, L=%.2f\n", temp, pres, hum, lux);   
            wait_us(500000);         
        }

        //Read DIP Switches (if fitted)
        #if MSB_VER != 2
        int u = dipSwitches;
        disp.cls();
        disp.locate(0,0);
        disp.printf("DIP A B C D");
        disp.locate(1,0);
        disp.printf("    %d %d %d %d\n", dipSwitches[0], dipSwitches[1], dipSwitches[2], dipSwitches[3]);
        wait_us(3000000);
        #endif      
         */

    }
}


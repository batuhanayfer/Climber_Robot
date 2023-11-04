#include "mbed.h"
#include "PM2_Libary.h"
//Workshop 2


#define CLIMB_STAIR     0
#define DESCEND_STAIR   1
#define DRIVE_FORWARD   2
#define DRIVE_BACKWARDS 3
#define CELEBRATE       4
#define CLIMB           5
#define DESCEND         6
#define RAISE_ROBOTER   7
#define DETECT_STAIR    8
#define GET_ON_STAIR    9
#define DRIVE_OVER_STAIR 10

// logical variable main task
bool do_execute_main_task = false;  // this variable will be toggled via the user button (blue button) to or not to execute the main task

// user button on nucleo board
Timer user_button_timer;            // create Timer object which we use to check if user button was pressed for a certain time (robust against signal bouncing)
InterruptIn user_button(PC_13);     // create InterruptIn interface object to evaluate user button falling and rising edge (no blocking code in ISR)
void user_button_pressed_fcn();     // custom functions which gets executed when user button gets pressed and released, definition below
void user_button_released_fcn();

// while loop gets executed every main_task_period_ms milliseconds
int main_task_period_ms = 50;   // define main task period time in ms e.g. 50 ms -> main task runns 20 times per second
Timer main_task_timer;          // create Timer object which we use to run the main task every main task period time in ms

// led on nucleo board
//DigitalOut user_led(LED1);      // create DigitalOut object to command user led

// additional Led
DigitalOut extra_led(PB_9);     // create DigitalOut object to command extra led (do add an aditional resistor, e.g. 220...500 Ohm)

// gate Pin 
DigitalOut gate_pin(PB_2);

// mechanical button
DigitalIn mechanical_button(PC_5);  // create DigitalIn object to evaluate extra mechanical button, you need to specify the mode for proper usage, see below

// Sharp GP2Y0A41SK0F, 4-40 cm IR Sensor
float ir_distance_mV = 0.0f;    // define variable to store measurement
AnalogIn ir_analog_in(PC_3);    // create AnalogIn object to read in infrared distance sensor, 0...3.3V are mapped to 0...1

// 78:1, 100:1, ... Metal Gearmotor 20Dx44L mm 12V CB
DigitalOut enable_motors(PB_15);    // create DigitalOut object to enable dc motors

float   pwm_period_s = 0.00005f;    // define pwm period time in seconds and create FastPWM objects to command dc motors
FastPWM pwm_M1(PB_13);              // motor M3 is closed-loop position controlled (angle controlled)
FastPWM pwm_M2(PA_9);               // motor M3 is closed-loop position controlled (angle controlled)
FastPWM pwm_M3(PA_10);              // motor M3 is closed-loop position controlled (angle controlled)

EncoderCounter  encoder_M1(PA_6, PC_7); // create encoder objects to read in the encoder counter values
EncoderCounter  encoder_M2(PB_6, PB_7);
EncoderCounter  encoder_M3(PA_0, PA_1);

// create SpeedController and PositionController objects, default parametrization is for 78.125:1 gear box
float max_voltage = 12.0f;                  // define maximum voltage of battery packs, adjust this to 6.0f V if you only use one batterypack
float counts_per_turn = 20.0f * 78.125f;    // define counts per turn at gearbox end: counts/turn * gearratio
float kn = 180.0f / 12.0f;                  // define motor constant in rpm per V
float k_gear = 100.0f / 78.125f;            // define additional ratio in case you are using a dc motor with a different gear box, e.g. 100:1
float kp = 0.1f;                            // define custom kp, this is the default speed controller gain for gear box 78.125:1


float max_speed_rps = kn * max_voltage;                 // define maximum speed that the position controller is changig the speed, has to be smaller or equal to kn * max_voltage
//PositionController Relais_Motor(counts_per_turn * k_gear, kn / k_gear, kp * k_gear, max_voltage, pwm_M3, encoder_M3); // parameters adjusted to 100:1 gear, we need a different speed controller gain here


PositionController Stelzenmotor_hinten(counts_per_turn * k_gear, kn / k_gear, kp * k_gear, max_voltage, pwm_M1, encoder_M1); // parameters adjusted to 100:1 gear, we need a different speed controller gain here
PositionController Radmotor(counts_per_turn * k_gear, kn / k_gear, kp * k_gear, max_voltage, pwm_M2, encoder_M2); // parameters adjusted to 100:1 gear, we need a different speed controller gain here
PositionController Relais_Motor(counts_per_turn * k_gear, kn / k_gear, kp * k_gear, max_voltage, pwm_M3, encoder_M3); // parameters adjusted to 100:1 gear, we need a different speed controller gain here

int loops_per_seconds = static_cast<int>(ceilf(1.0f/(0.001f*(float)main_task_period_ms))); // define loops per second
float getIrDistance_mm(void)
{
    float a = 1.22e+04f;
    float b = 0.0002677f;
    float c = 1.112f;
    return((a / (1.0e3f * ir_analog_in.read() * 3.3f)) + (b *(1.0e3f * ir_analog_in.read() * 3.3f)) - (c));
}



int main()
{
    // attach button fall and rise functions to user button object
    user_button.fall(&user_button_pressed_fcn);
    user_button.rise(&user_button_released_fcn);

    // start timer
    main_task_timer.start();

    // set pullup mode: add resistor between pin and 3.3 V, so that there is a defined potential
    mechanical_button.mode(PullUp);

    // enable hardwaredriver dc motors: 0 -> disabled, 1 -> enabled
    enable_motors = 1;


    int state = CLIMB, subState = DRIVE_FORWARD;
    float ir_distance_mm;
  
    float pi = 3.14159265359f;
    float radiusTrack_mm = 10.0f,//werte angepasst
          radiusWheel_mm = 9.5f,//werte angepasst
          radiusGear_mm = 5.0f,//werte angepasst
          tableLength_mm = 200.0f,//werte noch anpassen
          stepHeight_mm = 110.0f,
          StelzeHintenCurrentRotation = 0.0f,
          RadmotorCurrentRotation = 0.0f,
          RaupenmotorCurrentRotation = 0.0f,
          StelzeVorneCurrentRotation = 0.0f,
          rotationsNeeded,rotationsStelzeNeeded,
          rotationDifference = 0,
          tollerance = 0.01;
    bool heightFlag = false, baseDone = false;
    float speed = 1.0f;
    while (mechanical_button.read()) 
    {
        rotationsNeeded = ((stepHeight_mm - 20.0f) / (2*pi*radiusGear_mm));//werte vieleicht noch anpassen
        Stelzenmotor_hinten.setDesiredRotation(rotationsNeeded, 0.5f);   //Stelzenmotor hinten
        
        gate_pin = 1; //enable Stelze vorne
        thread_sleep_for(15);
        Relais_Motor.setDesiredRotation(-(rotationsNeeded), 0.5f); //Stelzenmotor vorne
    }
    Stelzenmotor_hinten.setDesiredRotation(Stelzenmotor_hinten.getRotation(), 0.0f); // Stelzenmotor hinten stoppen
    Relais_Motor.setDesiredRotation(Relais_Motor.getRotation(), 0.0f); // Stelzenmotor vorne stoppen
    while(false)
    {
        Radmotor.setDesiredRotation( + 100.0f, speed); //Radmotor
    }
    while (true) // this loop will run forever
    { 
        main_task_timer.reset();

        if (do_execute_main_task) 
        {
            
            if (state == CLIMB) 
            {
                switch (subState) 
                {
                case DRIVE_FORWARD:
                if(baseDone)
                {
                    rotationsNeeded =((tableLength_mm/6) / (2*pi*radiusTrack_mm)) + RaupenmotorCurrentRotation;//werte noch anpassen

                }
                else 
                {
                    rotationsNeeded =((tableLength_mm/2) / (2*pi*radiusTrack_mm)+RaupenmotorCurrentRotation);//werte noch anpassen
                }
                    
                    gate_pin = 0; //enable raupe
                    thread_sleep_for(15);
                    Relais_Motor.setDesiredRotation(rotationsNeeded + RaupenmotorCurrentRotation,speed);// Raupenmotor
                    if (Relais_Motor.getRotation() >= (rotationsNeeded - tollerance)) 
                    {
                        RaupenmotorCurrentRotation = Relais_Motor.getRotation();// Motoren noch anpassen
                        Relais_Motor.setDesiredRotation(RaupenmotorCurrentRotation, 0.0f);
                        gate_pin = 1; //enable Stelze vorne
                        thread_sleep_for(15);
                        StelzeVorneCurrentRotation = Relais_Motor.getRotation();
                        subState = RAISE_ROBOTER;
                    }                
                break;
                case RAISE_ROBOTER:
                    rotationsNeeded = ((stepHeight_mm - 20.0f) / (2*pi*radiusGear_mm));//werte vieleicht noch anpassen        
                    Stelzenmotor_hinten.setDesiredRotation(-100, speed);   //Stelzenmotor hinten
                    Relais_Motor.setDesiredRotation(100, speed); //Stelzenmotor vorne
                    
                    if(getIrDistance_mm() >= 9)
                    {
                        Stelzenmotor_hinten.setDesiredRotation(Stelzenmotor_hinten.getRotation(), 0.0f); // Stelzenmotor hinten stoppen
                        Relais_Motor.setDesiredRotation(Relais_Motor.getRotation(), 0.0f); // Stelzenmotor vorne stoppen
                        subState = DETECT_STAIR;
                    }

                break;
                case DETECT_STAIR:
                    Radmotor.setDesiredRotation( 100.0f,speed); //Radmotor
                    if (mechanical_button.read()) 
                    { // Wenn stufe erkannt
                        RadmotorCurrentRotation = Radmotor.getRotation();
                        Radmotor.setDesiredRotation(RadmotorCurrentRotation, 0.0f); // Radmotor stoppen
                        gate_pin = 1; //enable Stelze vorne
                        thread_sleep_for(15);
                        Relais_Motor.setDesiredRotation(100, speed); //Stelzenmotor vorne
                        Stelzenmotor_hinten.setDesiredRotation(-3.3f, speed);   //Stelzenmotor hinten                       
                        RadmotorCurrentRotation = Radmotor.getRotation();
                        subState = GET_ON_STAIR;
                    }
                    else if (Radmotor.getRotation() >= ((70.0f / (2*pi*radiusWheel_mm) + RadmotorCurrentRotation - tollerance))) 
                    { 
                        // wenn keine Stufe erkannt
                        gate_pin = 1; //enable Stelze vorne
                        thread_sleep_for(15);
                        Radmotor.setDesiredRotation(Radmotor.getRotation(), 0);                     
                        subState = CELEBRATE;
                    }
                break;
                case GET_ON_STAIR:
                
                    if(Stelzenmotor_hinten.getRotation() <= -3.4f)
                    {
                        Stelzenmotor_hinten.setDesiredRotation(Stelzenmotor_hinten.getRotation(), 0.0f);
                    }
                    if(((getIrDistance_mm() >= 13.1f) || heightFlag))
                    {
                       if(!heightFlag)
                        {
                            rotationDifference = Relais_Motor.getRotation() - StelzeVorneCurrentRotation;
                            rotationsStelzeNeeded = Relais_Motor.getRotation() - rotationDifference;
                        }
                        heightFlag = true;
                        Stelzenmotor_hinten.setDesiredRotation(Stelzenmotor_hinten.getRotation(), 0.0f); // Stelzenmotor hinten stoppen
                        Relais_Motor.setDesiredRotation(Relais_Motor.getRotation(), 0.0f); // Stelzenmotor vorne stoppen
                        rotationsNeeded = ((30.0f)/(2*pi*radiusWheel_mm));
                        Radmotor.setDesiredRotation(RadmotorCurrentRotation+ rotationsNeeded, speed); //Radmotor
                        if ((Radmotor.getRotation() >= (rotationsNeeded + RadmotorCurrentRotation - tollerance))) 
                        {
                            Radmotor.setDesiredRotation(Radmotor.getRotation(), 0.0f); //Radmotor stoppen                            
                            Relais_Motor.setDesiredRotation(rotationsStelzeNeeded, speed); //Stelzenmotor vorne einfahren
                            if(Relais_Motor.getRotation() <= (rotationsStelzeNeeded + tollerance))
                            {
                                gate_pin = 0; //enable raupe
                                thread_sleep_for(15);
                                Relais_Motor.setDesiredRotation(100.0f+Relais_Motor.getRotation(), speed);// Raupenmotor
                                Radmotor.setDesiredRotation(100.0f, speed * (radiusTrack_mm / radiusWheel_mm)); // Radmotor
                                RaupenmotorCurrentRotation = Relais_Motor.getRotation();
                                subState = DRIVE_OVER_STAIR;
                                thread_sleep_for(15);
                                heightFlag = false;
                            }                        
                        }
                    }
                break;
                case DRIVE_OVER_STAIR: 
                    gate_pin = 0; //enable raupe
                    if((getIrDistance_mm() <= 5.0f) && ((Relais_Motor.getRotation() - RaupenmotorCurrentRotation) >= 1.9f))
                    {
                        gate_pin = 0; //enable raupe
                        thread_sleep_for(15);
                        RaupenmotorCurrentRotation = Relais_Motor.getRotation();                      
                        Relais_Motor.setDesiredRotation(Relais_Motor.getRotation(), 0.0f);// Raupenmotor stoppen
                        Radmotor.setDesiredRotation(Radmotor.getRotation(), 0.0f); // Radmotor stoppen
                        RadmotorCurrentRotation = Radmotor.getRotation();
                        Stelzenmotor_hinten.setDesiredRotation(-0.1, speed); //Stelze hinten einfahren
                    }
                    if (Stelzenmotor_hinten.getRotation() >= (-0.1-tollerance)) 
                    {
                        Stelzenmotor_hinten.setDesiredRotation(Stelzenmotor_hinten.getRotation(), 0.0f); //Stelze hinten stoppen
                        RaupenmotorCurrentRotation = Relais_Motor.getRotation();
                        baseDone = true;
                        subState = DRIVE_FORWARD;
                    }
                break;
                case CELEBRATE:
                    //celebrate code here
                    return 1; // end of programm
                break;
                }
            }
            // Code für Treppe runtersteigen hier
            else if (state == DESCEND) 
            {
                switch (subState) 
                {
                case DRIVE_BACKWARDS:

                break;
                case DESCEND_STAIR:

                break;
                case CELEBRATE:
                    // celebrate code here
                    return 1; // end of programm
                break;
                }
            }

        } 
        int main_task_elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(main_task_timer.elapsed_time()).count();
        thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

void user_button_pressed_fcn()
{
    user_button_timer.start();
    user_button_timer.reset();
}

void user_button_released_fcn()
{
    // read timer and toggle do_execute_main_task if the button was pressed longer than the below specified time
    int user_button_elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(user_button_timer.elapsed_time()).count();
    user_button_timer.stop();
    if (user_button_elapsed_time_ms > 200) 
    {
        do_execute_main_task = !do_execute_main_task;
    }
}
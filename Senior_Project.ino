/**
 * @file Sensor_Node.ino
 * @author Miles Kuhn (kuhn0192@d.umn.edu)
 * @brief
 * @version 0.1
 * @date 2024-04-02
 *
 * @copyright Copyright (c) 2024
 *
 */

//------------------------------------------------
// Includes
//------------------------------------------------

#include "Senior_Project.h"

//The following preprocessor statement defines whether or not 
//The microcontroller should wait (in the loop section) before collecting more data
//WAIT 1 means waiting and WAIT 0 means the loop will take readings and transmit without pause
#define WAIT 1

//------------------------------------------------
// State Diagram
// Init
//      Setup: Notecard, I2C and Mux
//      Set the addresses to the sensors (In memory, NOT I2C at this stage )
// Loop
//      Collect Sensor Data
//      Send data over LoRa to gateway (or to a cell tower if LTE notecard is used)
//------------------------------------------------

//total time for each loop, takes into account times of actions
const uint32_t total_loop_minutes = 1; //thus each loop takes 30 minutes

//used to track the total loop time (so it restarts after 30 min)
unsigned long loop_start_time;

//instantiation of each of the sensors (they are constructed with no parameters)
PHSensor pHSensor;
DissolvedOxygenSensor dissolvedOxygenSensor;
TemperatureSensor temperatureSensor;
ConductivitySensor conductivitySensor;

void setup() {
    // --- Setup Console Debugging, the Notecard and I2C communication
    pinMode(indicator_pin, OUTPUT);

    console_debug.begin(9600); //beign debugging at 9600 baud - check serial output at this frequency
    Wire.begin(); // start I2C communication
    notecard.begin(); //start Notecard
    qwiic.begin(); //start Mux

    delay(1000);

    j_handler.setup_notecard(); 
    j_handler.setup_data_template();

    delay(1000);

    // -------
    
    //the sensors are setup using polymorphism (see the class Sensor for a better explanation)
    //each of the sensor addresses (in memory not I2C) must be set in the array
    sensor_array[0] = &temperatureSensor; 
    sensor_array[1] = &conductivitySensor;
    sensor_array[2] = &dissolvedOxygenSensor; 
    sensor_array[3] = &pHSensor; 

    //set the strings used in data transmission here (it doesn't work in the constructor)
    //it is IMPORTANT that all the strings match up between the software here and Datacake
    //edit this with caution
    for(Sensor* sensor: sensor_array) {
        strcpy(sensor->mean_str_, sensor->ezo_board.get_name()); 
        strcat(sensor->mean_str_, "_mean"); 
        
        strcpy(sensor->std_dev_str_, sensor->ezo_board.get_name()); 
        strcat(sensor->std_dev_str_, "_std_dev"); 
    }

    //set all the sensors to sleep initially
    for(Sensor* sensor: sensor_array) {
        sensor->sleep_command();
    }

    //put "all" the isolated carrier boards into low power mode
    j_handler.set_all_pins_low();
};

void loop() {

    loop_start_time = millis();

    //---COLLECT DATA---
    collectSensorData();  

    //---SEND DATA---
    j_handler.send_data(); 

    //--- WAIT ---
    
    // useful if want to perform action while waiting
    #if WAIT && N_DEBUG && LOOP_WAIT_ACTION
        while( !time_has_elapsed(loop_start_time, miliseconds_per_minute*total_loop_minutes) ) {
            
        }
    #elif WAIT
        //this is more optimized as there are fewer calls per thirty minutes - the wait time is precomputed   
        //this optimization is extremely minimal 
        delay(miliseconds_per_minute*total_loop_minutes - (millis() - loop_start_time));  
    #endif

};

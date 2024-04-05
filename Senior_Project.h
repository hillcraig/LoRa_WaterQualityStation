#include "Notecard.h"
#include <time.h>
#include <Wire.h>
#include <Server.h>
#include <array>
#include <Servo.h>
#include "Ezo_i2c.h"
#include <SparkFun_I2C_Mux_Arduino_Library.h>

#define console_debug Serial

#define N_DEBUG 1
#define LOOP_WAIT_ACTION 0

//The productUID defines the interaction with notehub - this must be added as parameter
//To hub.set upon notecard initialization (for the first time)
#define productUID "edu.umn.d.kuhn0192:lorawan"

//self explanatory constants
const int miliseconds_per_second = 1000;
const int miliseconds_per_minute = 60*miliseconds_per_second; 

//hexadecimal address of the mux (for I2C communication)
const int MUX_ADDR = 0x70;
//pin for the indicator light on the swan
const int indicator_pin = 10;

//Note: this is the default number of samples per sensor
//This can be overriden in the constructor if needed
const int SENSOR_SAMPLES = 10; 

// ----------- MUX PORTS ------------------

//these are the current ports that each sensor is 
//physically attached to on the MUX
//these will vary for each sensor station
const int PHSENSOR_PORT = 1;
const int TEMPERATURE_PORT = 2;
const int CONDUCTIVITY_PORT = 3; 
const int DISSOLVED_OXYGEN_PORT = 6; 
// -----------------------------------------


//These are the AUX pins on the notecarrier board
const int PHSENSOR_PIN = 1;
const int CONDUCTIVITY_PIN = 3;
const int DISSOLVED_OXYGEN_PIN = 2;
//

Notecard notecard;
QWIICMUX qwiic;  

class Sensor; //forward declaration of the class so the array can be declared
struct J_Handler; //forward declaration of the struct

//array containing each of the sensors
Sensor *sensor_array[4];

// checks whether or not the current time has surpased the 
// the duration specifed based on a starting time 
// safe against rollover!
bool time_has_elapsed(uint32_t start_time, uint32_t duration) {
    //this is safe against roll over because the type of each is an unsigned int
    return millis() - start_time > duration; 
}

//----- Stats -----

// Returns the average of the array arr of size N
float get_average(const float arr[], int N) {
    float sum = 0;
    for (int i = 0; i < N; ++i) {
        sum += arr[i];
    }
    return sum / N;
};

// Returns the standard deviation of the array arr of size N
float get_standard_deviation(const float arr[], int N) {
    float sum = 0;
    float mean = get_average(arr, N);

    for (int i = 0; i < N; ++i) {
        sum += (arr[i] - mean) * (arr[i] - mean);
    }

    return sqrt(sum / (N - 1));
};

//definition of the possible sensor reading return types 
//the sensor can either return a normal reading or there was 
//a problem with the communication between the controller
//and the sensor - in which case an error reading is returned
enum Sensor_Reading {
    NORMAL,
    SENSOR_ERROR,
};

//default temperature of the water set to 25 degrees centigrade
float global_temperature_average = 25; 

/*
* Super class Sensor overriden by the Temperature, Dissolved Oxygen, PH and Conductivity classes
* Each type derived from the Sensor super class inheret all the fields and functions
* Each class may override the functionality of the super class for unique behavior
* The temperature class in particular overrides several of the parent Sensor class
*
*/
class Sensor {
    public:

        int address = 0; //address refers to the I2C address of the ezoboard =
        int port = 0; //port refers to the physical mux port on the mux board
        int pin = 0;
        int number_of_samples; //number of samples to take during each reading

        float average = 0.0; //average of the N measurements
        float standard_deviation = 0.0; //standard deviation of the N measurements

        //example temperature_mean
        char mean_str_[50]; //mean_string used in sending data to datacake 
        //example temperature_std_dev
        char std_dev_str_[50];

        Ezo_board& ezo_board;

        //only the temperature sensor should override this method
        virtual void update_global_temp() {}

        //opens the mux port to communicate with an ezo board (of the Sensor calling the function)
        //and puts the ezo board in sleep mode - the sensor is immediately awakend by another command
        virtual void sleep_command() {
            qwiic.enablePort(port);
            ezo_board.send_cmd("Sleep");
            qwiic.disablePort(port);
        }

        //this is the default send_read_command used by most of the sensors
        //the temperature sensor is the exception - the other sensors are dependent on the reading 
        virtual void send_read_command() {
            ezo_board.send_read_with_temp_comp(global_temperature_average);
        }
        
        virtual Sensor_Reading take_reading() {
            //Note: remeber that the port and ezo_board is unique to the specific sensor calling the functions
            //open mux port to communicate with the associated ezo board
            qwiic.enablePort(port);
            
            for (int i = 0; i < number_of_samples; ++i) {
                //read from the sensor probe
                send_read_command();
                //wait to ensure the command has had time to be recieved
                delay(1000);

                //ensure that the value read from the sensor is valid (wasn't corrupted)
                if(ezo_board.receive_read_cmd() == Ezo_board::NOT_READ_CMD) {
                    #if N_DEBUG
                        console_debug.println("READING ERROR!");
                    #endif
                    return Sensor_Reading::SENSOR_ERROR;
                } else {
                    //if the data was not corrupted, update the data array with the reading
                    data[i] = ezo_board.get_last_received_reading(); 
                }                                         
            
            }

            //after all the readings have been taken put the ezo board back to sleep (the mux port is still enabled at this point)
            ezo_board.send_cmd("Sleep");

            //close communication with the sensor
            qwiic.disablePort(port);

            //calculate the mean and standard deviation of the readings 
            average = get_average(data, number_of_samples);
            standard_deviation = get_standard_deviation(data, number_of_samples); 

            //this is called by each sensor but only the temp sensor overrides this function and 
            //updates the global temperature
            //MOST OF THE TIME THIS WILL DO NOTHING
            update_global_temp(); 

            #if N_DEBUG
                console_debug.print(mean_str_); 
                console_debug.print(" ");
                console_debug.println(average);
                
                console_debug.print( std_dev_str_);
                console_debug.print(" ");
                console_debug.println(standard_deviation);
            #endif

            return Sensor_Reading::NORMAL;
        };
        
        //stores the data taken from the sensors during a reading
        //it is allocated on the heap so an arbitrary number of samples 
        //can be taken per sensor
        float* data;

        //a sensor must be initialized with params
        Sensor() = delete; 

        /*
        * Construtor used to instantiate Sensor derived objects
        * num_of_samples_param defines the number of samples to be taken during each sensor reading - the default is SENSOR_SAMPLES
        * ezo_board_param defines the specific ezo board for a sensor; 
        * example: Ezo_board ph_board = Ezo_board(99, name); this defines a new ezo_board with I2C address 99 and a name 
        */
        //NOTE: dont try and put debug statements in the constructor
        Sensor(int num_of_samples_param, Ezo_board& ezo_board_param, int mux_port, int pin_param) :
            ezo_board{ezo_board_param}, port(mux_port), number_of_samples(num_of_samples_param), pin(pin_param) {

            address = ezo_board_param.get_address(); //here the address of the board is set to a class member  
            data = static_cast<float*>(calloc(number_of_samples, sizeof(float))); //here the space for the data is allocated on the heap
        };

        //when a sensor object is destroyed the memory is freed and the pointer to the data set to NULL
        ~Sensor() {
            delete[] data;
            data = NULL; //THIS IS TOTALLY FINE DONT KNOW WHY INTELLISENSE DOESN'T LIKE IT
        };
};

/*
* struct J_Handler - it's a struct so things are automatically public - otherwise has no effect
*
* used to communicate with the Notecard and handles requests to the notecard
* note: card.attn sleep doesn't noticibly reduce the power draw
*/
struct J_Handler {
    J *req; //JSON object type J

    //------------------------------------------------
    // Note about inline functions: the compiler decides whether or not to inline the function
    // Inlining means copying and pasting the function body directly where it is called to 
    // avoid the jump instruction during execution - an incredibly mild optimization in this context
    // It is being used in the following code more for clarity than anything else (extracting a process
    // into its own function if it contains a logical sequence makes sense)
    //------------------------------------------------


    /*
    * This method sets the notecards productUID -- this defines where the traffic should be sent to
    * on notehub.io; It also sets the "mode" to minimum so syncs only occur when "sync" : true is
    * explicitly sent as a field in the request
    *
    * Notes: the "product" will be "overwritten" with the same value each time the setup method is called 
    * this operation only needs to happen once technically but it doesn't hurt anything
    */
    void setup_notecard() {
        notecard.setDebugOutputStream(console_debug); //set the debugging information to the console

        req = notecard.newRequest("hub.set");          //init JSON object for request
        JAddStringToObject(req, "product", productUID); //set productUID
        JAddBoolToObject(req, "sync", true);
        JAddStringToObject(req, "mode", "minimum");      //set mode to minimum

        if (!notecard.sendRequest(req)) {                 //send request for connection
            #if N_DEBUG
                console_debug.println("NOTECARD DIDNT CONNECT - PROBLEM");
                notecard.logDebug("FATAL: Failed to configure Notecard!\n");
            #endif
            JDelete(req);
        } else {
            #if N_DEBUG
                console_debug.println("NOTECARD SENT REQUEST!");
            #endif
        }

    };

    /*
    * Ensures that the template for sending data from the sensor station has been set up
    * This should technically only need to be done once but is included here for accidental factory reset
    * or if the template should be updated - if no changes have been made to the template - the template
    * is overwritten with itself.
    *
    * A template used to send data must be used in LoRa and can optionally be used for LTE
    * Using a template minimizes the amount of data sent over the network and minimizes power usage and 
    * air time that way
    */
    void setup_data_template() {
        
        req = NoteNewRequest("note.template");
        JAddStringToObject(req, "file", "data.qo");
        JAddNumberToObject(req, "port", 49); //arbitrary port between 1-100 (though this value must be the same when using the template)
        JAddStringToObject(req, "format", "compact");
        JAddBoolToObject(req, "sync", true);

        J *body = JCreateObject();

        JAddNumberToObject(body, "conductivity_mean", 14.1);
        JAddNumberToObject(body, "conductivity_std_dev", 14.1);
        JAddNumberToObject(body, "dissolved_oxygen_mean", 14.1);
        JAddNumberToObject(body, "dissolved_oxygen_std_dev", 14.1);
        JAddNumberToObject(body, "temperature_mean", 14.1);
        JAddNumberToObject(body, "temperature_std_dev", 14.1);
        JAddNumberToObject(body, "ph_mean", 14.1);
        JAddNumberToObject(body, "ph_std_dev", 14.1);

        JAddItemToObject(req, "body", body);

        if (!notecard.sendRequest(req)) {
            JDelete(req);
            #if N_DEBUG
                console_debug.println("Problem!");
            #endif
        }

        #if N_DEBUG
            console_debug.println("SET_UP DONE");
        #endif
    };

    /*
    * send_data sends a request to the notecard with the sensor data and attempts to sync with notehub
    * 
    * This is notecard agnostic (tested LoRa chips and WBNAW chips)
    *
    */
    void send_data() {
        //the data must be sent using a note.add request with information added to data.qo 
        //as the template has been setup to work with this file
        req = notecard.newRequest("note.add"); //creates a new request
        JAddStringToObject(req, "file", "data.qo"); //set the file to add data
        JAddNumberToObject(req, "port", 49); //arbitrary port defined by the template between 1-100
        JAddStringToObject(req, "format", "compact"); //will happen automatically for LoRa but sends a minimal data message     
        JAddBoolToObject(req, "sync", true); //sync to notehub
            
        J *body = JCreateObject(); //the body contains sensor data information
        
        //for each sensor, add the mean and standard deviation with the appropriate data
        for (Sensor* sensor : sensor_array) {
            JAddNumberToObject(body, sensor->mean_str_, sensor->average); 
            JAddNumberToObject(body, sensor->std_dev_str_, sensor->standard_deviation); 
        }

        //add the body to the request
        JAddItemToObject(req, "body", body);
        
        //send the information to the notecard (which will start syncing with Notehub because of the earlier field)
        if (!notecard.sendRequest(req)) {
            JDelete(req);
            #if N_DEBUG
                console_debug.println("Problem!");
            #endif
        }

    };

    /* 
    * ====== FIXING A PIN-4 LOW CALL ======
    * 1. Physically remove notecard from notecarrier board
    * 2. Flash "safe" software onto the microcontroller (introductory blink code for instance)
    * 3. Reconnect notecard to the notecarrier
    * 4. Factory reset the notecard using the usb port on the NOTECARRIER and the notecarrier quick start
    * 5. Reset the product UID on the notecard
    * 6. You can now reflash the microcontroller with software and test again
    * =====================================
    */

    /*
    * Sends a request to the notecard to pull pins AUX1 - AUX3 (AUX4 is left as is -- see note)
    * to LOW (pulls them to ground) which sets the isolated carrier boards to low power mode
    * 
    * Note on pin 4: setting the fourth pin to low even in software kills the notecard - while the
    * notecard is connected no new code can be flashed to the microcontroller - if this happens follow the 
    * procedure above. There is either a problem in the Notecard firmware or the Notecard API
    */
    void set_all_pins_low() {
        J *req = NoteNewRequest("card.aux");
        JAddStringToObject(req, "mode", "gpio");

        J *pins = JAddArrayToObject(req, "usage");
        JAddItemToArray(pins, JCreateString("low"));   // AUX1
        JAddItemToArray(pins, JCreateString("low"));   // AUX2
        JAddItemToArray(pins, JCreateString("low"));  // AUX3
        JAddItemToArray(pins, JCreateString("")); // this unfortunately is correct for the time being
        //the above statement needs to be fixed in the blues wireless API or firmware

        if (!notecard.sendRequest(req)) {                 //send request for connection
            #if N_DEBUG
                console_debug.println("NOTECARD PULL LOW PROBLEM");
            #endif
            JDelete(req);
        } else {
            #if N_DEBUG
                console_debug.println("NOTECARD PULL LOW SUCCESS");
            #endif
        }

        delay(1000);
    }

    void set_pin_common(const int pin_location, const char* high_low) {
        //it is expected that this function will be called in normal operation (the temperature sensor calls
        //this each time for instance)
        if(pin_location > 3 || pin_location < 1) {
            #if N_DEBUG
                console_debug.println("Trying to access a pin outside of the bounds - yes pin 4 is not allowed");
            #endif
            return;
        }

        req = NoteNewRequest("card.aux");
        JAddStringToObject(req, "mode", "gpio");

        J *pins = JAddArrayToObject(req, "usage");

        //adds three elements to the array
        for(int i=1; i<4; ++i) {
            if(i == pin_location) {
                JAddItemToArray(pins, JCreateString(high_low));
            } else {
                JAddItemToArray(pins, JCreateString(""));
            }
        }

        JAddItemToArray(pins, JCreateString("")); //there needs to be a fourth entry in the array - and it can't be low
    }


    /*
    * Sends a request to the notecard to pull the pin specified by pin_location to low
    * The pin_location is specified in each individual sensor's class (the constants are defined 
    * at the top of the file but they are set in the child classes)
    * 
    * The separate function has been created for debug statement differences and call clarity (no param added from main)
    */ 
    void set_pin_low(const int pin_location) {
        set_pin_common(pin_location, "low");

        if (!notecard.sendRequest(req)) {                 //send request for connection
            #if N_DEBUG
                console_debug.println("NOTECARD PULL LOW - Individual - PROBLEM");
            #endif
            JDelete(req);
        } else {
            #if N_DEBUG
                console_debug.println("NOTECARD PULL LOW - Individual - SUCCESS");
            #endif
        }

        delay(1000);
    }

    /*
    * Sends a request to the notecard to pull the pin specified by pin_location to high
    * The pin_location is specified in each individual sensor's class (the constants are defined 
    * at the top of the file but they are set in the child classes)
    * 
    * The separate function has been created for debug statement differences and call clarity (no param added from main)
    * 
    */ 
    void set_pin_high(const int pin_location) {
        set_pin_common(pin_location, "high");

        if (!notecard.sendRequest(req)) {                 //send request for connection
            #if N_DEBUG
                console_debug.println("NOTECARD PULL HIGH - Individual - PROBLEM");
            #endif
            JDelete(req);
        } else {
            #if N_DEBUG
                console_debug.println("NOTECARD PULL HIGH - Individual - SUCCESS");
            #endif
        }

        delay(1000);
    }
};

/*
* Child class DissolvedOxygenSensor of parent Sensor 
* Defines the name and mux port of the sensor
* Methods overrien in this class will be called instead of the sensor methods
*/
class DissolvedOxygenSensor : public Sensor {
public:

    const char* name = "dissolved_oxygen";
    Ezo_board dissoved_oxygen_board = Ezo_board(97, name); 

    DissolvedOxygenSensor( int num_of_samples_param = SENSOR_SAMPLES, int port_param = DISSOLVED_OXYGEN_PORT, int pin_param = DISSOLVED_OXYGEN_PIN) :
        Sensor( num_of_samples_param, dissoved_oxygen_board, port_param, pin_param) {};
};

/*
* Child PHSensor of parent Sensor 
* Defines the name and mux port of the sensor
* Methods overrien in this class will be called instead of the sensor methods
*/
class PHSensor : public Sensor {
public:

    const char* name = "ph";
    Ezo_board ph_board = Ezo_board(99, name); 

    PHSensor(int num_of_samples_param = SENSOR_SAMPLES, int port_param = PHSENSOR_PORT, int pin_param = PHSENSOR_PIN) : 
        Sensor( num_of_samples_param, ph_board, port_param, pin_param) {};  
};

/*
* Child class TemperatureSensor of parent Sensor 
* Defines the name and mux port of the sensor
* Methods overrien in this class will be called instead of the sensor methods
*/
class TemperatureSensor : public Sensor {
public:

    const char* name = "temperature";
    Ezo_board temp_board = Ezo_board(102, name); 
    // note init super class port call
    //currently temperature sensor isolated carrier board is not shut off
    TemperatureSensor(int num_of_samples_param = SENSOR_SAMPLES, int port_param = TEMPERATURE_PORT, int pin_param = 0) :
        Sensor( num_of_samples_param, temp_board, port_param, pin_param) {};

    //the temperature sensor won't call a temperature dependent reading (cause the temperature is being read)
    //instead the normal read command without a temperature adjustment is used
    //this overrides the method defined in the sensor class and is called when the temperature sensor Sensor 
    //calls this method
    void send_read_command() override {
        ezo_board.send_read_cmd(); 
    }

    //updates the global temperature average with the most recent average reading
    //all the other sensors have an empty function call when calling this method
    //thus there are no if statements - (which may lack some clarity but it makes the calls more concise)
    //the "instanceof" call in c++ is rather involved 
    //this overrides the method defined in the sensor class and is called when the temperature sensor Sensor 
    //calls this method
    virtual void update_global_temp() override {
        global_temperature_average = this->average; 
    }
};

/*
* Child class DissolvedOxygenSensor of parent Sensor 
* Defines the name and mux port of the sensor
* Methods overrien in this class will be called instead of the sensor methods
*/
class ConductivitySensor : public Sensor {
public:

    const char* name = "conductivity";
    Ezo_board conductivity_board = Ezo_board(100, name); 
    ConductivitySensor(int num_of_samples_param = SENSOR_SAMPLES, int port_param = CONDUCTIVITY_PORT, int pin_param = CONDUCTIVITY_PIN) : 
        Sensor(num_of_samples_param, conductivity_board, port_param, pin_param) {};
};

//object instantiation of the j_handler for handling notecard requests
J_Handler j_handler;

//reads sensor data from each sensor - is an easy wrapper function
void collectSensorData() {
    
    Sensor_Reading reading;  
    
    for (Sensor* sensor : sensor_array) {
        j_handler.set_pin_high(sensor->pin);

        reading = sensor->take_reading();

        j_handler.set_pin_low(sensor->pin);

        #if N_DEBUG
            if(reading == SENSOR_ERROR) {
                console_debug.println("There was error while reading from the " + String(sensor->ezo_board.get_name()) + " sensor"); 
            }
        #endif

    }
};
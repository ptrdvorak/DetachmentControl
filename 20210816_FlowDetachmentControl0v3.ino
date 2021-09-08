/*
 * ====================================================================================================================
 * Flow Detachment Control System
 * --------------------------------------------------------------------------------------------------------------------
 * - senses the flow state on top surface of aircraft wing with a number of phototransistors
 * - the phototransistors are covered by a flexible tape
 * - when the flow is attached to the wing surface, the tape blocks light from entering the phototransistors
 * - as the angle of attack of the wing increases, detachment starts to occur at the trailing edge of the wing
 * - tapes in the detached flow are lifted from the wing surface and hence light can hit the phototransistor
 * - this signal is sensed on input pins (aka sensor pins as referred to within this code)
 * - based on known position of the phototransistors, percentage of surface with attached flow is computed
 * - to take immediate corrective action, the system features a feed-forward control logic
 * - the control signal is superimposed on elevator signal {CH2} that is read from RC receiver via PPM
 * - the signal is then sent to flight computer via PPM
 * - in-fligt logging is performed by the flight computer - percentage of chord with attached flow is mapped to CH4
 * - other RC channels are resent without modification
 * - the system is designed to run on Arduino Mega2560 sitting between RC receiver and onboard flight computer
 * - the system is controlled by: 
 *      CH5 (momentary switch) for calibration of the sensor - should be performed in state of attached flow at low AOA 
 *      CH6 (switch) to enable the control signal mixing into the baseline elevator signal
 *      CH7 (linear potentiometer) to control the scaling of the control signal ... for easy in-situ tuning
 * - LED array is used to indicate the sensor states as well as other variables (configurable)
 * - Health monitoring of the system can be performed via serial port
 * --------------------------------------------------------------------------------------------------------------------
 * Created by Petr Dvorak @ Institute of Aerospace Engineering, Brno University of Technology
 * The code uses PPMEncoder library version 0.3.0 by Christopher Schirner (github@schinken.io)
 * The code uses PPMReader library by Aapo Nikkila
 * ====================================================================================================================
 */


#include <PPMEncoder.h>
#include <PPMReader.h>

#define PPMIN_PIN   2          // The RC receiver is connected to this pin
#define PPMOUT_PIN  3          // The Autopilot is connected to this pin
#define CHANNELS    8          // Number of PPM channels we read and write
#define LED_OUTPUT  1          // What do we illustrate with the LED array
                               // 0: Nothing
                               // 1: Sensor state
                               // 2: RC state

#define SERIAL_LOGGING         // Shall we log to serial port?
#define SERIAL_INTERVAL 1000   // How many ms between serial logs

#define FILTERING_ENABLED 1    // Shall we employ the sensor value filtering in time domain?

// Variables for I/O sensors
const int SENSORS=8;           // Number of sensors and LEDs
const int FilteringSamples = 50;    // Number of samples in the circular buffer
const int HighThresholdPercentage = 10; // Percentage of HIGH states in the circular buffer where the entire sensor is considered HIGH

const int StatusLEDPin=13;     //Status LED 
const int LEDPins[] = {32, 34, 36, 38, 40, 42, 44, 46};
int LEDStates[SENSORS];        //Array for LED states

const int SensorPins[SENSORS] = {16, 18, 20, 22, 24, 26, 28, 30};
const int SensorPositions[SENSORS] = {90, 80, 70, 60, 50, 40, 30, 20};  //Percentage of local camber for respective sensors
int SensorStates[SENSORS];     //Array for sensor states

int SensorBuffer[SENSORS][FilteringSamples];
int SensorBufferStorage[SENSORS][FilteringSamples];
int HighStateCounts[SENSORS] = {0, 0, 0, 0, 0, 0, 0, 0};      // Counts the number of HIGH states in the buffer
int FilteredStates[SENSORS];   // Filtered states in time domain
int CalibrationArray[SENSORS] = {1, 1, 1, 1, 1, 1, 1, 1}; //By default, all sensors are on
int CalibratedStates[SENSORS]; // Corrected for malfunctioning sensors

// Variables
int ppmIN[CHANNELS];
int ppmOUT[CHANNELS];
int RC2IN;
int RC5IN;
int RC6IN;
int RC7IN;
int RC2OUT;
int RC4OUT;
bool trigger;
bool triggermemory;
bool LEDOverride;              // variable for enabling special LED indication
int AttachedChordPercentage=100;
int PitchCorrection=0;
int ControlScaling=1;

// Counters etc.
unsigned int CycleCounter = 0;
uint32_t looptimer;            // timer for loop frequency measurememnt
float loopfrequency;           // frequency of the loop computation in kHz
uint32_t logstart;             // timer for logging 
uint32_t LEDTimerStart;        // timer for LED override

// Initialize a PPMReader 
PPMReader ppmread(PPMIN_PIN, CHANNELS);

// ================================ SETUP ===============================
void setup() {

// Initialize PPMEncoder
ppmEncoder.begin(PPMOUT_PIN,CHANNELS);


// Turn on serial link
  #ifdef SERIAL_LOGGING
      Serial.begin(9600);
  #endif
  
// Initialize sensors
    for (int sensor = 0; sensor < SENSORS; ++sensor) { 
    pinMode(SensorPins[sensor] , INPUT); //define the sensor pins as input
    
    }

// Initialize LEDs
    for (int sensor = 0; sensor < SENSORS; ++sensor) {
    pinMode(LEDPins[sensor] , OUTPUT); // define the LED pin as output
    digitalWrite(LEDPins[sensor],LOW); // Turn off the LED
    }
    
    pinMode(StatusLEDPin , OUTPUT); // define the LED pin as output
    digitalWrite(StatusLEDPin ,LOW); // Turn off the LED

// Signalise the setup is done - activate pattern
    
        for (int sensor = 0; sensor < SENSORS; ++sensor) {
        digitalWrite(LEDPins[sensor],HIGH);
        delay(50);
        }
        for (int sensor = 0; sensor < SENSORS; ++sensor) {
        digitalWrite(LEDPins[sensor],LOW);
        delay(50);
        }
        for (int sensor = SENSORS-1; sensor >= 0; --sensor) {
        digitalWrite(LEDPins[sensor],HIGH);
        delay(50);
        }
        for (int sensor = SENSORS-1; sensor >= 0; --sensor) {
        digitalWrite(LEDPins[sensor],LOW);
        delay(50);
        }
}

// =============================== LOOP =================================
void loop() {
    // PPM READ: Read latest valid values from all RC channels
    // WARNING - the PPMReader library adresses the channels starting from 1 instead of 0
    for (int channel = 0; channel < CHANNELS; ++channel) {
        unsigned long value = ppmread.latestValidChannelValue(channel+1, 0);
        ppmIN[channel]=value;       
    }
    RC2IN=ppmIN[1];
    RC5IN=ppmIN[4];
    RC6IN=ppmIN[5];
    RC7IN=ppmIN[6];

    // READ detachment sensors 
    // We are reverting the logic - HIGH is for sensor that detects light
    for (int sensor = 0; sensor < SENSORS; ++sensor) { 
    SensorStates[sensor] = !digitalRead(SensorPins[sensor]);
    }

    /* FILTER the sensor states in time domain
     * Filtering is based on FirstInLastOut circular buffer of sensor states.
     * HIGH states are then counted in the frame of this buffer and if their percentage is higher than set treshold, the sensor is considered HIGH in this cycle.
     */
    #if (FILTERING_ENABLED == 1) // Perform the filtering only if enabled
      // First, we backup the current sensor buffer:
      for (int sample = 0; sample < FilteringSamples; ++sample){
        for (int sensor = 0; sensor < SENSORS; ++sensor) { 
          SensorBufferStorage[sensor][sample]=SensorBuffer[sensor][sample];
        } 
      }
      // Then, we write the current sensor states to the top of the buffer:
      for (int sensor = 0; sensor < SENSORS; ++sensor) { 
          SensorBuffer[sensor][0]=SensorStates[sensor];
      }
      // Now, we attach the backed-up values past the current value. The last record from the buffer will be omitted
      for (int sample = 1; sample < FilteringSamples; ++sample){
        for (int sensor = 0; sensor < SENSORS; ++sensor) { 
          SensorBuffer[sensor][sample]=SensorBufferStorage[sensor][sample-1];
        } 
      }
      // We are ready to count the number of high states:
      // reset the counts for this cycle
      for (int sensor = 0; sensor < SENSORS; ++sensor) { 
          HighStateCounts[sensor] = 0;
      } 
      for (int sample = 0; sample < FilteringSamples; ++sample){
        for (int sensor = 0; sensor < SENSORS; ++sensor) { 
          if (SensorBuffer[sensor][sample]==1){
            ++HighStateCounts[sensor];
          }
        } 
      }
      // Is the count of high states in the buffer higher than set threshold? Then assign the filtered states array a high value 
      for (int sensor = 0; sensor < SENSORS; ++sensor) { 
        if (HighStateCounts[sensor]>(HighThresholdPercentage*FilteringSamples/100)){
          FilteredStates[sensor]=1;
        }
        else {
          FilteredStates[sensor]=0;
        }
      }

    #else   // BYPASS of the filtering function - assign the sensor states directly to filtered states
      for (int sensor = 0; sensor < SENSORS; ++sensor) { 
        FilteredStates[sensor] = SensorStates[sensor]; 
      }
    #endif 
    
    // CALIBRATE the sensors for malfunctions, stuck barbs etc - new state is read when CH5 trigger is pulled
    if (RC5IN > 1500){
      trigger = HIGH;
    }
    else{
      trigger = LOW;
    }
    if (trigger==HIGH && triggermemory==LOW){
      for (int sensor = 0; sensor < SENSORS; ++sensor) { 
      CalibrationArray[sensor] = !FilteredStates[sensor];
      LEDOverride = HIGH;                                       // Turn on the LED override parameter so that the CalibrationArray can be displayed
      }
    }
    triggermemory=trigger;

    // Use the calibration array to disable invalid sensors (= the flow there is considered to be always attached = LOW)
    for (int sensor = 0; sensor < SENSORS; ++sensor) { 
    CalibratedStates[sensor] = CalibrationArray[sensor]*FilteredStates[sensor];
    }

    /* Compute the Attached chord percentage ... we simply iterate through the sensors and if a sensor is HIGH, then we consider this sensor to be detached ...
     *  hence only the chord up to the sensor position is considered attached.
     *  Default value is 100 (percent) ... the entire chord is considered attached as a baseline
     *  CH4 value is mapped to the AttachedChordPercentage variable
    */
    AttachedChordPercentage=100;
    for (int sensor = 0; sensor < SENSORS; ++sensor) { 
      if (CalibratedStates[sensor]==1){
        AttachedChordPercentage=SensorPositions[sensor];    
      }
    }
    RC4OUT=map(AttachedChordPercentage, 0, 100, 1000, 2000);
    
    /* Create a pitch correction signal from the AttachedChordPercentage value:
     * 100% correction is considered to be at full detachment, 0% correction at fully attached flow
     * 100% correction is mapped to 500us PWM signal - i.e. half throw of the elevator stick range. 
     * Hence if the control stick is at neutral position (1500us PWM), we deduct 500us resulting in 1000us signal -> maximum pitch down command.
     * Additionally, the control signal is scaled by a CH7 input (control knob) to provide means for effective tuning of the controller during an experiment.
     * At middle of the scaling range, the factor is 1, when fully turned down, the scaling is zero - i.e. the control input is zero irrespective of the detachment. 
     */
    PitchCorrection=map(AttachedChordPercentage, 0, 100, 50, 0); //Mapping is performed to 1/10th of the desired value, so that the computation can be performed in integer math
    ControlScaling=map(RC7IN, 1000, 2000, 0, 20);                //Mapping is to 10-fold the real scaling factor in order to avoid float computations 

    // Mix a pitch correction signal with the raw pitch signal if mixing is enabled by switch on CH6
    // Scaling for the control signal is provided by CH7
    if (RC6IN>1500){
      RC2OUT=RC2IN-PitchCorrection*ControlScaling;
      RC2OUT=constrain(RC2OUT, 1000, 2000);       // Constrain the output signal to be in the valid bounds
    }
    else {
      RC2OUT=RC2IN;                               // if the mixing is not enabled by CH6, we simply copy the input signal to output.
    }

    // Assign values to PPMOUTput
    for (int channel = 0; channel < CHANNELS; ++channel) {
    ppmOUT[channel]=ppmIN[channel];
    }
    ppmOUT[3]=RC4OUT;
    ppmOUT[1]=RC2OUT;
    
    // Write channel values to PPM
    // WARNING - the PPMEncoder adds approx 5us to the signal ... therefore this offset has been compensated here:
    for (int channel = 0; channel < CHANNELS; ++channel) {
    ppmEncoder.setChannel(channel, ppmOUT[channel]-5);
    }

    // LED array control
    if (LEDOverride==HIGH) {                    // Enables special indication with the LED array
      if (millis()<LEDTimerStart+500){          // How long the indication takes place 
        for (int sensor = 0; sensor < SENSORS; ++sensor) {
        LEDStates[sensor]=CalibrationArray[sensor];
        }
      }
      else {
        LEDOverride = LOW;
      }
    }
    else {
      LEDTimerStart= millis();
      // Refresh the LED array with standard data 
      #if (LED_OUTPUT == 1)
          for (int sensor = 0; sensor < SENSORS; ++sensor) {
          LEDStates[sensor]=CalibratedStates[sensor];
          } 
      #endif  
      
  
      #if (LED_OUTPUT == 2)
          for (int channel = 0; channel < CHANNELS; ++channel) {
          if (ppmIN[channel]<1500) {LEDStates[channel]=LOW;}
          else {LEDStates[channel]=HIGH;}
          }
      #endif  
    }
    for (int sensor = 0; sensor < SENSORS; ++sensor) { 
    digitalWrite(LEDPins[sensor],LEDStates[sensor]); // Display the sensor states with LEDs
    }
    
    // Display heartbeat with Status LED, compute loop frequency
    ++CycleCounter;
    if (CycleCounter == 1000){
    digitalWrite(StatusLEDPin,HIGH);
    loopfrequency=1000000000.0/float(micros()-looptimer);
    looptimer=micros();
    }
    if (CycleCounter >= 2000){
    digitalWrite(StatusLEDPin,LOW);
    CycleCounter=0;
    }

    // Log to serial port ... only if enabled and only once in a while not to delay the main loop
    #ifdef SERIAL_LOGGING
    if (millis()-logstart>=SERIAL_INTERVAL) {
      Serial.print("freq: " + String(loopfrequency) + " Hz | ");
      Serial.print("CalibrationArray: ");
      for (int sensor = 0; sensor < SENSORS; ++sensor) { 
        Serial.print(String(CalibrationArray[sensor]) + " ");
        }
      Serial.print("| Attached: " + String(AttachedChordPercentage) + "% ");
      Serial.print("| Correction: " + String(PitchCorrection) + " ");
      Serial.print("| Scaling: " + String(ControlScaling) + " ");
      Serial.print("| PPM OUT: ");
      for (int channel = 0; channel < CHANNELS; ++channel) {
        Serial.print(String(ppmOUT[channel]) + " ");
        }
      Serial.println();
      logstart=millis(); 
      }
    #endif
}

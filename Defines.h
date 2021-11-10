#define SW_Ver 2
#define SW_Rev 0

// Global variables/defines
#define IN_CONFIG_MODE 0x5A5A

// GLOBAL channels section

// AXIS pin mappings
#define ROLL_ANALOG_PIN     A0  //CH1
#define PITCH_ANALOG_PIN    A1  //CH2
#define THROTTLE_ANALOG_PIN A2  //CH3
#define YAW_ANALOG_PIN      A3  //CH4

// RC channels defines
#define ROLL_CH_1		0 //PPM out to RC CH1 
#define PITCH_CH_2		1 //PPM out to RC CH2
#define THROTTLE_CH_3	2 //PPM out to RC CH3 
#define YAW_CH_4		3 //PPM out to RC CH4 
#define GEAR_CH_5		4 //PPM out to RC CH5 
#define FLAPS_CH_6		5 //PPM out to RC CH6 
#define HT_CH_7			6 //PPM out to RC CH7 
#define TRIGGER_CH_8	7 //PPM out to RC CH8 

//axis limits 
//analogAxisLimits->Roll_Max|Roll_Min|Pitch_Max|Pitch_Min|Throt_Max|Throt_Min|Yaw_Max|Yaw_Min
#define NUM_OF_AXIS 4
#define NUM_OF_AXIS_LIMITS (NUM_OF_AXIS * 2) // each axis has 2 limits (min & max)

//channels indexs
#define ROLL_MAX_IDX      0
#define ROLL_MIN_IDX      1
#define PITCH_MAX_IDX     2
#define PITCH_MIN_IDX     3
#define THROTTLE_MAX_IDX  4
#define THROTTLE_MIN_IDX  5
#define YAW_MAX_IDX       6
#define YAW_MIN_IDX       7

//ch CALIBRATION INIT VALUES
#define MAX_INIT_VAL 0
#define MIN_INIT_VAL 2000

// BUTTON / TRIGGERS pin mappings
#define NUM_OF_BUTTONS_PER_SELECTOR 4
#define HAT_SELECT_LOW_PIN  3
#define BTNS_SELECT_LOW_PIN 4
#define BTN_1_PIN           5
#define BTN_2_PIN           6
#define BTN_3_PIN           7
#define BTN_4_PIN           8
#define TOGGLE_1            A4
#define TOGGLE_2            A5// for motor killer

// OUTPUT pin mappings
#define BUZZER_PIN 12
#define LED_PIN 11
// INPUT pin mappings
#define IN_VOLTAGE_SENS_PIN A6

// EEPROM section
int EEPROM_addr = 0;
#define EEPROM_WRITE_REFRESHTIME 10000 // if needed, update eeprom every 10sec
#define NUM_OF_TRIMMED_CHANNELS  2

//Channel uSec section
#define PULSE_uS_MIN  1000  // pulse minimum width
#define PULSE_uS_CNTR 1500  // pulse center width
#define PULSE_uS_MAX  2000  // pulse maximum width

// PPM out section
#define NUM_OF_CHANNELS_IN_PPM_OUT 8
#define PPM_OUT_PIN                9 //set PPM signal output pin on the arduino
#define CHANNEL_NUMBER             8 //set the number of chanels

#define FRAME_LENGTH               22500 //set the PPM frame length in microseconds (1ms = 1000us)
#define PULSE_LENGTH               300 //set the pulse length
#define onState                    1 //set polarity of the pulses: 1 is positive, 0 is negative


//PPM in section (head tracker decoder)
#define PPM_IN_PIN            2

//Battery handle
#define BATTERY_LOW_VALUE 7 // in volts
#define BATTERY_VOLTAGE_FACTOR 72.6 // read value to voltage value -> 7V = (72.6 * 7) = 508
#define FLOAT_GET_BATTERY_VOLTAGE_FROM_ANALOG_READ(alanogVoltage) (float)(alanogVoltage / BATTERY_VOLTAGE_FACTOR)
#define LOW_BATTERY_TIMER 5000 // wait for low battery voltage for more than 5secs

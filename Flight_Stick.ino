
#include <EEPROM.h>
#include "Defines.h"

/****************************************************Variables*******************************************************************/

uint16_t inConfigMode = 0;
uint16_t analogAxisLimits[NUM_OF_AXIS_LIMITS];

//BUZZER section
byte BuzzerIsOn = 0, BuzzerLen = 0;
unsigned long BuzzerOnTime = 0;

//LED section
byte LEDIsOff = 0, LEDLen = 0;
unsigned long LEDOffTime = 0;

// TRIM section
int8_t CH_Trims[NUM_OF_TRIMMED_CHANNELS];  // each chennel can be trimmed from -127 to 128 usec
byte CHTrimPrevPressedMap = 0xFF;// bit1 - UP, bit2 - RIGHT, bit3 - LEFT, bit4 - DOWN
unsigned long previousMillis = 0, currentMillis;

// set ppm out double buffer to min uSec 8-channels
uint16_t PPMOutDoubleBuffer[2][CHANNEL_NUMBER] = {{PULSE_uS_MIN,PULSE_uS_MIN,PULSE_uS_MIN,PULSE_uS_MIN,PULSE_uS_MIN,PULSE_uS_MIN,PULSE_uS_MIN,PULSE_uS_MIN},
                                                  {PULSE_uS_MIN,PULSE_uS_MIN,PULSE_uS_MIN,PULSE_uS_MIN,PULSE_uS_MIN,PULSE_uS_MIN,PULSE_uS_MIN,PULSE_uS_MIN}};

unsigned long ch[9], t[10];
int pulse = 0;

//Buttons Section
byte btns_selector_state = 1; // 0 - HAT LOW(0) BTNS HIGH(1) - read HAT status
                              // 1 - HAT LOW(1) BTNS HIGH(0) - read BTNS status
int btns_pins[NUM_OF_BUTTONS_PER_SELECTOR] = { BTN_1_PIN, BTN_2_PIN, BTN_3_PIN, BTN_4_PIN };
unsigned long pressTime[NUM_OF_BUTTONS_PER_SELECTOR] = { 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF };
unsigned long releaseTime[NUM_OF_BUTTONS_PER_SELECTOR];
byte btns_prev_res[NUM_OF_BUTTONS_PER_SELECTOR];
byte btns_res[NUM_OF_BUTTONS_PER_SELECTOR];
unsigned long longPressDelay = 600;
unsigned long shortPressDelay = 25;

//Flaps Section
byte flapsStage = 0;
unsigned int flapsStages[5] = { PULSE_uS_MIN, 1150, 1400, 1750, PULSE_uS_MAX }; // 5 stages flaps

//Buffers Section
byte fill_buffer_idx = 0;      // buffer index to collect data into
byte ready_buffer_idx = 1;     // buffer index with fresh data to be transmit
byte buffer_sent_finished = 0; // buffer index that was transmitted

unsigned long lowBatteryVoltageTimer = 0;
bool batteryLowVoltageDetected = false;

//Motor Kill toggle
#define MOTOR_KILL_DISABLE_MINIMUM_THRESHOLD 1100
bool kill_Motor = false, kill_Motor_prev = false;

//HeadTracker and Trigger killer
bool disableHT_riggger = false;

/****************************************************Functions*******************************************************************/

void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR |= bit(digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR |= bit(digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

void versionBeepIndication(byte ver, byte rev)
{
    // Indicate Current Version using Beeps on sttartup
    int count;
    for (count = 0; count < ver; count++)
    {
        digitalWrite(BUZZER_PIN, HIGH);
        digitalWrite(LED_PIN, HIGH);
        delay(200);
        digitalWrite(BUZZER_PIN, LOW);
        digitalWrite(LED_PIN, LOW);
        delay(100);
    }
    delay(200);
    // Indicate Current Revision using Beeps on sttartup
    for (count = 0; count < rev; count++)
    {
        digitalWrite(BUZZER_PIN, HIGH);
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(BUZZER_PIN, LOW);
        digitalWrite(LED_PIN, LOW);
        delay(100);
    }
}

void axisCalibrationInit()
{
    for (int i = 0; i < NUM_OF_AXIS_LIMITS; i++)
    {
        if (i % 2)
        {//odd - MIN limits init
            analogAxisLimits[i] = MIN_INIT_VAL;
        }
        else
        {//even - MAX limits init
            analogAxisLimits[i] = MAX_INIT_VAL;
        }
    }
}

void checkForConfigMode()
{
    // check for config mode entring condition
    byte btn_state = 0;
    for (int i = 0; i < NUM_OF_BUTTONS_PER_SELECTOR; i++)
    {
        btns_res[i] = digitalRead(btns_pins[i]);
        if (btns_res[i] != 0)
        {
            btn_state |= (1 << i);
        }
    }

    if (btn_state == 0x4)
    {// only if buttons 2, 4, 1 are pressed together on power up - enter to config mode
        inConfigMode = IN_CONFIG_MODE;
    }

}

void setup()
{
    Serial.begin(115200);

    // setup input axis channels analog
    pinMode(PITCH_ANALOG_PIN, INPUT);
    pinMode(ROLL_ANALOG_PIN, INPUT);
    pinMode(YAW_ANALOG_PIN, INPUT);
    pinMode(THROTTLE_ANALOG_PIN, INPUT);

    // setup toggles
    pinMode(TOGGLE_1, INPUT_PULLUP);
    pinMode(TOGGLE_2, INPUT_PULLUP);

    // setup buttons selectors
    pinMode(HAT_SELECT_LOW_PIN, OUTPUT);
    pinMode(BTNS_SELECT_LOW_PIN, OUTPUT);

    // setup buzzer pin as output
    pinMode(BUZZER_PIN, OUTPUT);

    // setup LED pin as output
    pinMode(LED_PIN, OUTPUT);

    // setup IN_VOLTAGE_SENS_PIN
    pinMode(IN_VOLTAGE_SENS_PIN, INPUT);

    // setup input buttons pins
    for (int i = 0; i < NUM_OF_BUTTONS_PER_SELECTOR; i++)
        pinMode(btns_pins[i], INPUT_PULLUP);

    pinMode(PPM_OUT_PIN, OUTPUT); // sets the digital pin as output
    digitalWrite(PPM_OUT_PIN, !onState); //set the PPM signal pin to the default state (off)

    // read saved trims values from eeprom
    for (EEPROM_addr = 0; EEPROM_addr < NUM_OF_TRIMMED_CHANNELS; EEPROM_addr++)
    {
        CH_Trims[EEPROM_addr] = EEPROM.read(EEPROM_addr);
    }

    // beep version number
    versionBeepIndication(SW_Ver, SW_Rev);

    cli();
    TCCR1A = 0; // set entire TCCR1 register to 0
    TCCR1B = 0;

    OCR1A = 100; // compare match register, change this
    TCCR1B |= (1 << WGM12); // turn on CTC mode
    TCCR1B |= (1 << CS11); // 8 prescaler: 0,5 microseconds at 16mhz
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
    pciSetup(PPM_IN_PIN);
    sei();

    readAxisEndPointsFromEEPROM();

    checkForConfigMode();
    TurnOnTheLED();
}

void loop()
{
    if (inConfigMode == IN_CONFIG_MODE)
    {
        unsigned long currentMillis, time_stamp;

        // overwrite the loaded values with init values
        axisCalibrationInit();

        currentMillis = time_stamp = millis();
        while (1)
        {
            //loop only in config mode

            //Serial.println("In config mode");
            collectAxisMaxMinValues();

            // in config mode - beep every 2 sec
            currentMillis = millis();
            if ((currentMillis - time_stamp) > 2000)
            {
                time_stamp = currentMillis;
                BuzzTheBuzzer(10);
                TurnOffTheLED(20);
            }
            TurnOffTheBuzzerCycler();
            TurnOnTheLEDCycler();
            delay(50);

            // look for button 3 press for saving the current calibrated values
            if (digitalRead(btns_pins[2]) == LOW)
            {
                saveConfigurationToEEPROM();
                // exit config mode to normal operation
                inConfigMode = 0;
                TurnOnTheLED();
                break;
            }
        }
    }

    // normal opertaion starts here !
    collectToggles();
    collectAxisStatus(fill_buffer_idx);
    collectButonsStatus(fill_buffer_idx);

    // done collecting data check if buffer swap is required
    if (buffer_sent_finished == 1)
    {
        // done collecting data and last buffer was sent - switch buffers
        if (ready_buffer_idx == 1)
        {
            fill_buffer_idx = 1;
            ready_buffer_idx = 0;
        }
        else
        {
            fill_buffer_idx = 0;
            ready_buffer_idx = 1;
        }
    }

    // handle CH trims save to eeprom
    currentMillis = millis();
    if (((currentMillis - previousMillis) > EEPROM_WRITE_REFRESHTIME) && (buffer_sent_finished == 1))
    {// preform a single eeprom save if needed only if time is over and PPM out finished sending a buffer (in ppm silance time)
        previousMillis = currentMillis;
        if (EEPROM_addr >= NUM_OF_TRIMMED_CHANNELS)
        {// reset the eeprom address
            EEPROM_addr = 0;
        }
        EEPROM.update(EEPROM_addr, CH_Trims[EEPROM_addr]);
        EEPROM_addr++;
    }

    TurnOffTheBuzzerCycler();

    checkBatteryVoltage();

    handleSerialReads();
}

void collectToggles()
{
    if (digitalRead(TOGGLE_1) == LOW)
    {
        disableHT_riggger = false;
    }
    else
    {
        disableHT_riggger = true;
    }

    if (digitalRead(TOGGLE_2) == LOW)
    {
        kill_Motor = false;
    }
    else
    {
        kill_Motor = true;
        kill_Motor_prev = true;
    }
}

void collectAxisStatus(byte fill_buffer_idx)
{
    PPMOutDoubleBuffer[fill_buffer_idx][ROLL_CH_1] = (map(analogRead(ROLL_ANALOG_PIN), analogAxisLimits[ROLL_MIN_IDX], analogAxisLimits[ROLL_MAX_IDX], PULSE_uS_MIN, PULSE_uS_MAX) + CH_Trims[ROLL_CH_1]);

    PPMOutDoubleBuffer[fill_buffer_idx][PITCH_CH_2] = (map(analogRead(PITCH_ANALOG_PIN), analogAxisLimits[PITCH_MIN_IDX], analogAxisLimits[PITCH_MAX_IDX], PULSE_uS_MAX, PULSE_uS_MIN) + CH_Trims[PITCH_CH_2]);

    PPMOutDoubleBuffer[fill_buffer_idx][THROTTLE_CH_3] = (map(analogRead(THROTTLE_ANALOG_PIN), analogAxisLimits[THROTTLE_MIN_IDX], analogAxisLimits[THROTTLE_MAX_IDX], PULSE_uS_MAX, PULSE_uS_MIN));

    Serial.println(PPMOutDoubleBuffer[fill_buffer_idx][THROTTLE_CH_3]);
    if ((kill_Motor_prev == true) && (kill_Motor == false))
    {// FOR SAFETY!!!!!!!  -  check that throttle is at minimum in order to enable the throttle again
        if (PPMOutDoubleBuffer[fill_buffer_idx][THROTTLE_CH_3] < 1100)
        {
          Serial.println("released");
            kill_Motor = false;
            kill_Motor_prev = false;
        }
    }

    if ((kill_Motor == true) || (kill_Motor_prev == true))
    {// kill motor toggle is ON
        PPMOutDoubleBuffer[fill_buffer_idx][THROTTLE_CH_3] = PULSE_uS_MIN;
    }

    PPMOutDoubleBuffer[fill_buffer_idx][YAW_CH_4] = (map(analogRead(YAW_ANALOG_PIN), analogAxisLimits[YAW_MIN_IDX], analogAxisLimits[YAW_MAX_IDX], PULSE_uS_MAX, PULSE_uS_MIN));
    //Serial.println(PPMOutDoubleBuffer[fill_buffer_idx][YAW_CH_4]);

    //Serial.println(ch[5]);
    // if ch[5] value is invalid or if in safe mode set it to the middle
    if ((ch[5] > 2100) || (ch[5] < 900) || (disableHT_riggger == true))
    {
        ch[5] = PULSE_uS_CNTR;
    }
    PPMOutDoubleBuffer[fill_buffer_idx][HT_CH_7] = ch[5];
}

void readCurrentPushButtonsCondition()
{
    btns_selector_state = 0; // select only push buttons and not HAT buttons
    for (int i = 0; i < NUM_OF_BUTTONS_PER_SELECTOR; i++)
    {
        btns_res[i] = digitalRead(btns_pins[i]);
    }
}

void collectButonsStatus(byte fill_buffer_idx)
{
    int i = 0;
    Toggle_Selectors();

    // for later - input read optimization using PIND and PINB ports direct read
    // read buttons state from input ports PIND (for digital 3 - 7) and PINB (for digital 8)
    //byte allButtonsState = (byte) ((PINB & 0b00000001) <<5 ) | ((PIND & 0b11111000) >>3);
    //Serial.println(allButtonsState , BIN);

    for (i = 0; i < NUM_OF_BUTTONS_PER_SELECTOR; i++)
    {
        btns_res[i] = digitalRead(btns_pins[i]);

        if (btns_selector_state == 0)
        {
            if (btns_res[i] == 0)
            {// momentry press
                if (i == 0)
                {// set to maximum
                    if (disableHT_riggger == false)
                    {// enable trigger only if disableHT_riggger is false
                        PPMOutDoubleBuffer[0][TRIGGER_CH_8] = PULSE_uS_MAX;
                        PPMOutDoubleBuffer[1][TRIGGER_CH_8] = PULSE_uS_MAX;
                    }
                    //reset head tracker in case of emergency
                    ch[5] = PULSE_uS_CNTR;
                }
            }
            else
            {
                if (i == 0)
                {// set to minimum
                    PPMOutDoubleBuffer[0][TRIGGER_CH_8] = PULSE_uS_MIN;
                    PPMOutDoubleBuffer[1][TRIGGER_CH_8] = PULSE_uS_MIN;
                }
            }

            if ((btns_res[i] == 0) && (btns_prev_res[i] == 0))
            {// pressed detected start timer
                pressTime[i] = millis();
                btns_prev_res[i] = 1;
            }

            if (btns_res[i] == 1)
            {// released check if short press
                if (((millis() - pressTime[i]) > shortPressDelay) && (btns_prev_res[i] == 1))
                {// if short press time over
                    switch (i)
                    {
                        case 1:// button 2
                            PPMOutDoubleBuffer[0][GEAR_CH_5] = PULSE_uS_MAX;
                            PPMOutDoubleBuffer[1][GEAR_CH_5] = PULSE_uS_MAX;
                            break;
                        // button 3 short press
                        case 2:
                            if (flapsStage > 0)
                            {
                                flapsStage--;
                                PPMOutDoubleBuffer[0][FLAPS_CH_6] = flapsStages[flapsStage];
                                PPMOutDoubleBuffer[1][FLAPS_CH_6] = flapsStages[flapsStage];
                            }
                            break;
                        // button 4 short press
                        case 3:
                            if (flapsStage < 4)
                            {
                                flapsStage++;
                                PPMOutDoubleBuffer[0][FLAPS_CH_6] = flapsStages[flapsStage];
                                PPMOutDoubleBuffer[1][FLAPS_CH_6] = flapsStages[flapsStage];
                            }
                            break;
                    }
                }
                btns_prev_res[i] = 0;
                continue;
            }

            // check if long press
            if (((millis() - pressTime[i]) > longPressDelay) && (btns_prev_res[i] == 1))
            {// long press detected
                switch (i)
                {
                    // button 2 long press
                    case 1:
                        PPMOutDoubleBuffer[0][GEAR_CH_5] = PULSE_uS_MIN;
                        PPMOutDoubleBuffer[1][GEAR_CH_5] = PULSE_uS_MIN;
                        break;
                    case 2:// fold flaps to 0
                        flapsStage = 0;
                        PPMOutDoubleBuffer[0][FLAPS_CH_6] = flapsStages[flapsStage];
                        PPMOutDoubleBuffer[1][FLAPS_CH_6] = flapsStages[flapsStage];
                        break;
                }
                btns_prev_res[i] = 2;// 2 - for last long press
            }
        }
        else if (btns_selector_state == 1)
        {// button pressed        
            if (btns_res[i] == 0)
            {
                if (CHTrimPrevPressedMap & (1 << i))
                {// change trim
                    CHTrimPrevPressedMap &= ~(1 << i);

                    switch (i)
                    {
                        case 0:
                            // Dec 4 from Trim
                            if (CH_Trims[PITCH_CH_2] >= -122)
                            {
                                CH_Trims[PITCH_CH_2] -= 4;

                                // check if channel trim is "0" set buzzer to long buzz
                                if (CH_Trims[PITCH_CH_2] == 0) BuzzTheBuzzer(50); // long buzz
                                else BuzzTheBuzzer(10);                 // short buzz
                            }
                            break;

                        case 3:
                            // ADD 4 to Trim 
                            if (CH_Trims[PITCH_CH_2] <= 122)
                            {
                                CH_Trims[PITCH_CH_2] += 4;

                                // check if channel trim is "0" set buzzer to long buzz
                                if (CH_Trims[PITCH_CH_2] == 0) BuzzTheBuzzer(50); // long buzz
                                else BuzzTheBuzzer(10);                 // short buzz
                            }
                            break;

                        case 1:
                            // ADD 4 to Trim 
                            if (CH_Trims[ROLL_CH_1] <= 122)
                            {
                                CH_Trims[ROLL_CH_1] += 4;

                                // check if channel trim is "0" set buzzer to long buzz
                                if (CH_Trims[ROLL_CH_1] == 0) BuzzTheBuzzer(50); // long buzz
                                else BuzzTheBuzzer(10);                 // short buzz
                            }
                            break;

                        case 2:
                            // Dec 4 from Trim
                            if (CH_Trims[ROLL_CH_1] >= -122)
                            {
                                CH_Trims[ROLL_CH_1] -= 4;

                                // check if channel trim is "0" set buzzer to long buzz
                                if (CH_Trims[ROLL_CH_1] == 0) BuzzTheBuzzer(50); // long buzz
                                else BuzzTheBuzzer(10);                 // short buzz
                            }
                            break;

                    }
                }
            }
            else if ((CHTrimPrevPressedMap & (1 << i)) == 0)
            {// Release trim button
                CHTrimPrevPressedMap |= (1 << i);
            }
        }
    }
}

ISR(TIMER1_COMPA_vect)
{ //leave this alone
    static boolean state = true;

    TCNT1 = 0;

    if (state)
    { //start pulse
        digitalWrite(PPM_OUT_PIN, onState);
        //OCR1A = PULSE_LENGTH * 2;
        OCR1A = (PULSE_LENGTH << 1); // mult by 2
        state = false;
        buffer_sent_finished = 0;
    }
    else
    { //end pulse and calculate when to start the next pulse
        static byte cur_chan_numb;
        static unsigned int calc_rest;

        digitalWrite(PPM_OUT_PIN, !onState);
        state = true;

        if (cur_chan_numb >= CHANNEL_NUMBER)
        {
            cur_chan_numb = 0;
            calc_rest = calc_rest + PULSE_LENGTH;//
                                                 //OCR1A = (FRAME_LENGTH - calc_rest) * 2;
            OCR1A = ((FRAME_LENGTH - calc_rest) << 1); // mult by 2
            calc_rest = 0;
            buffer_sent_finished = 1;
        }
        else
        {
            //OCR1A = (PPMOutDoubleBuffer[ready_buffer_idx][cur_chan_numb] - PULSE_LENGTH) * 2;
            OCR1A = ((PPMOutDoubleBuffer[ready_buffer_idx][cur_chan_numb] - PULSE_LENGTH) << 1); // mult by 2
            calc_rest = calc_rest + PPMOutDoubleBuffer[ready_buffer_idx][cur_chan_numb];
            cur_chan_numb++;
        }
    }
}

ISR(PCINT2_vect) // handle pin change interrupt for D0 to D7 here
{
    if (PIND & B00000100)
    {// handle only D2 interrupt
        t[pulse] = micros();

        if (pulse == 0)
        {
            pulse++;
            return;
        }
        ch[pulse] = t[pulse] - t[pulse - 1];

        if (ch[pulse] > 3000)
        {
            t[0] = t[pulse];
            pulse = 1;
        }
        else
        {
            pulse++;
        }
    }
}

void Toggle_Selectors()
{
    if (btns_selector_state == 0)
    {// selecting the HAT inputs
        digitalWrite(HAT_SELECT_LOW_PIN, LOW);
        digitalWrite(BTNS_SELECT_LOW_PIN, HIGH);
        btns_selector_state = 1;
    }
    else if (btns_selector_state == 1)
    {// selecting the BTNS inputs
        digitalWrite(HAT_SELECT_LOW_PIN, HIGH);
        digitalWrite(BTNS_SELECT_LOW_PIN, LOW);
        btns_selector_state = 0;
    }
}

//BUZZER handling

void BuzzTheBuzzer(unsigned int buzz_len)
{
    digitalWrite(BUZZER_PIN, HIGH);
    BuzzerLen = buzz_len;
    BuzzerIsOn = 1;
    BuzzerOnTime = millis();
}


void TurnOffTheBuzzerCycler()
{
    if (BuzzerIsOn == 1)
    {
        if ((millis() - BuzzerOnTime) > (10 * BuzzerLen))
        {
            BuzzerIsOn = 0;
            digitalWrite(BUZZER_PIN, LOW);
        }
    }
}

void TurnOnTheBuzzer()
{
    digitalWrite(BUZZER_PIN, HIGH);
}

void TurnOffTheBuzz()
{
    digitalWrite(BUZZER_PIN, LOW);
    BuzzerIsOn = 0;
}

//LED handling

void TurnOffTheLED(unsigned int led_len)
{
    TurnOffTheLED();
    LEDLen = led_len;
    LEDIsOff = 1;
    LEDOffTime = millis();
}


void TurnOnTheLEDCycler()
{
    if (LEDIsOff == 1)
    {
        if ((millis() - LEDOffTime) > (10 * LEDLen))
        {
            LEDIsOff = 0;
            TurnOnTheLED();
        }
    }
}

void TurnOnTheLED()
{
    digitalWrite(LED_PIN, HIGH);
}

void TurnOffTheLED()
{
    digitalWrite(LED_PIN, LOW);
    LEDIsOff = 0;
}

void checkBatteryVoltage()
{
    float batt_voltage = FLOAT_GET_BATTERY_VOLTAGE_FROM_ANALOG_READ(analogRead(IN_VOLTAGE_SENS_PIN));
    if (batt_voltage < BATTERY_LOW_VALUE)
    {// low battery detected - open timer to test if its consistence - if its consistence for more than x secs buzz the buzzer !
        if (batteryLowVoltageDetected == false)
        {
            lowBatteryVoltageTimer = millis();
            batteryLowVoltageDetected = true;
        }
    }
    else if (batteryLowVoltageDetected == true)
    {
        batteryLowVoltageDetected = false;
        TurnOffTheBuzz();
    }

    if (((millis() - lowBatteryVoltageTimer) > LOW_BATTERY_TIMER) &&
       (batteryLowVoltageDetected == true))
    {
        BuzzTheBuzzer(150);
        Serial.print("Battery voltage: ");
        Serial.println(batt_voltage);

        Serial.print("Battery voltage below: ");
        Serial.println(BATTERY_LOW_VALUE);
    }
}

void handleSerialReads()
{
    byte incomingByte = 0;
    int battery_prec = analogRead(IN_VOLTAGE_SENS_PIN);
    byte status_tx[18] = {((PPMOutDoubleBuffer[fill_buffer_idx][ROLL_CH_1] >> 8) & 0xFF),(PPMOutDoubleBuffer[fill_buffer_idx][ROLL_CH_1] & 0xFF),
                    ((PPMOutDoubleBuffer[fill_buffer_idx][PITCH_CH_2] >> 8) & 0xFF),(PPMOutDoubleBuffer[fill_buffer_idx][PITCH_CH_2] & 0xFF),
                    ((PPMOutDoubleBuffer[fill_buffer_idx][THROTTLE_CH_3] >> 8) & 0xFF),(PPMOutDoubleBuffer[fill_buffer_idx][THROTTLE_CH_3] & 0xFF),
                    ((PPMOutDoubleBuffer[fill_buffer_idx][YAW_CH_4] >> 8) & 0xFF),(PPMOutDoubleBuffer[fill_buffer_idx][YAW_CH_4] & 0xFF),
                    ((PPMOutDoubleBuffer[fill_buffer_idx][GEAR_CH_5] >> 8) & 0xFF),(PPMOutDoubleBuffer[fill_buffer_idx][GEAR_CH_5] & 0xFF),
                    ((PPMOutDoubleBuffer[fill_buffer_idx][FLAPS_CH_6] >> 8) & 0xFF),(PPMOutDoubleBuffer[fill_buffer_idx][FLAPS_CH_6] & 0xFF),
                    ((PPMOutDoubleBuffer[fill_buffer_idx][HT_CH_7] >> 8) & 0xFF),(PPMOutDoubleBuffer[fill_buffer_idx][HT_CH_7] & 0xFF),
                    ((PPMOutDoubleBuffer[fill_buffer_idx][TRIGGER_CH_8] >> 8) & 0xFF),(PPMOutDoubleBuffer[fill_buffer_idx][TRIGGER_CH_8] & 0xFF),
                    ((battery_prec >> 8) & 0xFF),(battery_prec & 0xFF)};
    if (Serial.available() > 0)
    {
        // read the incoming byte:
        incomingByte = Serial.read();

        switch (incomingByte)
        {
            case 0x5A:// status message request

                Serial.write(status_tx, sizeof(status_tx));

                break;

            default:
                break;
        }
    }
}

void collectAxisMaxMinValues()
{
    int roll_read, pitch_read, throttle_read, yaw_read;
    roll_read = analogRead(ROLL_ANALOG_PIN);

    // roll limits
    if (roll_read < analogAxisLimits[ROLL_MIN_IDX])
    {
        analogAxisLimits[ROLL_MIN_IDX] = roll_read;
        Serial.println(analogAxisLimits[ROLL_MIN_IDX]);
    }
    else if (roll_read > analogAxisLimits[ROLL_MAX_IDX])
    {
        analogAxisLimits[ROLL_MAX_IDX] = roll_read;
        Serial.println(analogAxisLimits[ROLL_MAX_IDX]);
    }

    pitch_read = analogRead(PITCH_ANALOG_PIN);

    // pitch limits
    if (pitch_read < analogAxisLimits[PITCH_MIN_IDX])
    {
        analogAxisLimits[PITCH_MIN_IDX] = pitch_read;
        Serial.println(analogAxisLimits[PITCH_MIN_IDX]);
    }
    else if (pitch_read > analogAxisLimits[PITCH_MAX_IDX])
    {
        analogAxisLimits[PITCH_MAX_IDX] = pitch_read;
        Serial.println(analogAxisLimits[PITCH_MAX_IDX]);
    }

    throttle_read = analogRead(THROTTLE_ANALOG_PIN);

    // throttle limits
    if (throttle_read < analogAxisLimits[THROTTLE_MIN_IDX])
    {
        analogAxisLimits[THROTTLE_MIN_IDX] = throttle_read;
        Serial.println(analogAxisLimits[THROTTLE_MIN_IDX]);
    }
    else if (throttle_read > analogAxisLimits[THROTTLE_MAX_IDX])
    {
        analogAxisLimits[THROTTLE_MAX_IDX] = throttle_read;
        Serial.println(analogAxisLimits[THROTTLE_MAX_IDX]);
    }

    yaw_read = analogRead(YAW_ANALOG_PIN);

    // yaw limits
    if (yaw_read < analogAxisLimits[YAW_MIN_IDX])
    {
        analogAxisLimits[YAW_MIN_IDX] = yaw_read;
        Serial.println(analogAxisLimits[YAW_MIN_IDX]);
    }
    else if (yaw_read > analogAxisLimits[YAW_MAX_IDX])
    {
        analogAxisLimits[YAW_MAX_IDX] = yaw_read;
        Serial.println(analogAxisLimits[YAW_MAX_IDX]);
    }
}

void saveConfigurationToEEPROM()
{
    // reset trimmed channels back to 0
    for (EEPROM_addr = 0; EEPROM_addr < NUM_OF_TRIMMED_CHANNELS; EEPROM_addr++)
    {
        EEPROM.update(EEPROM_addr, 0x00);
    }

    writeAxisEndPointsIntoEEPROM();
}

//save analog limits into eeprom 
void writeAxisEndPointsIntoEEPROM()
{
    int arraycount = 0;
    for (int i = 0; i < (NUM_OF_AXIS_LIMITS * 2); i += 2) //every value stored n 2 bytes (16 bits values)
    {
        EEPROM.update((i + NUM_OF_TRIMMED_CHANNELS), ((analogAxisLimits[arraycount] >> 8) & 0xFF));//write 8 msb
        EEPROM.update(((i + 1) + NUM_OF_TRIMMED_CHANNELS), ((analogAxisLimits[arraycount]) & 0xFF));//write 8 lsb 
        arraycount++;
    }
    //indicate save done
    BuzzTheBuzzer(50);
    TurnOffTheLED(250);
    Serial.println();
    Serial.print("Roll - ");
    Serial.print(analogAxisLimits[ROLL_MAX_IDX]);
    Serial.print(" ");
    Serial.print(analogAxisLimits[ROLL_MIN_IDX]);
    Serial.println();
    Serial.print("Pitch - ");
    Serial.print(analogAxisLimits[PITCH_MAX_IDX]);
    Serial.print(" ");
    Serial.print(analogAxisLimits[PITCH_MIN_IDX]);
    Serial.println();
    Serial.print("Throttle - ");
    Serial.print(analogAxisLimits[THROTTLE_MAX_IDX]);
    Serial.print(" ");
    Serial.print(analogAxisLimits[THROTTLE_MIN_IDX]);
    Serial.println();
    Serial.print("Yaw - ");
    Serial.print(analogAxisLimits[YAW_MAX_IDX]);
    Serial.print(" ");
    Serial.print(analogAxisLimits[YAW_MIN_IDX]);
    Serial.println();
}

void readAxisEndPointsFromEEPROM()
{
    int arraycount = 0;
    for (int i = 0; i < (NUM_OF_AXIS_LIMITS * 2); i += 2) //every value stored n 2 bytes (16 bits values)
    {

        // write 8 msb
        analogAxisLimits[arraycount] = (uint16_t)((EEPROM.read(i + NUM_OF_TRIMMED_CHANNELS) << 8) & 0xFF00);
        // write 8 lsb
        analogAxisLimits[arraycount] |= (uint16_t)((EEPROM.read((i + NUM_OF_TRIMMED_CHANNELS) + 1)) & 0xFF);
        arraycount++;
    }
    //TODO: add value validation after read - if value invalid set to default (0-1023)
    Serial.println();
    Serial.print("Roll - ");
    Serial.print(analogAxisLimits[ROLL_MAX_IDX]);
    Serial.print(" ");
    Serial.print(analogAxisLimits[ROLL_MIN_IDX]);
    Serial.println();
    Serial.print("Pitch - ");
    Serial.print(analogAxisLimits[PITCH_MAX_IDX]);
    Serial.print(" ");
    Serial.print(analogAxisLimits[PITCH_MIN_IDX]);
    Serial.println();
    Serial.print("Throttle - ");
    Serial.print(analogAxisLimits[THROTTLE_MAX_IDX]);
    Serial.print(" ");
    Serial.print(analogAxisLimits[THROTTLE_MIN_IDX]);
    Serial.println();
    Serial.print("Yaw - ");
    Serial.print(analogAxisLimits[YAW_MAX_IDX]);
    Serial.print(" ");
    Serial.print(analogAxisLimits[YAW_MIN_IDX]);
    Serial.println();
}

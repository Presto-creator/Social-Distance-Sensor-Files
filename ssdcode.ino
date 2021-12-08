/////////////////////////////////////////////////////////////////////////////////////////////////////
//mux_dist_meas.ino
/////////////////////////////////////////////////////////////////////////////////////////////////////

// segment pin definitions
const uint8_t pinSegA = 7;
const uint8_t pinSegB = 3;      //note: pin 8 is used by input capture so I swapped it for pin 3 here
const uint8_t pinSegC = 9;
const uint8_t pinSegD = 10;
const uint8_t pinSegE = 11;
const uint8_t pinSegF = 12;
const uint8_t pinSegG = 13;

// common pins of the four digits definitions
//USING ANALOG A0-3 PINS, IN STARTUP FUNCTION
const uint8_t pinDigit1 = A0;
const uint8_t pinDigit2 = A1;
const uint8_t pinDigit3 = A2;
const uint8_t pinDigit4 = A3;
const uint8_t grDigitCommons[] = { pinDigit1, pinDigit2, pinDigit3, pinDigit4 };
const uint8_t grSegments[] = { pinSegA, pinSegB, pinSegC, pinSegD, pinSegE, pinSegF, pinSegG };
//
const uint8_t pinTrig = 2;
const uint8_t pinEcho = 8;          //input capture is PB0 which is pin '8' on Uno R3
const uint8_t pinButton = 4;
const uint8_t pinGreenLED = 5;
const uint8_t pinRedLED = 6;

//segment to digit mappings
const uint8_t grDigitSegs[] =
{
    //-abcdefg
    0b01111110, //0
    0b00110000, //1
    0b01101101, //2
    0b01111001, //3
    0b00110011, //4
    0b01011011, //5
    0b01011111, //6
    0b01110000, //7
    0b01111111, //8
    0b01111011, //9
    0b00000001, //10 --> creates a '-' character
    0b00000000  //11 --> creates a dark (blank) character  
    
};

//masks for each segment (i.e. to what segment a bit position in grDigitSegs[] refers)
#define SEG_A_MASK      0b01000000
#define SEG_B_MASK      0b00100000
#define SEG_C_MASK      0b00010000
#define SEG_D_MASK      0b00001000
#define SEG_E_MASK      0b00000100
#define SEG_F_MASK      0b00000010
#define SEG_G_MASK      0b00000001
//
#define DASH_SYMBOL         10          //when no valid data or timeout, display should show '----'
#define BLANK_SYMBOL        11          //when we want to show nothing on the digit(s)
//
#define ISR_FLAG_START      0           //used within ISR to sequence rising or falling edge input capture
#define ISR_FLAG_END        1
//
#define MEASUREMENT_TIME    100ul       //mS    100mS   number of mS between distance measurements
#define MEASURE_TIMEOUT     1000ul      //mS    1000mS  maximum number of mS to wait before timing out on measurement
#define MUX_DIGIT_TIME      5ul         //mS    5mS     time each digit is displayed while muxing
#define PING_DURATION       10ul        //uS    10uS    approx time of ultrasonic ping pulse on trigger pin
#define DISPLAY_RESULT_TIME 3000ul      //mS    3-sec   time to display result

// variables that will change
//**    volatile variables will be accessed in interrupts as well as mainline
volatile uint8_t
    isrDone,                //indicates to mainline that timing of pulse has completed
    isrFlag;                //tells IC interrupt what edge it's looking for
volatile uint16_t
    isrT1OVF;               //counts timer 1 overflows during measurement
volatile uint32_t    
    isrTimeStart,           //time at the rising edge of pulse; 32-bit because we roll in overflow counts
    isrTimeEnd,             //time at the rising edge of pulse; 32-bit because we roll in overflow counts
    isrTimeDuration;        //resultant duration of pulse; is end - start
    
uint8_t
    nowButton,
    lastButton,
    grDisplayDigits[4];
        
uint32_t
    timeNow;                //global 32-bit value of millis() or micros() as required
    
enum msmStates
{
    //measurement state machine states
    WAIT_MEASURE = 0,
    WAIT_PING = 1,
    WAIT_ECHO_LOW = 2,
    TIME_ECHO_PULSE = 3,
    DISPLAY_RESULT = 4
      
};

void setup() 
{
    Serial.begin( 9600 );
    pinMode( pinTrig, OUTPUT );
    pinMode( pinEcho, INPUT );

    //note: button is not currently used (system does measurements every 100mS)
    pinMode( pinButton, INPUT_PULLUP );
    lastButton = digitalRead( pinButton );

    //set the digit common pins as outputs
    for( uint8_t i=0; i<4; i++ )
        pinMode( grDigitCommons[i], OUTPUT );

    //set the digit segments as outputs
    for( uint8_t i=0; i<7; i++ )
        pinMode( grSegments[i], OUTPUT );

    //LED pins as outputs
    pinMode( pinGreenLED, OUTPUT );
    pinMode( pinRedLED, OUTPUT );

    //setup timer 1 input capture ICP1 (on PB0)
    //  COM1A1  COM1A0  COM1B1  COM1B0  -----   -----   WGM11   WGM10
    //  0       0       0       0       0       0       0       0
    TCCR1A = 0;

    //just set up prescaler to /8 here
    //other setup occurs in measurement state machine
    //  ICNC1   ICES1   -----   WGM13   WGM12   CS12    CS11    CS10
    //  0       0       0       0       0       0       1       0
    TCCR1B = (1<<CS11);

    //force digits to be blank out of reset
    for( uint8_t i=0; i<4; i++ )
        grDisplayDigits[i] = BLANK_SYMBOL;

    //and both LEDs off
    digitalWrite( pinGreenLED, LOW );
    digitalWrite( pinRedLED, LOW );
    
}//setup

void loop() 
{    
    MuxDisplay();
    MeasurementStateMachine();

}//loop

void MeasurementStateMachine( void )
{
    static uint32_t
        timePing,
        timeMeasure;
    static uint8_t
        stateMeasure = WAIT_MEASURE;

    timeNow = millis();
    switch( stateMeasure )
    {
        case    WAIT_MEASURE:
            //check the button once ever MEASUREMENT_TIME seconds            
            if( (timeNow - timeMeasure) >= MEASUREMENT_TIME )
            {
                timeMeasure = timeNow;

                //measurement will start when button is pressed and then released (i.e. goes high)
                nowButton = digitalRead( pinButton );
                if( nowButton != lastButton )
                {
                    lastButton = nowButton;
                    if( nowButton == HIGH )
                    {
                        timePing = micros();
                        digitalWrite( pinTrig, HIGH );
                        stateMeasure = WAIT_PING;
                        
                    }//if
                    
                }//if          
                
            }//if
            
        break;

        case    WAIT_PING:
            //we time the TRIG pulse here; when done, set the pin low...
            if( (micros() - timePing) >= PING_DURATION )
            {
                digitalWrite( pinTrig, LOW );                
                stateMeasure = WAIT_ECHO_LOW;
                
            }//if
            
        break;

        case    WAIT_ECHO_LOW:
            //wait for ECHO to show low, then look for high-going pulse
            if( digitalRead( pinEcho ) == LOW )
            {
                //set up for input capture positive-going pulse
                noInterrupts();
                isrDone = false;                    //ISR will set true when measurement done
                isrFlag = ISR_FLAG_START;           //indicates were looking for rising edge first
                isrT1OVF = 0;                       //clears overflow counter
                TCNT1 = 0;                          //clears timer 1 current count
                TCCR1B |= (1<<ICNC1) | (1<<ICES1);  //set input capture to look for rising edge and turn on noise filter
                TIMSK1 |= (1<<ICIE1) | (1<<TOIE1);  //enable the input capture and timer overlfow interrupts                
                TIFR1 |= (1<<ICF1) | (1<<TOV1);     //clear any current interrupt flags
                interrupts();                       //then enable interrupts
                
                timePing = millis();
                stateMeasure = TIME_ECHO_PULSE;
                
            }//if
            
        break;

        case    TIME_ECHO_PULSE:
            //have we waited a long time and not seen a result yet?
            if( (timeNow - timePing) >= MEASURE_TIMEOUT )
            {
                //yes; turn off interrupts and show "----" with red LED
                noInterrupts();
                TIMSK1 &= ~( (1<<ICIE1) | (1<<TOIE1) );  //disable input capture and overflow interrupts                
                interrupts();

                //no result so display '----'
                for( uint8_t i=0; i<4; i++ )
                    grDisplayDigits[i] = DASH_SYMBOL;

                //red LED if measurement timeout
                digitalWrite( pinGreenLED, LOW );
                digitalWrite( pinRedLED, HIGH );
                
                timeMeasure = timeNow;
                
                stateMeasure = DISPLAY_RESULT;
                
            }//if
            else
            {
                //haven't timed out yet; has ISR measured a high-going pulse?
                if( isrDone == true )
                {
                    //yes; shut off the interrupts
                    noInterrupts();
                    TIMSK1 &= ~( (1<<ICIE1) | (1<<TOIE1) );  //disable input capture and overflow interrupts                
                    interrupts();
                    
                    //NOTES:
                    //  - timer 1 ticks at 2MHz (prescaler is /8) giving 500nS/tick
                    //  - sound travels at 13503.96 inches per second
                    //      - thus it travels 0.00675198 inches per 500nS tick
                    //  - round trip distance is Duration * 0.00675198
                    //      - distance to object is half that or Duration * 0.00337599
                    //
                    // example: Suppose object is 316 inches away
                    //      - total trimp distance is 632 inches; assume SOS is 13503.96 in/sec
                    //      - travel time is 632in/13503.96in/sec == 46.801mS
                    //      - 500nS/tick timer counts 46.801mS/500nS/tick or 93602 counts
                    //
                    //      - float fInches = (float)isrTimeDuration * 0.00337599;
                    //          fInches == 93602 * 0.00337599 = 316.0
                    //
                    //      - uint8_t uFeet = (uint8_t)(fInches / 12.0);
                    //          uFeet = 316.0 / 12.0 == 26.333 (26)
                    //      
                    //      - uint8_t uInches = (uint8_t)(fInches - (12.0 * (float)uFeet) );
                    //          uInches = 316.0 - (12 * 26) == 4
                    float fInches = (float)isrTimeDuration * 0.00337599;
                    //
                    uint8_t uFeet = (uint8_t)(fInches / 12.0);
                    grDisplayDigits[0] = uFeet / 10;
                    grDisplayDigits[1] = uFeet - (10 * grDisplayDigits[0]);
                    //
                    uint8_t uInches = (uint8_t)(fInches - (12.0 * (float)uFeet) );
                    grDisplayDigits[2] = uInches / 10;
                    grDisplayDigits[3] = uInches - (10 * grDisplayDigits[2]);

                    //////////////////////////////////////////////////////////////////                        
                    //blank leading zeros?
                    //so if reading is 1ft-6 inches show " 1 6" instead of "0106"?
                    //comment out this block if you want the leading zeros
                    if( grDisplayDigits[0] == 0 )
                        grDisplayDigits[0] = BLANK_SYMBOL;
                    if( grDisplayDigits[2] == 0 )
                        grDisplayDigits[2] = BLANK_SYMBOL;
                    //////////////////////////////////////////////////////////////////                        

                    //green LED if measurement successful
                    if (uFeet >= 6) {
                    digitalWrite( pinGreenLED, HIGH );
                    digitalWrite( pinRedLED, LOW );
                    }
                    else if (uFeet < 6) {
                    digitalWrite( pinGreenLED, LOW );
                    digitalWrite( pinRedLED, HIGH );
                    }

                    timeMeasure = timeNow;
    
                    stateMeasure = DISPLAY_RESULT;
                    
                }//if
                
            }//else          
            
        break;

        case    DISPLAY_RESULT:
            //hold the display for DISPLAY_RESULT_TIME before blanking and returning to wait for button press
            //will show "----" or reading with LED state
            if( (timeNow - timeMeasure) >= DISPLAY_RESULT_TIME )
            {
                //finished showing result; now blank digits
                for( uint8_t i=0; i<4; i++ )
                    grDisplayDigits[i] = BLANK_SYMBOL;

                //both LEDs off
                digitalWrite( pinGreenLED, LOW );
                digitalWrite( pinRedLED, LOW );
                
                timeMeasure = timeNow;
                stateMeasure = WAIT_MEASURE;
                
            }//if
            
        break;
        
    }//switch
    
}//MeasurementStateMachine

ISR( TIMER1_CAPT_vect )
{
    uint16_t inputCapCount = ICR1;

    if( (TIFR1 & (1 << TOV1)) && (inputCapCount < 0x7FFF) )
    {
        TIFR1 |= (1<<TOV1);
        isrT1OVF++;
        
    }//if
    
    //        
    if( isrFlag == ISR_FLAG_START )    
    {        
        //rising edge
        isrTimeStart = ((uint32_t)isrT1OVF << 16) + inputCapCount;   //log the time of the edge
        isrFlag = ISR_FLAG_END;                             //tell ISR to go to falling edge next time in        
        TCCR1B &= ~(1<<ICES1);                              //set timer to capture falling edge

    }//if
    else
    {
        //this is the falling edge
        isrTimeEnd = ((uint32_t)isrT1OVF << 16) + inputCapCount;     //get the time of the falling edge
        TIMSK1 &= ~( (1<<ICIE1) | (1<<TOIE1) );                 //and disable more input captures & OVFs (will re-enable next measurement)
        
        isrTimeDuration = isrTimeEnd - isrTimeStart;        //compute duration of pulse as timeEnd - timeStart
        isrDone = true;                                     //indicate to mainline we have a completed measurement
        
    }//else
    
}//ISR( TIMER1_CAPT_vect )

ISR( TIMER1_OVF_vect )
{
    //on each timer overflow we bump this counter
    //giving us a gazillion (well, 32) bits of counting
    isrT1OVF++;
    
}//ISR( TIMER1_OVF_vect )

void MuxDisplay( void )
{
    static uint8_t
        lastIdxDigit = 3,
        idxDigit = 0;
    static uint32_t
        timeDisplay;

    timeNow = millis();
    
    if( (timeNow - timeDisplay) >= MUX_DIGIT_TIME )
    {
        timeDisplay = timeNow;
        
        //turn off current digit by setting its common-anode (?) low
        //set segments for this digit
        //then turn the digit on
        digitalWrite( grDigitCommons[lastIdxDigit], LOW );      //turn off currently-on digit common
        DriveSegs( grDisplayDigits[idxDigit] );                 //drive new segments for new digit
        digitalWrite( grDigitCommons[idxDigit], HIGH );         //and turn on its common

        //save the "last" digit so we know which to turn off next time through
        lastIdxDigit = idxDigit;

        //then bump digit count. the "& 0x03" ensures
        //the count is 0, 1, 2 or 3
        idxDigit = (idxDigit + 1) & 0x03;
        
    }//if
    
}//MuxDisplay

void DriveSegs( uint8_t val )
{
    uint8_t mask = 0b01000000;
    
    for( uint8_t i=0; i<7; i++ )
    {
        digitalWrite( grSegments[i], (grDigitSegs[val] & mask) ? LOW:HIGH );
        mask >>= 1;
        
    }//for
    
}//DriveSegs

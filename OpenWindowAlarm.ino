/*
 * OpenWindowAlarm.cpp
 * Every TEMPERATURE_SAMPLE_SECONDS (24) seconds a sample is taken of the attiny internal temperature sensor which has a resolution of 1 degree.
 * This delay is implemented by n (3) times sleeping at SLEEP_MODE_PWR_DOWN for a period of 8 seconds.
 *
 * Open Window is detected after (TEMPERATURE_COMPARE_AMOUNT * TEMPERATURE_SAMPLE_SECONDS) seconds of reading a temperature
 * which value is TEMPERATURE_DELTA_THRESHOLD_DEGREE lower than the temperature (TEMPERATURE_COMPARE_DISTANCE * TEMPERATURE_SAMPLE_SECONDS) seconds before.
 * The internal sensor has therefore 3 minutes time to adjust to the outer temperature, to get even small changes in temperature.
 * The greater the temperature change the earlier the change of sensor value and detection of an open window.
 * After open window detection Alarm is activated after OPEN_WINDOW_MINUTES if actual temperature is not higher than the (minimum temperature + 1) i.e. the window is not closed yet.
 *
 *  Copyright (C) 2018  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include <Arduino.h>

//#define DEBUG

#ifdef DEBUG
#include "TinySerialOut.h"
#endif

//#include "AVRUtils.h"

#include <avr/boot.h>  // needed for boot_signature_byte_get()
#include <avr/power.h> // needed for  clock_prescale_set()
#include <avr/sleep.h> // needed for sleep_enable()
#include <avr/wdt.h>   // needed for WDTO_8S


#define VERSION "1.0"

/*
 */
const uint8_t OPEN_WINDOW_ALARM_DELAY_MINUTES = 5;

const uint16_t TEMPERATURE_SAMPLE_SECONDS = 24;  // use multiple of 8
const uint8_t OPEN_WINDOW_SAMPLES = (OPEN_WINDOW_ALARM_DELAY_MINUTES * 60) / TEMPERATURE_SAMPLE_SECONDS;
const uint8_t TEMPERATURE_COMPARE_AMOUNT = 2;
const uint8_t TEMPERATURE_COMPARE_DISTANCE = 8;
// Array to hold enough values to compare TEMPERATURE_COMPARE_AMOUNT values with the same amount of values TEMPERATURE_COMPARE_DISTANCE positions before
uint16_t sTemperatureArray[(TEMPERATURE_COMPARE_AMOUNT + TEMPERATURE_COMPARE_DISTANCE + TEMPERATURE_COMPARE_AMOUNT)];
// 1 LSB  = 1 Degree Celsius
const uint16_t TEMPERATURE_DELTA_THRESHOLD_DEGREE = 2;

uint16_t sTemperatureMinimumAfterWindowOpen;
bool sOpenWindowDetected = false;
uint8_t sOpenWindowSampleDelayCounter;

//
// ATMEL ATTINY85
//
//                                 +-\/-+
//           RESET/ADC0 (D5) PB5  1|    |8  Vcc
//   Tone - ADC3/PCINT3 (D3) PB3  2|    |7  PB2 (D2) INT0/ADC1 - TX Debug output
//   Tone inv.   - ADC2 (D4) PB4  3|    |6  PB1 (D1) MISO/DO/AIN1/OC0B/OC1A/PCINT1 - (Digispark) LED
//                           GND  4|    |5  PB0 (D0) OC0A/AIN0 - Alarm Test if connected to ground
//

#define ALARM_TEST_PIN PB0
#define LED_PIN  PB1
#define TONE_PIN PB4
#define TONE_PIN_INVERTED PB3

#define ADC_TEMPERATURE_CHANNEL_MUX 15

#if (LED_PIN == TX_PIN)
#error "LED pin must not be equal TX pin."
#endif

const uint16_t LED_PULSE_LENGTH = 200; // 500 is well visible 200 is OK

uint8_t sMCUSRStored; // content of MCUSR register at startup

void signalReading();
void signal();
void initSleep(uint8_t tSleepMode);
void sleepWithWatchdog(uint8_t aWatchdogPrescaler);
void sleepDelay();
void delayMilliseconds(unsigned int aMillis);
uint16_t readADCChannelWithReferenceOversample(uint8_t aChannelNumber, uint8_t aReference, uint8_t aOversampleExponent);
void changeDigisparkClock();

/***********************************************************************************
 * Code starts here
 ***********************************************************************************/

void setup() {

    // store MCUSR early for later use
    sMCUSRStored = MCUSR;
    MCUSR = 0;

#ifdef DEBUG
    /*
     * Initialize the serial pin as an output for Serial.print like debugging
     */
    initTXPin();
#endif

    changeDigisparkClock();

    // initialize the LED pin as an output.
    pinMode(LED_PIN, OUTPUT);

#ifdef DEBUG
    writeString(F("START " __FILE__ "\nVersion " VERSION " from " __DATE__ "\nAlarm delay="));
    writeUnsignedByte(OPEN_WINDOW_ALARM_DELAY_MINUTES);
    writeString(F(" minutes\nMCUSR="));
    writeUnsignedByteHex(sMCUSRStored);
    write1Start8Data1StopNoParity('\n');
#endif

#ifdef TRACE
    writeString(F(" LFuse="));
    writeUnsignedByteHex(boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS));
    writeString(F(" OSCCAL="));
    writeUnsignedByteHex(tOSCCAL);
    write1Start8Data1StopNoParity('\n');
#endif

    /*
     * init sleep mode
     */
    initSleep(SLEEP_MODE_PWR_DOWN);

    pinMode(TONE_PIN_INVERTED, OUTPUT);
    pinMode(TONE_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(ALARM_TEST_PIN, INPUT);  // Instead of pinMode(ALARM_TEST_PIN, INPUT_PULLUP);
    digitalWrite(ALARM_TEST_PIN, 1); // set pullup

// disable Arduino delay() and millis() timer0 and also its interrupts which kills the deep sleep.
    TCCR0B = 0;

    /*
     * Initialize ADC channel and reference
     */
    readADCChannelWithReferenceOversample(ADC_TEMPERATURE_CHANNEL_MUX, INTERNAL1V1, 0);

    /*
     * Blink LED at startup to show OPEN_WINDOW_MINUTES
     */
    for (int i = 0; i < OPEN_WINDOW_ALARM_DELAY_MINUTES; ++i) {
        // activate LED
        digitalWrite(LED_PIN, 1);
        delayMilliseconds(200);  // delay() is disabled, so use delayMicroseconds()
        // deactivate LED
        digitalWrite(LED_PIN, 0);
        delayMilliseconds(200);
    }

    if (!digitalRead(ALARM_TEST_PIN)) {
#ifdef DEBUG
        writeString(F("Test signal out"));
        write1Start8Data1StopNoParity('\n');
#endif
        signal();
    }

// disable digital input buffer to save power
// do not disable buffer for outputs whose values are read back
    DIDR0 = (1 << ADC1D) | (1 << ADC2D) | (1 << ADC3D) | (1 << AIN1D) | (1 << AIN0D);

    /*
     * wait 8 seconds, since ATtinys temperature is increased after the digistump boot process
     */
    sleepWithWatchdog(WDTO_8S);
}

void loop() {
    uint16_t tTemperatureNewSum = 0;
    uint16_t tTemperatureOldSum = 0;

    uint8_t i = (sizeof(sTemperatureArray) / sizeof(uint16_t)) - 1;
    /*
     * shift values in temperature history array and insert new one at [0]
     */
    while (i >= TEMPERATURE_COMPARE_AMOUNT + TEMPERATURE_COMPARE_DISTANCE) {
        // shift TEMPERATURE_COMPARE_AMOUNT values to end and sum them up
        sTemperatureArray[i] = sTemperatureArray[i - 1];
        tTemperatureOldSum += sTemperatureArray[i - 1];
        i--;
    }

    while (i >= TEMPERATURE_COMPARE_AMOUNT) {
        // shift values to end
        sTemperatureArray[i] = sTemperatureArray[i - 1];
        i--;
    }

    while (i > 0) {
        // shift (TEMPERATURE_COMPARE_AMOUNT - 1) values to end and sum them up
        sTemperatureArray[i] = sTemperatureArray[i - 1];
        tTemperatureNewSum += sTemperatureArray[i - 1];
        i--;
    }

    /*
     * Read new Temperature (typical 280 - 320 at 25 C) and add to sum
     */
    sTemperatureArray[0] = readADCChannelWithReferenceOversample(ADC_TEMPERATURE_CHANNEL_MUX, INTERNAL1V1, 4);
    signalReading();
    tTemperatureNewSum += sTemperatureArray[0];

    /*
     * Check if reason for reset was "power on" (in contrast to "reset pin") and temperature is decreasing
     */
    if ((sMCUSRStored & (1 << PORF))
    /* we are a a power on boot */
    && (sTemperatureArray[(sizeof(sTemperatureArray) / sizeof(uint16_t)) - 1] == 0)
            && (sTemperatureArray[(sizeof(sTemperatureArray) / sizeof(uint16_t)) - 2] > 0)
            /*
             * array is almost full, so check if temperature is lower than at reset time which means,
             * we ported the sensor from a warm place to its final one
             */
            && (sTemperatureArray[0] < sTemperatureArray[(sizeof(sTemperatureArray) / sizeof(uint16_t)) - 2])) {
        // Start from beginning, clear temperature array
#ifdef DEBUG
        writeString(F("Detected porting to a colder place -> reset"));
        write1Start8Data1StopNoParity('\n');
#endif
        for (i = 0; i < (sizeof(sTemperatureArray) / sizeof(uint16_t)) - 1; ++i) {
            sTemperatureArray[i] = 0;
        }
    } else {

#ifdef DEBUG
        writeString(F("Temp="));
        writeUnsignedInt(sTemperatureArray[0]);
        writeString(F(" Old="));
        writeUnsignedInt(tTemperatureOldSum);
        writeString(F(" New="));
        writeUnsignedInt(tTemperatureNewSum);
        write1Start8Data1StopNoParity('\n');
#endif

        if (sOpenWindowDetected) {
            if (tTemperatureNewSum < sTemperatureMinimumAfterWindowOpen) {
                // set new minimum
                sTemperatureMinimumAfterWindowOpen = tTemperatureNewSum;
            }
            sOpenWindowSampleDelayCounter++;
            if (sOpenWindowSampleDelayCounter >= OPEN_WINDOW_SAMPLES) {
                /*
                 * After delay, check if already open
                 */
                if (tTemperatureNewSum <= (sTemperatureMinimumAfterWindowOpen + TEMPERATURE_COMPARE_AMOUNT)) {
                    // window still open
#ifdef DEBUG
                    writeString(F("Detected window still open -> alarm"));
                    write1Start8Data1StopNoParity('\n');
#endif
                    signal();
                } else {
                    // window already closed -> reset values for a new start
#ifdef DEBUG
                    writeString(F("Detected window already closed -> start again"));
                    write1Start8Data1StopNoParity('\n');
#endif
                    sOpenWindowDetected = false;
                }
            }
        } else {
            if (tTemperatureNewSum + (TEMPERATURE_DELTA_THRESHOLD_DEGREE * TEMPERATURE_COMPARE_AMOUNT) < tTemperatureOldSum) {
#ifdef DEBUG
                writeString(F("Detected window just opened -> check again in "));
                writeUnsignedByte(OPEN_WINDOW_ALARM_DELAY_MINUTES);
                writeString(F(" minutes\n"));
#endif
                sTemperatureMinimumAfterWindowOpen = tTemperatureNewSum;
                sOpenWindowDetected = true;
                sOpenWindowSampleDelayCounter = 0;
            }
        }
    } // after boot and temperature is decreasing

    sleepDelay();
}

/*
 * Code to change Digispark Bootloader clock settings to get the right CPU frequency
 * and to reset Digispark OCCAL tweak.
 * Call it if you want to use the standard ATtiny85 library, BUT do not call it, if you need Digispark USB functions available for 16MHz.
 */
void changeDigisparkClock() {
    uint8_t tLowFuse = boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);
    if ((tLowFuse & 0x0F) == 0x01) {
        /*
         * Here we have High Frequency PLL Clock ( 16 MHz)
         */
#if (F_CPU == 1000000)
        // Divide 16 MHz Pll clock by 16 for Digispark Boards to get the requested 1 MHz
        clock_prescale_set(clock_div_16);
//        CLKPR = (1 << CLKPCE);  // unlock function
//        CLKPR = (1 << CLKPS2); // %16
#endif
#if (F_CPU == 8000000)
        // Divide 16 MHz Pll clock by 2 for Digispark Boards to get the requested 8 MHz
        clock_prescale_set(clock_div_2);
//        CLKPR = (1 << CLKPCE);  // unlock function
//        CLKPR = (1 << CLKPS0);// %2
#endif
    }

    /*
     * Code to reset Digispark OCCAL tweak
     */
#define  SIGRD  5 // needed for boot_signature_byte_get()
    uint8_t tStoredOSCCAL = boot_signature_byte_get(1);
    if (OSCCAL != tStoredOSCCAL) {
#ifdef DEBUG
        uint8_t tOSCCAL = OSCCAL;
        writeString(F("Changed OSCCAL from "));
        writeUnsignedByteHex(tOSCCAL);
        writeString(F(" to "));
        writeUnsignedByteHex(tStoredOSCCAL);
        write1Start8Data1StopNoParity('\n');
#endif
        // retrieve the factory-stored oscillator calibration bytes to revert the digispark OSCCAL tweak
        OSCCAL = tStoredOSCCAL;
    }
}

void signal() {
    uint16_t tCounter = 300; // 600 seconds
    while (tCounter-- != 0) {

        // activate LED
        digitalWrite(LED_PIN, 1);
        delayMilliseconds(300);

        // deactivate LED
        digitalWrite(LED_PIN, 0);

        /*
         * use tone() only for specifying the frequency and then switch to OCR1B PWM mode
         * in order to use inverted output for increasing the volume
         */
        tone(TONE_PIN, 1100); // specify half frequency -> PWM doubles it
        TCCR1 = TCCR1 & 0x0F; // keep only prescaler
        GTCCR = (1 << PWM1B) | (1 << COM1B0); // PWM Mode with OCR1B (PB4) + !OCR1B (PB3) outputs enabled
        OCR1B = OCR1C / 2; // 50% PWM

        delayMilliseconds(1000);
        tone(TONE_PIN, 550); // specify half frequency -> PWM doubles it
    }
}

/*
 * Flash LED only for a short period to save power.
 * If open window detected, increase pulse length to give a visual feedback
 */
void signalReading() {
    // activate LED
    digitalWrite(LED_PIN, 1);
    if (sOpenWindowDetected) {
        delayMicroseconds(LED_PULSE_LENGTH * 4);
    } else {
        delayMicroseconds(LED_PULSE_LENGTH);
    }
    // deactivate LED
    digitalWrite(LED_PIN, 0);
}

void sleepDelay() {
    for (uint8_t i = 0; i < (TEMPERATURE_SAMPLE_SECONDS / 8); ++i) {
        sleepWithWatchdog(WDTO_8S);
    }
}

void delayMilliseconds(unsigned int aMillis) {
    for (unsigned int i = 0; i < aMillis; ++i) {
        delayMicroseconds(1000);
    }
}

#define ADC_PRESCALE8    3 // 104 microseconds per ADC conversion at 1 MHz

uint16_t readADCChannelWithReferenceOversample(uint8_t aChannelNumber, uint8_t aReference, uint8_t aOversampleExponent) {
    uint16_t tSumValue = 0;
    ADMUX = aChannelNumber | (aReference << REFS2);

// ADCSRB = 0; // free running mode if ADATE is 1 - is default
// ADSC-StartConversion ADATE-AutoTriggerEnable ADIF-Reset Interrupt Flag
    ADCSRA = (_BV(ADEN) | _BV(ADSC) | _BV(ADATE) | _BV(ADIF) | ADC_PRESCALE8);

    for (uint8_t i = 0; i < (1 << aOversampleExponent); i++) {
        /*
         * wait for free running conversion to finish.
         * Do not wait for ADSC here, since ADSC is only low for 1 ADC Clock cycle on free running conversion.
         */
        loop_until_bit_is_set(ADCSRA, ADIF);

        ADCSRA |= _BV(ADIF); // clear bit to recognize next conversion has finished
        // Add value
        tSumValue += (ADCH << 8) | ADCL;
    }
    ADCSRA &= ~_BV(ADATE); // Disable auto-triggering (free running mode)
    return (tSumValue >> aOversampleExponent);
}

void initSleep(uint8_t tSleepMode) {
    sleep_enable()
    ;
    set_sleep_mode(tSleepMode);
}

/*
 * aWatchdogPrescaler (see wdt.h) can be one of
 * WDTO_15MS, 30, 60, 120, 250, WDTO_500MS
 * WDTO_1S to WDTO_8S
 */

void sleepWithWatchdog(uint8_t aWatchdogPrescaler) {
    MCUSR &= ~_BV(WDRF); // Clear WDRF in MCUSR

    // use wdt_enable() since it handles that the WDP3 bit is in bit 5 of the WDTCSR register
    wdt_enable(aWatchdogPrescaler);

#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
#define WDTCSR  WDTCR
#endif
    WDTCSR |= _BV(WDIE) | _BV(WDIF); // Watchdog interrupt enable + reset interrupt flag
    sei();
    sleep_cpu()
    ;
    wdt_disable();
}

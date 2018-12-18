/*
 * OpenWindowAlarm.cpp
 *
 * Overview:
 * Every 24 seconds a sample is taken of the ATtiny internal temperature sensor which has a resolution of 1 degree.
 * If temperature is lower than "old" temperature value, then an alarm is issued 5 minutes later, if "the condition still holds".
 * Detection of an open window is indicated by a longer 20ms blink and a short click every 24 seconds.
 * A low battery (below 3.55 Volt) is indicated by beeping and flashing LED every 24 seconds. Only the beep (not the flash) is significantly longer than at open window detection.
 *
 * Detailed description:
 * Open window is detected after `TEMPERATURE_COMPARE_AMOUNT * TEMPERATURE_SAMPLE_SECONDS` seconds of reading a temperature
 * which value is `TEMPERATURE_DELTA_THRESHOLD_DEGREE` lower than the temperature `TEMPERATURE_COMPARE_DISTANCE * TEMPERATURE_SAMPLE_SECONDS` seconds before.
 * The delay is implemented by 3 times sleeping at `SLEEP_MODE_PWR_DOWN` for a period of 8 seconds in order to reduce power consumption.
 * Detection of an open window is indicated by a longer 20ms blink and a short click every 24 seconds.
 * The internal sensor has therefore 3 minutes time to adjust to the outer temperature, to get even small changes in temperature.
 * The greater the temperature change the earlier the change of sensor value and detection of an open window.
 * After open window detection Alarm is activated after `OPEN_WINDOW_MINUTES` if actual temperature is not higher than the minimum temperature (+ 1) i.e. the window is not closed yet.
 *
 * Every `VCC_MONITORING_DELAY_MIN` minutes the battery voltage is measured. A low battery (below `VCC_VOLTAGE_LOWER_LIMIT_MILLIVOLT` Millivolt)
 * is indicated by beeping and flashing LED every 24 seconds. Only the beep (not the flash) is significantly longer than at open window detection.
 *
 * Power consumption:
 * Power consumption is 6uA at sleep and 2.8 mA at at 1MHz active.
 * Loop needs 2.1 ms and with DEBUG 6.5 ms => active time is ca. 1/10k or 1/4k of total time and power consumption is 500 times more than sleep.
 *   => Loop adds 5% to 12% to total power consumption.
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

#define DEBUG // To see serial output with 115200 baud at pin2
//#define TRACE // To see serial output with 115200 baud at pin2
//#define ALARM_TEST // start alarm immediately if PB0 / D0 is connected to ground - incompatible with Pullup at 0 bootloader

#ifdef DEBUG
#include "TinySerialOut.h"
#endif

#include <avr/boot.h>  // needed for boot_signature_byte_get()
#include <avr/power.h> // needed for clock_prescale_set()
#include <avr/sleep.h> // needed for sleep_enable()
#include <avr/wdt.h>   // needed for WDTO_8S

#define VERSION "1.1"

#ifdef ALARM_TEST
#define ALARM_TEST_PIN PB0
#endif

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
uint16_t sTemperatureAtWindowOpen;
bool sOpenWindowDetected = false;
uint8_t sOpenWindowSampleDelayCounter;

/*
 * VCC monitoring
 */
const uint16_t VCC_VOLTAGE_LOWER_LIMIT_MILLIVOLT = 3550; // 3.7 Volt is normal operating voltage
bool sVCCVoltageTooLow = false;
const uint8_t VCC_MONITORING_DELAY_MIN = 60; // Check VCC every hour
uint16_t sVCCMonitoringDelayCounter = (VCC_MONITORING_DELAY_MIN * 60) / TEMPERATURE_SAMPLE_SECONDS;

//
// ATMEL ATTINY85
//
//                                 +-\/-+
//           RESET/ADC0 (D5) PB5  1|    |8  Vcc
//   Tone - ADC3/PCINT3 (D3) PB3  2|    |7  PB2 (D2) INT0/ADC1 - TX Debug output
//   Tone inv.   - ADC2 (D4) PB4  3|    |6  PB1 (D1) MISO/DO/AIN1/OC0B/OC1A/PCINT1 - (Digispark) LED
//                           GND  4|    |5  PB0 (D0) OC0A/AIN0 - Alarm Test if connected to ground
//

#define LED_PIN  PB1
#define TONE_PIN PB4
#define TONE_PIN_INVERTED PB3

#define ADC_TEMPERATURE_CHANNEL_MUX 15
#define ADC_1_1_VOLT_CHANNEL_MUX 12
#define SHIFT_VALUE_FOR_REFERENCE REFS2

#if (LED_PIN == TX_PIN)
#error "LED pin must not be equal TX pin."
#endif

#define LED_PULSE_LENGTH 200 // 500 is well visible, 200 is OK
#if (LED_PULSE_LENGTH < 150)
#error "LED_PULSE_LENGTH must at least be 150, since the code after digitalWrite(LED_PIN, 1) needs 150 us."
#endif

uint8_t sMCUSRStored; // content of MCUSR register at startup

void PWMtone(uint8_t aPin, unsigned int aFrequency, unsigned long aDurationMillis = 0);
void delayAndSignalOpenWindowDetectionAndLowVCC();
void alarm();
void initPeriodicSleepWithWatchdog(uint8_t tSleepMode, uint8_t aWatchdogPrescaler);
void sleepDelay(uint16_t aSecondsToSleep);
void delayMilliseconds(unsigned int aMillis);
uint16_t readADCChannelWithReferenceOversample(uint8_t aChannelNumber, uint8_t aReference, uint8_t aOversampleExponent);
void checkVCC();
void changeDigisparkClock();

/***********************************************************************************
 * Code starts here
 ***********************************************************************************/

void setup() {

    /*
     * store MCUSR early for later use
     */
    sMCUSRStored = MCUSR;
    MCUSR = 0; // to prepare for next reset or power on

#ifdef DEBUG
    /*
     * Initialize the serial pin as an output for Serial.print like debugging
     */
    initTXPin();
#endif

    /*
     * initialize the pins
     */
    pinMode(LED_PIN, OUTPUT);
    pinMode(TONE_PIN_INVERTED, OUTPUT);
    pinMode(TONE_PIN, OUTPUT);
#ifdef ALARM_TEST
    pinMode(ALARM_TEST_PIN, INPUT_PULLUP);
#endif

    changeDigisparkClock();

#ifdef DEBUG
    writeString(F("START " __FILE__ "\nVersion " VERSION " from " __DATE__ "\nAlarm delay = "));
    writeUnsignedByte(OPEN_WINDOW_ALARM_DELAY_MINUTES);
    writeString(F(" minutes\n"));
#endif

#ifdef TRACE
    writeString(F("MCUSR="));
    writeUnsignedByteHexWithPrefix(sMCUSRStored);
    writeString(F(" LFuse="));
    writeUnsignedByteHexWithPrefix(boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS));
    writeString(F(" WDTCR="));
    writeUnsignedByteHexWithPrefix(WDTCR);
    writeString(F(" OSCCAL="));
    writeUnsignedByteHexWithPrefix(OSCCAL);
    write1Start8Data1StopNoParity('\n');
#endif

    /*
     * init sleep mode and wakeup period
     */
    initPeriodicSleepWithWatchdog(SLEEP_MODE_PWR_DOWN, WDTO_8S);
// disable Arduino delay() and millis() timer0 and also its interrupts which kills the deep sleep.
    TCCR0B = 0;

    /*
     * Initialize ADC channel and reference
     */
    readADCChannelWithReferenceOversample(ADC_TEMPERATURE_CHANNEL_MUX, INTERNAL1V1, 0);

    if (sMCUSRStored & (1 << PORF)) {
        PWMtone(TONE_PIN, 2200, 100); // signal power on. Not entered after reset.
    }
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

#ifdef ALARM_TEST
    if (!digitalRead(ALARM_TEST_PIN)) {
#ifdef DEBUG
        writeString(F("Test signal out"));
        write1Start8Data1StopNoParity('\n');
#endif
        alarm();
    }
#endif

    checkVCC();

// disable digital input buffer to save power
// do not disable buffer for outputs whose values are read back
    DIDR0 = (1 << ADC1D) | (1 << ADC2D) | (1 << ADC3D) | (1 << AIN1D) | (1 << AIN0D);

    /*
     * wait 8 seconds, since ATtinys temperature is increased after the micronucleus boot process
     */
    sleep_cpu()
    ;
}

/*
 * Shift temperature history values ad insert new value.
 * Check if temperature decreases after power on.
 * Check if window was just opened.
 * If window was opened check if window still open -> ALARM
 * Loop needs 2.1 ms and with DEBUG 6.5 ms => active time is ca. 1/10k or 1/4k of total time and power consumption is 500 times more than sleep.
 * 2 ms for Temperature reading
 * 0.25 ms for processing
 * 0.05 ms for LED flashing
 *  + 4.4 ms for DEBUG
 */
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
     * needs 2 ms
     */
    sTemperatureArray[0] = readADCChannelWithReferenceOversample(ADC_TEMPERATURE_CHANNEL_MUX, INTERNAL1V1, 4);
    tTemperatureNewSum += sTemperatureArray[0];

#ifdef DEBUG
    // needs 4.4 ms
    writeString(F("Temp="));
    writeUnsignedInt(sTemperatureArray[0]);
    writeString(F(" Old="));
    writeUnsignedInt(tTemperatureOldSum);
    writeString(F(" New="));
    writeUnsignedInt(tTemperatureNewSum);
    write1Start8Data1StopNoParity('\n');
#endif
    // activate LED after reading to signal it. Do it here to reduce delay below.
    digitalWrite(LED_PIN, 1);

    /*
     * Check if reason for reset was "power on" (in contrast to "reset pin") and temperature is decreasing
     */
    if ((sMCUSRStored & (1 << PORF))
    /* we are a a power on boot */
    && (sTemperatureArray[(sizeof(sTemperatureArray) / sizeof(uint16_t)) - 1] == 0)
            && (sTemperatureArray[(sizeof(sTemperatureArray) / sizeof(uint16_t)) - 2] > 0)
            /*
             * array is almost full, so check if temperature is lower than at power on time which means,
             * we ported the sensor from a warm place to its final one.
             */
            && (sTemperatureArray[0] < sTemperatureArray[(sizeof(sTemperatureArray) / sizeof(uint16_t)) - 2])) {
        // Start from beginning, clear temperature array
#ifdef DEBUG
        writeString(F("Detected porting to a colder place -> reset\n"));
#endif
        for (i = 0; i < (sizeof(sTemperatureArray) / sizeof(uint16_t)) - 1; ++i) {
            sTemperatureArray[i] = 0;
        }
    } else {

        if (!sOpenWindowDetected) {
            /*
             * Check if window just opened
             */
            // tTemperatureOldSum can be 0 -> do not use tTemperatureNewSum < tTemperatureOldSum - (TEMPERATURE_DELTA_THRESHOLD_DEGREE * TEMPERATURE_COMPARE_AMOUNT)
            if (tTemperatureNewSum + (TEMPERATURE_DELTA_THRESHOLD_DEGREE * TEMPERATURE_COMPARE_AMOUNT) < tTemperatureOldSum) {
#ifdef DEBUG
                writeString(F("Detected window just opened -> check again in "));
                writeUnsignedByte(OPEN_WINDOW_ALARM_DELAY_MINUTES);
                writeString(F(" minutes\n"));
#endif
                sTemperatureMinimumAfterWindowOpen = tTemperatureNewSum;
                sTemperatureAtWindowOpen = tTemperatureNewSum;
                sOpenWindowDetected = true;
                sOpenWindowSampleDelayCounter = 0;
            }
        } else {

            /*
             * Check if window already closed -> start a new detection
             */
            if (tTemperatureNewSum > (sTemperatureMinimumAfterWindowOpen + TEMPERATURE_COMPARE_AMOUNT)) {
                sOpenWindowDetected = false;
#ifdef DEBUG
                writeString(F("Detected window already closed -> start again\n"));
#endif
                // reset history in order to avoid a new detection next sample, since tTemperatureNewSum may still be lower than tTemperatureOldSum
                for (i = 0; i < (sizeof(sTemperatureArray) / sizeof(uint16_t)) - 1; ++i) {
                    sTemperatureArray[i] = 0;
                }
            } else {
                if (tTemperatureNewSum < sTemperatureMinimumAfterWindowOpen) {
                    // set new minimum
                    sTemperatureMinimumAfterWindowOpen = tTemperatureNewSum;
                }

                /*
                 * Check for delay
                 */
                sOpenWindowSampleDelayCounter++;
                if (sOpenWindowSampleDelayCounter >= OPEN_WINDOW_SAMPLES) {
                    /*
                     * After delay, check if still open - Temperature must be 1 degree lower than temperature at time of open detection
                     */
                    if (tTemperatureNewSum <= sTemperatureAtWindowOpen - TEMPERATURE_COMPARE_AMOUNT) {
                        /*
                         * Window still open -> ALARM
                         */
#ifdef DEBUG
                        writeString(F("Detected window still open -> alarm"));
                        write1Start8Data1StopNoParity('\n');
#endif
                        alarm();
                    } else {
                        // Temperature not 1 degree lower than temperature at time of open detection
                        sOpenWindowDetected = false;
#ifdef DEBUG
                        writeString(F("Detected window maybe not really opened -> start again\n"));
#endif
                    }
                } // delay
            } // already closed
        } // !sOpenWindowDetected
    }  // after power on and temperature is decreasing

    /*
     * VCC check every hour
     */
    sVCCMonitoringDelayCounter--;
    if (sVCCMonitoringDelayCounter == 0) {
        sVCCMonitoringDelayCounter = (VCC_MONITORING_DELAY_MIN * 60) / TEMPERATURE_SAMPLE_SECONDS;
        checkVCC(); // needs 4.5 ms
    }

    delayAndSignalOpenWindowDetectionAndLowVCC();
    // deactivate LED before sleeping
    digitalWrite(LED_PIN, 0);

    sleepDelay(TEMPERATURE_SAMPLE_SECONDS);
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
         * Here we have High Frequency PLL Clock (16 or 16.5 MHz)
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
        writeUnsignedByteHexWithPrefix(tOSCCAL);
        writeString(F(" to "));
        writeUnsignedByteHexWithPrefix(tStoredOSCCAL);
        write1Start8Data1StopNoParity('\n');
#endif
        // retrieve the factory-stored oscillator calibration bytes to revert the digispark OSCCAL tweak
        OSCCAL = tStoredOSCCAL;
    }
}

/*
 * Like tone(), but use OCR1B (PB4) + !OCR1B (PB3)
 */
void PWMtone(uint8_t aPin, unsigned int aFrequency, unsigned long aDurationMillis) {
    tone(aPin, aFrequency / 2, aDurationMillis); // specify half frequency -> PWM doubles it
    TCCR1 = TCCR1 & 0x0F; // reset mode and disconnect OC1A pins, keep only prescaler
    GTCCR = (1 << PWM1B) | (1 << COM1B0); // Switch to PWM Mode with OCR1B (PB4) + !OCR1B (PB3) outputs enabled
    OCR1B = OCR1C / 2; // set PWM to 50%
}

/*
 * plays alarm signal for the specified seconds
 */
void playAlarmSignalSeconds(uint16_t aSecondsToPlay) {
#ifdef DEBUG
    writeString(F("Play alarm for "));
    writeUnsignedInt(aSecondsToPlay);
    writeString(F(" seconds\n"));
#endif
    uint16_t tCounter = (aSecondsToPlay * 10) / 13; // == ... * 1000 (ms per second) / (1300ms for a loop)
    while (tCounter-- != 0) {
        // activate LED
        digitalWrite(LED_PIN, 1);
        PWMtone(TONE_PIN, 1100);

        delayMilliseconds(300);

        // deactivate LED
        digitalWrite(LED_PIN, 0);
        PWMtone(TONE_PIN, 2200);

        delayMilliseconds(1000);
    }
}

/*
 * Generates a 2200 | 1100 Hertz tone signal for 10 minutes and then play it 10 seconds with intervals starting from 24 seconds up to 5 minutes.
 */
void alarm() {

    playAlarmSignalSeconds(600);  // 600 seconds / 10 Minutes

#ifdef DEBUG
    writeString(F("After 10 minutes alarm now play it for 10 s with 24 to 600 s delay\n"));
#endif

    uint16_t tDelay = 24;
    while (true) {
        sleepDelay(tDelay); // Start with 24 seconds
        playAlarmSignalSeconds(10);
        noTone(TONE_PIN);
        if (tDelay < 600) { // up to 5 minutes
            tDelay++;
        }
    }
}

/*
 * Delay to flash LED only for a short period to save power.
 * If open window detected, increase pulse length to give a visual feedback
 */
void delayAndSignalOpenWindowDetectionAndLowVCC() {
    if (sOpenWindowDetected) {
        PWMtone(TONE_PIN, 2200);
        delayMicroseconds(2000); // 2000 can be heard
        noTone(TONE_PIN);
        delayMicroseconds(20000); // to let the led light longer
    } else if (sVCCVoltageTooLow) {
        PWMtone(TONE_PIN, 1600);
        delayMicroseconds(20000);
        noTone(TONE_PIN);
    } else {
        delayMicroseconds(LED_PULSE_LENGTH - 150);  // - 150 for the duration from digitalWrite(LED_PIN, 1) until here
    }
}

void sleepDelay(uint16_t aSecondsToSleep) {
    ADCSRA = 0; // disable ADC -> saves 150 - 200 uA
    for (uint16_t i = 0; i < (aSecondsToSleep / 8); ++i) {
        sleep_cpu()
        ;
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
    ADMUX = aChannelNumber | (aReference << SHIFT_VALUE_FOR_REFERENCE);

// ADCSRB = 0; // free running mode if ADATE is 1 - is default
// ADSC-StartConversion ADATE-AutoTriggerEnable ADIF-Reset Interrupt Flag
    ADCSRA = (_BV(ADEN) | _BV(ADSC) | _BV(ADATE) | _BV(ADIF) | ADC_PRESCALE8);

    for (uint8_t i = 0; i < (1 << aOversampleExponent); i++) {
        /*
         * wait for free running conversion to finish.
         * Do not wait for ADSC here, since ADSC is only low for 1 ADC Clock cycle on free running conversion.
         */
        loop_until_bit_is_set(ADCSRA, ADIF);

        ADCSRA |= _BV(ADIF); // clear bit to recognize conversion has finished
        // Add value
        tSumValue += ADCL | (ADCH << 8);
//        tSumValue += (ADCH << 8) | ADCL; // this does NOT work!
    }
    ADCSRA &= ~_BV(ADATE); // Disable auto-triggering (free running mode)
    return (tSumValue >> aOversampleExponent);
}

uint16_t getVCCVoltageMillivolt(void) {
    // use AVCC with external capacitor at AREF pin as reference
    uint8_t tOldADMUX = ADMUX;
    /*
     * Must wait >= 200 us if reference has to be switched to VSS
     * Must wait >= 70 us if channel has to be switched to ADC_1_1_VOLT_CHANNEL_MUX
     */
    if ((ADMUX & (INTERNAL << SHIFT_VALUE_FOR_REFERENCE)) || ((ADMUX & 0x0F) != ADC_1_1_VOLT_CHANNEL_MUX)) {
        // switch AREF
        ADMUX = ADC_1_1_VOLT_CHANNEL_MUX | (DEFAULT << SHIFT_VALUE_FOR_REFERENCE);
        // and wait for settling
        delayMicroseconds(400); // experimental value is > 200 us
    }
    uint16_t tVCC = readADCChannelWithReferenceOversample(ADC_1_1_VOLT_CHANNEL_MUX, DEFAULT, 2);
    ADMUX = tOldADMUX;
    /*
     * Do not wait for reference to settle here, since it may not be necessary
     */
    return ((1024L * 1100) / tVCC);
}

void checkVCC() {
    uint16_t sVCCVoltageMillivolt = getVCCVoltageMillivolt();
#ifdef DEBUG
    writeString(F("VCC="));
    writeUnsignedInt(sVCCVoltageMillivolt);
    writeString(F("mV\n"));
#endif
    if (sVCCVoltageMillivolt < VCC_VOLTAGE_LOWER_LIMIT_MILLIVOLT) {
        sVCCVoltageTooLow = true;
    }
}

/*
 * Watchdog wakes CPU periodically and all we have to do is call sleep_cpu();
 * aWatchdogPrescaler (see wdt.h) can be one of
 * WDTO_15MS, 30, 60, 120, 250, WDTO_500MS
 * WDTO_1S to WDTO_8S
 */
void initPeriodicSleepWithWatchdog(uint8_t tSleepMode, uint8_t aWatchdogPrescaler) {
    sleep_enable()
    ;
    set_sleep_mode(tSleepMode);
    MCUSR = ~_BV(WDRF); // Clear WDRF in MCUSR

#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
#define WDTCSR  WDTCR
#endif
    // Watchdog interrupt enable + reset interrupt flag -> needs ISR(WDT_vect)
    uint8_t tWDTCSR = _BV(WDIE) | _BV(WDIF) | (aWatchdogPrescaler & 0x08 ? _WD_PS3_MASK : 0x00) | (aWatchdogPrescaler & 0x07); // handles that the WDP3 bit is in bit 5 of the WDTCSR register,
    WDTCSR = _BV(WDCE) | _BV(WDE); // clear lock bit for 4 cycles by writing 1 to WDCE AND WDE
    WDTCSR = tWDTCSR; // set final Value
}

/*
 * This interrupt wakes up the cpu from sleep
 */
ISR(WDT_vect) {
    ;
}

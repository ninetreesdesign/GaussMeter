/*  @name make a gaussmeter from a linear Hall-Effect Sensor, analog out
    @description read from original HES Si72B218 (can find no data for this part; guessing at sensitivity of 5mV/G.
                 plan to change to A1308 1.3mV/G for bigger range
                 HW is T3.2 and 128x64 I2C OLED
 */
 
#ifndef ARDUINO_H
#include <arduino.h>
#endif
#ifndef SPI_H
#include <SPI.h>
#endif
#ifndef WIRE_H
#include <Wire.h>
#endif

#include <Adafruit_GFX.h>       // https://learn.adafruit.com/adafruit-gfx-graphics-library/graphics-primitives
#include <Adafruit_SSD1306.h>   // OLED display
// printing shortcuts
#define HWSERIAL Serial1
#define CONSOLE Serial  // USB port debug console (IDE)
#define E echoString    // a generalized print fctn
#define P Serial.print
#define Pln Serial.println

///////////////////////////////////////////////////////////////////////////////////
const String   TITLE   = "Gauss Meter";    //
const String   VERSION_NUM   = "v1.1";    //
const uint16_t LED_INTERVAL  = 1000;
const uint16_t HES_INTERVAL  = 250;
const bool     HWSERIAL_FLAG = 0;           // send msgs to hardware serial port
const bool     CONSOLE_FLAG  = 1;           // Route text msgs to standard output port
///////////////////////////////////////////////////////////////////////////////////

const uint8_t HES_PIN        = A3;          // IN  analog Hall Effect Sensor
const uint8_t STATUS_LED_PIN =  9;          // OUT PWM
const uint8_t WARN_LED_PIN   = 10;          // OUT PWM
const uint8_t OLED_RESET_PIN = 14;          // not used?
const uint8_t MAG_PWR_PIN    = 12;

// font sizes
const uint8_t SMALL  = 1;
const uint8_t MED    = 2;
const uint8_t LARGE  = 3;
const uint8_t XLARGE = 4;

const uint16_t status_led_flash_duration = 12;
const uint16_t ADC_MAX_VAL = 16;            // number of bits

char    msg[40];                            // container for print strings
char    pole[2];
uint16_t sec = 0;

elapsedMillis since_sec     = 0;            // ms to count a sec
elapsedMillis since_hes     = 0;
elapsedMillis since_led     = 0;

Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET_PIN);
///

void setup() {
    initializeStuff();                  // IO pin states, serial comm
    readADC(HES_PIN);
}


void loop() {
    const float ofst = -0.210;
    const float Vref = 3.3;
    const float G_scale = -1 * 1/0.005;   // polarity and sensor's sensitivity    // 5mV per G
    float k = 0.80;
    static float V;
    static float V_prev = 0;

    if (since_sec >= 1000) {
        since_sec = 0;
        sec ++;
#if 0
        display.setTextSize(SMALL);
        display.setCursor(0,20);
        sprintf(msg,"%05d", sec);
        display.print(msg);
        display.display();
#endif
    }

    if (since_hes >= HES_INTERVAL) {
        since_hes = 0;
        
        V = Vref * ((float)readADC(HES_PIN)/65535);           // +-1.0 f.s.
        // filter it:         new_filtered_value = k * raw_sensor_value + (1 - k) * old_filtered_value
        V = V*k + V_prev*(1-k);
        V_prev = V;
        float G = G_scale * (V - ofst - Vref/2);    // mid value is 0 flux; N/S indicated by polarity
        if (abs(G) < 2.0) G = 0;    // ignore small fluctuations near zero
        if (sec>1) {                // wait for value to settle at beginning
            display.setTextSize(XLARGE);
            display.setCursor(0,36);   // shortens the minus sign if present
            sprintf(msg,"%4.0f", abs(G));
            display.print(msg); 
            display.setTextSize(LARGE);
            display.setCursor(0,33);
            pole[1] = '\0';
            (G > 0) ? pole[0] = 'N' : pole[0] = 'S';  // print the pole
            display.print(pole);
            display.display();
            // Serial.printf("    %5.3fV  %5.1f G ", V, G);
            Serial.printf("    %5.3f V   [%s] %5.1f G \r", pole, abs(G));
            E("\n");
        }
    }

#if 0
    // flash status led each interval (second)
    if (since_led >= LED_INTERVAL) {
        since_led = 0;
        analogWrite(STATUS_LED_PIN,21);
    }
    if (since_led >= status_led_flash_duration) {   // turn off led after N ms
        analogWrite(STATUS_LED_PIN,0);
    }
#endif
}


/* Functions */

uint16_t readADC(byte addr) {
        digitalWriteFast(MAG_PWR_PIN,1); delay(4);  // turn on sensors and let settle
        uint16_t ADC = 0;
        int N = 8;
        for (int i = 0; i < N; i++) {
            ADC += analogRead(addr);
            delayMicroseconds(100);
        }
        ADC /= N;
        digitalWriteFast(MAG_PWR_PIN,0);
        return ADC;
}

///
void splashLED(uint8_t pin, uint8_t n) {
    for (uint8_t i = 0; i < n; i++) {
        digitalWriteFast(pin, 1);
        delay(15);
        digitalWriteFast(pin, 0);
        delay(200);
    }
    delay(200);
}

/// Output text string to either or both serial ports: hardware serial, USB serial
void echoString(String str) {                   // print Arduino String
    // prints msg to selected port(s)
    if (CONSOLE_FLAG)  CONSOLE.print(str);      // print to console
    if (HWSERIAL_FLAG) HWSERIAL.print(str);     // print to HW serial port
}

///
void initializeStuff() {
    pinMode(STATUS_LED_PIN,OUTPUT);
    pinMode(LED_BUILTIN,OUTPUT);
    pinMode(MAG_PWR_PIN,OUTPUT);

    analogReadResolution(ADC_MAX_VAL);     // actual ADC resolution is 10 on T3.2
    analogWriteResolution(ADC_MAX_VAL);    // actual internal DAC is 12 on T3.2
    // set faster PWM clock                   https://www.pjrc.com/teensy/td_pulse.html
    analogWriteFrequency(4, 93750 );       // Hz (much faster clock to avoid flickering or audio whine
    analogWriteResolution(8);              // analogWrite value 0 to 2^N-1, or 2^N for steady ON
    
    splashLED(LED_BUILTIN, 3);

    // oled setup
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
        delay(1000);
        Serial.println(F("    OLED SSD1306 allocation failed."));
    }
    display.setTextWrap(false);
    display.setTextColor(WHITE,BLACK);

    display.clearDisplay();
    display.display();
    delay(200);
    display.setTextSize(MED);
    display.setCursor(76,0);
    display.print(VERSION_NUM);
    display.display();
    delay(600);
    display.clearDisplay();
    display.setTextSize(MED);
    display.setCursor(0,0);
    display.print(TITLE); 



    delay(400);  // initialize console Serial port
    if(CONSOLE_FLAG) {
        Serial.begin(115200);
        while (!Serial && (millis() < 5000)) {} // include timeout if print console isn't opened
        E(F("\n    Serial port(s) initialized.\n\n"));
    }
    Pln(VERSION_NUM);

}


/// EOF

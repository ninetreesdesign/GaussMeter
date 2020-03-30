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
#define HWSERIAL Serial1        // optional output to hardware serial
#define CONSOLE Serial          // USB port debug console (IDE)
#define P   Serial.print
#define Pf  Serial.printf
#define Pln Serial.println

///////////////////////////////////////////////////////////////////////////////////
const String   TITLE         = "Gauss Meter";
const String   VERSION_NUM   = "v1.1.1";    // added bar graph
const uint16_t LED_INTERVAL  = 1000;
const uint16_t HES_INTERVAL  = 350;
const bool     HWSERIAL_FLAG = 0;           // send msgs to hardware serial port
const bool     CONSOLE_FLAG  = 1;           // Route text msgs to standard output port
const bool     PRINT_TIME_FLAG = 0;
///////////////////////////////////////////////////////////////////////////////////

const uint8_t HES_PIN        = A3;          // IN  analog Hall Effect Sensor
const uint8_t STATUS_LED_PIN =  9;          // OUT PWM
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
const float ofst = 0; //-0.210;
const float V_ref = 3.3;
const float G_scale = -1 * 1/0.005;         // polarity and sensor's sensitivity    // 5mV per G
float k = 0.90;                             // smoothing factor

elapsedMillis since_sec     = 0;            // ms to count a sec
elapsedMillis since_hes     = 0;
elapsedMillis since_led     = 0;

Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET_PIN);   // create instance 

/// 
void setup() {
    initializeStuff();                      // IO pin states, serial comm
    readHES(HES_PIN);                       // stabilize initial reading
}

///
void loop() {
    static float V;
    static float V_prev = 0;
    pole[1] = '\0';

    if (since_hes >= HES_INTERVAL) {                // update reading to display
        since_hes = 0;
        uint16_t adc = analogRead(HES_PIN);
        V = V_ref * (float)adc/pow(2,ADC_MAX_VAL);  // +-1.0 f.s.
        V = V*k + V_prev*(1-k);                     // filter it
        V_prev = V;
        float G = G_scale * (V - V_ref/2 - ofst);   // mid value is 0 flux; N/S indicated by polarity
        if (abs(G) < 3.0) G = 0.0001;               // ignore small fluctuations near zero
        if (sec > 0) {                              // wait for value to settle at beginning
            oledDrawBarGraph(G);
            // print Gauss value
            display.setTextSize(XLARGE);
            display.setCursor(-4,37);
            sprintf(msg,"%4.0f", abs(G));
            display.print(msg);
            display.setTextSize(LARGE);
            display.setCursor(112,36);  // on bottom row of pixels 
            // print the pole
            if (abs(G) <= 3)  pole[0] = ' ';
            else if (G > 0)   pole[0] = 'N';
            else if (G < 0)   pole[0] = 'S';
            display.print(pole);
            display.display();
            Pf("    %5u %5.3fV %5.1fG %s \r", adc, V, abs(G), pole);
            P("\n");
        }
    }

    if (since_sec >= 1000) {
        since_sec = 0;
        sec ++;
        if (PRINT_TIME_FLAG) {
            display.setTextSize(SMALL);
            display.setCursor(0,20);
            sprintf(msg,"  %05d sec", sec);
            display.print(msg);
            display.display();
            P(msg);
            P("\n");
        }
    }

    // flash status led each interval (second)
    if (since_led >= LED_INTERVAL) {
        since_led = 0;
        analogWrite(STATUS_LED_PIN,11);             // set very dim
    }
    if (since_led >= status_led_flash_duration) {   // turn off led after N ms
        analogWrite(STATUS_LED_PIN,0);
    }

}


/* Functions */

/// read hall effect sensor
float readHES(byte addr) {
    byte N = 38;  // read N times
    // digitalWriteFast(MAG_PWR_PIN,1);
    // delay(24);  // turn on sensors and let settle
    uint16_t ADC = readADC(addr, N);
    // digitalWriteFast(MAG_PWR_PIN,0);
    return ADC;
}

/// read from internal ADC
uint16_t readADC(byte addr, byte N) {
    uint32_t ADC = 0;
    for (int i = 0; i < N; i++) {
        ADC += analogRead(addr);
        delayMicroseconds(100);
    }
    ADC /= N;
    Serial.println(ADC);

    return (uint16_t)ADC;
}

void oledDrawBarGraph(int val) {
    byte SCRN_W = 128;
    byte SCRN_W2 = SCRN_W / 2;
    byte TOP = 20;
    byte HT  = 5;
    int w = map(int(abs(val)),1,(V_ref*abs(G_scale)/2),0,SCRN_W2);
    display.fillRect(0,TOP,SCRN_W,HT-0,0);                      // blank the rows
    for (int i = 0; i < SCRN_W; i++) {
        display.drawLine(i,TOP+HT-1, i,TOP+HT-1, (i % 4 > 0));  // draw H axis
    }
    display.fillRect(SCRN_W2-1,TOP-4, 3,HT+3,  1);                // center mark
    // draw a bunch of short vertical lines to make bar graph
    if (val >= 0) { // NORTH
        for (int i = 0; i < w; i++) {
            display.drawLine(i+SCRN_W2,TOP, i+SCRN_W2,TOP+HT-1, (i==0 || i % 4 > 0));
        }
    }
    else {          // SOUTH
        // for (int i = w; i=0; i--) {
        for (int i = 0; i < w; i++) {
            display.drawLine(SCRN_W2 - i-1,TOP, SCRN_W2 - i-1,TOP+HT-1, (i==0 || i % 4 > 0));
        }
    }
    display.display();
}

///
void splashLED(uint8_t pin, uint8_t n) {
    for (uint8_t i = 0; i < n; i++) {
        digitalWriteFast(pin, 1);
        delay(15);
        digitalWriteFast(pin, 0);
        delay(200);
    }
    delay(100);
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
    // print splash to OLED and CONSOLE
    display.clearDisplay();
    display.display();
    delay(100);
    display.setTextSize(MED);
    display.setCursor(0,0);
    display.print(TITLE);
    display.setCursor(24,32);
    display.print(VERSION_NUM);
    display.display();
    delay(600);
    display.clearDisplay();
    display.setTextSize(SMALL);
    display.setCursor(51,0);
    display.print("Gauss");

    // initialize console Serial port
    Serial.begin(115200);
    while (!Serial && (millis() < 5000)) {} // include timeout if print console isn't opened
    P(F("\n    Serial port(s) initialized.\n"));
    
    Pln(VERSION_NUM);
}


/// EOF

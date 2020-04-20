/*  @name make a gaussmeter from a linear Hall-Effect Sensor, analog out
    @description Usethe Allegro A1308 1.3mV/G lowest sensitivity available for highest flux range
                 Uses the Teensy 3.2 and 128x64 I2C OLED
    todo:
        add a button input for user to zero the offset
        add a photo of device and a schematic to git repo
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
#define P   Serial.print
#define Pf  Serial.printf
#define Pln Serial.println

/////////////////////////////// User Settings //////////////////////////////////////////////////
const String   TITLE         = "Gauss Meter";
const String   VERSION_NUM   = "v1.2.2";    // removed unused code
const uint16_t LED_INTERVAL  = 1000;
const uint16_t HES_INTERVAL  = 5;           // fast output rate to capture changing field
const uint16_t OLED_INTERVAL = 200;
const bool     CONSOLE_FLAG  = 1;           // Route readings to standard output port
const bool     OLED_TIME_FLAG = 1;          // include elapsed time on oled screen
const bool     TESLA_UNITS_FLAG = 0;        // Gauss/Teslas
////////////////////////////////////////////////////////////////////////////////////////////////

const uint8_t HES_PIN        = A2;          // IN  analog Hall Effect Sensor
const uint8_t STATUS_LED_PIN =  9;          // OUT PWM
const uint8_t OLED_RESET_PIN = -1;          // none on board  
const uint8_t MAG_PWR_PIN    = 12;
const uint8_t OLED_PWR_PIN   = 21;          // powered from adjacent I/O pins for easy mounting
const uint8_t OLED_GND_PIN   = 20;          //    "

// font sizes
const uint8_t SMALL  = 1;
const uint8_t MED    = 2;
const uint8_t LARGE  = 3;
const uint8_t XLARGE = 4;

const uint16_t status_led_flash_duration = 12;
const uint16_t ADC_MAX_VAL = 16;            // number of bits
const float    HES_SENSITIVITY = 1.3e-3;    // PN A1308:1.3mV/G
const float    G_scale = -1 * 1 / HES_SENSITIVITY; // polarity and sensor's sensitivity
const float    ofst = (42-21) * HES_SENSITIVITY; // observed for each sensor
const float    V_ref = 3.297;
const float    alpha = 0.70;                // filter smoothing factor

char msg[40];                               // container for print strings
char pole[2];
String units_string = "";

uint32_t total_sec = 0;                     // 136 yrs worth of seconds

elapsedMillis since_sec     = 0;            // ms to count a sec
elapsedMillis since_hes     = 0;
elapsedMillis since_oled    = 0;
elapsedMillis since_led     = 0;

Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET_PIN);   // create instance

///
void setup() {
    pinMode(OLED_GND_PIN, OUTPUT);
    pinMode(OLED_PWR_PIN, OUTPUT);
    digitalWrite(OLED_GND_PIN, 0);
    digitalWrite(OLED_PWR_PIN, 1);
    initializeStuff();                      // IO pin states, serial comm
    readHES(HES_PIN);                       // stabilize initial reading
    delay(200);
}

///
void loop() {
    static float V;                                 // Voltage
    static float V_prev = 0;                        // keep for 1st order filter
    static float G;                                 // flux (Gauss)
    float T;                                        // flux (Tesla)
    if (since_hes >= HES_INTERVAL) {                // make flux measurement
        since_hes = 0;

        uint16_t adc = analogRead(HES_PIN);         // read Hall-effect sensor with ADC
        V = V_ref * (float)adc / pow(2, ADC_MAX_VAL);  // convert to voltage 
        V = V * alpha + V_prev * (1 - alpha);       // filter it
        V_prev = V;
        
        G = G_scale * (V - (V_ref / 2) - ofst);     // mid value is 0 flux; North = positive
        if (abs(G) < 2.0)
            G = 0.0001;                             // ignore small fluctuations near zero
        T = G/10000;

        if (TESLA_UNITS_FLAG == 0)
            Pf("  %5.1f G \n", G);                  // print in Gauss units
        else
            Pf("  %5.1f mT \n", 1000*T);            // print in mT units
    }

    if (since_oled >= OLED_INTERVAL) {              // update OLED screen
        since_oled = 0;
        oledUpdateValue(G);
    }

    if (since_sec >= 1000) {
        since_sec = 0;
        total_sec ++;
        uint16_t sec  = total_sec % 60;
        uint16_t hour = total_sec / 3600;
        uint16_t min  = (total_sec % 3600)/60;
        if (OLED_TIME_FLAG) {                       // display elapsed time since power-up
            display.setTextSize(SMALL);
            display.setCursor(0, 27);
            sprintf(msg, " %02d:%02d:%02d", hour, min, sec);
            display.print(msg);
        }
    }
}   // end of loop


/* Functions */

/// read hall effect sensor
float readHES(byte addr) {
    byte N = 8;                     // read N times
    int t  = 200;                   // delay between reads
    uint16_t adc = readADC(addr, N, t);
    return (float)adc;
}

/// read from internal ADC
uint16_t readADC(byte addr, byte N, int t) {
    uint32_t adc = 0;
    for (int i = 0; i < N; i++) {
        adc += analogRead(addr);
        delayMicroseconds(t);
    }
    adc /= N;
    return (uint16_t)adc;
}

///
void oledUpdateValue( float G) {
// read flux
// a high value means open probe
// use global units flag to convert to Tesla if set
    pole[1] = '\0';                             // display N or S
    // print Gauss value
    display.setTextSize(XLARGE);
    display.setCursor(-4, 37);

    if (abs(G) < 1200) {                        // high value
        oledDrawBarGraph(G);
        if (TESLA_UNITS_FLAG == 0)
            sprintf(msg, "%4.0f ", G);
        else
            sprintf(msg, "%5.1f mT", 1000*(G/10000));
        display.print(msg);
        display.setTextSize(LARGE);
        display.setCursor(104, 36);             // on bottom row of pixels
        // print the pole (negative = South)
        if (abs(G) <= 2.0)  pole[0] = ' ';
        else if (G > 0)     pole[0] = 'N';
        else if (G < 0)     pole[0] = 'S';
        display.print(pole);
    }
    else {              // a high value with a pulldn resistor indicates probe not connected
        oledDrawBarGraph(0);
        sprintf(msg, "     ");                  // erase prev value
        display.print(msg);
        display.setTextSize(LARGE);
        display.setCursor(5, 37);
        sprintf(msg, " Open?   ");
        display.print(msg);
        display.setCursor(111, 36);             // on bottom row of pixels
        pole[0] = ' ';
        display.print(pole);
    }
    display.display();                          // update the screen
}

///
void oledDrawBarGraph(int val) {
    byte SCRN_W = 128;
    byte SCRN_W2 = SCRN_W / 2;
    byte TOP = 20;
    byte HT  = 5;
    // scale bar width to max Gauss range 
    int w = map(int(abs(val)), 1, (V_ref/2 / HES_SENSITIVITY), 0, SCRN_W2); // ~ +-1200G
    display.fillRect(0, TOP, SCRN_W, HT - 0, 0);                // blank the rows
    for (int i = 0; i < SCRN_W; i++) {
        display.drawLine(i, TOP + HT - 1, i, TOP + HT - 1, (i % 4 > 0)); // draw H axis
    }
    display.fillRect(SCRN_W2 - 1, TOP - 4, 3, HT + 3,  1);        // center mark
    // draw a bunch of short vertical lines to make bar graph
    if (val >= 0) { // NORTH
        for (int i = 0; i < w; i++) {
            display.drawLine(i + SCRN_W2, TOP, i + SCRN_W2, TOP + HT - 1, (i == 0 || i % 4 > 0));
        }
    }
    else {          // SOUTH
        // for (int i = w; i=0; i--) {
        for (int i = 0; i < w; i++) {
            display.drawLine(SCRN_W2 - i - 1, TOP, SCRN_W2 - i - 1, TOP + HT - 1, (i == 0 || i % 4 > 0));
        }
    }
    display.display();
}

/// LED indicates program has started
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
void initOled() {
    // oled setup
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
        delay(800);
        Serial.println(F("OLED:   SSD1306 allocation failed."));
    }
    else
        Serial.println("OLED:   initialized");
    display.setTextWrap(false);
    display.setTextColor(WHITE, BLACK);
    display.clearDisplay();
    display.display();
    delay(100);
}

///
void initializeStuff() {
    pinMode(STATUS_LED_PIN, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(MAG_PWR_PIN, OUTPUT);
    // set matching resolutions for ADC & DAC
    analogReadResolution(ADC_MAX_VAL);     // actual ADC resolution is 10 on T3.2
    analogWriteResolution(ADC_MAX_VAL);    // actual internal DAC is 12 on T3.2
    // set faster PWM clock                   https://www.pjrc.com/teensy/td_pulse.html
    analogWriteFrequency(4, 93750 );       // Hz (much faster clock to avoid flickering or audio whine
    analogWriteResolution(8);              // analogWrite value 0 to 2^N-1, or 2^N for steady ON
    
    if (TESLA_UNITS_FLAG == 0) units_string = " Gauss";
    else units_string = "mTeslas";
    splashLED(LED_BUILTIN, 3);
    // set up OLED display
    initOled();
    // print title and version
    display.setTextSize(MED);
    display.setCursor(0, 0);
    display.print(TITLE);
    display.setCursor(24, 32);
    display.print(VERSION_NUM);
    display.display();
    delay(400);
    display.clearDisplay();
    display.setTextSize(SMALL);
    display.setCursor(44, 0);               // meter display title
    display.print(units_string);

    // initialize console Serial port
    Serial.begin(115200);
    while (!Serial && (millis() < 5000)) {} // include timeout if print console isn't opened
    P(F("USB Serial port initialized.\n"));
    
    // print configuration
    P("Version:     ");
    Pln(VERSION_NUM);
    Pf("Sensitivity: %3.1fmV/G\n", 1000*HES_SENSITIVITY);
    Pf("G_scale:     %5.1f \n", G_scale);
    Pf("Offset:      %5.3fV\n", ofst);
    Pf("V_ref:       %5.3fV\n", V_ref);
    P ("Units:       ");
    Pln(units_string);
}

/// EOF

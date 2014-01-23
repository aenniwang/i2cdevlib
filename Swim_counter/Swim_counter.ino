// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#define ENABLE_SD 1

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <LiquidCrystal.h>
#include <MsTimer2.h>
#ifdef ENABLE_SD
#include <SD.h>
#endif
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define TIMER_INTERVAL_MS 100
#define TIMER_INTERVAL_ADJ_MS 20
#define TIME_DIRECTION_CHANGE_MS 3500
/* TIME_DIRECTION_CHANGE_S*SECOND_PER_MS/TIMER_INTERVAL_MS
 * 30*1000/100=300
 */
#define TIME_DIRECTION_CHANGE_COUNT (TIME_DIRECTION_CHANGE_MS/TIMER_INTERVAL_MS)
bool is_yaw_adjusted = false;
float ypr_direction = 1000.0f;
uint16_t timer_temp;
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high
#ifdef ENABLE_SD
File logFile ;
#endif

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

 //uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

#define IO_LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
#define IO_LCD1602_RS 12   
#define IO_LCD1602_RW 11
#define IO_LCD1602_EN 10
#define IO_LCDD4 9
#define IO_LCDD5 8
#define IO_LCDD6 7
#define IO_LCDD7 6
#define IO_LCD1602_BL 5

#ifdef ENABLE_LCD
// rs, enable, d4, d5, d6, d7 
LiquidCrystal lcd(IO_LCD1602_RS, IO_LCD1602_EN, IO_LCDD4, IO_LCDD5, IO_LCDD6, IO_LCDD7);
#endif

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
//uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile static uint16_t timer;
// 100ms every time.
void updatetimer()
{
    timer++;
}

uint16_t get_time(uint16_t timer_0, uint16_t timer_1) /*timer_1 is a later time!*/
{
    if(timer_0<timer_1)
        return timer_1-timer_0;
    else
        return 65536-timer_0+timer_1;
}

/* 
    Firest Place: is_fsm_update_begin  
                  direction_change_count is no meaning now, use it as a counter
*/
uint8_t direction_change_count = 0;
bool is_fsm_update_begin = false;
#define YAW_FORWARD  true
#define YAW_BACKWARD false
bool direction=YAW_FORWARD;
void fsm_check_started(VectorFloat*acc)
{
#define ACC_FOR_START 4
float acc_max;
#define REPEAT_COUNT 8
#define timer_check_fsm timer_temp
#ifdef ENABLE_LCD
    lcd.setCursor(0,1);
    lcd.print(direction_change_count);
#endif
    acc_max = acc->x*acc->x+acc->y*acc->y+acc->z*acc->z;
    if(acc_max>ACC_FOR_START)
    {
            /* At least 1.5 second interval less 
             * than 6 seconds need to repeat a acc check.*/
            if(get_time(timer_check_fsm,timer)<15)
            {
                return;
            }
            if(get_time(timer_check_fsm,timer)<60) /*6 seconds*/
            {
                timer_check_fsm = timer;
                direction_change_count++;

                if(direction_change_count>=REPEAT_COUNT)
                {
                    direction_change_count = 0;
                    direction = YAW_FORWARD;
#ifdef ENABLE_LCD
                    lcd.clear();
                    lcd.setCursor(0,0);
                    lcd.print("D");
                    lcd.setCursor(2,0);
                    if(direction == YAW_FORWARD)
                        lcd.print("F");
                    else
                        lcd.print("R");
                    lcd.setCursor(10,0);
                    lcd.print("R");
                    lcd.setCursor(12,0);
                    lcd.print(direction_change_count);
#endif
                    is_fsm_update_begin=true;
                    return;
                }
            }

            timer_check_fsm =timer;
    }
#undef timer_check_fsm 
}


void fsm_update(VectorFloat*acc,Quaternion * q,float fsm_ypr)
{
    static bool last_direction;
    bool current_direction;
    static bool first_time=true;
    static uint16_t last_direction_change_time;
    float y;

    y= fsm_ypr - ypr_direction;

    if(y>M_PI)
        y=y-2*M_PI;
    else if(y<-M_PI)
        y=2*M_PI+y;

#ifdef ENABLE_LCD
    lcd.setCursor(0,1);
    lcd.print("Angel");
    lcd.setCursor(6,1);
    lcd.print(y*180/M_PI);
#endif
    
// debug
//    Serial.print(ypr_direction*180/M_PI);
 //   Serial.print("\t");
  //  Serial.print(fsm_ypr*180/M_PI);
   // Serial.print("\t");
    //Serial.print(y*180/M_PI);
    //Serial.print("\n");
    // angel -90 ---- 90 
    // TODO: should limit the yaw range??? for example -45 -- 45??
    if((y<0 && y>(-M_PI/2))||(y>=0)&&(y<(M_PI/2)))
        current_direction= YAW_FORWARD;
    else    
        current_direction= YAW_BACKWARD;

    if(first_time)
    {
#ifdef ENABLE_LCD
        bl_set_on();
#endif
        first_time = false;
        last_direction = current_direction;
        direction_change_count = 0;
        last_direction_change_time = timer;
    }

    //direction not changed, do nothing
    if(current_direction!= last_direction)
    {
        // Only change the direction if the direction remains a TIME_DIRECTION_CHANGE_S time.
        if(get_time(last_direction_change_time,timer) > TIME_DIRECTION_CHANGE_COUNT )
        {
            if(last_direction != direction)
            {
                direction_change_count++;
                direction = current_direction;

#ifdef ENABLE_LCD
                bl_set_on();
                lcd.setCursor(0,0);
                lcd.print("D");
                lcd.setCursor(2,0);
                if(direction == YAW_FORWARD)
                    lcd.print("F");
                else
                    lcd.print("R");
                lcd.setCursor(10,0);
                lcd.print("R");
                lcd.setCursor(12,0);
                lcd.print(direction_change_count);
#endif
                //            Serial.print("Direction changed, count=");
                //            Serial.println(direction_change_count);
                /* Direction changed!*/
            }
        }
        last_direction_change_time = timer;
        last_direction = current_direction;
    }
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

uint16_t schedule_timer;
uint16_t sd_sync_schedule_timer;
void setup() {
    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // lcd
#ifdef ENABLE_LCD
    pinMode(IO_LCD1602_RW, OUTPUT);
    digitalWrite(IO_LCD1602_RW, LOW);
    pinMode(IO_LCD1602_BL,OUTPUT);
    digitalWrite(IO_LCD1602_BL, HIGH);
    lcd.begin(16,2);
#endif

#ifdef ENABLE_SD
    //
    // Initialize SD
#define IO_SD_SPI_CS 4
    Serial.print("Initializing SD card...");
    pinMode(10, OUTPUT);

#ifdef ENABLE_LCD
    //
    // LCD_STATUS BIT 14
    // 'E' - SD Failed
    // 'S' - SD OK
    // 'W' - SD Writing
    lcd.setCursor(13,0);
    lcd.print("E");
#endif
    if (!SD.begin(IO_SD_SPI_CS)) {
        Serial.println("initialization failed!");
        return;
    }
    Serial.println("initialization done.");
#if 0
    if(SD.exists("MPU.txt"))
    {
        logFile = SD.open("test.txt");
    }
    else
#endif
    {
        logFile = SD.open("MPU.txt", FILE_WRITE);
    }
    if (logFile) {
        Serial.println("Create a MPU.txt");
    } else {
        // if the file didn't open, print an error:
        Serial.println("error opening test.txt");
    }
#if 0
    // read from the file until there's nothing else in it:
    while (logFile.available()) {
        logFile.read();
    }
#endif
    
    if(logFile)
    {
        logFile.println("-----------------------------------------------");
        logFile.println("A NEW LOG FILE");
        logFile.println("-----------------------------------------------");
    }
#ifdef ENABLE_LCD
    lcd.print("S");
#endif
#endif

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif


    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    MsTimer2::set(TIMER_INTERVAL_MS - TIMER_INTERVAL_ADJ_MS, updatetimer);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
//    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
//    while (Serial.available() && Serial.read()); // empty buffer
//    while (!Serial.available());                 // wait for data
//    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(IO_LED_PIN, OUTPUT);

    MsTimer2::start();
    schedule_timer = timer;
    sd_sync_schedule_timer = timer;
}
#ifdef ENABLE_LCD
uint16_t timer_bl;
#define BL_ON_MS 40
void bl_set_on()
{
    digitalWrite(IO_LCD1602_BL, HIGH);
    timer_bl =timer;
}

void bl_schedule()
{
    if(timer_bl)
    {
        if(get_time(timer_bl,timer)>BL_ON_MS)
        {
            digitalWrite(IO_LCD1602_BL, HIGH);
//            digitalWrite(IO_LCD1602_BL, LOW);

            timer_bl = 0;
        }
    }
    else
        //digitalWrite(IO_LCD1602_BL,LOW);
        digitalWrite(IO_LCD1602_BL,HIGH);
}
#endif

void yaw_auto_adjust(float ypr)
{
#define timer_last timer_temp
#define TIME_YAW_ADJUST_INTERVAL 3000
#define TIME_YAW_ADJUST_COUNT (TIME_YAW_ADJUST_INTERVAL/TIMER_INTERVAL_MS)

    if(get_time(timer_last,timer)>TIME_YAW_ADJUST_COUNT)
    {
        Serial.print("YAW Adjust...      ");
        Serial.print(timer);
        Serial.print("\t");
        Serial.print(timer_last);
        Serial.print("\t");
        Serial.print(ypr_direction);
       Serial.print("\t");
        Serial.println(ypr);
#ifdef ENABLE_LCD
        lcd.setCursor(0,0);
        lcd.print("YAW Adjust ...");
#endif
#define YAW_ADJUST_LIMITION (M_PI/360) // 1 degreen
        if(((ypr_direction>ypr) && (ypr_direction-ypr<YAW_ADJUST_LIMITION))
                ||((ypr_direction<ypr)&&((ypr-ypr_direction)<YAW_ADJUST_LIMITION)))
        {
//            Serial.println("Adjust finished.");
            is_yaw_adjusted = true;
#ifdef ENABLE_LCD
            lcd.clear();
#endif
            return ;
        }

        ypr_direction = ypr;
        timer_last = timer;

#ifdef ENABLE_LCD
        lcd.setCursor(0,1);
        lcd.print(ypr_direction*180/M_PI);
#endif
    }
#undef timer_last
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
bool zero_motion =false;
bool bsync_sd=false;
void loop() {
 
RESTART:
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (bsync_sd ||( !mpuInterrupt && fifoCount < packetSize)) {
#ifdef ENABLE_SD
        if(bsync_sd && logFile)
        {
            logFile.flush();
            bsync_sd = false;
            goto RESTART;
        }
#endif
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println("FIFO overflow!");

    }
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        if(!is_yaw_adjusted)
        {
            yaw_auto_adjust(ypr[0]);
            goto RESTART;
        }

        VectorFloat g;
#if 1
        g.x=aa.x/4096.0f;
        g.y=aa.y/4096.0f;
        g.z=aa.z/4096.0f;
#endif
        if(!is_fsm_update_begin)
        {
#ifdef ENABLE_LCD
            lcd.setCursor(0,0);
            lcd.print("Prepare");
#endif
            fsm_check_started(&g);
        }

        if(get_time(schedule_timer,timer)>2)
        {
            // Serial.println("Schedule fsm_update");
            // Serial.print("ypr\t");
            // Serial.print(ypr[0] * 180/M_PI);
            // Serial.print("\t");
            // Serial.print(ypr[1] * 180/M_PI);
            // Serial.print("\t");
            // Serial.println(ypr[2] * 180/M_PI);

            if(is_fsm_update_begin)
            {
                fsm_update(&g,&q,ypr[0]);
#ifdef ENABLE_LCD
                bl_schedule();
#endif
            }
#ifdef ENABLE_SD
            logFile.print(timer);logFile.print(" ## ");
            logFile.print("ACC: ");
            logFile.print(g.x);logFile.print(" ");
            logFile.print(g.y);logFile.print(" ");
            logFile.print(g.z);logFile.print(" ");
            logFile.print("YPR: ");
            logFile.print(ypr[0]);logFile.print(" ");
            logFile.print(ypr[1]);logFile.print(" ");
            logFile.println(ypr[2]);
#endif
            schedule_timer = timer;
        }

#ifdef ENABLE_SD
        if(get_time(sd_sync_schedule_timer,timer)>100)
        {
            bsync_sd = true;
            sd_sync_schedule_timer = timer;
        }
#endif
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(IO_LED_PIN, blinkState);
    }
}
/////////////////////////////////////////////////////////
 

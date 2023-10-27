// RP2040 Pico Servo CAM board Arduino Sketch Template
// J. Bradshaw 20231027
// Overclocked to 250MHz processor clock (core temp of ~31 deg C)

#include <Wire.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <SPI.h>              // RasPi Pico uses RX(4), CSn(5), SCK(2), and TX(3)
#include <Servo.h>

// CAN defines and variables
#define MY_CAN_ID 0x1234ABCD

#define SPI_CS_PIN 5      // GP5 is default CS
#define CAN_INT_PIN 6     // GP6 is default Interrupt in pin

MCP_CAN can(SPI_CS_PIN); // Set CS pin

unsigned char can_flagRecv = 0;
unsigned char can_rxBufLen = 0;
unsigned char can_rx_buf[8];
unsigned long can_rxId = 0;

unsigned long can_txId = MY_CAN_ID; // 
unsigned char can_txBufLen = 0;
unsigned char can_tx_buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};

static unsigned long can_millisLastTx1 = 0;

int led = LED_BUILTIN; // the PWM pin the LED is attached to
int brightness = 0;    // how bright the LED is
int fadeAmount = 20;    // how many points to fade the LED by
unsigned long tm;  // Used to capture millis() timestamp

Servo servo1;  // create servo object to control a servo
Servo servo2;  // create servo object to control a servo
Servo servo3;  // create servo object to control a servo
Servo servo4;  // create servo object to control a servo
Servo servo5;  // create servo object to control a servo
Servo servo6;  // create servo object to control a servo

int SERVO_MIN = 750;
int SERVO_MAX = 2250;
int SERVO_CENT = (SERVO_MIN + SERVO_MAX) / 2;
int SERVO_SPAN = (SERVO_MAX - SERVO_MIN);

void setup() {
    
  delay(100); // short delay upon startup for voltages to settle on IMU and Xbee

  // Initialize I2C bus
  Wire1.setSDA(26); // set pins for I2C1_SCL (GP27) and I2C1_SDA (GP26)
  Wire1.setSCL(27);
  Wire1.begin();

  // Serial is the USB Interface
  Serial.begin(115200); // USB Interface by default (no need to define pins here)
  
  // Initialize Serial 1 (UART0 !!)  on pin GP0 (TX) and pin GP1 (RX)
  pinMode(0,OUTPUT);  // pin 0 is TX UART0
  pinMode(1,INPUT);   // pin 1 is RX UART0
  Serial1.setTX(0);
  Serial1.setRX(1);   
  Serial1.begin(115200);

  // Setup CAN SPI bus for MCP2515 CAN Controller IC
  SPI.setSCK(2);
  SPI.setTX(3);
  SPI.setRX(4);  
  SPI.setCS(5);

  // Initialize MCP2515 running at 16MHz with a baudrate of 250kb/s and the masks and filters disabled.
  if(can.begin(MCP_ANY, CAN_250KBPS, MCP_16MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");
  
  can.setMode(MCP_NORMAL);   // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(CAN_INT_PIN, INPUT);  // Configuring pin for /INT input
  //attachInterrupt(CAN_INT_PIN, MCP2515_ISR, FALLING); // start interrupt  
}

void setup1() {
    // Initialize the led
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);

  servo1.attach(16, SERVO_MIN, SERVO_MAX);  // attaches the servo on GPIO_16 to the servo object
  servo2.attach(17, SERVO_MIN, SERVO_MAX);  // attaches the servo on GPIO_17 to the servo object
  servo3.attach(18, SERVO_MIN, SERVO_MAX);  // attaches the servo on GPIO_18 to the servo object
  servo4.attach(19, SERVO_MIN, SERVO_MAX);  // attaches the servo on GPIO_19 to the servo object
  servo5.attach(20, SERVO_MIN, SERVO_MAX);  // attaches the servo on GPIO_20 to the servo object
  servo6.attach(21, SERVO_MIN, SERVO_MAX);  // attaches the servo on GPIO_21 to the servo object
}


// main loop core 0
void loop() {
  static unsigned long serial_last = 0;
  tm = millis();

  //Serial.println(tm);

  float yawPosRad = sinf(tm * .01) + 1.0/3.0;
  float pitchPosRad = sinf(tm * .01) + 2.0/3.0;
  float rollPosRad = sinf(tm * .01);
  
  // check if a CAN receive Interrupt has occured
  if(CAN_MSGAVAIL == can.checkReceive()) {
    can_flagRecv = 1;
    // read data,  len: data length, buf: data buf
    can.readMsgBuf(&can_rxId, &can_rxBufLen, can_rx_buf);
  }
    
  if (can_flagRecv) {
    // check if get data
    can_flagRecv = 0;                   // clear flag

    Serial.printf("CAN RX id=0x%5X data: ", can_rxId);
    for (int i = 0; i < can_rxBufLen; i++) {
      Serial.printf(" %2X", can_rx_buf[i]);
    }
    Serial.println();
  }// if flagRecv

  // Transmit CAN data to ID can_txId
  if(millis() > can_millisLastTx1 + 100){
    can_millisLastTx1 = millis();
    // send data:  id = 0x00, standard frame, data len = 8, can_tx_buf: data buf
      
    int yawRaw = (int)(((yawPosRad / 2.0*PI) - .5) * 65535);
    int byteLow = yawRaw & 0x00ff;
    int byteHigh = (yawRaw >> 8) & 0x00ff;
    can_tx_buf[0] = byteLow;
    can_tx_buf[1] = byteHigh;

    int pitchRaw = (int)((pitchPosRad / PI) * 65535);
    byteLow = pitchRaw & 0x00ff;
    byteHigh = (pitchRaw >> 8) & 0x00ff;
    can_tx_buf[2] = byteLow;
    can_tx_buf[3] = byteHigh;

    int rollRaw = (int)((rollPosRad / PI) * 65535);
    byteLow = rollRaw & 0x00ff;
    byteHigh = (rollRaw >> 8) & 0x00ff;
    can_tx_buf[4] = byteLow;
    can_tx_buf[5] = byteHigh;

    can_tx_buf[6] = 0;
    can_tx_buf[7] = 0;

    can_txId = MY_CAN_ID;
    can.sendMsgBuf(can_txId, 8, can_tx_buf);  // transmit the CAN data on the bus
  }  

  // serial transmit
  if(tm > serial_last + 10){
    serial_last = tm;

    Serial.printf("Core Temp: %.1fC ", analogReadTemp());
    Serial.printf("%d CAN TX id=%d data: ", millis(), can_txId);
    for(int i=0;i<8;i++){
      Serial.printf(" %2X", can_tx_buf[i]);
    }
    Serial.printf("\r\n");    
  }
}

//main loop for core 1
void loop1(){
  tm = millis();

  static unsigned long led_last = 0;
  static unsigned long servo1_last = 0;

  // flash led to indiccate activity
  analogWrite(led, brightness);    // set the brightness
  // change the brightness for next time through the loop:
  brightness = brightness + fadeAmount;
  if (brightness <= 0 || brightness >= 255) {    // reverse the direction of the fading at the ends of the fade
    fadeAmount = -fadeAmount;
  }

  // update servo 1 every 80 milliseconds
  if(tm > servo1_last + 80){
    float scaledServoValNorm1 = sinf(tm * .005) / (PI/2.0) * 1/6;
    float scaledServoValNorm2 = sinf(tm * .005) / (PI/2.0) * 2/6;
    float scaledServoValNorm3 = sinf(tm * .005) / (PI/2.0) * 3/6;
    float scaledServoValNorm4 = sinf(tm * .005) / (PI/2.0) * 4/6;
    float scaledServoValNorm5 = sinf(tm * .005) / (PI/2.0) * 5/6;
    float scaledServoValNorm6 = sinf(tm * .005) / (PI/2.0);
  
    int servoOut_us1 = (int)(SERVO_CENT + (scaledServoValNorm1*(SERVO_SPAN/2.0)));
    int servoOut_us2 = (int)(SERVO_CENT + (scaledServoValNorm2*(SERVO_SPAN/2.0)));
    int servoOut_us3 = (int)(SERVO_CENT + (scaledServoValNorm3*(SERVO_SPAN/2.0)));
    int servoOut_us4 = (int)(SERVO_CENT + (scaledServoValNorm4*(SERVO_SPAN/2.0)));
    int servoOut_us5 = (int)(SERVO_CENT + (scaledServoValNorm5*(SERVO_SPAN/2.0)));
    int servoOut_us6 = (int)(SERVO_CENT + (scaledServoValNorm6*(SERVO_SPAN/2.0)));
    
    // servo.write(servoOut); // Used to write 'Degrees"
    servo1.writeMicroseconds(servoOut_us1);
    servo2.writeMicroseconds(servoOut_us2);
    servo3.writeMicroseconds(servoOut_us3);
    servo4.writeMicroseconds(servoOut_us4);
    servo5.writeMicroseconds(servoOut_us5);
    servo6.writeMicroseconds(servoOut_us6);
  }
  delay(20);
} // loop1

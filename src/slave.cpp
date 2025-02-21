// Feather9x_TX
// It is designed to work with the other example Feather9x_RX
#ifdef SLAVE


#include <SPI.h>
#include <RH_RF95.h>
#include <Servo.h>
#include <Adafruit_BNO055.h>
#include <MS5607.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>


#define RFM95_CS    8
#define RFM95_INT   7
#define RFM95_RST   4

#define RXPin 11  // GPS TX
#define TXPin 10  // GPS RX
#define GPSBaud 9600



// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

#define ESC_PIN 9
#define YAW_SERVO 12
#define PITCH_SERVO 13

int throttle;


// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
Servo esc;
Servo yaw;
Servo pitch;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
MS5607 P_Sens;
TinyGPSPlus gps;

SoftwareSerial gpsSerial(RXPin, TXPin);

float P_val,T_val,H_val;

bool SHUTDOWN;


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  delay(100);

  Serial.println("Feather LoRa RX");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

    // Attach the ESC to the Arduino
  esc.attach(ESC_PIN);

  pitch.attach(PITCH_SERVO);
  yaw.attach(YAW_SERVO);


  // Step 1: Send maximum throttle for calibration
  Serial.println("Calibrating ESC - Sending max throttle...");
  esc.writeMicroseconds(2000); // Maximum throttle
  delay(2000);                 // Wait for ESC to register max throttle

  // Step 2: Send minimum throttle to complete calibration
  Serial.println("Calibrating ESC - Sending min throttle...");
  esc.writeMicroseconds(1000); // Minimum throttle
  delay(2000);                 // Wait for ESC to register min throttle


  // ESC should now be armed
  Serial.println("ESC calibrated and armed.");
  throttle = 1000;


  

  if (!bno.begin()) {
    Serial.println("IMU not intialized!");
  }

  if(!P_Sens.begin()){
    Serial.println("Error in Communicating with sensor, check your connections!");
  }else{
    Serial.println("MS5607 initialization successful!");
  }

  gpsSerial.begin(GPSBaud);
  Serial.println("GPS Module Initialized...");

  SHUTDOWN = false;
}

void loop() {
  // Read from RF95 (LoRa) module
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (!rf95.recv(buf, &len)) {
      Serial.println("Receive failed");
      return;
    }

    buf[len] = '\0';  // Null-terminate the received string
    Serial.print("Received: ");
    Serial.println((char*)buf);

    if (strcmp((char*)buf, "off") == 0) {
      throttle = 1000;
      esc.writeMicroseconds(throttle);
      SHUTDOWN = true;
      return;
    } 
    else if (strcmp((char*)buf, "on") == 0) {
      SHUTDOWN = false;
      return;
    }

    if (!SHUTDOWN) {
      // Parse three integers from received data
      int t, p, y;
      if (sscanf((char*)buf, "%d %d %d", &t, &p, &y) == 3) {
        // Clamp values to safe ranges
        throttle = constrain(t, 1000, 2000);
        int pitch_val = constrain(p, 0, 180);
        int yaw_val = constrain(y, 0, 180);

        // Apply values to ESC and servos
        esc.writeMicroseconds(throttle);
        pitch.write(pitch_val);
        yaw.write(yaw_val);

        Serial.print("Throttle set to: ");
        Serial.print(throttle);
        Serial.print(" | Pitch set to: ");
        Serial.print(pitch_val);
        Serial.print("° | Yaw set to: ");
        Serial.print(yaw_val);
        Serial.println("°");

        // Send acknowledgment
        char response[32];
        snprintf(response, sizeof(response), "ACK: %d %d %d", throttle, pitch_val, yaw_val);
        rf95.send((uint8_t*)response, strlen(response));
        rf95.waitPacketSent();
      } 
      else {
        Serial.println("Invalid command format");
      }
    }
  }
}


#endif
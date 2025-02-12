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

int throttle;


// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
Servo esc;
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

    if (strcmp((char*)buf, "off") == 0) {
        throttle = 1000;
        esc.writeMicroseconds(throttle);
        SHUTDOWN = true;
    } else if (strcmp((char*)buf, "on") == 0) {
      SHUTDOWN = false;
    }

    
    if (!SHUTDOWN) {
      if (rf95.recv(buf, &len)) {
        digitalWrite(LED_BUILTIN, HIGH);
        throttle = atoi((char*)buf);


        // Clamp throttle between 1000 and 2000
        throttle = constrain(throttle, 1000, 2000);
        esc.writeMicroseconds(throttle);

        // Send a reply
        uint8_t data[15];
        sprintf((char *)data, "received: %s", buf);
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
          
        digitalWrite(LED_BUILTIN, LOW);
      } else {
        Serial.print("Receive failed");
      }
    } else {
      if (rf95.recv(buf, &len)) {
        uint8_t data[15];
        sprintf((char *)data, "SHUTDOWN: command %s not executed", buf);
        Serial.print("SHUTDOWN");
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
      }
    }
    
    
  }
  if (!SHUTDOWN) {
    // Read sensor data (Temperature, Pressure, Altitude)
    if (P_Sens.readDigitalValue()) {
      T_val = P_Sens.getTemperature();
      P_val = P_Sens.getPressure();
      H_val = P_Sens.getAltitude();
    } else {
      Serial.print(" | Sensor Error ");
    }

    // Read IMU (BNO055) orientation data
    sensors_event_t event; 
    bno.getEvent(&event);
    float imu_x = event.orientation.x;
    float imu_y = event.orientation.y;
    float imu_z = event.orientation.z;

    // Read GPS data
    float latitude = 0.0, longitude = 0.0;
    if (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
      if (gps.location.isUpdated()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
      }
    }

    // Print final formatted line
    Serial.print("Throttle: ");
    Serial.print(throttle);
    Serial.print(" | Temp: ");
    Serial.print(T_val);
    Serial.print("C | Pressure: ");
    Serial.print(P_val);
    Serial.print("mBar | Altitude: ");
    Serial.print(H_val);
    Serial.print("m | IMU: ");
    Serial.print(imu_x, 2);
    Serial.print("°, ");
    Serial.print(imu_y, 2);
    Serial.print("°, ");
    Serial.print(imu_z, 2);
    Serial.print("° | GPS: ");
    
    if (latitude != 0.0 && longitude != 0.0) {
      Serial.print(latitude, 6);
      Serial.print(", ");
      Serial.print(longitude, 6);
    } else {
      Serial.print("No Fix");
    }
    Serial.println(); // Move to a new line after one complete update
  }
}

#endif
//***********************************************************************
// Hexapod Slave ESP32
// - Receives pre-computed servo angles from Master ESP32 via ESP-NOW
// - Directly drives Legs 4, 5, 6 (9 servos on its own GPIO pins)
// - No IK math, no PS5, no gait logic — just receive and write
//
// Hardware: ESP32 DevKit WROOM-32D/32U
// Servos:   MG996R (PWM 610us-2400us, 50Hz)
// Library:  ESP32Servo by Kevin Harrington (Arduino Library Manager)
//           ESP-NOW: built into ESP32 Arduino core — NO install needed
//
// IMPORTANT — ONE-TIME SETUP:
//   1. Flash this file to the Slave ESP32 FIRST.
//   2. Open Serial Monitor at 115200 baud.
//   3. Note the MAC address printed at boot: "Slave MAC: XX:XX:XX:XX:XX:XX"
//   4. Copy that MAC into SLAVE_MAC_ADDRESS[] in Hexapod_MASTER.ino.
//   5. Then flash the Master.
//
// NO WIRES needed between Master and Slave (beyond shared power/GND
// for the servo supply — the ESP-NOW link is purely wireless).
//***********************************************************************

#include <esp_now.h>    // Built-in — no install needed
#include <WiFi.h>       // Built-in — needed to init Wi-Fi for ESP-NOW
#include <ESP32Servo.h> // Install: Arduino Library Manager "ESP32Servo"


//***********************************************************************
// Slave Angle Packet — must match Master struct EXACTLY (same field order,
// same types, no checksum field). Compiler packing matches because both
// ESPs use the identical ESP32 toolchain.
//***********************************************************************
struct SlavePacket {
  int16_t coxa4;
  int16_t femur4;
  int16_t tibia4;
  int16_t coxa5;
  int16_t femur5;
  int16_t tibia5;
  int16_t coxa6;
  int16_t femur6;
  int16_t tibia6;
};

// Double-buffer: ESP-NOW callback writes to rxBuf, main loop reads from
// activePkt. Prevents a half-written packet being applied mid-frame.
volatile SlavePacket rxBuf;
SlavePacket activePkt;
volatile bool newPacket = false;


//***********************************************************************
// Pin Definitions — Slave ESP32 (Legs 4, 5, 6 only)
// Safe PWM-capable GPIO pins on ESP32 WROOM-32.
// Avoid: 0,2,12,15 (boot strapping), 6-11 (flash), 34-39 (input only)
// Same GPIO numbers as Master's leg 1-3 layout — symmetric wiring.
//***********************************************************************
const int COXA4_SERVO  = 13;
const int FEMUR4_SERVO = 14;
const int TIBIA4_SERVO = 27;
const int COXA5_SERVO  = 26;
const int FEMUR5_SERVO = 25;
const int TIBIA5_SERVO = 33;
const int COXA6_SERVO  = 32;
const int FEMUR6_SERVO = 23;
const int TIBIA6_SERVO = 22;


//***********************************************************************
// Servo Objects — Slave owns Legs 4, 5, 6 only
//***********************************************************************
Servo coxa4_servo;
Servo femur4_servo;
Servo tibia4_servo;
Servo coxa5_servo;
Servo femur5_servo;
Servo tibia5_servo;
Servo coxa6_servo;
Servo femur6_servo;
Servo tibia6_servo;

// Watchdog: track time of last valid packet for timeout detection
unsigned long lastPacketTime = 0;
const unsigned long PACKET_TIMEOUT_MS = 500; // 500ms = 25 missed frames at 20ms


//***********************************************************************
// ESP-NOW Receive Callback
// Called automatically by ESP-NOW stack when a packet arrives.
// Runs in Wi-Fi task context — keep it short, no servo writes here.
// Just copy data into rxBuf and set the flag for main loop to consume.
//***********************************************************************
void on_data_recv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  if (data_len != sizeof(SlavePacket)) {
    Serial.printf("ESP-NOW: unexpected packet size %d (expected %d)\n",
                  data_len, (int)sizeof(SlavePacket));
    return;
  }

  memcpy((void*)&rxBuf, data, sizeof(SlavePacket));
  newPacket = true;
  lastPacketTime = millis();
}


//***********************************************************************
// setup()
//***********************************************************************
void setup()
{
  Serial.begin(115200);

  // --- ESP-NOW Init ---
  // Station mode must be set before esp_now_init().
  // Slave does NOT connect to any router — Wi-Fi hardware used only for ESP-NOW.
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Print own MAC address — paste this into Master's SLAVE_MAC_ADDRESS[]
  Serial.print("Slave MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAILED — halting");
    while (true) delay(1000);
  }

  // Register receive callback — fires on every incoming ESP-NOW packet
  esp_now_register_recv_cb(on_data_recv);

  Serial.println("ESP-NOW ready — listening for Master packets");

  // --- ESP32Servo Init ---
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Attach Legs 4-6 servos (MG996R pulse range: 610-2400 us at 50Hz)
  coxa4_servo.attach(COXA4_SERVO,   610, 2400);
  femur4_servo.attach(FEMUR4_SERVO, 610, 2400);
  tibia4_servo.attach(TIBIA4_SERVO, 610, 2400);
  coxa5_servo.attach(COXA5_SERVO,   610, 2400);
  femur5_servo.attach(FEMUR5_SERVO, 610, 2400);
  tibia5_servo.attach(TIBIA5_SERVO, 610, 2400);
  coxa6_servo.attach(COXA6_SERVO,   610, 2400);
  femur6_servo.attach(FEMUR6_SERVO, 610, 2400);
  tibia6_servo.attach(TIBIA6_SERVO, 610, 2400);

  // Move legs 4-6 to neutral position on startup
  coxa4_servo.write(90);  femur4_servo.write(90);  tibia4_servo.write(90);
  coxa5_servo.write(90);  femur5_servo.write(90);  tibia5_servo.write(90);
  coxa6_servo.write(90);  femur6_servo.write(90);  tibia6_servo.write(90);

  lastPacketTime = millis();
}


//***********************************************************************
// apply_packet()
// Atomically copies rxBuf to activePkt, then writes angles to servos.
// Only called from main loop when newPacket flag is true.
// Constraining to 0-180 is a safety net against any unexpected value.
//***********************************************************************
void apply_packet()
{
  // Disable interrupts briefly to get a consistent snapshot of rxBuf
  noInterrupts();
  activePkt = rxBuf;
  newPacket = false;
  interrupts();

  coxa4_servo.write(constrain((int)activePkt.coxa4,  0, 180));
  femur4_servo.write(constrain((int)activePkt.femur4, 0, 180));
  tibia4_servo.write(constrain((int)activePkt.tibia4, 0, 180));

  coxa5_servo.write(constrain((int)activePkt.coxa5,  0, 180));
  femur5_servo.write(constrain((int)activePkt.femur5, 0, 180));
  tibia5_servo.write(constrain((int)activePkt.tibia5, 0, 180));

  coxa6_servo.write(constrain((int)activePkt.coxa6,  0, 180));
  femur6_servo.write(constrain((int)activePkt.femur6, 0, 180));
  tibia6_servo.write(constrain((int)activePkt.tibia6, 0, 180));
}


//***********************************************************************
// Main Loop
// Lightweight — all packet arrival work happens in on_data_recv().
// Main loop checks the flag and applies the latest packet immediately.
//***********************************************************************
void loop()
{
  if (newPacket) {
    apply_packet();
  }

  // Watchdog: warn if master has gone silent
  // Servos naturally hold their last commanded position — no jerk on timeout.
  if ((millis() - lastPacketTime) > PACKET_TIMEOUT_MS) {
    Serial.println("WARNING: No ESP-NOW packet from Master for 500ms");
    lastPacketTime = millis(); // reset timer to avoid flooding serial

    // Optional failsafe — uncomment to return legs 4-6 to neutral if master lost:
    /*
    coxa4_servo.write(90);  femur4_servo.write(90);  tibia4_servo.write(90);
    coxa5_servo.write(90);  femur5_servo.write(90);  tibia5_servo.write(90);
    coxa6_servo.write(90);  femur6_servo.write(90);  tibia6_servo.write(90);
    */
  }
}

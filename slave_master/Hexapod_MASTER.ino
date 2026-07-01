//***********************************************************************
// Hexapod Master ESP32
// - Connects to PS5 DualSense via Bluetooth
// - Runs ALL IK and gait math for all 6 legs
// - Directly drives Legs 1, 2, 3 (9 servos on its own GPIO pins)
// - Sends pre-computed servo angles for Legs 4, 5, 6 to Slave ESP32
//   via ESP-NOW (Wi-Fi based, no wires needed between ESPs)
//
// Hardware: ESP32 DevKit WROOM-32D/32U
// Servos:   MG996R (PWM 610us-2400us, 50Hz)
// Library:  ESP32Servo by Kevin Harrington (Arduino Library Manager)
//           PS5Controller by lemmingDev   (Arduino Library Manager)
//           ESP-NOW: built into ESP32 Arduino core — NO install needed
//
// IMPORTANT — ONE-TIME SETUP:
//   Flash Hexapod_SLAVE.ino first, open Serial Monitor at 115200 baud,
//   and note the MAC address printed on boot (e.g. "AA:BB:CC:DD:EE:FF").
//   Paste that MAC into SLAVE_MAC_ADDRESS below, then flash this file.
//***********************************************************************

#include <PS5Controller.h>   // Install: Arduino Library Manager "PS5Controller" by lemmingDev
#include <ESP32Servo.h>      // Install: Arduino Library Manager "ESP32Servo"
#include <esp_now.h>         // Built-in — no install needed
#include <WiFi.h>            // Built-in — needed to init Wi-Fi for ESP-NOW
#include <math.h>


//***********************************************************************
// ESP-NOW Configuration
// Paste the Slave ESP32's MAC address here (printed on Slave boot)
//***********************************************************************
uint8_t SLAVE_MAC_ADDRESS[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}; // <-- REPLACE THIS

// Callback: fires after each esp_now_send() completes
// Used only for debug — not required for operation
void on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  if (status != ESP_NOW_SEND_SUCCESS)
    Serial.println("ESP-NOW send FAILED");
}


//***********************************************************************
// Pin Definitions — Master ESP32 (Legs 1, 2, 3 only)
// Safe PWM-capable GPIO pins on ESP32 WROOM-32
// Avoid: 0,2,12,15 (boot strapping), 6-11 (flash), 34-39 (input only)
//***********************************************************************
const int COXA1_SERVO  = 13;
const int FEMUR1_SERVO = 14;
const int TIBIA1_SERVO = 27;
const int COXA2_SERVO  = 26;
const int FEMUR2_SERVO = 25;
const int TIBIA2_SERVO = 33;
const int COXA3_SERVO  = 32;
const int FEMUR3_SERVO = 23;
const int TIBIA3_SERVO = 22;

// Battery voltage divider input (input-only ADC pin, safe)
// Use a 100kΩ + 33kΩ divider to bring 12.6V → ~3.1V
const int BATT_VOLTAGE = 34;


//***********************************************************************
// Leg Geometry Constants (unchanged from original)
//***********************************************************************
const int COXA_LENGTH  = 51;
const int FEMUR_LENGTH = 65;
const int TIBIA_LENGTH = 121;

const int TRAVEL = 30;

const long A12DEG = 209440;
const long A30DEG = 523599;

const int FRAME_TIME_MS = 20;

const float HOME_X[6] = {  82.0,   0.0, -82.0,  -82.0,    0.0,  82.0};
const float HOME_Y[6] = {  82.0, 116.0,  82.0,  -82.0, -116.0, -82.0};
const float HOME_Z[6] = { -80.0, -80.0, -80.0,  -80.0,  -80.0, -80.0};

const float BODY_X[6] = { 110.4,  0.0, -110.4, -110.4,    0.0, 110.4};
const float BODY_Y[6] = {  58.4, 90.8,   58.4,  -58.4,  -90.8, -58.4};
const float BODY_Z[6] = {   0.0,  0.0,    0.0,    0.0,    0.0,   0.0};

const int COXA_CAL[6]  = {2, -1, -1, -3, -2, -3};
const int FEMUR_CAL[6] = {4, -2,  0, -1,  0,  0};
const int TIBIA_CAL[6] = {0, -3, -3, -2, -3, -1};


//***********************************************************************
// Slave Angle Packet
// Master computes these angles inside leg_IK() for legs 4-5-6,
// stores them here instead of writing to servos, then sends via ESP-NOW.
// No manual checksum needed — ESP-NOW has built-in hardware CRC.
// NOTE: ESP-NOW max payload is 250 bytes. This struct is 18 bytes — fine.
//***********************************************************************
struct SlavePacket {
  int16_t coxa4;   // leg 4 angles (0-180 degrees)
  int16_t femur4;
  int16_t tibia4;
  int16_t coxa5;
  int16_t femur5;
  int16_t tibia5;
  int16_t coxa6;
  int16_t femur6;
  int16_t tibia6;
  // No checksum field — ESP-NOW handles data integrity internally
};

SlavePacket slavePkt;


//***********************************************************************
// PS5 Button Helpers
// PS5Controller library returns bool for buttons, int8_t for sticks
//***********************************************************************
// Joystick reads: ps5.LStickX(), LStickY(), RStickX(), RStickY()
// Returns -127 to 127. Center = 0.
// Buttons: ps5.Cross(), Circle(), Square(), Triangle()
//          ps5.L1(), R1(), L2(), R2()
//          ps5.Up(), Down(), Left(), Right()   (D-pad)
//          ps5.Options()  = old START
//          ps5.Create()   = old SELECT

// Track previous button state for rising-edge ("ButtonPressed") detection
bool prev_cross     = false;
bool prev_circle    = false;
bool prev_square    = false;
bool prev_triangle  = false;
bool prev_l1        = false;
bool prev_r1        = false;
bool prev_l2        = false;
bool prev_r2        = false;
bool prev_options   = false;
bool prev_create    = false;
bool prev_dpad_up   = false;
bool prev_dpad_down = false;
bool prev_dpad_left = false;
bool prev_dpad_right= false;

// Helper: true only on the frame the button went from released to pressed
#define PRESSED(current, prev) ((current) && !(prev))


//***********************************************************************
// Variable Declarations (same as original, LED vars removed)
//***********************************************************************
unsigned long currentTime;
unsigned long previousTime;

int temp;
int mode;
int gait;
int gait_speed;
int reset_position;
int capture_offsets;

int batt_voltage;
int batt_voltage_index;
int batt_voltage_array[50];
long batt_voltage_sum;

float L0, L3;
float gamma_femur;
float phi_tibia, phi_femur;
float theta_tibia, theta_femur, theta_coxa;

int leg1_IK_control, leg6_IK_control;
float leg1_coxa, leg1_femur, leg1_tibia;
float leg6_coxa, leg6_femur, leg6_tibia;

int leg_num;
int totalX, totalY, totalZ;
int tick, duration, numTicks;
int z_height_left, z_height_right;
int commandedX, commandedY, commandedR;
int translateX, translateY, translateZ;
float step_height_multiplier;
float strideX, strideY, strideR;
float sinRotX, sinRotY, sinRotZ;
float cosRotX, cosRotY, cosRotZ;
float rotOffsetX, rotOffsetY, rotOffsetZ;
float amplitudeX, amplitudeY, amplitudeZ;
float offset_X[6], offset_Y[6], offset_Z[6];
float current_X[6], current_Y[6], current_Z[6];

int tripod_case[6]   = {1,2,1,2,1,2};
int ripple_case[6]   = {2,6,4,1,3,5};
int wave_case[6]     = {1,2,3,4,5,6};
int tetrapod_case[6] = {1,3,2,1,2,3};


//***********************************************************************
// Servo Objects — Master owns Legs 1, 2, 3 only
//***********************************************************************
Servo coxa1_servo;
Servo femur1_servo;
Servo tibia1_servo;
Servo coxa2_servo;
Servo femur2_servo;
Servo tibia2_servo;
Servo coxa3_servo;
Servo femur3_servo;
Servo tibia3_servo;


//***********************************************************************
// setup()
//***********************************************************************
void setup()
{
  Serial.begin(115200);

  // --- ESP-NOW Init ---
  // ESP-NOW uses Wi-Fi hardware but does NOT connect to any router.
  // Station mode must be set before esp_now_init().
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(); // make sure we're not associated with any AP

  Serial.print("Master MAC: ");
  Serial.println(WiFi.macAddress()); // print own MAC for reference

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAILED — halting");
    while (true) delay(1000);
  }

  // Register send callback (optional, useful for debugging)
  esp_now_register_send_cb(on_data_sent);

  // Register the Slave as a peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, SLAVE_MAC_ADDRESS, 6);
  peerInfo.channel = 0;    // 0 = use current Wi-Fi channel
  peerInfo.encrypt = false; // no encryption (add if needed)

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ESP-NOW add peer FAILED — check SLAVE_MAC_ADDRESS");
    while (true) delay(1000);
  }

  Serial.println("ESP-NOW ready");

  // --- ESP32Servo Init ---
  // Required by ESP32Servo before attaching any servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Attach Legs 1-3 servos (MG996R pulse range)
  coxa1_servo.attach(COXA1_SERVO,   610, 2400);
  femur1_servo.attach(FEMUR1_SERVO, 610, 2400);
  tibia1_servo.attach(TIBIA1_SERVO, 610, 2400);
  coxa2_servo.attach(COXA2_SERVO,   610, 2400);
  femur2_servo.attach(FEMUR2_SERVO, 610, 2400);
  tibia2_servo.attach(TIBIA2_SERVO, 610, 2400);
  coxa3_servo.attach(COXA3_SERVO,   610, 2400);
  femur3_servo.attach(FEMUR3_SERVO, 610, 2400);
  tibia3_servo.attach(TIBIA3_SERVO, 610, 2400);

  // --- PS5 Controller Init ---
  // First time pairing: hold CREATE + PS button until light bar flashes fast.
  // Replace MAC below with your PS5 controller's Bluetooth address,
  // OR use ps5.begin() with no argument to accept any PS5 controller.
  ps5.begin("AA:BB:CC:DD:EE:FF");  // <-- REPLACE WITH YOUR PS5 CONTROLLER MAC ADDRESS
  // Alternative (accept any PS5 controller): ps5.begin();

  Serial.println("Waiting for PS5 controller...");
  while (!ps5.isConnected()) {
    delay(100);
  }
  Serial.println("PS5 Controller connected!");

  // Initialize battery monitor array
  for (batt_voltage_index = 0; batt_voltage_index < 50; batt_voltage_index++)
    batt_voltage_array[batt_voltage_index] = 0;
  batt_voltage_sum = 0;
  batt_voltage_index = 0;

  // Clear offsets
  for (leg_num = 0; leg_num < 6; leg_num++) {
    offset_X[leg_num] = 0.0;
    offset_Y[leg_num] = 0.0;
    offset_Z[leg_num] = 0.0;
  }
  capture_offsets = false;
  step_height_multiplier = 1.0;

  mode = 0;
  gait = 0;
  gait_speed = 0;
  reset_position = true;
  leg1_IK_control = true;
  leg6_IK_control = true;
}


//***********************************************************************
// send_slave_packet()
// Sends pre-computed leg 4-5-6 angles to Slave ESP32 via ESP-NOW.
// ESP-NOW is fire-and-forget with built-in CRC — no manual checksum needed.
// Latency: ~1ms. Max payload: 250 bytes. This struct: 18 bytes.
//***********************************************************************
void send_slave_packet()
{
  esp_err_t result = esp_now_send(
    SLAVE_MAC_ADDRESS,
    (uint8_t*)&slavePkt,
    sizeof(SlavePacket)
  );

  if (result != ESP_OK)
    Serial.println("esp_now_send error");
}


//***********************************************************************
// Main Loop
//***********************************************************************
void loop()
{
  currentTime = millis();
  if ((currentTime - previousTime) > FRAME_TIME_MS)
  {
    previousTime = currentTime;

    if (!ps5.isConnected()) {
      Serial.println("PS5 disconnected...");
      delay(100);
      return;
    }

    // Process PS5 inputs and update mode/gait/flags
    process_gamepad();

    // Reset legs to home position when commanded
    if (reset_position == true) {
      for (leg_num = 0; leg_num < 6; leg_num++) {
        current_X[leg_num] = HOME_X[leg_num];
        current_Y[leg_num] = HOME_Y[leg_num];
        current_Z[leg_num] = HOME_Z[leg_num];
      }
      reset_position = false;
    }

    // Run IK for all 6 legs.
    // Legs 0-2: writes directly to local servos.
    // Legs 3-5: stores angles in slavePkt instead of writing servos.
    if (mode < 99) {
      for (leg_num = 0; leg_num < 6; leg_num++)
        leg_IK(leg_num,
               current_X[leg_num] + offset_X[leg_num],
               current_Y[leg_num] + offset_Y[leg_num],
               current_Z[leg_num] + offset_Z[leg_num]);
    }

    // Reset leg lift first-pass flags if not in leg lift mode
    if (mode != 4) {
      leg1_IK_control = true;
      leg6_IK_control = true;
    }

    // Send computed angles for legs 4-5-6 to Slave ESP32
    send_slave_packet();

    battery_monitor();
    print_debug();

    // Mode dispatch (gait and body control)
    if (mode == 1) {
      if (gait == 0) tripod_gait();
      if (gait == 1) wave_gait();
      if (gait == 2) ripple_gait();
      if (gait == 3) tetrapod_gait();
    }
    if (mode == 2) translate_control();
    if (mode == 3) rotate_control();
    if (mode == 4) one_leg_lift();
    if (mode == 99) set_all_90();

    // Update previous button states for next frame edge detection
    prev_cross      = ps5.Cross();
    prev_circle     = ps5.Circle();
    prev_square     = ps5.Square();
    prev_triangle   = ps5.Triangle();
    prev_l1         = ps5.L1();
    prev_r1         = ps5.R1();
    prev_l2         = ps5.L2();
    prev_r2         = ps5.R2();
    prev_options    = ps5.Options();
    prev_create     = ps5.Create();
    prev_dpad_up    = ps5.Up();
    prev_dpad_down  = ps5.Down();
    prev_dpad_left  = ps5.Left();
    prev_dpad_right = ps5.Right();
  }
}


//***********************************************************************
// Process PS5 Controller Inputs
// Replaces original process_gamepad() which used PS2X library.
// PS5 sticks: -127 to 127 (center = 0)
// PS5 buttons: bool (true = held)
// PRESSED() macro detects rising edge (just-pressed this frame)
//***********************************************************************
void process_gamepad()
{
  if (PRESSED(ps5.Down(),  prev_dpad_down))  { mode = 0; gait = 0; reset_position = true; }
  if (PRESSED(ps5.Left(),  prev_dpad_left))  { mode = 0; gait = 1; reset_position = true; }
  if (PRESSED(ps5.Up(),    prev_dpad_up))    { mode = 0; gait = 2; reset_position = true; }
  if (PRESSED(ps5.Right(), prev_dpad_right)) { mode = 0; gait = 3; reset_position = true; }

  if (PRESSED(ps5.Triangle(), prev_triangle)) { mode = 1; reset_position = true; }
  if (PRESSED(ps5.Square(),   prev_square))   { mode = 2; reset_position = true; }
  if (PRESSED(ps5.Circle(),   prev_circle))   { mode = 3; reset_position = true; }
  if (PRESSED(ps5.Cross(),    prev_cross))    { mode = 4; reset_position = true; }

  if (PRESSED(ps5.Options(), prev_options)) {
    gait_speed = (gait_speed == 0) ? 1 : 0;
  }
  if (PRESSED(ps5.Create(), prev_create)) {
    mode = 99;  // set all servos to 90 degrees for calibration
  }

  if (PRESSED(ps5.L1(), prev_l1) || PRESSED(ps5.R1(), prev_r1)) {
    capture_offsets = true;
  }
  if (PRESSED(ps5.L2(), prev_l2) || PRESSED(ps5.R2(), prev_r2)) {
    for (leg_num = 0; leg_num < 6; leg_num++) {
      offset_X[leg_num] = 0;
      offset_Y[leg_num] = 0;
      offset_Z[leg_num] = 0;
    }
    leg1_IK_control = true;
    leg6_IK_control = true;
    step_height_multiplier = 1.0;
  }
}


//***********************************************************************
// Leg IK Routine
// Legs 0, 1, 2 (physical legs 1, 2, 3): write directly to local servos
// Legs 3, 4, 5 (physical legs 4, 5, 6): store angles in slavePkt
//***********************************************************************
void leg_IK(int leg_number, float X, float Y, float Z)
{
  L0 = sqrt(sq(X) + sq(Y)) - COXA_LENGTH;
  L3 = sqrt(sq(L0) + sq(Z));

  if ((L3 < (TIBIA_LENGTH + FEMUR_LENGTH)) && (L3 > (TIBIA_LENGTH - FEMUR_LENGTH)))
  {
    // Tibia angle
    phi_tibia   = acos((sq(FEMUR_LENGTH) + sq(TIBIA_LENGTH) - sq(L3)) / (2 * FEMUR_LENGTH * TIBIA_LENGTH));
    theta_tibia = phi_tibia * RAD_TO_DEG - 23.0 + TIBIA_CAL[leg_number];
    theta_tibia = constrain(theta_tibia, 0.0, 180.0);

    // Femur angle
    gamma_femur = atan2(Z, L0);
    phi_femur   = acos((sq(FEMUR_LENGTH) + sq(L3) - sq(TIBIA_LENGTH)) / (2 * FEMUR_LENGTH * L3));
    theta_femur = (phi_femur + gamma_femur) * RAD_TO_DEG + 14.0 + 90.0 + FEMUR_CAL[leg_number];
    theta_femur = constrain(theta_femur, 0.0, 180.0);

    // Coxa angle
    theta_coxa = atan2(X, Y) * RAD_TO_DEG + COXA_CAL[leg_number];

    switch (leg_number)
    {
      // ---- MASTER: Legs 1, 2, 3 — write directly to local servos ----

      case 0:  // Leg 1
        if (leg1_IK_control == true) {
          theta_coxa = theta_coxa + 45.0;
          theta_coxa = constrain(theta_coxa, 0.0, 180.0);
          coxa1_servo.write(int(theta_coxa));
          femur1_servo.write(int(theta_femur));
          tibia1_servo.write(int(theta_tibia));
        }
        break;

      case 1:  // Leg 2
        theta_coxa = theta_coxa + 90.0;
        theta_coxa = constrain(theta_coxa, 0.0, 180.0);
        coxa2_servo.write(int(theta_coxa));
        femur2_servo.write(int(theta_femur));
        tibia2_servo.write(int(theta_tibia));
        break;

      case 2:  // Leg 3
        theta_coxa = theta_coxa + 135.0;
        theta_coxa = constrain(theta_coxa, 0.0, 180.0);
        coxa3_servo.write(int(theta_coxa));
        femur3_servo.write(int(theta_femur));
        tibia3_servo.write(int(theta_tibia));
        break;

      // ---- SLAVE: Legs 4, 5, 6 — store angles in slavePkt ----

      case 3:  // Leg 4
        if (theta_coxa < 0)
          theta_coxa = theta_coxa + 225.0;
        else
          theta_coxa = theta_coxa - 135.0;
        theta_coxa = constrain(theta_coxa, 0.0, 180.0);
        slavePkt.coxa4  = int(theta_coxa);
        slavePkt.femur4 = int(theta_femur);
        slavePkt.tibia4 = int(theta_tibia);
        break;

      case 4:  // Leg 5
        if (theta_coxa < 0)
          theta_coxa = theta_coxa + 270.0;
        else
          theta_coxa = theta_coxa - 90.0;
        theta_coxa = constrain(theta_coxa, 0.0, 180.0);
        slavePkt.coxa5  = int(theta_coxa);
        slavePkt.femur5 = int(theta_femur);
        slavePkt.tibia5 = int(theta_tibia);
        break;

      case 5:  // Leg 6
        if (leg6_IK_control == true) {
          if (theta_coxa < 0)
            theta_coxa = theta_coxa + 315.0;
          else
            theta_coxa = theta_coxa - 45.0;
          theta_coxa = constrain(theta_coxa, 0.0, 180.0);
          slavePkt.coxa6  = int(theta_coxa);
          slavePkt.femur6 = int(theta_femur);
          slavePkt.tibia6 = int(theta_tibia);
        }
        break;
    }
  }
}


//***********************************************************************
// Tripod Gait
//***********************************************************************
void tripod_gait()
{
  // PS5 sticks return -127 to 127 (center 0). Remap to match original logic.
  commandedX = -(int)ps5.RStickY();  // forward/back
  commandedY =  (int)ps5.RStickX();  // strafe left/right
  commandedR = -(int)ps5.LStickX();  // rotate

  if ((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick > 0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 2.0);
    for (leg_num = 0; leg_num < 6; leg_num++) {
      compute_amplitudes();
      switch (tripod_case[leg_num]) {
        case 1:
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cos(M_PI * tick / numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cos(M_PI * tick / numTicks);
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ) * sin(M_PI * tick / numTicks);
          if (tick >= numTicks - 1) tripod_case[leg_num] = 2;
          break;
        case 2:
          current_X[leg_num] = HOME_X[leg_num] + amplitudeX * cos(M_PI * tick / numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] + amplitudeY * cos(M_PI * tick / numTicks);
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1) tripod_case[leg_num] = 1;
          break;
      }
    }
    if (tick < numTicks - 1) tick++;
    else tick = 0;
  }
}


//***********************************************************************
// Wave Gait
//***********************************************************************
void wave_gait()
{
  commandedX = -(int)ps5.RStickY();
  commandedY =  (int)ps5.RStickX();
  commandedR = -(int)ps5.LStickX();

  if ((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick > 0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 6.0);
    for (leg_num = 0; leg_num < 6; leg_num++) {
      compute_amplitudes();
      switch (wave_case[leg_num]) {
        case 1:
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cos(M_PI * tick / numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cos(M_PI * tick / numTicks);
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ) * sin(M_PI * tick / numTicks);
          if (tick >= numTicks - 1) wave_case[leg_num] = 6;
          break;
        case 2:
          current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.5;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.5;
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1) wave_case[leg_num] = 1;
          break;
        case 3:
          current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.5;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.5;
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1) wave_case[leg_num] = 2;
          break;
        case 4:
          current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.5;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.5;
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1) wave_case[leg_num] = 3;
          break;
        case 5:
          current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.5;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.5;
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1) wave_case[leg_num] = 4;
          break;
        case 6:
          current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.5;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.5;
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1) wave_case[leg_num] = 5;
          break;
      }
    }
    if (tick < numTicks - 1) tick++;
    else tick = 0;
  }
}


//***********************************************************************
// Ripple Gait
//***********************************************************************
void ripple_gait()
{
  commandedX = -(int)ps5.RStickY();
  commandedY =  (int)ps5.RStickX();
  commandedR = -(int)ps5.LStickX();

  if ((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick > 0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 6.0);
    for (leg_num = 0; leg_num < 6; leg_num++) {
      compute_amplitudes();
      switch (ripple_case[leg_num]) {
        case 1:
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cos(M_PI * tick / (numTicks * 2));
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cos(M_PI * tick / (numTicks * 2));
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ) * sin(M_PI * tick / (numTicks * 2));
          if (tick >= numTicks - 1) ripple_case[leg_num] = 2;
          break;
        case 2:
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cos(M_PI * (numTicks + tick) / (numTicks * 2));
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cos(M_PI * (numTicks + tick) / (numTicks * 2));
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ) * sin(M_PI * (numTicks + tick) / (numTicks * 2));
          if (tick >= numTicks - 1) ripple_case[leg_num] = 3;
          break;
        case 3:
          current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.0;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.0;
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1) ripple_case[leg_num] = 4;
          break;
        case 4:
          current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.0;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.0;
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1) ripple_case[leg_num] = 5;
          break;
        case 5:
          current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.0;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.0;
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1) ripple_case[leg_num] = 6;
          break;
        case 6:
          current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.0;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.0;
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1) ripple_case[leg_num] = 1;
          break;
      }
    }
    if (tick < numTicks - 1) tick++;
    else tick = 0;
  }
}


//***********************************************************************
// Tetrapod Gait
//***********************************************************************
void tetrapod_gait()
{
  commandedX = -(int)ps5.RStickY();
  commandedY =  (int)ps5.RStickX();
  commandedR = -(int)ps5.LStickX();

  if ((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick > 0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 3.0);
    for (leg_num = 0; leg_num < 6; leg_num++) {
      compute_amplitudes();
      switch (tetrapod_case[leg_num]) {
        case 1:
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cos(M_PI * tick / numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cos(M_PI * tick / numTicks);
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ) * sin(M_PI * tick / numTicks);
          if (tick >= numTicks - 1) tetrapod_case[leg_num] = 2;
          break;
        case 2:
          current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks;
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1) tetrapod_case[leg_num] = 3;
          break;
        case 3:
          current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks;
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1) tetrapod_case[leg_num] = 1;
          break;
      }
    }
    if (tick < numTicks - 1) tick++;
    else tick = 0;
  }
}


//***********************************************************************
// Compute Strides
//***********************************************************************
void compute_strides()
{
  strideX = 90 * commandedX / 127;
  strideY = 90 * commandedY / 127;
  strideR = 35 * commandedR / 127;

  sinRotZ = sin(radians(strideR));
  cosRotZ = cos(radians(strideR));

  if (gait_speed == 0) duration = 1080;
  else duration = 3240;
}


//***********************************************************************
// Compute Amplitudes
//***********************************************************************
void compute_amplitudes()
{
  totalX = HOME_X[leg_num] + BODY_X[leg_num];
  totalY = HOME_Y[leg_num] + BODY_Y[leg_num];

  rotOffsetX = totalY * sinRotZ + totalX * cosRotZ - totalX;
  rotOffsetY = totalY * cosRotZ - totalX * sinRotZ - totalY;

  amplitudeX = ((strideX + rotOffsetX) / 2.0);
  amplitudeY = ((strideY + rotOffsetY) / 2.0);
  amplitudeX = constrain(amplitudeX, -50, 50);
  amplitudeY = constrain(amplitudeY, -50, 50);

  if (abs(strideX + rotOffsetX) > abs(strideY + rotOffsetY))
    amplitudeZ = step_height_multiplier * (strideX + rotOffsetX) / 4.0;
  else
    amplitudeZ = step_height_multiplier * (strideY + rotOffsetY) / 4.0;
}


//***********************************************************************
// Translate Control (mode 2)
// PS5 sticks: -127 to 127. Remapped to match original travel range.
//***********************************************************************
void translate_control()
{
  translateX = map((int)ps5.RStickY(), -127, 127, 2*TRAVEL, -2*TRAVEL);
  for (leg_num = 0; leg_num < 6; leg_num++)
    current_X[leg_num] = HOME_X[leg_num] + translateX;

  translateY = map((int)ps5.RStickX(), -127, 127, -2*TRAVEL, 2*TRAVEL);
  for (leg_num = 0; leg_num < 6; leg_num++)
    current_Y[leg_num] = HOME_Y[leg_num] + translateY;

  // Left stick Y: negative (up) = raise body, positive (down) = lower body
  int ly = (int)ps5.LStickY();
  if (ly < 0)
    translateZ = map(ly, -127, 0, TRAVEL, 0);
  else
    translateZ = map(ly, 0, 127, 0, -3*TRAVEL);
  for (leg_num = 0; leg_num < 6; leg_num++)
    current_Z[leg_num] = HOME_Z[leg_num] + translateZ;

  if (capture_offsets == true) {
    for (leg_num = 0; leg_num < 6; leg_num++) {
      offset_X[leg_num] += translateX;
      offset_Y[leg_num] += translateY;
      offset_Z[leg_num] += translateZ;
      current_X[leg_num] = HOME_X[leg_num];
      current_Y[leg_num] = HOME_Y[leg_num];
      current_Z[leg_num] = HOME_Z[leg_num];
    }
    capture_offsets = false;
    mode = 0;
  }
}


//***********************************************************************
// Rotate Control (mode 3)
//***********************************************************************
void rotate_control()
{
  int rx = (int)ps5.RStickX();
  int ry = (int)ps5.RStickY();
  int lx = (int)ps5.LStickX();
  int ly = (int)ps5.LStickY();

  sinRotX = sin((map(rx, -127, 127,  A12DEG, -A12DEG)) / 1000000.0);
  cosRotX = cos((map(rx, -127, 127,  A12DEG, -A12DEG)) / 1000000.0);
  sinRotY = sin((map(ry, -127, 127,  A12DEG, -A12DEG)) / 1000000.0);
  cosRotY = cos((map(ry, -127, 127,  A12DEG, -A12DEG)) / 1000000.0);
  sinRotZ = sin((map(lx, -127, 127, -A30DEG,  A30DEG)) / 1000000.0);
  cosRotZ = cos((map(lx, -127, 127, -A30DEG,  A30DEG)) / 1000000.0);

  if (ly < 0)
    translateZ = map(ly, -127, 0, TRAVEL, 0);
  else
    translateZ = map(ly, 0, 127, 0, -3*TRAVEL);

  for (int leg_num = 0; leg_num < 6; leg_num++) {
    totalX = HOME_X[leg_num] + BODY_X[leg_num];
    totalY = HOME_Y[leg_num] + BODY_Y[leg_num];
    totalZ = HOME_Z[leg_num] + BODY_Z[leg_num];

    rotOffsetX =  totalX*cosRotY*cosRotZ + totalY*sinRotX*sinRotY*cosRotZ + totalY*cosRotX*sinRotZ - totalZ*cosRotX*sinRotY*cosRotZ + totalZ*sinRotX*sinRotZ - totalX;
    rotOffsetY = -totalX*cosRotY*sinRotZ - totalY*sinRotX*sinRotY*sinRotZ + totalY*cosRotX*cosRotZ + totalZ*cosRotX*sinRotY*sinRotZ + totalZ*sinRotX*cosRotZ - totalY;
    rotOffsetZ =  totalX*sinRotY         - totalY*sinRotX*cosRotY                                  + totalZ*cosRotX*cosRotY                                  - totalZ;

    current_X[leg_num] = HOME_X[leg_num] + rotOffsetX;
    current_Y[leg_num] = HOME_Y[leg_num] + rotOffsetY;
    current_Z[leg_num] = HOME_Z[leg_num] + rotOffsetZ + translateZ;

    if (capture_offsets == true) {
      offset_X[leg_num] += rotOffsetX;
      offset_Y[leg_num] += rotOffsetY;
      offset_Z[leg_num] += rotOffsetZ + translateZ;
      current_X[leg_num] = HOME_X[leg_num];
      current_Y[leg_num] = HOME_Y[leg_num];
      current_Z[leg_num] = HOME_Z[leg_num];
    }
  }
  if (capture_offsets == true) {
    capture_offsets = false;
    mode = 0;
  }
}


//***********************************************************************
// One Leg Lift (mode 4)
// Right stick controls Leg 1, Left stick controls Leg 6
// PS5 stick center = 0 (not 128 like PS2)
//***********************************************************************
void one_leg_lift()
{
  if (leg1_IK_control == true) {
    leg1_coxa  = coxa1_servo.read();
    leg1_femur = femur1_servo.read();
    leg1_tibia = tibia1_servo.read();
    leg1_IK_control = false;
  }
  // NOTE: leg6 servos live on Slave. Master cannot read them back.
  // leg6 IK control is disabled in this mode (leg6_IK_control tracks it).

  // Right stick X → Leg 1 coxa rotation
  temp = map((int)ps5.RStickX(), -127, 127, 45, -45);
  coxa1_servo.write(constrain(int(leg1_coxa + temp), 45, 135));

  // Right stick Y: negative (up) = lift leg 1
  int ry = (int)ps5.RStickY();
  if (ry < 0) {
    temp = map(ry, -127, 0, 24, 0);
    femur1_servo.write(constrain(int(leg1_femur + temp), 0, 170));
    tibia1_servo.write(constrain(int(leg1_tibia + 4*temp), 0, 170));
  } else {
    z_height_right = map(constrain(ry, 20, 127), 20, 127, 1, 8);
  }

  // Left stick Y: negative (up) = lift leg 6
  // Leg 6 angles go into slavePkt for the slave to execute
  int ly = (int)ps5.LStickY();
  if (ly < 0) {
    temp = map(ly, -127, 0, 24, 0);
    // We don't have direct servo objects for leg 6, so pack into slavePkt manually
    slavePkt.femur6 = constrain(int(leg6_femur + temp), 0, 170);
    slavePkt.tibia6 = constrain(int(leg6_tibia + 4*temp), 0, 170);
  } else {
    z_height_left = map(constrain(ly, 20, 127), 20, 127, 1, 8);
  }

  // Left stick X → Leg 6 coxa
  temp = map((int)ps5.LStickX(), -127, 127, 45, -45);
  slavePkt.coxa6 = constrain(int(leg6_coxa + temp), 45, 135);

  if (z_height_left > z_height_right) z_height_right = z_height_left;

  if (capture_offsets == true) {
    step_height_multiplier = 1.0 + ((z_height_right - 1.0) / 3.0);
    capture_offsets = false;
  }
}


//***********************************************************************
// Set All Servos to 90 Degrees (calibration mode 99)
// Legs 1-3: write locally. Legs 4-6: pack into slavePkt.
//***********************************************************************
void set_all_90()
{
  coxa1_servo.write(90 + COXA_CAL[0]);
  femur1_servo.write(90 + FEMUR_CAL[0]);
  tibia1_servo.write(90 + TIBIA_CAL[0]);

  coxa2_servo.write(90 + COXA_CAL[1]);
  femur2_servo.write(90 + FEMUR_CAL[1]);
  tibia2_servo.write(90 + TIBIA_CAL[1]);

  coxa3_servo.write(90 + COXA_CAL[2]);
  femur3_servo.write(90 + TIBIA_CAL[2]);
  tibia3_servo.write(90 + TIBIA_CAL[2]);

  slavePkt.coxa4  = 90 + COXA_CAL[3];
  slavePkt.femur4 = 90 + FEMUR_CAL[3];
  slavePkt.tibia4 = 90 + TIBIA_CAL[3];

  slavePkt.coxa5  = 90 + COXA_CAL[4];
  slavePkt.femur5 = 90 + FEMUR_CAL[4];
  slavePkt.tibia5 = 90 + TIBIA_CAL[4];

  slavePkt.coxa6  = 90 + COXA_CAL[5];
  slavePkt.femur6 = 90 + FEMUR_CAL[5];
  slavePkt.tibia6 = 90 + TIBIA_CAL[5];
}


//***********************************************************************
// Battery Monitor
// ESP32: 12-bit ADC (0-4095), 3.3V reference
// Use voltage divider: 100kΩ + 33kΩ to scale 12.6V → ~3.1V at pin
//***********************************************************************
void battery_monitor()
{
  batt_voltage_sum -= batt_voltage_array[batt_voltage_index];
  batt_voltage_array[batt_voltage_index] = map(analogRead(BATT_VOLTAGE), 0, 4095, 0, 1497);
  batt_voltage_sum += batt_voltage_array[batt_voltage_index];
  batt_voltage_index++;
  if (batt_voltage_index > 49) batt_voltage_index = 0;
  batt_voltage = batt_voltage_sum / 50;
}


//***********************************************************************
// Print Debug
//***********************************************************************
void print_debug()
{
  currentTime = millis();
  Serial.print(currentTime - previousTime);
  Serial.print(" ms | Batt: ");
  Serial.print(float(batt_voltage) / 100.0);
  Serial.print("V | Mode: ");
  Serial.print(mode);
  Serial.print(" | Gait: ");
  Serial.println(gait);
}

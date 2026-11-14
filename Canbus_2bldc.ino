// ODrive S1 serial console over CAN (CAN Simple protocol)
// Teensy 4.1, CAN1: pin 22 = CTX1 -> CAN Pal TX, pin 23 = CRX1 -> CAN Pal RX
// Serial Monitor: 115200 baud, line ending = "Newline" (or "Both NL & CR")

#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;

// ---------------- Settings ----------------
const uint8_t  NODE_ID   = 0;
const uint32_t CAN_BAUD  = 250000;     // must match odrv0.can.config.baud_rate
const uint32_t RAMP_STEP_MS = 20;      // software velocity ramp update period

// ---------------- CAN Simple command IDs ----------------
enum : uint8_t {
  CMD_HEARTBEAT        = 0x01,
  CMD_ESTOP            = 0x02,
  CMD_SET_AXIS_STATE   = 0x07,
  CMD_GET_ENCODER_EST  = 0x09,
  CMD_SET_CTRL_MODE    = 0x0B,
  CMD_SET_INPUT_POS    = 0x0C,
  CMD_SET_INPUT_VEL    = 0x0D,
  CMD_SET_INPUT_TORQUE = 0x0E,
  CMD_SET_LIMITS       = 0x0F,
  CMD_SET_TRAJ_VEL     = 0x11,
  CMD_SET_TRAJ_ACCEL   = 0x12,
  CMD_GET_IQ           = 0x14,
  CMD_REBOOT           = 0x16,
  CMD_GET_VBUS         = 0x17,
  CMD_CLEAR_ERRORS     = 0x18,
  CMD_SET_POS_GAIN     = 0x1A,
  CMD_SET_VEL_GAINS    = 0x1B,
};

// Axis states / modes
const uint32_t STATE_IDLE = 1;
const uint32_t STATE_CLOSED_LOOP = 8;
const int32_t  CTRL_TORQUE = 1, CTRL_VELOCITY = 2, CTRL_POSITION = 3;
const int32_t  INPUT_PASSTHROUGH = 1, INPUT_TRAP_TRAJ = 5;

// ---------------- State ----------------
enum Mode { MODE_VEL, MODE_TORQUE, MODE_POS };
Mode curMode = MODE_VEL;

float cmdVel = 0;                      // last velocity commanded (for ramp start)

// software velocity ramp
bool     rampActive = false;
float    rampStartV = 0, rampTargetV = 0;
uint32_t rampStartMs = 0, rampDurMs = 0, rampLastMs = 0;

// last heartbeat
uint32_t hbError = 0;
uint8_t  hbState = 0, hbResult = 0, hbTraj = 0;
bool     hbSeen = false;
// print-on-request flags (ODrive may broadcast these cyclically; only show them when asked)
volatile bool wantEnc = false, wantIq = false, wantVbus = false;
uint32_t lastPrintedErr = 0xFFFFFFFF;
int      lastPrintedState = -1;

// ---------------- CAN helpers ----------------
void sendFrame(uint8_t cmd, const void* data, uint8_t len, bool rtr = false) {
  CAN_message_t m;
  m.id = ((uint32_t)NODE_ID << 5) | cmd;
  m.len = len;
  m.flags.remote = rtr ? 1 : 0;
  if (len && data) memcpy(m.buf, data, len);
  can.write(m);
}

void sendRequest(uint8_t cmd) { sendFrame(cmd, nullptr, 8, true); }

void sendU32(uint8_t cmd, uint32_t v) { sendFrame(cmd, &v, 4); }
void sendF32(uint8_t cmd, float v)    { sendFrame(cmd, &v, 4); }

void sendF32x2(uint8_t cmd, float a, float b) {
  uint8_t buf[8];
  memcpy(buf, &a, 4);
  memcpy(buf + 4, &b, 4);
  sendFrame(cmd, buf, 8);
}

void setAxisState(uint32_t s) { sendU32(CMD_SET_AXIS_STATE, s); }

void setControllerMode(int32_t ctrl, int32_t input) {
  uint8_t buf[8];
  memcpy(buf, &ctrl, 4);
  memcpy(buf + 4, &input, 4);
  sendFrame(CMD_SET_CTRL_MODE, buf, 8);
}

void sendVel(float v) {
  cmdVel = v;
  sendF32x2(CMD_SET_INPUT_VEL, v, 0.0f);          // vel, torque feedforward
}

void sendTorque(float nm) { sendF32(CMD_SET_INPUT_TORQUE, nm); }

void sendPos(float turns) {
  uint8_t buf[8];
  int16_t velFF = 0, torqueFF = 0;                // scaled x0.001
  memcpy(buf, &turns, 4);
  memcpy(buf + 2 + 2, &velFF, 2);
  memcpy(buf + 6, &torqueFF, 2);
  sendFrame(CMD_SET_INPUT_POS, buf, 8);
}

void applyMode(Mode m) {
  curMode = m;
  rampActive = false;
  switch (m) {
    case MODE_VEL:
      setControllerMode(CTRL_VELOCITY, INPUT_PASSTHROUGH);
      sendVel(0.0f);                              // start from zero for safety
      Serial.println("Mode: velocity");
      break;
    case MODE_TORQUE:
      setControllerMode(CTRL_TORQUE, INPUT_PASSTHROUGH);
      sendTorque(0.0f);
      Serial.println("Mode: torque");
      break;
    case MODE_POS:
      setControllerMode(CTRL_POSITION, INPUT_TRAP_TRAJ);
      Serial.println("Mode: position (trapezoidal trajectory)");
      Serial.println("  note: send 'enc' then 'p <current pos>' first to avoid a jump.");
      break;
  }
}

// ---------------- Receive ----------------
void printHeartbeat() {
  if (!hbSeen) { Serial.println("No heartbeat received yet."); return; }
  Serial.printf("Heartbeat: state=%u error=0x%lX result=%u traj_done=%u\n",
                hbState, (unsigned long)hbError, hbResult, hbTraj);
}

void onRx(const CAN_message_t &m) {
  if ((m.id >> 5) != NODE_ID) return;
  uint8_t cmd = m.id & 0x1F;

  switch (cmd) {
    case CMD_HEARTBEAT: {
      memcpy(&hbError, m.buf, 4);
      hbState  = m.buf[4];
      hbResult = m.buf[5];
      hbTraj   = m.buf[6];
      hbSeen = true;
      // only print on change so the console stays usable
      if (hbError != lastPrintedErr || hbState != lastPrintedState) {
        lastPrintedErr = hbError;
        lastPrintedState = hbState;
        printHeartbeat();
      }
      break;
    }
    case CMD_GET_ENCODER_EST: {
      if (!wantEnc) break;
      wantEnc = false;
      float pos, vel;
      memcpy(&pos, m.buf, 4);
      memcpy(&vel, m.buf + 4, 4);
      Serial.printf("Encoder: pos=%.4f turns  vel=%.4f turns/s\n", pos, vel);
      break;
    }
    case CMD_GET_IQ: {
      if (!wantIq) break;
      wantIq = false;
      float setp, meas;
      memcpy(&setp, m.buf, 4);
      memcpy(&meas, m.buf + 4, 4);
      Serial.printf("Iq: setpoint=%.3f A  measured=%.3f A\n", setp, meas);
      break;
    }
    case CMD_GET_VBUS: {
      if (!wantVbus) break;
      wantVbus = false;
      float v, i;
      memcpy(&v, m.buf, 4);
      memcpy(&i, m.buf + 4, 4);
      Serial.printf("Vbus: %.2f V  Ibus: %.3f A\n", v, i);
      break;
    }
    default: break;
  }
}

// ---------------- Software velocity ramp ----------------
void startRamp(float target, float secs) {
  if (curMode != MODE_VEL) { Serial.println("Not in velocity mode (use 'mv')."); return; }
  if (secs <= 0) { sendVel(target); rampActive = false; return; }
  rampStartV = cmdVel;
  rampTargetV = target;
  rampStartMs = millis();
  rampDurMs = (uint32_t)(secs * 1000.0f);
  rampLastMs = 0;
  rampActive = true;
  Serial.printf("Ramp %.3f -> %.3f turns/s over %.2f s\n", rampStartV, target, secs);
}

void updateRamp() {
  if (!rampActive) return;
  uint32_t now = millis();
  if (now - rampLastMs < RAMP_STEP_MS) return;
  rampLastMs = now;

  uint32_t el = now - rampStartMs;
  if (el >= rampDurMs) {
    sendVel(rampTargetV);
    rampActive = false;
    Serial.println("Ramp done.");
  } else {
    float f = (float)el / (float)rampDurMs;
    sendVel(rampStartV + (rampTargetV - rampStartV) * f);
  }
}

// ---------------- Command parsing ----------------
void printHelp() {
  Serial.print(
    "Commands:\n"
    "  a               arm (clear errors, request CLOSED_LOOP_CONTROL)\n"
    "  i               idle (coast stop)\n"
    "  e               estop\n"
    "  c               clear errors\n"
    "  mv / mt / mp    select mode: velocity / torque / position\n"
    "  v <turns/s>     velocity setpoint (immediate, v 0 stops cleanly)\n"
    "  rv <v> <secs>   ramp velocity to <v> over <secs> (software ramp)\n"
    "  t <Nm>          torque setpoint\n"
    "  p <turns>       absolute position setpoint\n"
    "  ac <a> <d>      trajectory accel/decel limits, turns/s^2 (position mode only)\n"
    "  tv <turns/s>    trajectory velocity limit (position mode only)\n"
    "  l <v> <i>       set velocity limit + current limit\n"
    "  vg <g> <ig>     set velocity gain + velocity integrator gain\n"
    "  pg <g>          set position gain\n"
    "  iq              request measured/setpoint current\n"
    "  enc             request encoder position + velocity estimate\n"
    "  ?               request Vbus voltage\n"
    "  r               print last heartbeat (state, error)\n"
    "  reboot          reboot the ODrive\n"
    "  h               this help\n");
}

bool needArgs(int have, int need, const char* usage) {
  if (have >= need) return true;
  Serial.print("Usage: ");
  Serial.println(usage);
  return false;
}

void handleLine(char* line) {
  // tokenise (max 3 tokens)
  char* tok[3] = {nullptr, nullptr, nullptr};
  int n = 0;
  char* p = strtok(line, " \t");
  while (p && n < 3) { tok[n++] = p; p = strtok(nullptr, " \t"); }
  if (n == 0) return;

  const char* c = tok[0];
  float a1 = n > 1 ? atof(tok[1]) : 0;
  float a2 = n > 2 ? atof(tok[2]) : 0;

  if      (!strcmp(c, "a")) {
    sendFrame(CMD_CLEAR_ERRORS, nullptr, 0);
    delay(20);
    // re-apply current mode with a safe zero setpoint before arming
    if (curMode == MODE_VEL)        { setControllerMode(CTRL_VELOCITY, INPUT_PASSTHROUGH); sendVel(0); }
    else if (curMode == MODE_TORQUE){ setControllerMode(CTRL_TORQUE, INPUT_PASSTHROUGH);   sendTorque(0); }
    else                            { setControllerMode(CTRL_POSITION, INPUT_TRAP_TRAJ); }
    rampActive = false;
    delay(20);
    setAxisState(STATE_CLOSED_LOOP);
    Serial.println("Arming: requested CLOSED_LOOP_CONTROL (check heartbeat state=8).");
  }
  else if (!strcmp(c, "i"))  { rampActive = false; setAxisState(STATE_IDLE); Serial.println("Idle requested."); }
  else if (!strcmp(c, "e"))  { rampActive = false; sendFrame(CMD_ESTOP, nullptr, 0); Serial.println("ESTOP sent. Use 'c' then 'a' to recover."); }
  else if (!strcmp(c, "c"))  { sendFrame(CMD_CLEAR_ERRORS, nullptr, 0); Serial.println("Clear errors sent."); }
  else if (!strcmp(c, "mv")) applyMode(MODE_VEL);
  else if (!strcmp(c, "mt")) applyMode(MODE_TORQUE);
  else if (!strcmp(c, "mp")) applyMode(MODE_POS);
  else if (!strcmp(c, "v")) {
    if (!needArgs(n, 2, "v <turns/s>")) return;
    if (curMode != MODE_VEL) { Serial.println("Not in velocity mode (use 'mv')."); return; }
    rampActive = false;
    sendVel(a1);
    Serial.printf("Velocity = %.3f turns/s\n", a1);
  }
  else if (!strcmp(c, "rv")) {
    if (!needArgs(n, 3, "rv <v> <secs>")) return;
    startRamp(a1, a2);
  }
  else if (!strcmp(c, "t")) {
    if (!needArgs(n, 2, "t <Nm>")) return;
    if (curMode != MODE_TORQUE) { Serial.println("Not in torque mode (use 'mt')."); return; }
    sendTorque(a1);
    Serial.printf("Torque = %.3f Nm\n", a1);
  }
  else if (!strcmp(c, "p")) {
    if (!needArgs(n, 2, "p <turns>")) return;
    if (curMode != MODE_POS) { Serial.println("Not in position mode (use 'mp')."); return; }
    sendPos(a1);
    Serial.printf("Position = %.3f turns\n", a1);
  }
  else if (!strcmp(c, "ac")) {
    if (!needArgs(n, 3, "ac <accel> <decel>")) return;
    sendF32x2(CMD_SET_TRAJ_ACCEL, a1, a2);
    Serial.printf("Traj accel=%.3f decel=%.3f turns/s^2\n", a1, a2);
  }
  else if (!strcmp(c, "tv")) {
    if (!needArgs(n, 2, "tv <turns/s>")) return;
    sendF32(CMD_SET_TRAJ_VEL, a1);
    Serial.printf("Traj vel limit=%.3f turns/s\n", a1);
  }
  else if (!strcmp(c, "l")) {
    if (!needArgs(n, 3, "l <vel limit> <current limit>")) return;
    sendF32x2(CMD_SET_LIMITS, a1, a2);
    Serial.printf("Limits: vel=%.3f turns/s  current=%.3f A\n", a1, a2);
  }
  else if (!strcmp(c, "vg")) {
    if (!needArgs(n, 3, "vg <vel gain> <vel integrator gain>")) return;
    sendF32x2(CMD_SET_VEL_GAINS, a1, a2);
    Serial.printf("Vel gain=%.4f  integrator gain=%.4f\n", a1, a2);
  }
  else if (!strcmp(c, "pg")) {
    if (!needArgs(n, 2, "pg <pos gain>")) return;
    sendF32(CMD_SET_POS_GAIN, a1);
    Serial.printf("Pos gain=%.3f\n", a1);
  }
  else if (!strcmp(c, "iq"))  { wantIq = true;   sendRequest(CMD_GET_IQ); }
  else if (!strcmp(c, "enc")) { wantEnc = true;  sendRequest(CMD_GET_ENCODER_EST); }
  else if (!strcmp(c, "?"))   { wantVbus = true; sendRequest(CMD_GET_VBUS); }
  else if (!strcmp(c, "r"))   printHeartbeat();
  else if (!strcmp(c, "reboot")) {
    uint8_t action = 0;                           // 0 = reboot
    sendFrame(CMD_REBOOT, &action, 1);
    rampActive = false;
    Serial.println("Reboot sent. Wait a few seconds, then 'a' to arm.");
  }
  else if (!strcmp(c, "h"))   printHelp();
  else Serial.println("Unknown command. Type 'h' for help.");
}

// ---------------- Setup / loop ----------------
char lineBuf[64];
uint8_t lineLen = 0;

void setup() {
  Serial.begin(115200);
  can.begin();
  can.setBaudRate(CAN_BAUD);
  can.enableFIFO();
  can.enableFIFOInterrupt();
  can.onReceive(onRx);

  delay(1000);
  Serial.println("ODrive S1 CAN console ready.");
  printHelp();
  Serial.println("\nSafe start: motor is NOT armed. Type 'a' to arm.");
}

void loop() {
  can.events();
  updateRamp();

  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r' || ch == '\n') {
      if (lineLen > 0) {
        lineBuf[lineLen] = 0;
        handleLine(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = ch;
    }
  }
}

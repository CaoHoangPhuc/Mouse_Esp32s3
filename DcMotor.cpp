#include "esp32-hal-gpio.h"
#include "DcMotor.h"
#include "Config.h"
#include "driver/gpio.h"

#if defined(ESP32)
  #include <esp32-hal-ledc.h>
#endif

DcMotor* DcMotor::_instances[4] = {nullptr, nullptr, nullptr, nullptr};
static portMUX_TYPE sMotorPidTraceMux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t sMotorPidTraceCounterL = 0;
static uint32_t sMotorPidTraceCounterR = 0;

DcMotor::DcMotor() {}

int DcMotor::allocSlot(DcMotor* m) {
  for (int i = 0; i < 4; i++) {
    if (_instances[i] == nullptr) { _instances[i] = m; return i; }
  }
  return -1;
}

// PWM write wrapper: Arduino-ESP32 core v3 writes by pin, core v2 writes by channel
void DcMotor::pwmWriteDuty(uint32_t duty) {
  if (duty > _pwmMax) duty = _pwmMax;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(_p.pwm, duty);     // core v3
#else
  ledcWrite(_pwmCh, duty);     // core v2
#endif
}

bool DcMotor::begin(const Pins& pins,
                    uint8_t pwmChannel,
                    uint32_t pwmFreq,
                    uint8_t pwmResolutionBits)
{
  _p = pins;
  _pwmCh = pwmChannel;
  _pwmFreq = pwmFreq;
  _pwmResBits = pwmResolutionBits;

  pinMode(_p.in1, OUTPUT);
  pinMode(_p.in2, OUTPUT);
  pinMode(_p.pwm, OUTPUT);

  pinMode(_p.encA, INPUT_PULLUP);
  pinMode(_p.encB, INPUT_PULLUP);

  _pwmMax = (1UL << _pwmResBits) - 1;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  // Arduino-ESP32 core v3
  if (!ledcAttachChannel(_p.pwm, _pwmFreq, _pwmResBits, _pwmCh)) return false;
  pwmWriteDuty(0);
#else
  // Arduino-ESP32 core v2
  ledcSetup(_pwmCh, _pwmFreq, _pwmResBits);
  ledcAttachPin(_p.pwm, _pwmCh);
  pwmWriteDuty(0);
#endif

  // allocate instance slot for ISR
  _slot = allocSlot(this);
  if (_slot < 0) return false;

  // Attach ONLY encA interrupt (CHANGE)
  switch (_slot) {
    case 0: attachInterrupt(digitalPinToInterrupt(_p.encA), isrEncA0, RISING); break;
    case 1: attachInterrupt(digitalPinToInterrupt(_p.encA), isrEncA1, RISING); break;
    case 2: attachInterrupt(digitalPinToInterrupt(_p.encA), isrEncA2, RISING); break;
    case 3: attachInterrupt(digitalPinToInterrupt(_p.encA), isrEncA3, RISING); break;
  }

  // init runtime state
  _ticks = 0;
  _lastMicros = micros();
  _lastTicks = 0;
  _tps = 0.0f;
  _speedAccumTicks = 0;
  _speedAccumUs = 0;

  _speedCtrlEnabled = false;
  _targetTPS = 0.0f;

  // safe starter defaults (TPS-based) - tune later
  _kp = 0.00045f;
  _ki = 0.0080f;
  _kd = 0.00005f;
  _outLimit = 0.90f;
  _iLimit = 0.50f;
  _dFilterHz = 25.0f;
  _slewRate = 3.0f;

  resetPID();

  // stop motor
  coastStop();

  return true;
}

void DcMotor::applyDuty(int32_t duty) {
  // Handle inversion
  if (_p.invertDir) duty = -duty;

  // Clamp range
  if (duty > (int32_t)_pwmMax) duty = _pwmMax;
  if (duty < -(int32_t)_pwmMax) duty = -_pwmMax;

  if (duty > 0 && duty < DEADZONE) duty = 0;
  if (duty < 0 && duty > -DEADZONE) duty = 0;

  // Direction
  if (duty > 0) {
    // forward
    digitalWrite(_p.in1, HIGH);
    digitalWrite(_p.in2, LOW);
    pwmWriteDuty(duty);
  } else {
    // backward
    digitalWrite(_p.in1, LOW);
    digitalWrite(_p.in2, HIGH);
    pwmWriteDuty(-duty);  // use absolute value
  }
}

void DcMotor::setPower(float power) {
  _speedCtrlEnabled = false;
  power = clampf(power, -1.0f, 1.0f);

  int32_t duty = (int32_t)(power * (float)_pwmMax + (power >= 0 ? 0.5f : -0.5f));
  applyDuty(duty);
}

void DcMotor::enableSpeedControl(bool en) {
  _speedCtrlEnabled = en;
  if (!en) resetPID();
}

void DcMotor::setSpeedTPS(float tps) {
  _targetTPS = tps;
  _speedCtrlEnabled = true;
}

void DcMotor::coastStop() {
  digitalWrite(_p.in1, LOW);
  digitalWrite(_p.in2, LOW);
  pwmWriteDuty(0);
}

void DcMotor::brakeStop() {
  applyDuty(0);
}

void DcMotor::hardStop() {
  setSpeedTPS(0.0f);
}

void DcMotor::setSpeedPID(float kp, float ki, float kd,
                          float outLimit, float iLimit,
                          float dFilterHz, float slewRatePerSec)
{
  _kp = kp; _ki = ki; _kd = kd;
  _outLimit  = clampf(outLimit, 0.05f, 1.0f);
  _iLimit    = clampf(iLimit, 0.0f, _outLimit);
  _dFilterHz = (dFilterHz <= 0.0f) ? 0.0f : dFilterHz;
  _slewRate  = (slewRatePerSec < 0.0f) ? 0.0f : slewRatePerSec;
  resetPID();
}

void DcMotor::resetPID() {
  _iTerm = 0.0f;
  _dMeas = 0.0f;
  _lastTPSForD = _tps;
  _lastOut = 0.0f;
}

void DcMotor::update() {
  uint32_t now = micros();
  const uint32_t fallbackUs = (AppConfig::Tasks::MOTOR_LOOP_PERIOD_MS > 0)
                              ? (AppConfig::Tasks::MOTOR_LOOP_PERIOD_MS * 1000UL)
                              : 5000UL;
  uint32_t elapsedUs = (_lastMicros == 0) ? fallbackUs : (now - _lastMicros);
  if (elapsedUs == 0) elapsedUs = 1;
  const float dt = elapsedUs / 1e6f;

  // Always update speed estimate even if very fast
  int32_t ticksNow;
  ticksNow = _ticks;

  int32_t dTicks = ticksNow - _lastTicks;

  // Period-based speed estimate from a fixed accumulation window.
  const uint32_t windowUs = (AppConfig::Motors::TPS_ESTIMATE_WINDOW_MS > 0)
                            ? (AppConfig::Motors::TPS_ESTIMATE_WINDOW_MS * 1000UL)
                            : 5000UL;
  _speedAccumTicks += dTicks;
  _speedAccumUs += elapsedUs;
  if (_speedAccumUs >= windowUs) {
    float tpsInstant = _speedAccumTicks / (_speedAccumUs / 1e6f);
    const float alpha = AppConfig::Motors::TPS_LPF_ALPHA;
    _tps = _tps + alpha * (tpsInstant - _tps);
    _speedAccumTicks = 0;
    _speedAccumUs = 0;
  }

  _lastMicros = now;
  _lastTicks = ticksNow;

  if (!_speedCtrlEnabled) return;

  float err = _targetTPS - _tps;
  float pTerm = _kp * err;

  // D on measurement
  float dMeasRaw = (_tps - _lastTPSForD) / dt;
  _lastTPSForD = _tps;

  float dTerm = 0.0f;
  if (_kd != 0.0f) {
    if (_dFilterHz > 0.0f) {
      float rc = 1.0f / (2.0f * 3.1415926f * _dFilterHz);
      float aD = dt / (dt + rc);
      _dMeas = _dMeas + aD * (dMeasRaw - _dMeas);
      dTerm = -_kd * _dMeas;
    } else {
      dTerm = -_kd * dMeasRaw;
    }
  }

  float preSat = pTerm + dTerm;

  // Anti-windup integrator
  float outUnsat = preSat + _iTerm;
  float outSat = clampf(outUnsat, -_outLimit, _outLimit);

  bool saturated = (outUnsat != outSat);
  bool helpsUnsat =
    (!saturated) ||
    (outUnsat >  _outLimit && err < 0) ||
    (outUnsat < -_outLimit && err > 0);

  if (_ki != 0.0f && helpsUnsat) {
    _iTerm += (err * _ki * dt);
    _iTerm = clampf(_iTerm, -_iLimit, _iLimit);
  }

  float out = preSat + _iTerm;

  // Clamp output
  out = clampf(out, -_outLimit, _outLimit);

  // Slew-rate limiting (smooth torque)
  if (_slewRate > 0.0f) {
    float maxStep = _slewRate * dt;
    float delta = out - _lastOut;
    if (delta >  maxStep) out = _lastOut + maxStep;
    if (delta < -maxStep) out = _lastOut - maxStep;
  }
  _lastOut = out;

  int32_t duty = (int32_t)(out * (float)_pwmMax + (out >= 0 ? 0.5f : -0.5f));
  if (AppConfig::Debug::MOTOR_PID_TRACE) {
    const char* side = "?";
    uint32_t* ctr = nullptr;
    if (_pwmCh == AppConfig::Motors::LEFT_PWM_CHANNEL) {
      side = "L";
      ctr = &sMotorPidTraceCounterL;
    } else if (_pwmCh == AppConfig::Motors::RIGHT_PWM_CHANNEL) {
      side = "R";
      ctr = &sMotorPidTraceCounterR;
    }
    const uint8_t everyN = (AppConfig::Debug::MOTOR_PID_TRACE_EVERY_N == 0)
                           ? 1
                           : AppConfig::Debug::MOTOR_PID_TRACE_EVERY_N;
    bool shouldPrint = true;
    if (ctr != nullptr) {
      (*ctr)++;
      shouldPrint = ((*ctr % everyN) == 0);
    }
    if (shouldPrint) {
      char buf[180];
      snprintf(buf, sizeof(buf),
               "PID %s tgt=%.1f tps=%.1f err=%.2f P=%.4f I=%.4f D=%.4f out=%.4f duty=%ld",
               side, _targetTPS, _tps, err, pTerm, _iTerm, dTerm, out, (long)duty);
      if (_logFn != nullptr) {
        _logFn(String(buf));
      } else if (AppConfig::Debug::ENABLE_SERIAL_OUTPUT) {
        portENTER_CRITICAL(&sMotorPidTraceMux);
        Serial.println(buf);
        portEXIT_CRITICAL(&sMotorPidTraceMux);
      }
    }
  }
  applyDuty(duty);
}

int32_t DcMotor::getTicks() const {
  int32_t t;
  t = _ticks;
  return t;
}

void DcMotor::resetTicks(int32_t value) {
  _ticks = value;
  _lastTicks = value;
  _speedAccumTicks = 0;
  _speedAccumUs = 0;
}

// ---------------- Encoder ISR (simple, encA only) ----------------
// Attach interrupt on encA CHANGE. When it triggers, read A and B.
// Simple rule: step = (A == B) ? +1 : -1
// If direction is reversed, set invertEnc=true for that motor.
// If still wrong, change the rule to (A != B) ? +1 : -1.
void DcMotor::handleEncAChange() { // rename ok, still called by ISR

  bool B = gpio_get_level((gpio_num_t)_p.encB);

  // For A rising, direction depends on B (might need to flip)
  int8_t step = B ? -1 : +1;

  if (_p.invertEnc) step = -step;
  _ticks += step;
}


// ISR wrappers (only these have IRAM_ATTR)
void IRAM_ATTR DcMotor::isrEncA0(){ if(_instances[0]) _instances[0]->handleEncAChange(); }
void IRAM_ATTR DcMotor::isrEncA1(){ if(_instances[1]) _instances[1]->handleEncAChange(); }
void IRAM_ATTR DcMotor::isrEncA2(){ if(_instances[2]) _instances[2]->handleEncAChange(); }
void IRAM_ATTR DcMotor::isrEncA3(){ if(_instances[3]) _instances[3]->handleEncAChange(); }

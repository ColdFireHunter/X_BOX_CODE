#include <Arduino.h>
#include <pins.h>
#include <global.h>
#include <AsyncLED.h>
#include <gpio.h>

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
gpio GPIO;
AsyncLED asyncLED;

// LED DMA1 CH1
// ADC DMA1 CH7

static uint32_t lastPotTimeout = 0;
static bool ledsActive = false;
static uint32_t cycleEndTime = 0;

static bool doorUp = false; // false = down, true = up
static bool lastWisch1, lastWisch2, retried;
static bool pendingReq1 = false, pendingReq2 = false;

static bool lastDip1State = false;

// FSM states
enum class DoorState
{
  IDLE,
  BOOST_START,
  WAIT_PG,
  ENABLE_OUTPUT,
  SEND_WISCH,
  TRACK_MOTION,
  FINISH_DELAY,
  DISABLE_ALL
};
enum class ErrorReason
{
  NO_PG_SIGNAL,
  MOTOR_NO_RESPONSE,
  MOTOR_NO_END
};

// bit-flags for each possible trigger source
enum InputSource : uint8_t
{
  SRC_NONE = 0,
  SRC_KEY1 = 1 << 0,
  SRC_RF1 = 1 << 1,
  SRC_KEY2 = 1 << 2,
  SRC_RF2 = 1 << 3,
  SRC_FRONTBTN = 1 << 4,
};

static constexpr uint32_t BOOST_STABILIZE_MS = 250;
static constexpr uint32_t TECH_POWERUP_MS = 2000;
static constexpr uint32_t MOTOR_TIMEOUT_MS = 2500;
static constexpr uint32_t END_TIMEOUT_MS = 60000;
static constexpr uint32_t FINISH_DELAY_MS = 5000;
static constexpr uint32_t OPTO_DEBOUNCE_MS = 400;

static Neotimer boostTimer(BOOST_STABILIZE_MS);
static Neotimer techTimer(TECH_POWERUP_MS);
static Neotimer motorTimer(MOTOR_TIMEOUT_MS);
static Neotimer endTimeoutTimer(END_TIMEOUT_MS);
static Neotimer finishTimer(FINISH_DELAY_MS);
static Neotimer motionPresentDebounce(OPTO_DEBOUNCE_MS);
static Neotimer motionAbsentDebounce(OPTO_DEBOUNCE_MS);

static DoorState state = DoorState::IDLE;

inline void resetAllTimers()
{
  for (Neotimer *t : {&boostTimer, &techTimer, &motorTimer, &endTimeoutTimer,
                      &finishTimer, &motionPresentDebounce, &motionAbsentDebounce})
  {
    if (t->started())
      t->stop();
    t->reset();
  }
}

void errorHandler(ErrorReason reason)
{
  DEBUG_SERIAL.print("Error: ");
  switch (reason)
  {
  case ErrorReason::NO_PG_SIGNAL:
    DEBUG_SERIAL.println("Boost-Good (PG) signal never detected");
    break;
  case ErrorReason::MOTOR_NO_RESPONSE:
    DEBUG_SERIAL.println("Motor did not respond to Wisch pulse");
    break;
  case ErrorReason::MOTOR_NO_END:
    DEBUG_SERIAL.println("Motor did not finish moving in time");
    break;
  }
}

bool pollRequest(bool &want1, bool &want2, uint8_t &sourceFlags)
{
  want1 = want2 = false;
  sourceFlags = SRC_NONE;

  if (GPIO.getKeySwitch1())
  {
    want1 = true;
    sourceFlags |= SRC_KEY1;
  }
  if (GPIO.getRF_In1())
  {
    want1 = true;
    sourceFlags |= SRC_RF1;
  }
  if (GPIO.getKeySwitch2())
  {
    want2 = true;
    sourceFlags |= SRC_KEY2;
  }
  if (GPIO.getRF_In2())
  {
    want2 = true;
    sourceFlags |= SRC_RF2;
  }
  if (GPIO.getFrontPanelButton())
  {
    want1 = true;
    sourceFlags |= SRC_FRONTBTN;
  }

  return sourceFlags != SRC_NONE;
}

void updateDoorState()
{
  bool req1, req2;
  uint8_t src;
  bool hasReq = pollRequest(req1, req2, src);

  switch (state)
  {
  case DoorState::IDLE:
    if (hasReq)
    {
      DEBUG_SERIAL.print("INFO: Request from:");
      if (src & SRC_KEY1)
        DEBUG_SERIAL.print(" KEY1");
      if (src & SRC_RF1)
        DEBUG_SERIAL.print(" RF1");
      if (src & SRC_KEY2)
        DEBUG_SERIAL.print(" KEY2");
      if (src & SRC_RF2)
        DEBUG_SERIAL.print(" RF2");
      if (src & SRC_FRONTBTN)
        DEBUG_SERIAL.print(" FRONTBTN");
      DEBUG_SERIAL.println();

      asyncLED.fillRGBW(255, 255, 255, 255); // sofort volle Helligkeit
      asyncLED.show();
      ledsActive = true;
      cycleEndTime = 0;

      if (GPIO.getDip1())
      {
        lastWisch1 = req1;
        lastWisch2 = req2;
      }
      else
      {
        lastWisch1 = true;
        lastWisch2 = false;
      }

      retried = false;
      GPIO.setBoostEnable(true);
      boostTimer.reset();
      boostTimer.start();
      state = DoorState::BOOST_START;
    }
    break;

  case DoorState::BOOST_START:
    if (boostTimer.done())
    {
      DEBUG_SERIAL.println("INFO: BOOST stabilized → WAIT_PG");
      state = DoorState::WAIT_PG;
    }
    break;

  case DoorState::WAIT_PG:
    if (GPIO.getBoostStatus())
    {
      DEBUG_SERIAL.println("INFO: PG high → ENABLE_OUTPUT");
      GPIO.setOutputEnable(true);
      techTimer.reset();
      techTimer.start();
      state = DoorState::ENABLE_OUTPUT;
    }
    break;

  case DoorState::ENABLE_OUTPUT:
    if (techTimer.done())
    {
      DEBUG_SERIAL.println("INFO: Control electronics powered → SEND_WISCH");
      state = DoorState::SEND_WISCH;
    }
    break;

  case DoorState::SEND_WISCH:
    if (lastWisch1)
    {
      GPIO.triggerWisch1();
      DEBUG_SERIAL.println("INFO: Fired Wisch 1");
    }
    if (lastWisch2)
    {
      GPIO.triggerWisch2();
      DEBUG_SERIAL.println("Fired Wisch 2");
    }
    motorTimer.reset();
    motorTimer.start();
    motionPresentDebounce.stop();
    motionPresentDebounce.reset();
    motionAbsentDebounce.stop();
    motionAbsentDebounce.reset();
    state = DoorState::TRACK_MOTION;
    break;

  case DoorState::TRACK_MOTION:
  {
    uint8_t dir = GPIO.getMotorDirection();

    if (dir != 0)
    {
      // Motor hat geantwortet → Response-Timeout stoppen
      if (motorTimer.started())
        motorTimer.stop();

      // Beginn-Confirmation
      if (!motionPresentDebounce.started())
      {
        motionPresentDebounce.reset();
        motionPresentDebounce.start();
      }
      if (motionPresentDebounce.done())
      {
        doorUp = (dir == 1);
        if (!endTimeoutTimer.started())
        {
          endTimeoutTimer.reset();
          endTimeoutTimer.start();
        }
        if (motionAbsentDebounce.started())
        {
          motionAbsentDebounce.stop();
          motionAbsentDebounce.reset();
        }
      }
    }
    else // dir == 0
    {
      if (motionPresentDebounce.started() && !motionPresentDebounce.done())
      {
        motionPresentDebounce.stop();
        motionPresentDebounce.reset();
      }

      if (endTimeoutTimer.started())
      {
        if (!motionAbsentDebounce.started())
        {
          motionAbsentDebounce.reset();
          motionAbsentDebounce.start();
        }
        if (motionAbsentDebounce.done())
        {
          DEBUG_SERIAL.println("INFO: Motor stopped (debounced) → Finish Delay");
          endTimeoutTimer.stop();
          finishTimer.reset();
          finishTimer.start();
          state = DoorState::FINISH_DELAY;
        }
      }

      if (motorTimer.done() && !endTimeoutTimer.started())
      {
        if (!retried)
        {
          retried = true;
          if (lastWisch1)
          {
            GPIO.triggerWisch1();
            DEBUG_SERIAL.println("INFO: Retry Wisch 1");
          }
          if (lastWisch2)
          {
            GPIO.triggerWisch2();
            DEBUG_SERIAL.println("INFO: Retry Wisch 2");
          }
          motorTimer.reset();
          motorTimer.start();
        }
        else
        {
          errorHandler(ErrorReason::MOTOR_NO_RESPONSE);
          finishTimer.reset();
          finishTimer.start();
          state = DoorState::FINISH_DELAY;
        }
      }
    }

    if (endTimeoutTimer.done())
    {
      errorHandler(ErrorReason::MOTOR_NO_END);
      finishTimer.reset();
      finishTimer.start();
      state = DoorState::FINISH_DELAY;
    }

    // neue Requests nur merken
    if (hasReq)
    {
      if (GPIO.getDip1())
      {
        pendingReq1 |= req1;
        pendingReq2 |= req2;
      }
      else
      {
        pendingReq1 = true;
        pendingReq2 = false;
      }
    }
  }
  break;

  case DoorState::FINISH_DELAY:
    if (finishTimer.done())
    {
      finishTimer.stop();
      finishTimer.reset();
      DEBUG_SERIAL.println("INFO: Finish delay over");

      if (pendingReq1 || pendingReq2)
      {
        lastWisch1 = pendingReq1;
        lastWisch2 = pendingReq2;
        pendingReq1 = pendingReq2 = false;
        retried = false;
        state = DoorState::SEND_WISCH;
        cycleEndTime = 0;
      }
      else
      {
        if (ledsActive && cycleEndTime == 0)
        {
          cycleEndTime = millis();
        }
        state = DoorState::DISABLE_ALL;
      }
    }
    break;

  case DoorState::DISABLE_ALL:
    GPIO.setOutputEnable(false);
    GPIO.setBoostEnable(false);
    resetAllTimers();
    DEBUG_SERIAL.println("INFO: OUTPUT & BOOST disabled → back to IDLE");
    state = DoorState::IDLE;
    break;
  }
}

void handleLEDDisable()
{
  uint32_t potTime = GPIO.getPotiTime();
  if (potTime != lastPotTimeout)
  {
    lastPotTimeout = potTime;
    DEBUG_SERIAL.print("INFO: Pot timeout set to ");
    DEBUG_SERIAL.print(potTime);
    DEBUG_SERIAL.println(" ms");
  }

  if (!ledsActive || cycleEndTime == 0)
    return;

  uint32_t elapsed = millis() - cycleEndTime;
  uint32_t disableMs = GPIO.getPotiTime();
  if (elapsed >= disableMs)
  {
    asyncLED.clearRGBW();
    asyncLED.show();
    ledsActive = false;
    DEBUG_SERIAL.println("INFO: LEDs disabled after pot timeout");
  }
}

void handleDip1Change()
{
  bool curr = GPIO.getDip1();
  if (curr != lastDip1State)
  {
    lastDip1State = curr;
    DEBUG_SERIAL.print("INFO: DIP1 changed → ");
    if (curr)
      DEBUG_SERIAL.println("INFO: Simultaneous mode ENABLED");
    else
      DEBUG_SERIAL.println("INFO: Independent mode ENABLED");
  }
}

// --- Battery & Solar Charging State Machine (integer millivolts) -------------
// Event-driven debug only (minimal):
//  - Logs only when chargers enable/disable and when battery health/absence changes.
//  - Debounced charger-status used silently (0 = finished, 1 = active). No status prints/counters.
//  - Enable rule (updated): Vsol must be at least Battery + 50 mV (no upper limit).
//  - Keep-enabled rule (updated): keep only while Vsol >= Battery - 100 mV.
//  - Extra safety: if solar voltage drops fast, disable chargers immediately.
//  - Parallel charging allowed when solar >= 30V (still must satisfy each charger’s enable rule).
//  - OCV health check: every 30s disable chargers for 5s to classify ABSENT / LOW / CRITICAL.
//  - Termination logic: if a charger reports finished (0), keep it enabled; allow the other too.

static constexpr uint32_t SAMPLE_INTERVAL_MS = 50;
static constexpr uint32_t DECISION_INTERVAL_MS = 500;
static constexpr uint8_t AVG_WINDOW_SAMPLES = DECISION_INTERVAL_MS / SAMPLE_INTERVAL_MS; // 10
static_assert(AVG_WINDOW_SAMPLES >= 2, "AVG_WINDOW_SAMPLES must be >= 2");

static constexpr int32_t BAT_PRESENT_MIN_MV = 1000; // >= 1.0 V means present

// Hysteresis thresholds (UPDATED per request)
static constexpr int32_t START_HYS_MV = 50; // enable if Vsol >= Vbat + 0.05 V
static constexpr int32_t STOP_HYS_MV = 100; // disable if Vsol <  Vbat - 0.10 V

static constexpr int32_t SWITCH_DELTA_MV = 50;      // priority switch deadband
static constexpr int32_t PARALLEL_SOLAR_MV = 30000; // 30.0 V threshold to allow second charger

// Fast solar drop protection
static constexpr int32_t SOLAR_FAST_DROP_MV = 1500; // if Vsol drops by ≥1.5 V between decisions → disable

// Debounce for charger status pins
static constexpr uint8_t STATUS_STABLE_CYCLES = 5; // require 5 decision ticks stable

// Battery health thresholds (24V SLA pack, open-circuit; approx)
static constexpr int32_t BAT_LOW_THRESH_MV = 23600;  // ~20% SoC
static constexpr int32_t BAT_CRIT_THRESH_MV = 22000; // critical cutoff
static constexpr int32_t RISING_EPS_MV = 30;         // rising detection eps during OCV window

// OCV measurement scheduling
static constexpr uint32_t MEASURE_PERIOD_MS = 30000; // every 30s
static constexpr uint32_t MEASURE_WINDOW_MS = 5000;  // disable for 5s

// ---- Local helpers: ring-buffer averaging ----
template <size_t N>
struct IntFifoAvg
{
  int32_t buf[N] = {0};
  uint8_t idx = 0;
  uint8_t count = 0;
  int64_t sum = 0;
  void push(int32_t v)
  {
    if (count < N)
    {
      buf[idx] = v;
      sum += v;
      idx = (idx + 1) % N;
      count++;
    }
    else
    {
      sum -= buf[idx];
      buf[idx] = v;
      sum += v;
      idx = (idx + 1) % N;
    }
  }
  int32_t avg() const
  {
    if (count == 0)
      return 0;
    return static_cast<int32_t>((sum + (count / 2)) / count);
  }
};

static IntFifoAvg<AVG_WINDOW_SAMPLES> avgB1;
static IntFifoAvg<AVG_WINDOW_SAMPLES> avgB2;
static IntFifoAvg<AVG_WINDOW_SAMPLES> avgSol;

static uint32_t lastSampleMs = 0;
static uint32_t lastDecisionMs = 0;

// ---- Charger state tracking ----
enum class ChargeState
{
  IDLE,
  CHARGE_B1,
  CHARGE_B2,
  CHARGE_BOTH
};
static ChargeState chgState = ChargeState::IDLE;

static bool ch1Enabled = false;
static bool ch2Enabled = false;

// hysteresis latches per charger
struct HysLatch
{
  bool mayEnable = true;
};
static HysLatch hys1, hys2;

// --- Status debounce bookkeeping (silent) ---
struct StatusDebounce
{
  uint8_t stable = 1; // debounced status (0=finished, 1=active). Default 1.
  uint8_t candidate = 1;
  uint8_t count = 0;
};
static StatusDebounce ch1StatDb{};
static StatusDebounce ch2StatDb{};

// --- Battery health flags (for other FSMs) ---
static bool bat1Absent = false;
static bool bat2Absent = false;
static bool bat1Low = false;
static bool bat2Low = false;
static bool bat1Critical = false;
static bool bat2Critical = false;

// Track last reported flags to reduce log spam
static bool bat1LowPrev = false, bat2LowPrev = false;
static bool bat1CritPrev = false, bat2CritPrev = false;
static bool bat1AbsPrev = false, bat2AbsPrev = false;

// --- OCV measurement scheduling state ---
static uint32_t lastMeasureCycleStartMs = 0;
static bool measureActive = false;
static uint32_t measureActiveStartMs = 0;
static int32_t lastOCVB1mV = 0, lastOCVB2mV = 0;

// Track last solar (decision-averaged) for fast-drop detection
static int32_t lastDecisionVsol = 0;

// --- GPIO wrappers ---
static void setCharger1(bool on)
{
  if (ch1Enabled == on)
    return;
  if (on)
  {
    DEBUG_SERIAL.println("[CH1] Enabling");
    ch1Enabled = true;
    GPIO.enableBatteryCharger1(1);
    hys1.mayEnable = false;
  }
  else
  {
    DEBUG_SERIAL.println("[CH1] Disabling");
    ch1Enabled = false;
    GPIO.enableBatteryCharger1(0);
  }
}

static void setCharger2(bool on)
{
  if (ch2Enabled == on)
    return;
  if (on)
  {
    DEBUG_SERIAL.println("[CH2] Enabling");
    ch2Enabled = true;
    GPIO.enableBatteryCharger2(1);
    hys2.mayEnable = false;
  }
  else
  {
    DEBUG_SERIAL.println("[CH2] Disabling");
    ch2Enabled = false;
    GPIO.enableBatteryCharger2(0);
  }
}

// --- Solar rules (UPDATED) ---
// Enable only if Vsol >= Vbat + 50 mV (no upper bound).
static bool solarAllowsEnable(int32_t vSol, int32_t vBat, HysLatch &hys)
{
  if (hys.mayEnable)
  {
    return (vSol >= (vBat + START_HYS_MV));
  }
  else
  {
    if (vSol >= (vBat + START_HYS_MV))
    {
      hys.mayEnable = true; // re-arm silently
    }
    return false;
  }
}

// Keep enabled only while Vsol >= Vbat - 100 mV.
static bool solarKeepEnabled(int32_t vSol, int32_t vBat)
{
  return (vSol >= (vBat - STOP_HYS_MV));
}

static bool batteryPresent(int32_t vBat) { return vBat >= BAT_PRESENT_MIN_MV; }

// Debounce one status pin; return true if debounced change occurred
static bool debounceStatus(StatusDebounce &db, uint8_t raw)
{
  if (raw == db.candidate)
  {
    if (db.count < 255)
      db.count++;
  }
  else
  {
    db.candidate = raw;
    db.count = 1;
  }
  if (db.candidate != db.stable && db.count >= STATUS_STABLE_CYCLES)
  {
    db.stable = db.candidate;
    db.count = STATUS_STABLE_CYCLES;
    return true;
  }
  return false;
}

// --- Status management (silent; no prints) ---
static void updateStatusPinsSilent()
{
  const uint8_t s1raw = GPIO.getBatteryCharger1Status();
  const uint8_t s2raw = GPIO.getBatteryCharger2Status();
  (void)debounceStatus(ch1StatDb, s1raw);
  (void)debounceStatus(ch2StatDb, s2raw);
}

// --- Battery health classification (event logs only) ---
static void classifyBatteryHealthOCV(int32_t vB1, int32_t vB2)
{
  const bool newBat1Absent = !batteryPresent(vB1);
  const bool newBat2Absent = !batteryPresent(vB2);

  const bool b1Rising = (vB1 >= lastOCVB1mV + RISING_EPS_MV);
  const bool b2Rising = (vB2 >= lastOCVB2mV + RISING_EPS_MV);

  lastOCVB1mV = vB1;
  lastOCVB2mV = vB2;

  if (newBat1Absent != bat1Absent)
  {
    bat1Absent = newBat1Absent;
    DEBUG_SERIAL.print("[B1][ABSENT] ");
    DEBUG_SERIAL.println(bat1Absent ? "YES" : "NO");
  }
  if (newBat2Absent != bat2Absent)
  {
    bat2Absent = newBat2Absent;
    DEBUG_SERIAL.print("[B2][ABSENT] ");
    DEBUG_SERIAL.println(bat2Absent ? "YES" : "NO");
  }

  bool newB1Low = bat1Low, newB1Crit = bat1Critical;
  if (!bat1Absent && !b1Rising)
  {
    if (vB1 <= BAT_CRIT_THRESH_MV)
    {
      newB1Crit = true;
      newB1Low = true;
    }
    else if (vB1 <= BAT_LOW_THRESH_MV)
    {
      newB1Low = true;
      newB1Crit = false;
    }
    else
    {
      newB1Low = false;
      newB1Crit = false;
    }
  }

  bool newB2Low = bat2Low, newB2Crit = bat2Critical;
  if (!bat2Absent && !b2Rising)
  {
    if (vB2 <= BAT_CRIT_THRESH_MV)
    {
      newB2Crit = true;
      newB2Low = true;
    }
    else if (vB2 <= BAT_LOW_THRESH_MV)
    {
      newB2Low = true;
      newB2Crit = false;
    }
    else
    {
      newB2Low = false;
      newB2Crit = false;
    }
  }

  if (newB1Low != bat1Low || newB1Crit != bat1Critical)
  {
    bat1Low = newB1Low;
    bat1Critical = newB1Crit;
    DEBUG_SERIAL.print("[B1][HEALTH] ");
    if (bat1Critical)
      DEBUG_SERIAL.println("CRITICAL");
    else if (bat1Low)
      DEBUG_SERIAL.println("LOW");
    else
      DEBUG_SERIAL.println("NORMAL");
  }
  if (newB2Low != bat2Low || newB2Crit != bat2Critical)
  {
    bat2Low = newB2Low;
    bat2Critical = newB2Crit;
    DEBUG_SERIAL.print("[B2][HEALTH] ");
    if (bat2Critical)
      DEBUG_SERIAL.println("CRITICAL");
    else if (bat2Low)
      DEBUG_SERIAL.println("LOW");
    else
      DEBUG_SERIAL.println("NORMAL");
  }
}

// --- Main update ---
void updateBatterySolarCharger()
{
  const uint32_t now = millis();

  // sampling
  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS)
  {
    lastSampleMs = now;
    avgB1.push(GPIO.getBattery1Voltage());
    avgB2.push(GPIO.getBattery2Voltage());
    avgSol.push(GPIO.getSolarVoltage());
  }

  // decision
  if (now - lastDecisionMs < DECISION_INTERVAL_MS)
    return;
  lastDecisionMs = now;

  const int32_t vB1 = avgB1.avg();
  const int32_t vB2 = avgB2.avg();
  const int32_t vSol = avgSol.avg();

  // Fast drop protection (compare to previous decision tick)
  if (lastDecisionVsol != 0)
  {
    const int32_t drop = lastDecisionVsol - vSol;
    if (drop >= SOLAR_FAST_DROP_MV)
    {
      // Immediate safety shutdown
      if (ch1Enabled || ch2Enabled)
      {
        DEBUG_SERIAL.println("[SYS] Solar fast drop → disable all");
        setCharger1(false);
        setCharger2(false);
      }
    }
  }
  lastDecisionVsol = vSol;

  const bool b1PresentNow = batteryPresent(vB1);
  const bool b2PresentNow = batteryPresent(vB2);

  // OCV measurement scheduling (silent start/stop)
  if (!measureActive && (now - lastMeasureCycleStartMs >= MEASURE_PERIOD_MS))
  {
    measureActive = true;
    measureActiveStartMs = now;
    lastMeasureCycleStartMs = now;
  }

  if (measureActive)
  {
    if (ch1Enabled)
      setCharger1(false);
    if (ch2Enabled)
      setCharger2(false);

    // Debounce status silently; we still want stable status info for after OCV
    updateStatusPinsSilent();

    // Classify health (prints only on changes)
    classifyBatteryHealthOCV(vB1, vB2);

    if ((now - measureActiveStartMs) >= MEASURE_WINDOW_MS)
    {
      measureActive = false;
    }
    return;
  }

  // --- Normal logic below (only when not in OCV window) ---

  // Update debounced status pins (silent)
  updateStatusPinsSilent();

  // Use debounced status for logic (0=finished, 1=active)
  const uint8_t s1 = ch1StatDb.stable;
  const uint8_t s2 = ch2StatDb.stable;

  const bool b1Present = b1PresentNow;
  const bool b2Present = b2PresentNow;
  const bool bothPresent = b1Present && b2Present;
  const bool neitherPresent = !b1Present && !b2Present;

  if (neitherPresent)
  {
    if (ch1Enabled || ch2Enabled)
    {
      DEBUG_SERIAL.println("[SYS] No batteries → disable all");
      setCharger1(false);
      setCharger2(false);
    }
    chgState = ChargeState::IDLE;
    return;
  }

  // Preference: lower voltage battery first (with small deadband)
  bool preferB1 = false, preferB2 = false;
  if (bothPresent)
  {
    if (vB1 + SWITCH_DELTA_MV < vB2)
      preferB1 = true;
    else if (vB2 + SWITCH_DELTA_MV < vB1)
      preferB2 = true;
    else
      preferB1 = true; // tie → B1
  }
  else
  {
    if (b1Present)
      preferB1 = true;
    if (b2Present)
      preferB2 = true;
  }

  // Termination logic (debounced): if finished (status==0) keep enabled; allow other to run as well
  const bool b1Finished = (b1Present && (s1 == 0));
  const bool b2Finished = (b2Present && (s2 == 0));

  if (b1Finished && b2Finished)
  {
    setCharger1(true);
    setCharger2(true);
    chgState = ChargeState::CHARGE_BOTH;
    return;
  }

  auto canEnable1 = [&]
  { return solarAllowsEnable(vSol, vB1, hys1); };
  auto canEnable2 = [&]
  { return solarAllowsEnable(vSol, vB2, hys2); };
  auto keep1 = [&]
  { return solarKeepEnabled(vSol, vB1); };
  auto keep2 = [&]
  { return solarKeepEnabled(vSol, vB2); };

  if (preferB1)
  {
    if (b1Finished)
    {
      setCharger1(true);
    }
    else
    {
      if (ch1Enabled)
      {
        if (!keep1())
          setCharger1(false);
      }
      else if (canEnable1())
        setCharger1(true);
    }

    if (b2Present)
    {
      if (b1Finished)
      {
        if (ch2Enabled)
        {
          if (!keep2())
            setCharger2(false);
        }
        else if (canEnable2())
          setCharger2(true);
      }
      else
      {
        // Normal parallel rule: require ≥30V AND satisfy own enable/keep rules
        if (ch2Enabled)
        {
          if (!(vSol >= PARALLEL_SOLAR_MV && keep2()))
            setCharger2(false);
        }
        else
        {
          if (vSol >= PARALLEL_SOLAR_MV && canEnable2())
            setCharger2(true);
        }
      }
    }
  }
  else if (preferB2)
  {
    if (b2Finished)
    {
      setCharger2(true);
    }
    else
    {
      if (ch2Enabled)
      {
        if (!keep2())
          setCharger2(false);
      }
      else if (canEnable2())
        setCharger2(true);
    }

    if (b1Present)
    {
      if (b2Finished)
      {
        if (ch1Enabled)
        {
          if (!keep1())
            setCharger1(false);
        }
        else if (canEnable1())
          setCharger1(true);
      }
      else
      {
        if (ch1Enabled)
        {
          if (!(vSol >= PARALLEL_SOLAR_MV && keep1()))
            setCharger1(false);
        }
        else
        {
          if (vSol >= PARALLEL_SOLAR_MV && canEnable1())
            setCharger1(true);
        }
      }
    }
  }

  // Dynamic priority switch enable (or when one is finished)
  if (ch1Enabled && !ch2Enabled && b2Present && (vB2 + SWITCH_DELTA_MV < vB1))
  {
    if (b1Finished)
    {
      if (canEnable2())
        setCharger2(true);
    }
    else
    {
      if (vSol >= PARALLEL_SOLAR_MV && canEnable2())
        setCharger2(true);
    }
  }
  else if (ch2Enabled && !ch1Enabled && b1Present && (vB1 + SWITCH_DELTA_MV < vB2))
  {
    if (b2Finished)
    {
      if (canEnable1())
        setCharger1(true);
    }
    else
    {
      if (vSol >= PARALLEL_SOLAR_MV && canEnable1())
        setCharger1(true);
    }
  }

  // Safety: drop chargers if battery absent; or if solar collapses (unless finished)
  if (ch1Enabled && !batteryPresent(vB1))
    setCharger1(false);
  if (ch2Enabled && !batteryPresent(vB2))
    setCharger2(false);

  if (ch1Enabled && !b1Finished && !keep1())
    setCharger1(false);
  if (ch2Enabled && !b2Finished && !keep2())
    setCharger2(false);

  // Resolve state (no continuous debug)
  if (ch1Enabled && ch2Enabled)
    chgState = ChargeState::CHARGE_BOTH;
  else if (ch1Enabled)
    chgState = ChargeState::CHARGE_B1;
  else if (ch2Enabled)
    chgState = ChargeState::CHARGE_B2;
  else
    chgState = ChargeState::IDLE;
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV3;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO_PF2, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);
}
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV16;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 6;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_79CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_79CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  /* DMA1_Ch4_7_DMAMUX1_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Ch4_7_DMAMUX1_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Ch4_7_DMAMUX1_OVR_IRQn);
}

void setup()
{
  DEBUG_SERIAL.begin(115200);
  asyncLED.begin();
  asyncLED.setBrightness(160); // Set brightness to 60%
  asyncLED.clearRGBW();        // Clear the LED matrix
  asyncLED.show();

  GPIO.init();
  GPIO.setSensEnable(true);

  MX_DMA_Init();
  MX_ADC1_Init();

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_values, 6);

  DEBUG_SERIAL.println("INFO: System initialized successfully.");
}

void loop()
{
  GPIO.handler();
  updateDoorState(); // drive the door-opener FSM
  updateBatterySolarCharger();
  handleLEDDisable();
  handleDip1Change();
}

extern "C" void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if (hadc->Instance == ADC1)
  {
    /* USER CODE BEGIN ADC1_MspInit 0 */

    /* USER CODE END ADC1_MspInit 0 */

    /** Initializes the peripherals clocks
     */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_ADC_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA0     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PA2     ------> ADC1_IN2
    PA3     ------> ADC1_IN3
    PA4     ------> ADC1_IN4
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance = DMA1_Channel7;
    hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc1);

    /* USER CODE BEGIN ADC1_MspInit 1 */

    /* USER CODE END ADC1_MspInit 1 */
  }
}
extern "C" void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    /* USER CODE BEGIN ADC1_MspDeInit 0 */

    /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA0     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PA2     ------> ADC1_IN2
    PA3     ------> ADC1_IN3
    PA4     ------> ADC1_IN4
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(hadc->DMA_Handle);
    /* USER CODE BEGIN ADC1_MspDeInit 1 */

    /* USER CODE END ADC1_MspDeInit 1 */
  }
}
extern "C" void DMA1_Ch4_7_DMAMUX1_OVR_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Ch4_7_DMAMUX1_OVR_IRQn 0 */

  /* USER CODE END DMA1_Ch4_7_DMAMUX1_OVR_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Ch4_7_DMAMUX1_OVR_IRQn 1 */

  /* USER CODE END DMA1_Ch4_7_DMAMUX1_OVR_IRQn 1 */
}
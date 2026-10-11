// WAV Controller firmware - Revision WAV-R2.1-2026-10-11
// Door-close suspension raise now waits for ignition plus 20 seconds.
// After 20 minutes with ignition off, pending raising is cancelled before deep sleep.
// Entry, exit, or ignition wakes the controller.

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <TCA9554.h>
#include <esp_wifi.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>

#define SDA_PIN 42
#define SCL_PIN 41

const int BTN_STOP = 4;
const int BTN_OPEN = 5;
const int BTN_CLOSE = 6;
const int IGNITION_INPUT = 11;

const int COMPRESSOR = 0;
const int EXHAUST = 1;
const int L_BAG = 2;
const int SUPPLY = 3;
const int R_BAG = 4;
const int RAMP = 5;
const int D_CLOSE = 6;
const int D_OPEN = 7;

const unsigned long SOFT_OPEN_BRAKE_MS = 250;
const unsigned long IGNITION_RAISE_DELAY_MS = 20UL * 1000UL;
const unsigned long LOW_POWER_DELAY_MS = 20UL * 60UL * 1000UL;
const unsigned long BAG_SETTLE_MS = 1500;
const unsigned long BAG_BALANCE_TIMEOUT_MS = 15000;
const int BAG_BALANCE_DEADBAND = 50;
const unsigned long SUSPENSION_CHECK_INTERVAL_MS = 20UL * 60UL * 1000UL;
const unsigned long SUSPENSION_STABILITY_SAMPLE_MS = 10000;
const unsigned long SUSPENSION_TOPUP_TIMEOUT_MS = 15000;
const int SUSPENSION_STABILITY_PERCENT = 2;
const int SUSPENSION_LOW_PERCENT = 20;

Adafruit_ADS1115 ads;
TCA9554 tca(0x20);
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
Preferences prefs;

int L_DUMP = 4154;
int L_70 = 7792;
int R_DUMP = 5050;
int R_70 = 7551;
int T_0 = 716;
int T_100 = 7288;
int D_OPEN_LIM = 19486;
int D_CLOSE_LIM = 8585;
int D_SLOW_OPEN = 12055;

int rawL, rawR, rawP, rawD;

enum State {
    IDLE,
    LOWERING,
    OPENING_DOOR,
    SOFT_OPEN_BRAKE,
    DEPLOYING_RAMP,
    CLOSING_RAMP,
    CLOSING_DOOR,
    WAITING_FOR_IGNITION,
    IGNITION_RAISE_DELAY,
    RAISING,
    BALANCING_BAGS,
    CHECKING_SUSPENSION,
    SUSPENSION_TOPUP
};

State currentStep = IDLE;
unsigned long stepTimer = 0;
unsigned long softOpenBrakeTimer = 0;
unsigned long lastJogTime = 0;
bool manualJogActive = false;
bool softOpenBrakeApplied = false;
bool leftRaiseComplete = false;
bool rightRaiseComplete = false;
unsigned long bagBalanceTimer = 0;
unsigned long bagSettleTimer = 0;
bool bagBalanceFilling = false;
bool balanceFillLeft = false;
bool balanceFillRight = false;
bool ignitionWasOn = false;
bool ignitionSuspensionActive = false;
bool manualDoorOpeningActive = false;
unsigned long lastSuspensionCheck = 0;
unsigned long suspensionStabilityTimer = 0;
int suspensionMinL = 0;
int suspensionMaxL = 0;
int suspensionMinR = 0;
int suspensionMaxR = 0;
bool suspensionFillLeft = false;
bool suspensionFillRight = false;
unsigned long ignitionRaiseTimer = 0;
unsigned long ignitionOffTimer = 0;
bool ignitionOffTimerRunning = false;
RTC_DATA_ATTR bool suspensionRaisePending = false;

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
  <title>WAV Controller</title>
  <style>
    body { font-family: sans-serif; background: #1c1c1e; color: #fff; margin: 0; text-align: center; user-select: none; }
    .page { display: none; padding: 15px; } .active { display: block; }
    .nav { background: #2c2c2e; padding: 12px; display: flex; justify-content: space-around; border-bottom: 1px solid #444; position: sticky; top: 0; }
    .nav-btn { color: #0a84ff; cursor: pointer; font-weight: bold; }
    .card { background: #2c2c2e; border-radius: 12px; padding: 12px; margin-bottom: 12px; text-align: left; border-left: 4px solid #0a84ff; }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
    button { cursor: pointer; border: none; border-radius: 8px; font-weight: bold; }
    .jog { background: #3a3a3c; color: #fff; padding: 18px 5px; touch-action: manipulation; }
    .rec-btn { background: #30d158; color: #000; padding: 10px; font-size: 11px; }
    .action-btn { background: #0a84ff; color: #fff; padding: 18px; width: 100%; margin: 5px 0; border-radius: 12px; font-size: 16px; }
    .stop-btn { background: #ff3b30; color: #fff; width: 100%; padding: 15px; margin-top: 10px; border-radius: 12px; }
    .val { font-family: monospace; color: #00ff41; float: right; font-size: 1.1em; }
    .revision { color: #8e8e93; font-size: 12px; padding: 18px 0 10px; }
  </style>
</head>
<body oncontextmenu="return false;">
  <div class="nav"><div class="nav-btn" onclick="show('main')">OP</div><div class="nav-btn" onclick="show('cal')">CAL</div></div>
  <div id="main" class="page active">
    <h3>STATUS: <span id="status" style="color:#ff9f0a">READY</span></h3>
    <div class="card">TANK: <span id="pPSI" class="val">0.0</span> PSI</div>
    <button class="action-btn" style="background:#30d158;color:#000" onclick="send('START_ENTRY')">AUTO ENTRY (OPEN)</button>
    <button class="action-btn" onclick="send('START_EXIT')">AUTO EXIT (CLOSE)</button>
    <button class="stop-btn" onclick="send('STOP')">STOP</button>
  </div>
  <div id="cal" class="page">
    <div class="card">L BAG <span id="v0" class="val">0</span><div class="grid">
      <button class="jog" onmousedown="mD(event,'L_DOWN')" onmouseup="mU(event)" ontouchstart="mD(event,'L_DOWN')" ontouchend="mU(event)">DOWN</button>
      <button class="jog" onmousedown="mD(event,'L_UP')" onmouseup="mU(event)" ontouchstart="mD(event,'L_UP')" ontouchend="mU(event)">UP</button>
      <button class="rec-btn" onclick="rec(this,'REC_L_D')">SET DUMP</button><button class="rec-btn" onclick="rec(this,'REC_L_70')">SET 70</button>
    </div></div>
    <div class="card">R BAG <span id="v1" class="val">0</span><div class="grid">
      <button class="jog" onmousedown="mD(event,'R_DOWN')" onmouseup="mU(event)" ontouchstart="mD(event,'R_DOWN')" ontouchend="mU(event)">DOWN</button>
      <button class="jog" onmousedown="mD(event,'R_UP')" onmouseup="mU(event)" ontouchstart="mD(event,'R_UP')" ontouchend="mU(event)">UP</button>
      <button class="rec-btn" onclick="rec(this,'REC_R_D')">SET DUMP</button><button class="rec-btn" onclick="rec(this,'REC_R_70')">SET 70</button>
    </div></div>
    <div class="card">DOOR <span id="v3" class="val">0</span>
      <div>Slow open point: <span id="slowOpen">0</span></div><div class="grid">
      <button class="jog" onmousedown="mD(event,'D_OPEN')" onmouseup="mU(event)" ontouchstart="mD(event,'D_OPEN')" ontouchend="mU(event)">OPEN</button>
      <button class="jog" onmousedown="mD(event,'D_CLOSE')" onmouseup="mU(event)" ontouchstart="mD(event,'D_CLOSE')" ontouchend="mU(event)">CLOSE</button>
      <button class="rec-btn" onclick="rec(this,'REC_D_O')">SET OPEN</button><button class="rec-btn" onclick="rec(this,'REC_D_C')">SET CLOSE</button>
      <button class="rec-btn" onclick="rec(this,'REC_D_SLOW')">SET SLOW OPEN</button>
    </div></div>
    <div class="card">TANK <span id="v2" class="val">0</span><div class="grid">
      <button class="jog" onmousedown="mD(event,'FILL_T')" onmouseup="mU(event)" ontouchstart="mD(event,'FILL_T')" ontouchend="mU(event)">FILL</button>
      <button class="jog" onmousedown="mD(event,'VENT_T')" onmouseup="mU(event)" ontouchstart="mD(event,'VENT_T')" ontouchend="mU(event)">VENT</button>
      <button class="rec-btn" onclick="rec(this,'REC_P_0')">SET 0</button><button class="rec-btn" onclick="rec(this,'REC_P_100')">SET 100</button>
    </div></div>
    <button class="action-btn" style="background:#bf5af2" onclick="send('SAVE')">SAVE SETTINGS</button>
  </div>
  <div class="revision">Firmware revision: WAV-R2.1-2026-10-11</div>
  <script>
    const websocket = new WebSocket(`ws://${window.location.hostname}/ws`);
    let jogInterval;
    function send(command) { if (websocket.readyState === WebSocket.OPEN) websocket.send(command); }
    function mD(event, command) { event.preventDefault(); send(command); clearInterval(jogInterval); jogInterval = setInterval(() => send(command), 150); }
    function mU(event) { event.preventDefault(); clearInterval(jogInterval); send('STOP'); }
    function rec(button, command) { send(command); const old = button.style.background; button.style.background = '#0a84ff'; setTimeout(() => button.style.background = old, 400); }
    function show(id) { document.querySelectorAll('.page').forEach(page => page.classList.remove('active')); document.getElementById(id).classList.add('active'); }
    websocket.onmessage = event => {
      const data = JSON.parse(event.data);
      document.getElementById('v0').innerText = data.a0; document.getElementById('v1').innerText = data.a1;
      document.getElementById('v2').innerText = data.a2; document.getElementById('v3').innerText = data.a3;
      document.getElementById('pPSI').innerText = data.psi.toFixed(1); document.getElementById('slowOpen').innerText = data.slowOpen;
      document.getElementById('status').innerText = data.step;
    };
  </script>
</body>
</html>
)rawliteral";

void stopRelays() {
    for (int relay = 0; relay < 8; relay++) tca.write1(relay, LOW);
}

bool ignitionOn() {
    return digitalRead(IGNITION_INPUT) == HIGH;
}

int suspensionLowThreshold(int dumpPoint, int savedLevel) {
    long calibratedRange = (long)savedLevel - dumpPoint;
    return savedLevel - (calibratedRange * SUSPENSION_LOW_PERCENT / 100L);
}

int suspensionStabilityLimit(int dumpPoint, int savedLevel) {
    long calibratedRange = abs((long)savedLevel - dumpPoint);
    int limit = calibratedRange * SUSPENSION_STABILITY_PERCENT / 100L;
    return limit < 25 ? 25 : limit;
}

void beginSuspensionCheck(unsigned long now) {
    stopRelays();
    ignitionSuspensionActive = true;
    suspensionStabilityTimer = now;
    suspensionMinL = suspensionMaxL = rawL;
    suspensionMinR = suspensionMaxR = rawR;
    currentStep = CHECKING_SUSPENSION;
}

void handleIgnitionControl() {
    unsigned long now = millis();
    bool isOn = ignitionOn();

    if (isOn && !ignitionWasOn) lastSuspensionCheck = now;

    bool doorOpeningSequence = currentStep == LOWERING || currentStep == OPENING_DOOR ||
                               currentStep == SOFT_OPEN_BRAKE || currentStep == DEPLOYING_RAMP;
    if (isOn && (doorOpeningSequence || manualDoorOpeningActive)) {
        stopRelays();
        currentStep = IDLE;
        manualJogActive = false;
        manualDoorOpeningActive = false;
    }

    if (!isOn && ignitionWasOn && ignitionSuspensionActive) {
        stopRelays();
        ignitionSuspensionActive = false;
        currentStep = suspensionRaisePending ? WAITING_FOR_IGNITION : IDLE;
    }

    ignitionWasOn = isOn;

    if (isOn && currentStep == IDLE && now - lastSuspensionCheck >= SUSPENSION_CHECK_INTERVAL_MS) {
        lastSuspensionCheck = now;
        beginSuspensionCheck(now);
    }
}

void startStableWiFiAP() {
    WiFi.persistent(false);
    WiFi.disconnect(true, true);
    delay(100);
    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
    WiFi.softAP("WAV_System", "12345678", 6, false, 4);
    esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);
}

void enterLowPowerMode() {
    stopRelays();
    ws.closeAll();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);

    // Entry and exit are active LOW and use RTC-capable GPIOs on the ESP32-S3.
    rtc_gpio_pullup_en((gpio_num_t)BTN_OPEN);
    rtc_gpio_pulldown_dis((gpio_num_t)BTN_OPEN);
    rtc_gpio_pullup_en((gpio_num_t)BTN_CLOSE);
    rtc_gpio_pulldown_dis((gpio_num_t)BTN_CLOSE);
    uint64_t buttonWakeMask = (1ULL << BTN_OPEN) | (1ULL << BTN_CLOSE);
    esp_sleep_enable_ext1_wakeup(buttonWakeMask, ESP_EXT1_WAKEUP_ANY_LOW);

    // Ignition is active HIGH, so it has its own wake source.
    esp_sleep_enable_ext0_wakeup((gpio_num_t)IGNITION_INPUT, HIGH);

    Serial.flush();
    esp_deep_sleep_start();
}

void handleLowPowerMode() {
    unsigned long now = millis();

    if (ignitionOn()) {
        ignitionOffTimerRunning = false;
        return;
    }

    if (!ignitionOffTimerRunning) {
        ignitionOffTimer = now;
        ignitionOffTimerRunning = true;
        return;
    }

    bool safeToSleep = (currentStep == IDLE || currentStep == WAITING_FOR_IGNITION) &&
                       !manualJogActive &&
                       digitalRead(BTN_OPEN) == HIGH &&
                       digitalRead(BTN_CLOSE) == HIGH;

    if (safeToSleep && now - ignitionOffTimer >= LOW_POWER_DELAY_MS) {
        // Treat an unraised close/exit sequence as abandoned after 20 minutes.
        suspensionRaisePending = false;
        if (currentStep == WAITING_FOR_IGNITION) currentStep = IDLE;
        enterLowPowerMode();
    }
}

void restoreStateAfterWake() {
    esp_sleep_wakeup_cause_t wakeCause = esp_sleep_get_wakeup_cause();

    if (suspensionRaisePending) currentStep = WAITING_FOR_IGNITION;

    if (wakeCause != ESP_SLEEP_WAKEUP_EXT1) return;

    uint64_t wakePins = esp_sleep_get_ext1_wakeup_status();
    if (wakePins & (1ULL << BTN_OPEN)) {
        // Capture even a short entry pulse that may end before setup completes.
        suspensionRaisePending = false;
        currentStep = LOWERING;
        stepTimer = millis();
    } else if (wakePins & (1ULL << BTN_CLOSE)) {
        currentStep = CLOSING_RAMP;
        stepTimer = millis();
    }
}

void checkPhysicalButtons() {
    if (digitalRead(BTN_STOP) == LOW) {
        currentStep = IDLE;
        manualJogActive = false;
        manualDoorOpeningActive = false;
        ignitionSuspensionActive = false;
        suspensionRaisePending = false;
        stopRelays();
        return;
    }
    if ((currentStep == IDLE || currentStep == WAITING_FOR_IGNITION) &&
        !ignitionOn() && digitalRead(BTN_OPEN) == LOW) {
        suspensionRaisePending = false;
        currentStep = LOWERING;
        stepTimer = millis();
    } else if (currentStep == IDLE && digitalRead(BTN_CLOSE) == LOW) {
        currentStep = CLOSING_RAMP;
        stepTimer = millis();
    }
}

void handleSequence() {
    if (currentStep == IDLE) return;
    unsigned long now = millis();

    switch (currentStep) {
        case LOWERING:
            tca.write1(SUPPLY, LOW);
            tca.write1(RAMP, HIGH);
            if ((rawL > L_DUMP || rawR > R_DUMP) && now - stepTimer < 15000) {
                tca.write1(EXHAUST, HIGH);
                tca.write1(L_BAG, rawL > L_DUMP);
                tca.write1(R_BAG, rawR > R_DUMP);
            } else {
                stopRelays();
                softOpenBrakeApplied = false;
                currentStep = OPENING_DOOR;
                stepTimer = now;
            }
            break;

        case OPENING_DOOR:
            tca.write1(SUPPLY, LOW);
            tca.write1(RAMP, HIGH);
            tca.write1(COMPRESSOR, HIGH);
            tca.write1(D_CLOSE, LOW);
            tca.write1(D_OPEN, HIGH);
            if (rawD >= D_OPEN_LIM || now - stepTimer > 10000) {
                tca.write1(D_OPEN, LOW);
                if (now - stepTimer > 2500) {
                    stopRelays();
                    currentStep = DEPLOYING_RAMP;
                    stepTimer = now;
                }
            } else if (!softOpenBrakeApplied && rawD >= D_SLOW_OPEN) {
                tca.write1(D_OPEN, LOW);
                tca.write1(D_CLOSE, HIGH);
                softOpenBrakeApplied = true;
                softOpenBrakeTimer = now;
                currentStep = SOFT_OPEN_BRAKE;
            }
            break;

        case SOFT_OPEN_BRAKE:
            tca.write1(SUPPLY, LOW);
            tca.write1(RAMP, HIGH);
            tca.write1(COMPRESSOR, HIGH);
            tca.write1(D_OPEN, LOW);
            tca.write1(D_CLOSE, HIGH);
            if (now - softOpenBrakeTimer >= SOFT_OPEN_BRAKE_MS) {
                tca.write1(D_CLOSE, LOW);
                currentStep = OPENING_DOOR;
            }
            break;

        case DEPLOYING_RAMP:
            tca.write1(SUPPLY, LOW);
            tca.write1(RAMP, LOW);
            tca.write1(COMPRESSOR, HIGH);
            if (now - stepTimer > 6000) {
                stopRelays();
                currentStep = IDLE;
            }
            break;

        case CLOSING_RAMP:
            tca.write1(SUPPLY, LOW);
            tca.write1(RAMP, HIGH);
            tca.write1(COMPRESSOR, HIGH);
            if (now - stepTimer > 6000) {
                stopRelays();
                currentStep = CLOSING_DOOR;
                stepTimer = now;
            }
            break;

        case CLOSING_DOOR:
            tca.write1(SUPPLY, LOW);
            tca.write1(RAMP, HIGH);
            tca.write1(COMPRESSOR, HIGH);
            tca.write1(D_OPEN, LOW);
            tca.write1(D_CLOSE, HIGH);
            if (rawD <= D_CLOSE_LIM || now - stepTimer > 10000) {
                tca.write1(D_CLOSE, LOW);
                if (now - stepTimer > 2500) {
                    stopRelays();
                    leftRaiseComplete = false;
                    rightRaiseComplete = false;
                    suspensionRaisePending = true;
                    currentStep = WAITING_FOR_IGNITION;
                }
            }
            break;

        case WAITING_FOR_IGNITION:
            stopRelays();
            if (ignitionOn()) {
                ignitionRaiseTimer = now;
                currentStep = IGNITION_RAISE_DELAY;
            }
            break;

        case IGNITION_RAISE_DELAY:
            stopRelays();
            if (!ignitionOn()) {
                // Require one continuous 20-second ignition-on period.
                currentStep = WAITING_FOR_IGNITION;
            } else if (now - ignitionRaiseTimer >= IGNITION_RAISE_DELAY_MS) {
                leftRaiseComplete = false;
                rightRaiseComplete = false;
                ignitionSuspensionActive = true;
                stepTimer = now;
                currentStep = RAISING;
            }
            break;

        case RAISING:
            tca.write1(RAMP, HIGH);
            if (rawL >= L_70) leftRaiseComplete = true;
            if (rawR >= R_70) rightRaiseComplete = true;
            if ((!leftRaiseComplete || !rightRaiseComplete) && now - stepTimer < 20000) {
                tca.write1(COMPRESSOR, rawP < (T_100 * 0.85));
                bool openL = !leftRaiseComplete;
                bool openR = !rightRaiseComplete;
                tca.write1(L_BAG, openL);
                tca.write1(R_BAG, openR);
                tca.write1(SUPPLY, openL || openR);
            } else {
                stopRelays();
                if (leftRaiseComplete && rightRaiseComplete) {
                    bagBalanceTimer = now;
                    bagSettleTimer = now;
                    bagBalanceFilling = false;
                    balanceFillLeft = false;
                    balanceFillRight = false;
                    currentStep = BALANCING_BAGS;
                } else {
                    suspensionRaisePending = false;
                    ignitionSuspensionActive = false;
                    currentStep = IDLE;
                }
            }
            break;

        case BALANCING_BAGS:
            if (now - bagBalanceTimer >= BAG_BALANCE_TIMEOUT_MS) {
                stopRelays();
                ignitionSuspensionActive = false;
                suspensionRaisePending = false;
                currentStep = IDLE;
                break;
            }

            if (!bagBalanceFilling) {
                if (now - bagSettleTimer < BAG_SETTLE_MS) break;

                balanceFillLeft = rawL < (L_70 - BAG_BALANCE_DEADBAND);
                balanceFillRight = rawR < (R_70 - BAG_BALANCE_DEADBAND);
                if (!balanceFillLeft && !balanceFillRight) {
                    stopRelays();
                    ignitionSuspensionActive = false;
                    suspensionRaisePending = false;
                    currentStep = IDLE;
                    break;
                }
                bagBalanceFilling = true;
            }

            if (balanceFillLeft && rawL >= L_70) balanceFillLeft = false;
            if (balanceFillRight && rawR >= R_70) balanceFillRight = false;

            if (!balanceFillLeft && !balanceFillRight) {
                stopRelays();
                bagBalanceFilling = false;
                bagSettleTimer = now;
                break;
            }

            tca.write1(COMPRESSOR, rawP < (T_100 * 0.85));
            tca.write1(L_BAG, balanceFillLeft);
            tca.write1(R_BAG, balanceFillRight);
            tca.write1(SUPPLY, balanceFillLeft || balanceFillRight);
            break;

        case CHECKING_SUSPENSION:
            if (rawL < suspensionMinL) suspensionMinL = rawL;
            if (rawL > suspensionMaxL) suspensionMaxL = rawL;
            if (rawR < suspensionMinR) suspensionMinR = rawR;
            if (rawR > suspensionMaxR) suspensionMaxR = rawR;

            if (now - suspensionStabilityTimer >= SUSPENSION_STABILITY_SAMPLE_MS) {
                bool leftStable = suspensionMaxL - suspensionMinL <= suspensionStabilityLimit(L_DUMP, L_70);
                bool rightStable = suspensionMaxR - suspensionMinR <= suspensionStabilityLimit(R_DUMP, R_70);

                if (leftStable && rightStable) {
                    suspensionFillLeft = rawL < suspensionLowThreshold(L_DUMP, L_70);
                    suspensionFillRight = rawR < suspensionLowThreshold(R_DUMP, R_70);
                } else {
                    suspensionFillLeft = false;
                    suspensionFillRight = false;
                }

                if (suspensionFillLeft || suspensionFillRight) {
                    stepTimer = now;
                    currentStep = SUSPENSION_TOPUP;
                } else {
                    ignitionSuspensionActive = false;
                    currentStep = IDLE;
                }
            }
            break;

        case SUSPENSION_TOPUP:
            if (suspensionFillLeft && rawL >= L_70) suspensionFillLeft = false;
            if (suspensionFillRight && rawR >= R_70) suspensionFillRight = false;

            if (!suspensionFillLeft && !suspensionFillRight) {
                stopRelays();
                bagBalanceTimer = now;
                bagSettleTimer = now;
                bagBalanceFilling = false;
                balanceFillLeft = false;
                balanceFillRight = false;
                currentStep = BALANCING_BAGS;
            } else if (now - stepTimer >= SUSPENSION_TOPUP_TIMEOUT_MS) {
                stopRelays();
                ignitionSuspensionActive = false;
                currentStep = IDLE;
            } else {
                tca.write1(COMPRESSOR, rawP < (T_100 * 0.85));
                tca.write1(L_BAG, suspensionFillLeft);
                tca.write1(R_BAG, suspensionFillRight);
                tca.write1(SUPPLY, suspensionFillLeft || suspensionFillRight);
            }
            break;

        case IDLE:
            break;
    }
}

void saveSettings() {
    prefs.begin("wav_cal", false);
    prefs.putInt("ld", L_DUMP); prefs.putInt("l7", L_70);
    prefs.putInt("rd", R_DUMP); prefs.putInt("r7", R_70);
    prefs.putInt("t0", T_0); prefs.putInt("t1", T_100);
    prefs.putInt("do", D_OPEN_LIM); prefs.putInt("dc", D_CLOSE_LIM);
    prefs.putInt("dslow", D_SLOW_OPEN);
    prefs.end();
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t length) {
    if (type != WS_EVT_DATA) return;
    String command;
    for (size_t index = 0; index < length; index++) command += (char)data[index];

    if (command == "STOP") {
        currentStep = IDLE; manualJogActive = false; manualDoorOpeningActive = false; ignitionSuspensionActive = false; suspensionRaisePending = false; stopRelays();
    } else if (command == "START_ENTRY" && !ignitionOn()) {
        manualDoorOpeningActive = false;
        suspensionRaisePending = false;
        currentStep = LOWERING; stepTimer = millis();
    } else if (command == "START_EXIT") {
        manualDoorOpeningActive = false;
        ignitionSuspensionActive = false;
        suspensionRaisePending = false;
        currentStep = CLOSING_RAMP; stepTimer = millis();
    } else if (command == "SAVE") {
        saveSettings();
    } else if (command == "REC_L_D") L_DUMP = rawL;
    else if (command == "REC_L_70") L_70 = rawL;
    else if (command == "REC_R_D") R_DUMP = rawR;
    else if (command == "REC_R_70") R_70 = rawR;
    else if (command == "REC_P_0") T_0 = rawP;
    else if (command == "REC_P_100") T_100 = rawP;
    else if (command == "REC_D_O") D_OPEN_LIM = rawD;
    else if (command == "REC_D_C") D_CLOSE_LIM = rawD;
    else if (command == "REC_D_SLOW") D_SLOW_OPEN = rawD;
    else if (command == "L_UP") { tca.write1(COMPRESSOR, HIGH); tca.write1(SUPPLY, HIGH); tca.write1(L_BAG, HIGH); manualJogActive = true; lastJogTime = millis(); }
    else if (command == "L_DOWN") { tca.write1(EXHAUST, HIGH); tca.write1(L_BAG, HIGH); manualJogActive = true; lastJogTime = millis(); }
    else if (command == "R_UP") { tca.write1(COMPRESSOR, HIGH); tca.write1(SUPPLY, HIGH); tca.write1(R_BAG, HIGH); manualJogActive = true; lastJogTime = millis(); }
    else if (command == "R_DOWN") { tca.write1(EXHAUST, HIGH); tca.write1(R_BAG, HIGH); manualJogActive = true; lastJogTime = millis(); }
    else if (command == "D_OPEN" && !ignitionOn()) { tca.write1(D_CLOSE, LOW); tca.write1(COMPRESSOR, HIGH); tca.write1(D_OPEN, HIGH); tca.write1(SUPPLY, LOW); manualJogActive = true; manualDoorOpeningActive = true; lastJogTime = millis(); }
    else if (command == "D_CLOSE") { tca.write1(D_OPEN, LOW); tca.write1(COMPRESSOR, HIGH); tca.write1(D_CLOSE, HIGH); tca.write1(SUPPLY, LOW); manualJogActive = true; manualDoorOpeningActive = false; lastJogTime = millis(); }
    else if (command == "FILL_T") { tca.write1(COMPRESSOR, HIGH); tca.write1(SUPPLY, HIGH); manualJogActive = true; lastJogTime = millis(); }
    else if (command == "VENT_T") { tca.write1(EXHAUST, HIGH); tca.write1(SUPPLY, HIGH); manualJogActive = true; lastJogTime = millis(); }
}

void setup() {
    Serial.begin(115200);
    Serial.println("WAV Controller firmware WAV-R2.1-2026-10-11");
    prefs.begin("wav_cal", true);
    L_DUMP = prefs.getInt("ld", L_DUMP); L_70 = prefs.getInt("l7", L_70);
    R_DUMP = prefs.getInt("rd", R_DUMP); R_70 = prefs.getInt("r7", R_70);
    T_0 = prefs.getInt("t0", T_0); T_100 = prefs.getInt("t1", T_100);
    D_OPEN_LIM = prefs.getInt("do", D_OPEN_LIM); D_CLOSE_LIM = prefs.getInt("dc", D_CLOSE_LIM);
    D_SLOW_OPEN = prefs.getInt("dslow", D_SLOW_OPEN);
    prefs.end();

    Wire.begin(SDA_PIN, SCL_PIN);
    tca.begin();
    for (int relay = 0; relay < 8; relay++) tca.pinMode1(relay, OUTPUT);
    pinMode(BTN_STOP, INPUT_PULLUP); pinMode(BTN_OPEN, INPUT_PULLUP); pinMode(BTN_CLOSE, INPUT_PULLUP);
    pinMode(IGNITION_INPUT, INPUT_PULLUP);
    stopRelays();
    ads.begin(0x48);
    startStableWiFiAP();
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) { request->send_P(200, "text/html", index_html); });
    server.begin();
    restoreStateAfterWake();
}

void loop() {
    static unsigned long wifiCheck = 0;
    if (millis() - wifiCheck > 10000) {
        wifiCheck = millis();
        if (WiFi.getMode() != WIFI_AP) startStableWiFiAP();
    }
    ws.cleanupClients();
    rawL = ads.readADC_SingleEnded(0); rawR = ads.readADC_SingleEnded(1);
    rawP = ads.readADC_SingleEnded(2); rawD = ads.readADC_SingleEnded(3);
    handleIgnitionControl();
    checkPhysicalButtons();
    handleSequence();
    if (manualJogActive && millis() - lastJogTime > 400) {
        stopRelays();
        manualJogActive = false;
        manualDoorOpeningActive = false;
    }
    handleLowPowerMode();

    static unsigned long lastTelemetry = 0;
    if (millis() - lastTelemetry > 300) {
        float psi = (float)(rawP - T_0) * 100.0 / (T_100 - T_0);
        String stepNames[] = {"READY", "SQUATTING", "DOOR OPENING", "DOOR SLOWING", "RAMP DROP", "RAMP STOW", "DOOR CLOSING", "WAITING FOR IGNITION", "IGNITION DELAY 20S", "RAISING", "BAG BALANCING", "SUSPENSION CHECK", "SUSPENSION TOP-UP"};
        String json = "{\"a0\":" + String(rawL) + ",\"a1\":" + String(rawR) + ",\"a2\":" + String(rawP) + ",\"a3\":" + String(rawD) +
                      ",\"psi\":" + String(psi) + ",\"slowOpen\":" + String(D_SLOW_OPEN) + ",\"step\":\"" + stepNames[currentStep] + "\"}";
        if (ws.count() > 0) ws.textAll(json);
        else if (currentStep == IDLE) stopRelays();
        lastTelemetry = millis();
    }
}

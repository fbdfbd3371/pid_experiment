// PracticeV2_Experiment_EN_RelayCalOnly_UISet.ino
// ESP8266 + PID balance with web UI (STA → phone AP "s10").
// Changes vs previous:
//   • Relay turns ON only right before calibration (arming). Never toggled afterwards.
//   • 10 s boot hold before calibration.
//   • Before Start: PID OFF, ESCs at MIN, motors do not spin.
//   • On Start: first Slew to Base, then run PID for 20 s; Wi‑Fi/HTTP paused during run.
//   • Web UI (EN): Kp/Ki/Kd + Invert checkbox, big red "EXPERIMENT STARTED" banner.
//   • New /set endpoint. Inputs auto-apply with debounce; refresh won't overwrite recent edits.
//
// ---------------------------------------------------------------

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Servo.h>

// =============================
// Hardware pins and basics
// =============================
static const int PinRelay = D0;  // Motor power relay (active-LOW: LOW=ON)
static const int PinEsc1  = D2;  // ESC1 signal
static const int PinEsc2  = D1;  // ESC2 signal
static const int PinPot   = A0;  // Potentiometer (arm position)

// ESC pulse widths (microseconds)
static const int PulseMinUs = 1100;
static const int PulseMaxUs = 1500;
static       int BaseUs     = 1250;   // center (used only during experiment)
static       int DeltaMaxUs = 120;
static       int SlewStepUs = 4;

// Pot / filters / center
static       int   PotCenter  = 660;
static const int   PotMinSafe = 300;
static const int   PotMaxSafe = 900;
static       float PotAlpha   = 0.20f;
static       float DAlpha     = 0.20f;
static       bool  InvertDirection = false;

static const int LoopDtMs = 10; // ~100 Hz

// Boot hold delay (ms) before calibration
static const uint32_t BootHoldMs = 10000;

// =============================
// PID
// =============================
static       float KpUsPerCount       = 0.25f;
static       float KiUsPerCountSec    = 0.10f;
static       float KdUsPerCountPerSec = 0.10f;
static       int   IntegralMaxUs      = 80;

static       float IntUs       = 0.0f;
static       float DerivFilt   = 0.0f;
static       int   PotFiltered = 0;
static       uint32_t LastTsMs = 0;

// =============================
// ESC and relay
// =============================
Servo Esc1, Esc2;
static int CmdUs1 = PulseMinUs;
static int CmdUs2 = PulseMinUs;

// =============================
// Web/STA and experiment
// =============================
ESP8266WebServer Web(80);

// Phone AP credentials
static const char* StaSsid = "s10";
static const char* StaPass = "5560832b";

static const uint32_t ExperimentDurationMs = 20000;

static volatile bool ExperimentRunning = false;
static uint32_t ExperimentEndMs = 0;

// Turn Wi-Fi fully off during experiment for minimal jitter
#define DisableWifiDuringExperiment 1

// Serial printing throttle
static const uint32_t PrintEveryMs = 120;
static uint32_t LastPrintMs = 0;

// =============================
// Utils
// =============================
#define LOGF(fmt, ...) do { Serial.printf("[%9lu ms] " fmt, millis(), ##__VA_ARGS__); } while(0)
inline int ClampInt(int v, int lo, int hi){ return (v<lo)?lo:(v>hi)?hi:v; }
inline void RelayOn(){ digitalWrite(PinRelay, LOW); }
inline void RelayOff(){ digitalWrite(PinRelay, HIGH); }
inline void ServiceWeb(){ Web.handleClient(); yield(); }

// Keep MIN output to both ESCs
inline void HoldEscMin(){
  CmdUs1 = PulseMinUs;
  CmdUs2 = PulseMinUs;
  Esc1.writeMicroseconds(CmdUs1);
  Esc2.writeMicroseconds(CmdUs2);
}

// =============================
// Smooth command helpers
// =============================
void SlewBothTo(int target1, int target2){
  target1 = ClampInt(target1, PulseMinUs, PulseMaxUs);
  target2 = ClampInt(target2, PulseMinUs, PulseMaxUs);
  bool done=false;
  while(!done){
    done=true;
    if (CmdUs1!=target1){ int st=ClampInt(target1-CmdUs1,-SlewStepUs,SlewStepUs); CmdUs1+=st; done=false; }
    if (CmdUs2!=target2){ int st=ClampInt(target2-CmdUs2,-SlewStepUs,SlewStepUs); CmdUs2+=st; done=false; }
    Esc1.writeMicroseconds(CmdUs1);
    Esc2.writeMicroseconds(CmdUs2);
    if (!ExperimentRunning) ServiceWeb();
    delay(LoopDtMs);
  }
}

// =============================
// ESC calibration (MAX → MIN) = ARMING
// =============================
void CalibrateBothMaxMin(){
  LOGF("Calibrate (arming): MAX\n");
  Esc1.writeMicroseconds(PulseMaxUs);
  Esc2.writeMicroseconds(PulseMaxUs);
  delay(5000);
  LOGF("Calibrate (arming): MIN\n");
  Esc1.writeMicroseconds(PulseMinUs);
  Esc2.writeMicroseconds(PulseMinUs);
  delay(6000);
  HoldEscMin();
}

// =============================
// State print
// =============================
void PrintStateOnce(){
  uint32_t now=millis();
  if(now-LastPrintMs<PrintEveryMs) return;
  LastPrintMs=now;
  Serial.printf("[%9lu] run=%d adc_f=%d cmd1=%d cmd2=%d Kp=%.3f Ki=%.4f Kd=%.3f Int=%.1f d=%.1f inv=%d\n",
    now, ExperimentRunning?1:0, PotFiltered, CmdUs1, CmdUs2,
    KpUsPerCount, KiUsPerCountSec, KdUsPerCountPerSec, IntUs, DerivFilt, InvertDirection?1:0);
}

// =============================
// PID step (only used during experiment)
// =============================
void ControlStepPID(){
  int raw = analogRead(PinPot);
  if (raw<PotMinSafe || raw>PotMaxSafe){
    HoldEscMin();
    LOGF("WARN: Pot out of range (raw=%d). Safe MIN.\n", raw);
    delay(200);
    return;
  }

  if (PotFiltered==0) PotFiltered=raw;
  PotFiltered = (int)((1.0f-PotAlpha)*PotFiltered + PotAlpha*raw);

  int err = PotCenter - PotFiltered;
  if (InvertDirection) err = -err;

  uint32_t now=millis();
  if (LastTsMs==0) LastTsMs=now;
  float dtSec = (now-LastTsMs)/1000.0f;
  if (dtSec<=0.0f) dtSec = LoopDtMs/1000.0f;
  LastTsMs=now;

  static int lastY=0;
  if (lastY==0) lastY=PotFiltered;
  float dY = (PotFiltered - lastY) / dtSec;
  lastY = PotFiltered;

  DerivFilt = (1.0f-DAlpha)*DerivFilt + DAlpha*dY;

  IntUs += KiUsPerCountSec * err * dtSec;
  if (IntUs> IntegralMaxUs) IntUs =  IntegralMaxUs;
  if (IntUs<-IntegralMaxUs) IntUs = -IntegralMaxUs;

  float uDiff = KpUsPerCount*err + IntUs - KdUsPerCountPerSec*DerivFilt;
  if (uDiff> DeltaMaxUs) uDiff= DeltaMaxUs;
  if (uDiff<-DeltaMaxUs) uDiff=-DeltaMaxUs;

  int u1 = ClampInt(BaseUs + (int)uDiff, PulseMinUs, PulseMaxUs);
  int u2 = ClampInt(BaseUs - (int)uDiff, PulseMinUs, PulseMaxUs);

  int step1 = ClampInt(u1-CmdUs1, -SlewStepUs, SlewStepUs);
  int step2 = ClampInt(u2-CmdUs2, -SlewStepUs, SlewStepUs);
  CmdUs1 += step1;
  CmdUs2 += step2;

  Esc1.writeMicroseconds(CmdUs1);
  Esc2.writeMicroseconds(CmdUs2);
}

// =============================
// Web page (EN) with red banner and live apply
// =============================
static const char* HTML_PAGE PROGMEM = R"HTML(
<!doctype html>
<meta name=viewport content="width=device-width, initial-scale=1">
<title>BalanceRig — Experiment</title>
<style>
  body{font-family:system-ui,Segoe UI,Arial,sans-serif;max-width:720px;margin:24px auto;padding:0 12px}
  fieldset{border:1px solid #ddd;border-radius:12px;padding:12px;margin-bottom:12px}
  label{display:block;margin:8px 0 4px}
  input[type=number]{width:140px;padding:6px}
  button{padding:10px 14px;border-radius:12px;border:1px solid #bbb;cursor:pointer}
  #status{font:13px/1.5 monospace;background:#fafafa;border:1px solid #eee;border-radius:12px;padding:10px;white-space:pre-wrap}
  #banner{font-size:28px;color:#C00020;font-weight:900;text-align:center;margin:14px 0;display:none}
</style>
<h1>BalanceRig — Experiment (20 s)</h1>

<div id="banner">EXPERIMENT STARTED</div>

<fieldset>
  <legend>PID</legend>
  <label>Kp <input id=kp type=number step=0.01></label>
  <label>Ki <input id=ki type=number step=0.001></label>
  <label>Kd <input id=kd type=number step=0.01></label>
  <label><input id=invert type=checkbox> Invert direction</label>
  <button id=btn onclick="startExp()">Start Experiment (20 s)</button>
</fieldset>

<fieldset>
  <legend>Status</legend>
  <div id=status>Ready. Motors are OFF until you press Start.</div>
</fieldset>

<script>
let dirtyUntil = 0;     // time until which refresh must not overwrite fields after a local edit
let debounceT = null;

function debounce(fn, ms){
  return function(...args){
    clearTimeout(debounceT);
    debounceT = setTimeout(()=>fn.apply(this,args), ms);
  }
}
const applyNow = debounce(async function(){
  const params = new URLSearchParams({
    kp: document.getElementById('kp').value,
    ki: document.getElementById('ki').value,
    kd: document.getElementById('kd').value,
    invert: document.getElementById('invert').checked ? '1' : '0'
  });
  dirtyUntil = Date.now() + 800; // prevent refresh from overwriting for a short time
  try{ await fetch('/set?'+params.toString()).then(r=>r.json()); }catch(e){}
}, 220);

['kp','ki','kd','invert'].forEach(id=>{
  const el = document.getElementById(id);
  const ev = (id==='invert') ? 'change' : 'input';
  el.addEventListener(ev, ()=>{
    dirtyUntil = Date.now() + 800;
    applyNow();
  });
});

async function refresh(){
  const s = await fetch('/state').then(r=>r.json()).catch(()=>({ok:false}));
  const st = document.getElementById('status');
  const btn = document.getElementById('btn');
  if(!s.ok){
    st.textContent='No connection to device... (server might be paused during experiment)';
    return;
  }
  const now = Date.now();
  const F = (id,val)=>{
    const el=document.getElementById(id);
    if(!el) return;
    const active = (document.activeElement===el);
    if(active || now < dirtyUntil) return; // do not overwrite while editing or right after
    if(id==='invert'){ el.checked=!!val; } else { el.value=val; }
  };
  F('kp', s.kp); F('ki', s.ki); F('kd', s.kd); F('invert', s.invert);
  st.textContent =
    `WiFi: ${s.wifi}\nRunning: ${s.running}\nRemainMs: ${s.remain_ms}\n`+
    `ADC:${s.adc} Err:${s.err} Cmd1:${s.m1} Cmd2:${s.m2}\n`+
    `Motors: ${s.running? 'ON' : 'OFF (waiting for Start)'}`;
  btn.disabled = s.running ? true : false;
  document.getElementById('banner').style.display = s.running ? 'block' : 'none';
}

async function startExp(){
  document.getElementById('banner').style.display = 'block';
  document.getElementById('status').textContent = 'Experiment starting...';
  document.getElementById('btn').disabled = true;
  // ensure latest edits are applied to device before start
  await applyNow();
  try{ await fetch('/start').then(r=>r.json()); }catch(e){}
}

setInterval(refresh, 500);
refresh();
</script>
)HTML";

// =============================
// HTTP helpers
// =============================
static void SendJsonOk(){ Web.send(200, "application/json", "{\"ok\":true}"); }

// =============================
// HTTP handlers
// =============================
void HandleRoot(){ Web.send_P(200, "text/html", HTML_PAGE); }

void HandleState(){
  int adc = analogRead(PinPot);
  int err = PotCenter - PotFiltered;
  if (InvertDirection) err = -err;

  String wifi = (WiFi.status()==WL_CONNECTED) ? String("STA ")+WiFi.localIP().toString() : String("OFFLINE");
  uint32_t now = millis();
  int32_t remain = ExperimentRunning ? (int32_t)(ExperimentEndMs - now) : 0;
  if (remain < 0) remain = 0;

  String j = "{";
  j += "\"ok\":true,";
  j += "\"wifi\":\""+wifi+"\",";
  j += "\"running\":" + String(ExperimentRunning?1:0) + ",";
  j += "\"remain_ms\":" + String(remain) + ",";
  j += "\"kp\":" + String(KpUsPerCount,3) + ",";
  j += "\"ki\":" + String(KiUsPerCountSec,4) + ",";
  j += "\"kd\":" + String(KdUsPerCountPerSec,3) + ",";
  j += "\"invert\":" + String(InvertDirection?1:0) + ",";
  j += "\"adc\":" + String(adc) + ",";
  j += "\"err\":" + String(err) + ",";
  j += "\"m1\":" + String(CmdUs1) + ",";
  j += "\"m2\":" + String(CmdUs2);
  j += "}";
  Web.send(200, "application/json", j);
}

void HandleSet(){
  if (Web.hasArg("kp")) KpUsPerCount = constrain(Web.arg("kp").toFloat(), 0.0f, 5.0f);
  if (Web.hasArg("ki")) KiUsPerCountSec = constrain(Web.arg("ki").toFloat(), 0.0f, 1.0f);
  if (Web.hasArg("kd")) KdUsPerCountPerSec = constrain(Web.arg("kd").toFloat(), 0.0f, 2.0f);
  if (Web.hasArg("invert")) InvertDirection = (Web.arg("invert").toInt()!=0);
  SendJsonOk();
}

void HandleStart(){
  if (ExperimentRunning) { Web.send(200, "application/json", "{\"ok\":false,\"msg\":\"already_running\"}"); return; }

  // Clean PID state
  IntUs = 0.0f;
  DerivFilt = 0.0f;
  LastTsMs = millis();

  // Move both channels softly to Base before enabling PID loop
  SlewBothTo(BaseUs, BaseUs);

  ExperimentRunning = true;
  ExperimentEndMs = millis() + ExperimentDurationMs;

  Web.send(200, "application/json", "{\"ok\":true,\"msg\":\"started\"}");
  delay(1);
  Web.close();
#if DisableWifiDuringExperiment
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1);
#endif
}

// =============================
// Wi-Fi STA + HTTP init
// =============================
void SetupWiFiSta(){
  WiFi.mode(WIFI_STA);
  WiFi.begin(StaSsid, StaPass);
  LOGF("WiFi STA connecting to SSID=%s ...\n", StaSsid);
  uint32_t t0=millis();
  while (WiFi.status()!=WL_CONNECTED && millis()-t0<15000){
    delay(250); Serial.print(".");
  }
  Serial.println();
  if (WiFi.status()==WL_CONNECTED){
    LOGF("WiFi connected, IP=%s\n", WiFi.localIP().toString().c_str());
  } else {
    LOGF("WiFi connect timeout, continuing offline.\n");
  }
}

void StartHttpServer(){
  Web.on("/", HandleRoot);
  Web.on("/state", HandleState);
  Web.on("/set", HandleSet);
  Web.on("/start", HandleStart);
  Web.begin();
  LOGF("HTTP server started.\n");
}

// =============================
// setup()
// =============================
void setup(){
  Serial.begin(115200);
  delay(50);
  LOGF("Boot\n");

  pinMode(PinRelay, OUTPUT);
  RelayOff(); // relay OFF until calibration begins

  // Prepare ESCs at MIN (safe)
  Esc1.attach(PinEsc1);
  Esc2.attach(PinEsc2);
  HoldEscMin();

  // -------- 10-second boot hold (relay OFF) --------
  LOGF("Boot hold: waiting %lu ms before calibration...\n", (unsigned long)BootHoldMs);
  for (int sec = BootHoldMs/1000; sec > 0; --sec){
    LOGF("Starting in %d s\n", sec);
    delay(1000);
  }
  // -------------------------------------------------

  // Turn relay ON only now (just before calibration) — and NEVER touch it again
  RelayOn();

  // Calibration = arming
  CalibrateBothMaxMin(); // leaves MIN on outputs

  // Bring up Wi-Fi and HTTP
  SetupWiFiSta();
  if (WiFi.status()==WL_CONNECTED) StartHttpServer();
}

// =============================
// loop()
// =============================
void loop(){
  if (!ExperimentRunning){
    // Web active while waiting
    if (WiFi.getMode()==WIFI_STA && WiFi.status()!=WL_CONNECTED){
      static uint32_t retryAt=0;
      if (millis()>retryAt){ retryAt = millis()+5000; WiFi.reconnect(); }
    }
    if (WiFi.status()==WL_CONNECTED) ServiceWeb();

    // Keep ESC at MIN and do NOT run PID
    HoldEscMin();
    PrintStateOnce();
    delay(LoopDtMs);
    return;
  }

  // === Experiment running ===
  ControlStepPID();
  PrintStateOnce();

  if ((int32_t)(millis()-ExperimentEndMs) >= 0){
    // Stop experiment: set outputs to MIN; relay remains ON (unchanged)
    ExperimentRunning=false;
    HoldEscMin();

#if DisableWifiDuringExperiment
    WiFi.forceSleepWake();
    delay(1);
    SetupWiFiSta();
#endif
    if (WiFi.status()==WL_CONNECTED) StartHttpServer();
    LOGF("Experiment finished. Web server resumed. Relay stays ON.\n");
  }
  delay(LoopDtMs);
}

// ---------------------------------------------------------------
// End of file
// ---------------------------------------------------------------

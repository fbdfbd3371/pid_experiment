// PracticeV2_Experiment_EN_RelayCalOnly_UISet_Chart.ino
// ESP8266 + PID balance with web UI (STA → phone AP "s10").
// Additions in this version:
//   • During experiment, sample potentiometer every 50 ms (raw + filtered + error).
//   • After experiment, the page can fetch samples and render a line chart (no external libs).
//   • Data exposed via /data in compact JSON.
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
static       float PotAlpha   = 0.20f; // EMA for measurement
static       float DAlpha     = 0.20f; // EMA for derivative
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
// Sampling buffer (50 ms period, up to 500 samples)
// =============================
static const uint16_t SamplePeriodMs = 50;
static const int      MaxSamples     = 500; // 20 s / 50 ms = 400; reserve some headroom

static uint16_t SampleCount = 0;
static uint16_t Tms   [MaxSamples]; // time since start, ms (fits up to 65s)
static uint16_t AdcRaw[MaxSamples]; // 0..1023
static int16_t  AdcErr[MaxSamples]; // error (center - filtered), may be negative
static uint16_t AdcFilt[MaxSamples];// filtered value for reference
static int16_t  PUs[MaxSamples]; // P term (µs, signed)
static int16_t  IUs[MaxSamples]; // I term (µs, signed)
static int16_t  DUs[MaxSamples]; // D term (µs, signed)

static uint32_t NextSampleAtMs = 0;
static uint32_t ExperimentStartMs = 0;
static volatile bool SamplesReady = false;

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
// Sampling during experiment
// =============================
inline void TrySample(){
  if (!ExperimentRunning) return;
  uint32_t now = millis();
  if (now < NextSampleAtMs) return;
  if (SampleCount >= MaxSamples) return;

  int raw = analogRead(PinPot);
  // PotFiltered already updated in ControlStepPID()
  int err = PotCenter - PotFiltered;
  if (InvertDirection) err = -err;

  uint16_t t = (uint16_t)(now - ExperimentStartMs);
  Tms   [SampleCount] = t;
  AdcRaw[SampleCount] = (uint16_t)ClampInt(raw, 0, 1023);
  AdcFilt[SampleCount]= (uint16_t)ClampInt(PotFiltered, 0, 1023);
  AdcErr[SampleCount] = (int16_t)ClampInt(err, -32768, 32767);

  // PID components for logging (µs)
  float uP = KpUsPerCount * err;
  float uI = IntUs;
  float uD = -KdUsPerCountPerSec * DerivFilt;
  auto clamp16 = [](float v, int lim){ if(v>lim) v=lim; if(v<-lim) v=-lim; return (int16_t)v; };
  const int limUs = 2000;
  PUs[SampleCount] = clamp16(uP, limUs);
  IUs[SampleCount] = clamp16(uI, limUs);
  DUs[SampleCount] = clamp16(uD, limUs);

  SampleCount++;
  NextSampleAtMs += SamplePeriodMs;
}

// =============================
// Web page (EN) with red banner, live apply, and chart
// =============================
static const char* HTML_PAGE PROGMEM = R"HTML(
<!doctype html>
<meta name=viewport content="width=device-width, initial-scale=1">
<title>BalanceRig — Experiment</title>
<style>
  body{font-family:system-ui,Segoe UI,Arial,sans-serif;max-width:760px;margin:24px auto;padding:0 12px}
  fieldset{border:1px solid #ddd;border-radius:12px;padding:12px;margin-bottom:12px}
  label{display:block;margin:8px 0 4px}
  input[type=number]{width:140px;padding:6px}
  button{padding:10px 14px;border-radius:12px;border:1px solid #bbb;cursor:pointer}
  #status{font:13px/1.5 monospace;background:#fafafa;border:1px solid #eee;border-radius:12px;padding:10px;white-space:pre-wrap}
  #banner{font-size:28px;color:#C00020;font-weight:900;text-align:center;margin:14px 0;display:none}
  #chartWrap{border:1px solid #eee;border-radius:12px;padding:10px}
  canvas{width:100%; height:280px;}
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

<fieldset id="chartWrap">
  <legend>Chart (after experiment)</legend>
  <div style="display:flex;gap:8px;align-items:center">
    <button onclick="loadChart()">Load chart</button>
    <button onclick="savePng()">Save PNG</button>
    <small id=meta>—</small>
  </div>
  <div id="chartWrap">
    <div style="font-weight:600;margin:6px 0">Potentiometer (raw & filtered), ADC</div>
    <canvas id="chartPot" width="800" height="220"></canvas>
    <div style="font-weight:600;margin:14px 0 6px">Error, ADC</div>
    <canvas id="chartErr" width="800" height="180"></canvas>
    <div style="font-weight:600;margin:14px 0 6px">PID components, µs (P / I / D)</div>
    <canvas id="chartPid" width="800" height="220"></canvas>
  </div>
</fieldset>

<script>
let dirtyUntil = 0;
let debounceT = null;
let wasRunning = false;

function debounce(fn, ms){
  return function(...args){
    clearTimeout(debounceT);
    debounceT = setTimeout(()=>fn.apply(this,args), ms);
  }
}

function savePng(){
  const c = document.getElementById('chart');
  if(!c){ alert('Chart not found.'); return; }
  const link = document.createElement('a');
  link.href = c.toDataURL('image/png');
  link.download = `experiment_${Date.now()}.png`;
  if (typeof link.download === 'undefined') {
    window.open(link.href, '_blank');
  } else {
    link.click();
  }
}
const applyNow = debounce(async function(){
  const params = new URLSearchParams({
    kp: document.getElementById('kp').value,
    ki: document.getElementById('ki').value,
    kd: document.getElementById('kd').value,
    invert: document.getElementById('invert').checked ? '1' : '0'
  });
  dirtyUntil = Date.now() + 800;
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
    if(active || now < dirtyUntil) return;
    if(id==='invert'){ el.checked=!!val; } else { el.value=val; }
  };
  F('kp', s.kp); F('ki', s.ki); F('kd', s.kd); F('invert', s.invert);
  st.textContent =
    `WiFi: ${s.wifi}\nRunning: ${s.running}\nRemainMs: ${s.remain_ms}\n`+
    `ADC:${s.adc} Err:${s.err} Cmd1:${s.m1} Cmd2:${s.m2}\n`+
    `Motors: ${s.running? 'ON' : 'OFF (waiting for Start)'}`;
  btn.disabled = s.running ? true : false;
  document.getElementById('banner').style.display = s.running ? 'block' : 'none';

  // Auto-load chart once when experiment ends
  if (wasRunning && !s.running) {
    loadChart();
  }
  wasRunning = !!s.running;
}

async function startExp(){
  document.getElementById('banner').style.display = 'block';
  document.getElementById('status').textContent = 'Experiment starting...';
  document.getElementById('btn').disabled = true;
  await applyNow();
  try{ await fetch('/start').then(r=>r.json()); }catch(e){}
}

async function loadChart(){
  const meta = document.getElementById('meta');
  meta.textContent = 'Loading...';
  let d = null;
  try{
    d = await fetch('/data').then(r=>r.json());
  }catch(e){
    meta.textContent = 'No data (run an experiment first).';
    return;
  }
  if(!d || !d.ok || !d.count){
    meta.textContent = 'No data (run an experiment first).';
    return;
  }
  meta.textContent = `Samples: ${d.count}, dt=${d.dt_ms} ms`;
  drawPotChart(d.t, d.adc_raw, d.adc_filt, %POT_CENTER%);
  drawErrChart(d.t, d.err);
  drawPidChart(d.t, d.p, d.i, d.d);
}

function drawPotChart(t, raw, filt, potCenter){
  const c = document.getElementById("chartPot");
  const ctx = c.getContext("2d");
  const W = c.width, H = c.height;
  ctx.clearRect(0,0,W,H); ctx.save();
  ctx.strokeStyle = "#999"; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(40,10); ctx.lineTo(40,H-30); ctx.lineTo(W-10,H-30); ctx.stroke();
  const n = raw.length; const xmin = 0, xmax = t[n-1]||1;
  const ymin = 0, ymax = 1023;
  const x = (i)=> 40 + (W-50)*(t[i]-xmin)/(xmax-xmin||1);
  const y = (v)=> (H-30) - (H-40)*(v-ymin)/(ymax-ymin||1);
  ctx.strokeStyle = "#eee";
  for(let ms=0; ms<=xmax; ms+=1000){ const gx=40 + (W-50)*(ms-xmin)/(xmax-xmin||1); ctx.beginPath(); ctx.moveTo(gx,10); ctx.lineTo(gx,H-30); ctx.stroke(); }
  ctx.fillStyle = "#333"; ctx.fillText("time, ms", W-70, H-12); ctx.fillText("ADC", 8, 20);
  ctx.strokeStyle = "#444"; ctx.lineWidth = 1.2; ctx.beginPath(); for(let i=0;i<n;i++){ const xi=x(i), yi=y(raw[i]); if(i===0) ctx.moveTo(xi,yi); else ctx.lineTo(xi,yi);} ctx.stroke();
  ctx.strokeStyle = "#1a73e8"; ctx.lineWidth = 1.4; ctx.beginPath(); for(let i=0;i<n;i++){ const xi=x(i), yi=y(filt[i]); if(i===0) ctx.moveTo(xi,yi); else ctx.lineTo(xi,yi);} ctx.stroke();
  ctx.strokeStyle = "#e8711a"; ctx.setLineDash([4,4]); ctx.beginPath(); const yc = y(potCenter); ctx.moveTo(40,yc); ctx.lineTo(W-10,yc); ctx.stroke(); ctx.setLineDash([]);
  ctx.restore();
}

function drawErrChart(t, err){
  const c = document.getElementById("chartErr");
  const ctx = c.getContext("2d"); const W=c.width, H=c.height;
  ctx.clearRect(0,0,W,H); ctx.save();
  ctx.strokeStyle="#999"; ctx.lineWidth=1; ctx.beginPath(); ctx.moveTo(40,10); ctx.lineTo(40,H-30); ctx.lineTo(W-10,H-30); ctx.stroke();
  const n = err.length; const xmin=0, xmax=t[n-1]||1;
  let emin=0, emax=0; for(let i=0;i<n;i++){ if(err[i]<emin) emin=err[i]; if(err[i]>emax) emax=err[i]; }
  const eabs=Math.max(Math.abs(emin), Math.abs(emax), 10); const ymin=-eabs, ymax=eabs;
  const x=(i)=> 40 + (W-50)*(t[i]-xmin)/(xmax-xmin||1);
  const y=(v)=> (H-30) - (H-40)*(v-ymin)/(ymax-ymin||1);
  ctx.strokeStyle="#eee"; for(let ms=0; ms<=xmax; ms+=1000){ const gx=40 + (W-50)*(ms-xmin)/(xmax-xmin||1); ctx.beginPath(); ctx.moveTo(gx,10); ctx.lineTo(gx,H-30); ctx.stroke(); }
  ctx.strokeStyle="#bbb"; ctx.setLineDash([4,4]); ctx.beginPath(); const y0=y(0); ctx.moveTo(40,y0); ctx.lineTo(W-10,y0); ctx.stroke(); ctx.setLineDash([]);
  ctx.fillStyle="#333"; ctx.fillText("err (ADC)", 8, 20); ctx.fillText("time, ms", W-70, H-12);
  ctx.beginPath(); ctx.lineWidth=1.4; for(let i=0;i<n;i++){ const xi=x(i), yi=y(err[i]); if(i===0) ctx.moveTo(xi,yi); else ctx.lineTo(xi,yi);} ctx.stroke();
  ctx.restore();
}

function drawPidChart(t, p, i, d){
  const c = document.getElementById("chartPid"); const ctx=c.getContext("2d"); const W=c.width, H=c.height;
  ctx.clearRect(0,0,W,H); ctx.save();
  ctx.strokeStyle="#999"; ctx.lineWidth=1; ctx.beginPath(); ctx.moveTo(40,10); ctx.lineTo(40,H-30); ctx.lineTo(W-10,H-30); ctx.stroke();
  if(!p || !i || !d || !p.length){ ctx.restore(); return; }
  const n = Math.min(p.length, i.length, d.length, t.length);
  const xmin=0, xmax=t[n-1]||1;
  let vmin=0, vmax=0; for(let arr of [p,i,d]) for(let v of arr){ if(v<vmin) vmin=v; if(v>vmax) vmax=v; }
  const vabs=Math.max(Math.abs(vmin), Math.abs(vmax), 10); const ymin=-vabs, ymax=vabs;
  const x=(idx)=> 40 + (W-50)*(t[idx]-xmin)/(xmax-xmin||1);
  const y=(v)=> (H-30) - (H-40)*(v-ymin)/(ymax-ymin||1);
  ctx.strokeStyle="#eee"; for(let ms=0; ms<=xmax; ms+=1000){ const gx=40 + (W-50)*(ms-xmin)/(xmax-xmin||1); ctx.beginPath(); ctx.moveTo(gx,10); ctx.lineTo(gx,H-30); ctx.stroke(); }
  ctx.strokeStyle="#bbb"; ctx.setLineDash([4,4]); ctx.beginPath(); const y0=y(0); ctx.moveTo(40,y0); ctx.lineTo(W-10,y0); ctx.stroke(); ctx.setLineDash([]);
  ctx.fillStyle="#333"; ctx.fillText("µs", 12, 20); ctx.fillText("time, ms", W-70, H-12);
  const draw=(arr)=>{ ctx.beginPath(); ctx.lineWidth=1.4; for(let i=0;i<n;i++){ const xi=x(i), yi=y(arr[i]); if(i===0) ctx.moveTo(xi,yi); else ctx.lineTo(xi,yi);} ctx.stroke(); };
  ctx.strokeStyle = '#d32f2f'; draw(p); // P - red
  ctx.strokeStyle = '#1976d2'; draw(i); // I - blue
  ctx.strokeStyle = '#388e3c'; draw(d); // D - green
  draw(p); draw(i); draw(d);
  ctx.restore();
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
void HandleRoot(){
  // Inject PotCenter value into HTML once (for center line)
  String page = String(HTML_PAGE);
  page.replace("%POT_CENTER%", String(PotCenter));
  Web.send(200, "text/html", page);
}

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

  // Reset PID state
  IntUs = 0.0f;
  DerivFilt = 0.0f;
  LastTsMs = millis();

  // Reset sampling
  SampleCount = 0;
  SamplesReady = false;
  ExperimentStartMs = millis();
  NextSampleAtMs = ExperimentStartMs; // sample immediately at t=0

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

void HandleData(){
  if (!SamplesReady || SampleCount==0){
    Web.send(200, "application/json", "{\"ok\":false,\"msg\":\"no_data\"}");
    return;
  }
  String j = "{";
  j += "\"ok\":true,";
  j += "\"count\":" + String(SampleCount) + ",";
  j += "\"dt_ms\":" + String(SamplePeriodMs) + ",";

  // Times
  j += "\"t\":[";
  for (uint16_t i=0;i<SampleCount;i++){ j += String(Tms[i]); if (i+1<SampleCount) j += ","; }
  j += "],";

  // Raw
  j += "\"adc_raw\":[";
  for (uint16_t i=0;i<SampleCount;i++){ j += String(AdcRaw[i]); if (i+1<SampleCount) j += ","; }
  j += "],";

  // Filtered
  j += "\"adc_filt\":[";
  for (uint16_t i=0;i<SampleCount;i++){ j += String(AdcFilt[i]); if (i+1<SampleCount) j += ","; }
  j += "],";

  // Error
  j += "\"err\":[";
  for (uint16_t i=0;i<SampleCount;i++){ j += String(AdcErr[i]); if (i+1<SampleCount) j += ","; }
  j += "]";

  // P term (µs)
  j += ",\"p\":[";
  for (uint16_t i=0;i<SampleCount;i++){ j += String(PUs[i]); if (i+1<SampleCount) j += ","; }
  j += "]";

  // I term (µs)
  j += ",\"i\":[";
  for (uint16_t i=0;i<SampleCount;i++){ j += String(IUs[i]); if (i+1<SampleCount) j += ","; }
  j += "]";

  // D term (µs)
  j += ",\"d\":[";
  for (uint16_t i=0;i<SampleCount;i++){ j += String(DUs[i]); if (i+1<SampleCount) j += ","; }
  j += "]";


  j += "}";
  Web.send(200, "application/json", j);
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
  Web.on("/data", HandleData);
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
  TrySample(); // sample every 50 ms
  PrintStateOnce();

  if ((int32_t)(millis()-ExperimentEndMs) >= 0){
    // Stop experiment: set outputs to MIN; relay remains ON (unchanged)
    ExperimentRunning=false;
    HoldEscMin();
    SamplesReady = true; // allow /data to be served

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
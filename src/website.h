#pragma once

#ifndef PROGMEM
#define PROGMEM
#endif

const char pageIndex[] PROGMEM = R"====(
<!doctype html>
<html>
<head>
<meta charset='utf-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>LoRa APRS Weather Station PRO</title>

<style>
body{
  background:#0f172a;
  color:#e5e7eb;
  font-family:Arial, sans-serif;
  margin:0;
  padding:0;
}
main{
  max-width:900px;
  margin:auto;
  padding:20px;
}
.card{
  background:#111827;
  border:1px solid #334155;
  border-radius:10px;
  padding:16px;
  margin:14px 0;
}
h2,h3{
  margin-top:0;
}
input,select{
  width:100%;
  padding:6px;
  margin:4px 0 10px 0;
  background:#1f2937;
  color:#e5e7eb;
  border:1px solid #334155;
  border-radius:6px;
}
button{
  background:#2563eb;
  border:none;
  padding:10px 16px;
  border-radius:6px;
  color:white;
  cursor:pointer;
}
button:hover{
  background:#1d4ed8;
}
.row{
  display:grid;
  grid-template-columns:1fr 1fr;
  gap:12px;
}
.switch{
  display:flex;
  align-items:center;
  gap:10px;
}
.footer{
  font-size:12px;
  opacity:0.6;
  margin-top:20px;
}
</style>
</head>

<body>
<main>

<h2>📡 LoRa APRS Weather Station PRO</h2>

<!-- DASHBOARD -->
<div class='card'>
<h3>Live Data</h3>
<div>Temperatura: <b id='t'>--</b></div>
<div>Umidità: <b id='h'>--</b></div>
<div>Pressione: <b id='p'>--</b></div>
<div>Batteria: <b id='v'>--</b></div>
<div>IAQ: <b id='iaq'>--</b></div>
<div>CO2 eq: <b id='co2'>--</b></div>
<div>VOC: <b id='voc'>--</b></div>
<div>Gas: <b id='gas'>--</b></div>
<div>RSSI LoRa: <b id='rssi'>--</b></div>
<div>Uptime: <b id='uptime'>--</b></div>
</div>

<!-- CONFIG -->
<div class='card'>
<h3>APRS Configuration</h3>
<div class='row'>
<div>
Callsign
<input id='callsign'>
</div>
<div>
SSID
<input id='ssid' type='number'>
</div>
<div>
Latitude
<input id='lat' type='number' step='0.000001'>
</div>
<div>
Longitude
<input id='lon' type='number' step='0.000001'>
</div>
<div>
APRS Interval (ms)
<input id='interval' type='number'>
</div>
<div>
Variation Threshold
<input id='variation' type='number' step='0.01'>
</div>
</div>

<div class='switch'>
<label>Beacon Enabled</label>
<input id='beacon' type='checkbox'>
</div>

<hr>

<h3>WiFi</h3>
<div class='row'>
<div>
SSID
<input id='wifiSsid'>
</div>
<div>
Password
<input id='wifiPass' type='password'>
</div>
<div>
WiFi Timeout (ms)
<input id='wifiTimeout' type='number'>
</div>
</div>

<div class='switch'>
<label>WiFi Always ON</label>
<input id='wifiAlwaysOn' type='checkbox'>
</div>

<br>
<button onclick='saveConfig()'>💾 Save & Reboot</button>
<button onclick='reboot()'>🔄 Reboot</button>
<a href='/update'><button>⬆ OTA Update</button></a>

<div id='status'></div>
</div>

<div class='footer'>
Firmware PRO – LoRa APRS Station
</div>

</main>

<script>

// ===== AUTO REFRESH DASHBOARD =====
function loadData(){
  fetch('/api')
  .then(r=>r.json())
  .then(j=>{
    t.innerHTML=j.t.toFixed(1)+" °C";
    h.innerHTML=j.h.toFixed(1)+" %";
    p.innerHTML=j.p.toFixed(1)+" hPa";
    v.innerHTML=j.v.toFixed(2)+" V";
    iaq.innerHTML=j.iaq.toFixed(1);
    co2.innerHTML=j.co2.toFixed(0)+" ppm";
    voc.innerHTML=j.voc.toFixed(2);
    gas.innerHTML=j.gas.toFixed(0);
    rssi.innerHTML=j.rssi+" dBm";
    uptime.innerHTML=j.uptime+" s";
  });
}
setInterval(loadData,3000);
loadData();

// ===== LOAD CONFIG =====
function loadConfig(){
  fetch('/config')
  .then(r=>r.json())
  .then(c=>{
    callsign.value=c.callsign;
    ssid.value=c.ssid;
    lat.value=c.latitude;
    lon.value=c.longitude;
    interval.value=c.aprsIntervalMs;
    variation.value=c.variationThreshold;
    beacon.checked=c.beaconEnabled;
    wifiSsid.value=c.wifiSsid;
    wifiPass.value=c.wifiPass;
    wifiTimeout.value=c.wifiTimeoutMs;
    wifiAlwaysOn.checked=c.wifiAlwaysOn;
  });
}
loadConfig();

// ===== SAVE CONFIG =====
function saveConfig(){

  const data={
    callsign:callsign.value,
    ssid:parseInt(ssid.value),
    latitude:parseFloat(lat.value),
    longitude:parseFloat(lon.value),
    aprsIntervalMs:parseInt(interval.value),
    variationThreshold:parseFloat(variation.value),
    beaconEnabled:beacon.checked,
    wifiSsid:wifiSsid.value,
    wifiPass:wifiPass.value,
    wifiTimeoutMs:parseInt(wifiTimeout.value),
    wifiAlwaysOn:wifiAlwaysOn.checked
  };

  fetch('/config',{
    method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify(data)
  }).then(()=>{
    status.innerHTML="Saved. Rebooting...";
    setTimeout(()=>{location.reload();},4000);
  });
}

// ===== REBOOT =====
function reboot(){
  fetch('/reboot',{method:'POST'});
}

</script>

</body>
</html>
)====";

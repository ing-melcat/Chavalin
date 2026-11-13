#pragma once
#include <pgmspace.h>

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>SEGUIDOR PRO v1 - CONTROL MONITOR</title>
  <style>
    :root {
      --bg: #080d1a;
      --card: #11192c;
      --border: #1e2d4a;
      --cyan: #00e5ff;
      --orange: #ff9100;
      --purple: #b388ff;
      --green: #00e676;
      --red: #ff3d57;
      --text: #e2e8f0;
      --muted: #7c8ba1;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: 'Segoe UI', system-ui, sans-serif; }
    body { background: var(--bg); color: var(--text); padding: 12px; }
    .container { max-width: 1280px; margin: 0 auto; display: flex; flex-direction: column; gap: 12px; }

    /* Header */
    .header {
      display: flex; flex-wrap: wrap; justify-content: space-between; align-items: center;
      background: var(--card); border: 1px solid var(--border); border-radius: 10px; padding: 12px 16px; gap: 10px;
    }
    .brand { display: flex; flex-direction: column; }
    .brand h1 { font-size: 1.15rem; font-weight: 800; letter-spacing: 0.8px; color: #fff; }
    .brand span { font-size: 0.78rem; color: var(--orange); display: flex; align-items: center; gap: 6px; margin-top: 2px; }
    .dot { width: 8px; height: 8px; border-radius: 50%; background: var(--red); display: inline-block; }
    .dot.online { background: var(--green); box-shadow: 0 0 8px var(--green); }

    .actions { display: flex; flex-wrap: wrap; gap: 8px; }
    button {
      background: #1a2640; color: var(--text); border: 1px solid var(--border);
      padding: 7px 14px; border-radius: 6px; font-size: 0.82rem; font-weight: 600;
      cursor: pointer; transition: all 0.15s;
    }
    button:hover { border-color: var(--cyan); color: #fff; }
    button.primary { background: #006680; border-color: var(--cyan); color: #fff; }
    button.danger { background: #5a1220; border-color: var(--red); color: #fff; }
    button.active { background: #006e3a; border-color: var(--green); color: #fff; }

    /* Telemetry & PID grid */
    .params-grid {
      display: grid; grid-template-columns: repeat(auto-fit, minmax(130px, 1fr)); gap: 8px;
    }
    .param-box {
      background: var(--card); border: 1px solid var(--border); border-radius: 8px; padding: 8px 12px;
      display: flex; flex-direction: column; gap: 4px;
    }
    .param-box label { font-size: 0.7rem; color: var(--muted); text-transform: uppercase; letter-spacing: 0.5px; }
    .param-box .val { font-size: 1.1rem; font-weight: 700; color: #fff; font-variant-numeric: tabular-nums; }
    .param-box input {
      background: #090e1a; border: 1px solid var(--border); color: var(--cyan);
      padding: 4px 8px; border-radius: 4px; font-size: 0.95rem; font-weight: 700; width: 100%;
    }

    /* Main Visuals Row */
    .main-row {
      display: grid; grid-template-columns: 220px 1fr 220px; gap: 12px; align-items: stretch;
    }
    @media (max-width: 900px) {
      .main-row { grid-template-columns: 1fr; }
    }
    .card {
      background: var(--card); border: 1px solid var(--border); border-radius: 10px; padding: 12px;
      display: flex; flex-direction: column; align-items: center; justify-content: center;
    }
    .card-title { font-size: 0.75rem; color: var(--muted); text-transform: uppercase; letter-spacing: 0.8px; margin-bottom: 8px; font-weight: 700; }

    /* Gauges */
    .gauge-wrap { position: relative; width: 170px; height: 120px; display: flex; align-items: center; justify-content: center; }
    .gauge-val { position: absolute; bottom: 18px; font-size: 1.6rem; font-weight: 800; font-variant-numeric: tabular-nums; }
    .gauge-sub { font-size: 0.72rem; color: var(--muted); margin-top: 2px; }

    /* 16-Sensor Bar Chart */
    .sensors-wrap { width: 100%; height: 196px; display: flex; flex-direction: column; justify-content: flex-end; }
    .bars-container {
      display: grid; grid-template-columns: repeat(16, 1fr); gap: 4px; height: 145px; align-items: flex-end;
      border-bottom: 1px solid var(--border); padding: 0 4px;
    }
    .bar-col { display: flex; flex-direction: column; align-items: center; height: 100%; justify-content: flex-end; }
    .bar-fill {
      width: 100%; background: var(--cyan); border-radius: 3px 3px 0 0; min-height: 4px;
      transition: height 0.06s linear; box-shadow: 0 0 8px rgba(0, 229, 255, 0.35);
    }
    .bar-fill.idle { background: #145369; box-shadow: none; }
    .bar-labels {
      display: grid; grid-template-columns: repeat(16, 1fr); gap: 4px; padding: 4px 4px 0 4px;
      text-align: center; font-size: 0.65rem; color: var(--text); font-weight: 600;
    }
    .bar-mvs {
      display: grid; grid-template-columns: repeat(16, 1fr); gap: 4px; padding: 2px 4px 0 4px;
      text-align: center; font-size: 0.56rem; color: var(--muted); font-variant-numeric: tabular-nums;
    }

    /* Time-series chart */
    .chart-card { padding: 12px 16px; align-items: stretch; }
    .chart-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 6px; }
    .legend { display: flex; gap: 14px; font-size: 0.75rem; }
    .legend span { display: flex; align-items: center; gap: 5px; }
    .legend i { width: 10px; height: 3px; display: inline-block; }
    canvas#historyChart { width: 100%; height: 190px; background: #090e1a; border-radius: 6px; border: 1px solid var(--border); }
  </style>
</head>
<body>
<div class="container">
  <!-- Header -->
  <div class="header">
    <div class="brand">
      <h1>SEGUIDOR PRO v1 &bull; CONTROL MONITOR</h1>
      <span><i id="connDot" class="dot"></i> <span id="connText">Conectando al ESP32-C3...</span></span>
    </div>
    <div class="actions">
      <button onclick="sendCmd('calibrate')">Calibrar Sensores</button>
      <button id="btnRun" class="primary" onclick="toggleRun()">Arrancar Seguidor</button>
      <button onclick="sendCmd('test')">Test Motores</button>
      <button class="danger" onclick="sendCmd('stop')">PARO (E-STOP)</button>
      <button onclick="clearChart()">Limpiar Gráfica</button>
    </div>
  </div>

  <!-- PID & Telemetry Readouts -->
  <div class="params-grid">
    <div class="param-box"><label>Estado</label><div class="val" id="valMode">MANUAL</div></div>
    <div class="param-box"><label>Error Línea</label><div class="val" id="valError">0</div></div>
    <div class="param-box"><label>Base Speed</label><input type="number" id="inSpeed" value="120" min="0" max="255"></div>
    <div class="param-box"><label>Kp</label><input type="number" id="inKp" value="0.35" step="0.05"></div>
    <div class="param-box"><label>Ki</label><input type="number" id="inKi" value="0.00" step="0.01"></div>
    <div class="param-box"><label>Kd</label><input type="number" id="inKd" value="0.15" step="0.05"></div>
    <div class="param-box" style="justify-content:center;">
      <button class="primary" style="height:100%;" onclick="applyPID()">Aplicar PID</button>
    </div>
  </div>

  <!-- Gauges + 16 Sensors -->
  <div class="main-row">
    <div class="card">
      <div class="card-title">MOTOR IZQUIERDO</div>
      <div class="gauge-wrap">
        <svg viewBox="0 0 160 110" width="170" height="115">
          <path d="M 20 95 A 60 60 0 1 1 140 95" fill="none" stroke="#1e2d4a" stroke-width="12" stroke-linecap="round"/>
          <path id="arcL" d="M 20 95 A 60 60 0 1 1 140 95" fill="none" stroke="#00e5ff" stroke-width="12" stroke-linecap="round" stroke-dasharray="251" stroke-dashoffset="251"/>
        </svg>
        <div class="gauge-val" id="txtMotL" style="color:var(--cyan)">0</div>
      </div>
      <div class="gauge-sub">PWM: -255 a +255</div>
    </div>

    <div class="card">
      <div class="card-title">BARRA DE LÍNEA NORMALIZADA 0-1000 (IZQ I0 &larr; &rarr; I15 DER)</div>
      <div class="sensors-wrap">
        <div class="bars-container" id="barsContainer"></div>
        <div class="bar-labels" id="barLabels"></div>
        <div class="bar-mvs" id="barMvs"></div>
      </div>
    </div>

    <div class="card">
      <div class="card-title">MOTOR DERECHO</div>
      <div class="gauge-wrap">
        <svg viewBox="0 0 160 110" width="170" height="115">
          <path d="M 20 95 A 60 60 0 1 1 140 95" fill="none" stroke="#1e2d4a" stroke-width="12" stroke-linecap="round"/>
          <path id="arcR" d="M 20 95 A 60 60 0 1 1 140 95" fill="none" stroke="#ff9100" stroke-width="12" stroke-linecap="round" stroke-dasharray="251" stroke-dashoffset="251"/>
        </svg>
        <div class="gauge-val" id="txtMotR" style="color:var(--orange)">0</div>
      </div>
      <div class="gauge-sub">PWM: -255 a +255</div>
    </div>
  </div>

  <!-- Time-series Chart -->
  <div class="card chart-card">
    <div class="chart-header">
      <div class="card-title" style="margin:0;">Comandos calculados de motor (&plusmn;255) y Error</div>
      <div class="legend">
        <span><i style="background:var(--cyan)"></i> Motor Izq</span>
        <span><i style="background:var(--orange)"></i> Motor Der</span>
        <span><i style="background:var(--purple)"></i> Error/3</span>
      </div>
    </div>
    <canvas id="historyChart"></canvas>
  </div>
</div>

<script>
  const barsContainer = document.getElementById('barsContainer');
  const barLabels = document.getElementById('barLabels');
  const barMvs = document.getElementById('barMvs');
  const barEls = [];
  const mvEls = [];
  for (let i = 0; i < 16; i++) {
    const col = document.createElement('div');
    col.className = 'bar-col';
    const fill = document.createElement('div');
    fill.className = 'bar-fill idle';
    fill.style.height = '5%';
    col.appendChild(fill);
    barsContainer.appendChild(col);
    barEls.push(fill);

    const lbl = document.createElement('div');
    lbl.textContent = 'I' + i;
    barLabels.appendChild(lbl);

    const mv = document.createElement('div');
    mv.textContent = '----';
    barMvs.appendChild(mv);
    mvEls.push(mv);
  }

  let running = false;
  const maxPoints = 120;
  const histL = new Array(maxPoints).fill(0);
  const histR = new Array(maxPoints).fill(0);
  const histE = new Array(maxPoints).fill(0);

  function updateGauge(arcId, txtId, val) {
    document.getElementById(txtId).textContent = val;
    const pct = Math.min(1, Math.abs(val) / 255);
    const offset = 251 - (pct * 251);
    document.getElementById(arcId).style.strokeDashoffset = offset;
  }

  function drawChart() {
    const canvas = document.getElementById('historyChart');
    const ctx = canvas.getContext('2d');
    const w = canvas.width = canvas.clientWidth;
    const h = canvas.height = canvas.clientHeight;
    ctx.clearRect(0, 0, w, h);

    // Center zero line
    ctx.strokeStyle = '#1e2d4a';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(0, h / 2);
    ctx.lineTo(w, h / 2);
    ctx.stroke();

    function plotLine(data, color) {
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      ctx.beginPath();
      for (let i = 0; i < data.length; i++) {
        const x = (i / (maxPoints - 1)) * w;
        const y = (h / 2) - ((data[i] / 260) * (h / 2 - 10));
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
    }

    plotLine(histE, '#b388ff');
    plotLine(histL, '#00e5ff');
    plotLine(histR, '#ff9100');
  }

  function clearChart() {
    histL.fill(0); histR.fill(0); histE.fill(0);
    drawChart();
  }

  async function pollData() {
    const ctrl = new AbortController();
    const timer = setTimeout(() => ctrl.abort(), 600);
    try {
      const res = await fetch('/api/data', { signal: ctrl.signal, cache: 'no-store' });
      clearTimeout(timer);
      if (!res.ok) throw new Error('HTTP ' + res.status);
      const d = await res.json();

      document.getElementById('connDot').className = 'dot online';
      document.getElementById('connText').textContent = 'Conectado a ESP32-C3 (Wi-Fi Telemetría Activa)';

      running = d.run;
      const btnRun = document.getElementById('btnRun');
      btnRun.textContent = running ? 'Detener Seguidor' : 'Arrancar Seguidor';
      btnRun.className = running ? 'active' : 'primary';
      document.getElementById('valMode').textContent = running ? 'AUTO (PID)' : 'MANUAL';
      document.getElementById('valError').textContent = d.err;

      updateGauge('arcL', 'txtMotL', d.mL);
      updateGauge('arcR', 'txtMotR', d.mR);

      for (let i = 0; i < 16; i++) {
        const v = d.s[i]; // 0 - 1000
        const pct = Math.max(4, Math.min(100, v / 10));
        barEls[i].style.height = pct + '%';
        barEls[i].className = v > 60 ? 'bar-fill' : 'bar-fill idle';
        if (d.mv) mvEls[i].textContent = d.mv[i];
      }

      histL.push(d.mL); histL.shift();
      histR.push(d.mR); histR.shift();
      histE.push(Math.max(-255, Math.min(255, d.err / 3))); histE.shift();
      drawChart();
    } catch (e) {
      clearTimeout(timer);
      document.getElementById('connDot').className = 'dot';
      document.getElementById('connText').textContent = 'Sin conexión con el robot...';
    }
    setTimeout(pollData, 80);
  }

  async function sendCmd(cmd) {
    await fetch('/api/cmd?c=' + cmd);
  }

  function toggleRun() {
    sendCmd(running ? 'stop' : 'start');
  }

  async function applyPID() {
    const spd = document.getElementById('inSpeed').value;
    const kp  = document.getElementById('inKp').value;
    const ki  = document.getElementById('inKi').value;
    const kd  = document.getElementById('inKd').value;
    await fetch(`/api/pid?spd=${spd}&kp=${kp}&ki=${ki}&kd=${kd}`);
  }

  pollData();
</script>
</body>
</html>
)rawliteral";

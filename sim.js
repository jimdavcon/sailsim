<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Sailboat: Debugger Fixed Layout</title>
    <style>
        /* Force the body to be exactly the size of the window */
        html, body { margin: 0; padding: 0; width: 100%; height: 100%; overflow: hidden; background: #002b36; font-family: 'Courier New', monospace; }

        /* Use Grid to define the main areas */
        .app-container {
            display: grid;
            grid-template-columns: 1fr 200px; /* Simulation/Chart | Sidebar */
            grid-template-rows: 1fr 150px;    /* Simulation | Stripchart */
            width: 100vw;
            height: 100vh;
        }

        #world-container {
            grid-column: 1;
            grid-row: 1;
            background: #002b36;
            position: relative;
        }

        #chart-container {
            grid-column: 1;
            grid-row: 2;
            background: #001e26;
            border-top: 2px solid #586e75;
            position: relative;
        }

        #ui-sidebar {
            grid-column: 2;
            grid-row: 1 / span 2; /* Sidebar takes full height */
            background: #073642;
            padding: 15px;
            color: #859900;
            border-left: 1px solid #586e75;
            box-sizing: border-box;
        }

        canvas { width: 100%; height: 100%; display: block; }

        /* UI Styling */
        .section { margin-bottom: 20px; border-bottom: 1px solid #586e75; padding-bottom: 10px; }
        .label { font-size: 10px; color: #93a1a1; text-transform: uppercase; margin-bottom: 5px; }
        input { width: 100%; accent-color: #859900; cursor: pointer; }
        #telemetry { color: #268bd2; font-size: 11px; white-space: pre-wrap; line-height: 1.4; }
        h3 { font-size: 13px; margin: 0 0 15px 0; color: #b58900; }
        .chart-legend { position: absolute; top: 5px; right: 10px; font-size: 10px; color: #93a1a1; z-index: 20; }
        
        /* Bar Gauges */
        .wing-row { margin: 10px 0; }
        .bar-bg { width: 100%; height: 8px; background: #002b36; position: relative; border-radius: 4px; border: 1px solid #586e75; }
        .bar-fill { height: 100%; background: #2aa198; width: 0%; position: absolute; left: 50%; }
        .center-line { position: absolute; left: 50%; top: -2px; height: 12px; width: 1px; background: #93a1a1; }
    </style>
</head>
<body>

<div class="app-container">
    <div id="world-container">
        <canvas id="simCanvas"></canvas>
    </div>

    <div id="chart-container">
        <div class="chart-legend">
            <span style="color:#b58900">■ CTE (ft)</span> | 
            <span style="color:#2aa198">■ BEARING (rad)</span>
        </div>
        <canvas id="chartCanvas"></canvas>
    </div>

    <div id="ui-sidebar">
        <h3>WING AoA</h3>
        <div class="section">
            <div class="label">Forward Wing</div>
            <div class="wing-row"><div class="bar-bg"><div id="bar0" class="bar-fill"></div><div class="center-line"></div></div></div>
            <div class="label">Aft Wing</div>
            <div class="wing-row"><div class="bar-bg"><div id="bar1" class="bar-fill"></div><div class="center-line"></div></div></div>
        </div>
        <div class="section">
            <div class="label">Wind</div>
            <input type="range" id="windDir" min="0" max="360" value="45"> <span id="wdVal">45</span>°
            <input type="range" id="windSpd" min="0" max="40" value="0"> <span id="wsVal">0</span> kts
        </div>
        <div id="telemetry">Status: Active</div>
    </div>
</div>

<script>
const canvas = document.getElementById('simCanvas');
const ctx = canvas.getContext('2d');
const chartCanvas = document.getElementById('chartCanvas');
const chartCtx = chartCanvas.getContext('2d');

let WORLD_SIZE_FT = 2200;
let PX_PER_FT;
const KNOTS_TO_FTS = 1.68781;
const STALL_LIMIT = 0.25;

let wind = { angle: 45 * Math.PI/180, speed: 0 };
let waypoints = [];
let currentWPIndex = 0;
let tackState = 0; 
let history = [];

const boat = { x: 0, y: 0, theta: 0, vx: 0, vy: 0, omega: 0, sails: [{ x_off: 3.5, alpha: 0, angle: 0 }, { x_off: -3.5, alpha: 0, angle: 0 }] };

function norm(a) {
    while (a > Math.PI) a -= 2 * Math.PI;
    while (a < -Math.PI) a += 2 * Math.PI;
    return a;
}

function resize() {
    // Explicitly set canvas dimensions to their pixel-rendered size
    canvas.width = canvas.clientWidth;
    canvas.height = canvas.clientHeight;
    chartCanvas.width = chartCanvas.clientWidth;
    chartCanvas.height = 150;
    PX_PER_FT = Math.min(canvas.width, canvas.height) / WORLD_SIZE_FT;
    initSquareMission();
}

function initSquareMission() {
    const d = 500 * PX_PER_FT; const r = 100 * PX_PER_FT; 
    waypoints = [
        { x: d, y: -d, r: r, id: "WP1" }, { x: d, y: d, r: r, id: "WP2" },
        { x: -d, y: d, r: r, id: "WP3" }, { x: -d, y: -d, r: r, id: "WP4" },
        { x: 0, y: 0, r: r, id: "ORIGIN" }
    ];
}

document.getElementById('windDir').oninput = e => { 
    wind.angle = e.target.value * Math.PI/180; 
    document.getElementById('wdVal').innerText = e.target.value; 
};
document.getElementById('windSpd').oninput = e => { 
    wind.speed = e.target.value * KNOTS_TO_FTS; 
    document.getElementById('wsVal').innerText = e.target.value; 
};

function update(dt) {
    const pA = (currentWPIndex === 0) ? waypoints[waypoints.length - 1] : waypoints[currentWPIndex - 1];
    const pB = waypoints[currentWPIndex];
    
    const distToTarget = Math.hypot(pB.x - boat.x, pB.y - boat.y);
    if (distToTarget <= pB.r) { 
        currentWPIndex = (currentWPIndex + 1) % waypoints.length; 
        boat.omega *= 0.1; 
        tackState = 0; 
    }

    const dx = pB.x - pA.x, dy = pB.y - pA.y;
    const pathLen = Math.hypot(dx, dy);
    const uX = dx / pathLen, uY = dy / pathLen;
    const atd = (boat.x - pA.x) * uX + (boat.y - pA.y) * uY;
    const cte = (boat.x - pA.x) * (-uY) + (boat.y - pA.y) * uX;

    if (Math.abs(cte) > pB.r) tackState = (cte > 0) ? -1 : 1; 
    else tackState = 0;

    let tx = pA.x + (atd + 400 * PX_PER_FT) * uX + (tackState * pB.r) * (-uY);
    let ty = pA.y + (atd + 400 * PX_PER_FT) * uY + (tackState * pB.r) * uX;

    const targetBearing = Math.atan2(ty - boat.y, tx - boat.x);
    const hErr = norm(targetBearing - boat.theta);
    
    history.push({ cte: cte / PX_PER_FT, bearing: targetBearing });
    if (history.length > chartCanvas.width) history.shift();

    const normErr = Math.min(1, Math.abs(hErr) / (Math.PI / 2)); 
    const steerGain = 2.0 + (3.0 * normErr); 
    const speedScalar = 1.0 - (1.0 * normErr); 
    const baseSpeed = wind.speed * 0.4; 

    boat.omega += (hErr * steerGain - boat.omega * 5.0) * dt;
    boat.theta = norm(boat.theta + boat.omega * dt); 
    boat.vx = Math.cos(boat.theta) * baseSpeed * speedScalar;
    boat.vy = Math.sin(boat.theta) * baseSpeed * speedScalar;
    boat.x += boat.vx; boat.y += boat.vy;

    boat.sails.forEach((s, i) => {
        let relWind = norm((wind.angle + Math.PI) - boat.theta); 
        s.alpha = Math.max(-STALL_LIMIT, Math.min(STALL_LIMIT, (Math.sign(Math.sin(relWind)) * 0.21 * (1 - normErr)) + (hErr * -0.6 * normErr)));
        s.angle = (wind.angle + Math.PI) + s.alpha;
        const pct = (s.alpha / STALL_LIMIT) * 50;
        const bar = document.getElementById(`bar${i}`);
        bar.style.width = Math.abs(pct) + "%"; bar.style.left = pct < 0 ? (50 + pct) + "%" : "50%";
        bar.style.background = Math.abs(s.alpha) >= STALL_LIMIT * 0.95 ? "#dc322f" : "#2aa198";
    });
}

function draw() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.save(); ctx.translate(canvas.width / 2, canvas.height / 2);

    waypoints.forEach((wp, i) => {
        const prev = (i === 0) ? waypoints[waypoints.length - 1] : waypoints[i-1];
        const dx = wp.x - prev.x, dy = wp.y - prev.y, ang = Math.atan2(dy, dx);
        ctx.strokeStyle = (i === currentWPIndex) ? "rgba(133, 153, 0, 0.4)" : "rgba(88, 110, 117, 0.1)";
        ctx.beginPath();
        ctx.moveTo(prev.x + wp.r * Math.cos(ang + Math.PI/2), prev.y + wp.r * Math.sin(ang + Math.PI/2));
        ctx.lineTo(wp.x + wp.r * Math.cos(ang + Math.PI/2), wp.y + wp.r * Math.sin(ang + Math.PI/2));
        ctx.moveTo(prev.x + wp.r * Math.cos(ang - Math.PI/2), prev.y + wp.r * Math.sin(ang - Math.PI/2));
        ctx.lineTo(wp.x + wp.r * Math.cos(ang - Math.PI/2), wp.y + wp.r * Math.sin(ang - Math.PI/2));
        ctx.stroke();

        ctx.strokeStyle = (i === currentWPIndex) ? "#b58900" : "rgba(88, 110, 117, 0.2)";
        ctx.beginPath(); ctx.arc(wp.x, wp.y, wp.r, 0, Math.PI*2); ctx.stroke();

        ctx.save(); ctx.translate(wp.x, wp.y); ctx.rotate(wind.angle);
        ctx.strokeStyle = "#268bd2"; ctx.beginPath(); ctx.moveTo(-20, 0); ctx.lineTo(20, 0); ctx.lineTo(12, -5); ctx.moveTo(20, 0); ctx.lineTo(12, 5); ctx.stroke();
        ctx.restore();
    });

    ctx.save(); ctx.translate(boat.x, boat.y); ctx.rotate(boat.theta);
    ctx.strokeStyle = "#eee8d5"; ctx.strokeRect(-12, -6, 24, 12);
    ctx.fillStyle = "#dc322f"; ctx.beginPath(); ctx.arc(12, 0, 2.5, 0, Math.PI*2); ctx.fill();
    boat.sails.forEach(s => {
        ctx.save(); ctx.translate(s.x_off, 0); ctx.rotate(s.angle - boat.theta);
        ctx.fillStyle = "rgba(42, 161, 152, 0.8)"; ctx.fillRect(-10, -2, 20, 4);
        ctx.fillStyle = "#dc322f"; ctx.beginPath(); ctx.arc(10, 0, 2, 0, Math.PI*2); ctx.fill();
        ctx.restore();
    });
    ctx.restore(); ctx.restore();

    // STRIPCHART DRAWING
    chartCtx.fillStyle = "#001e26";
    chartCtx.fillRect(0, 0, chartCanvas.width, chartCanvas.height);
    
    // Baseline
    chartCtx.strokeStyle = "#586e75"; chartCtx.lineWidth = 1;
    chartCtx.beginPath(); chartCtx.moveTo(0, 75); chartCtx.lineTo(chartCanvas.width, 75); chartCtx.stroke();
    
    // Plot CTE (Yellow) - Scale: 1px = 2ft
    chartCtx.strokeStyle = "#b58900"; chartCtx.lineWidth = 2;
    chartCtx.beginPath();
    history.forEach((pt, i) => {
        let y = 75 - (pt.cte * 0.5); 
        if (i === 0) chartCtx.moveTo(i, y); else chartCtx.lineTo(i, y);
    });
    chartCtx.stroke();

    // Plot Bearing (Cyan) - Scale: +/- PI maps to +/- 60px
    chartCtx.strokeStyle = "#2aa198"; chartCtx.lineWidth = 2;
    chartCtx.beginPath();
    history.forEach((pt, i) => {
        let y = 75 - (pt.bearing * (60/Math.PI)); 
        if (i === 0) chartCtx.moveTo(i, y); else chartCtx.lineTo(i, y);
    });
    chartCtx.stroke();
}

function loop() { 
    update(0.016); 
    draw(); 
    requestAnimationFrame(loop); 
}

window.addEventListener('resize', resize);
resize();
loop();
</script>
</body>
</html>

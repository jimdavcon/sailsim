<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Sailboat: Triple-Pane Debugger</title>
    <style>
        /* Force screen to be exact window size */
        html, body { margin: 0; padding: 0; width: 100%; height: 100%; overflow: hidden; background: #002b36; font-family: monospace; color: #93a1a1; }

        /* Vertical Stack Layout */
        #app-wrapper { display: flex; flex-direction: column; width: 100vw; height: 100vh; }

        /* Pane 1: Top Bar (Telemetry) */
        #telemetry-pane { height: 40px; background: #073642; display: flex; align-items: center; padding: 0 20px; border-bottom: 1px solid #586e75; font-size: 12px; gap: 20px; flex-shrink: 0; }
        .stat-item { color: #268bd2; }

        /* Pane 2: World Map (Middle) */
        #world-pane { flex-grow: 1; background: #002b36; position: relative; overflow: hidden; }
        #simCanvas { position: absolute; top:0; left:0; width: 100%; height: 100%; }

        /* Pane 3: Stripchart (Bottom) - FORCED HEIGHT */
        #chart-pane { height: 180px; background: #001e26; border-top: 2px solid #dc322f; /* RED BORDER TO DEBUG */ position: relative; flex-shrink: 0; }
        #chartCanvas { width: 100%; height: 100%; display: block; }

        /* Floating Controls on Right */
        #controls-overlay { position: absolute; top: 50px; right: 10px; width: 160px; background: rgba(7, 54, 66, 0.9); padding: 15px; border: 1px solid #586e75; z-index: 100; border-radius: 5px; }
        
        input { width: 100%; cursor: pointer; accent-color: #859900; }
        label { font-size: 10px; display: block; margin-top: 10px; }
        .legend { position: absolute; bottom: 5px; right: 10px; font-size: 11px; }
    </style>
</head>
<body>

<div id="app-wrapper">
    <div id="telemetry-pane">
        <div>STATUS: <span id="status" class="stat-item">ACTIVE</span></div>
        <div>CTE: <span id="cte-val" class="stat-item">0.0</span> ft</div>
        <div>TARGET: <span id="wp-val" class="stat-item">WP1</span></div>
    </div>

    <div id="world-pane">
        <canvas id="simCanvas"></canvas>
        <div id="controls-overlay">
            <h4 style="margin:0 0 10px 0; color:#b58900">ENVIRONMENT</h4>
            <label>WIND DIR</label>
            <input type="range" id="windDir" min="0" max="360" value="45">
            <label>WIND SPD (kts)</label>
            <input type="range" id="windSpd" min="0" max="40" value="10">
        </div>
    </div>

    <div id="chart-pane">
        <canvas id="chartCanvas"></canvas>
        <div class="legend">
            <span style="color:#b58900">■ CTE</span> | <span style="color:#2aa198">■ BEARING CMD</span>
        </div>
    </div>
</div>

<script>
const simCanvas = document.getElementById('simCanvas');
const sCtx = simCanvas.getContext('2d');
const chartCanvas = document.getElementById('chartCanvas');
const cCtx = chartCanvas.getContext('2d');

let PX_PER_FT;
const KNOTS_TO_FTS = 1.68781;
let wind = { angle: 0.78, speed: 10 * KNOTS_TO_FTS };
let waypoints = [];
let currentWPIndex = 0;
let history = [];
let boat = { x: 0, y: 0, theta: 0, omega: 0, sails: [{off: 3.5}, {off: -3.5}] };

function norm(a) {
    while (a > Math.PI) a -= 2 * Math.PI;
    while (a < -Math.PI) a += 2 * Math.PI;
    return a;
}

function resize() {
    simCanvas.width = simCanvas.offsetWidth;
    simCanvas.height = simCanvas.offsetHeight;
    chartCanvas.width = chartCanvas.offsetWidth;
    chartCanvas.height = chartCanvas.offsetHeight;
    PX_PER_FT = Math.min(simCanvas.width, simCanvas.height) / 2000;
    
    const d = 500 * PX_PER_FT; const r = 100 * PX_PER_FT;
    waypoints = [
        {x:d, y:-d, r:r, id:"WP1"}, {x:d, y:d, r:r, id:"WP2"},
        {x:-d, y:d, r:r, id:"WP3"}, {x:-d, y:-d, r:r, id:"WP4"}
    ];
}

document.getElementById('windDir').oninput = e => wind.angle = e.target.value * Math.PI/180;
document.getElementById('windSpd').oninput = e => wind.speed = e.target.value * KNOTS_TO_FTS;

function update() {
    let pA = currentWPIndex === 0 ? waypoints[waypoints.length-1] : waypoints[currentWPIndex-1];
    let pB = waypoints[currentWPIndex];

    // Physics & CTE
    let dx = pB.x - pA.x, dy = pB.y - pA.y, len = Math.hypot(dx, dy);
    let ux = dx/len, uy = dy/len;
    let atd = (boat.x - pA.x)*ux + (boat.y - pA.y)*uy;
    let cte = (boat.x - pA.x)*(-uy) + (boat.y - pA.y)*ux;

    // WP Logic
    if (Math.hypot(pB.x - boat.x, pB.y - boat.y) < pB.r) {
        currentWPIndex = (currentWPIndex + 1) % waypoints.length;
        boat.omega *= 0.1;
    }

    // Steering logic (Targeting a point 400ft ahead on the line)
    let tackState = Math.abs(cte) > pB.r ? Math.sign(cte) * -1 : 0;
    let tx = pA.x + (atd + 400*PX_PER_FT)*ux + (tackState * pB.r)*(-uy);
    let ty = pA.y + (atd + 400*PX_PER_FT)*uy + (tackState * pB.r)*ux;
    
    let targetB = Math.atan2(ty - boat.y, tx - boat.x);
    let err = norm(targetB - boat.theta);

    boat.omega += (err * 3.0 - boat.omega * 5.0) * 0.016;
    boat.theta = norm(boat.theta + boat.omega * 0.016);
    
    let speed = wind.speed * 0.4 * (1.0 - Math.min(1, Math.abs(err)));
    boat.x += Math.cos(boat.theta) * speed;
    boat.y += Math.sin(boat.theta) * speed;

    // UI Updates
    document.getElementById('cte-val').innerText = (cte/PX_PER_FT).toFixed(1);
    document.getElementById('wp-val').innerText = pB.id;
    history.push({cte: cte/PX_PER_FT, brg: targetB});
    if(history.length > chartCanvas.width) history.shift();
}

function draw() {
    // 1. Draw World
    sCtx.clearRect(0,0,simCanvas.width, simCanvas.height);
    sCtx.save(); sCtx.translate(simCanvas.width/2, simCanvas.height/2);
    
    waypoints.forEach((wp, i) => {
        sCtx.strokeStyle = (i===currentWPIndex) ? "#859900" : "#586e75";
        sCtx.beginPath(); sCtx.arc(wp.x, wp.y, wp.r, 0, 7); sCtx.stroke();
    });

    sCtx.save(); sCtx.translate(boat.x, boat.y); sCtx.rotate(boat.theta);
    sCtx.strokeStyle = "#eee8d5"; sCtx.strokeRect(-12,-6,24,12);
    sCtx.restore(); sCtx.restore();

    // 2. Draw Chart
    cCtx.fillStyle = "#001e26"; cCtx.fillRect(0,0,chartCanvas.width, chartCanvas.height);
    cCtx.strokeStyle = "#586e75"; cCtx.beginPath(); cCtx.moveTo(0,90); cCtx.lineTo(chartCanvas.width, 90); cCtx.stroke();
    
    cCtx.lineWidth = 2;
    // CTE Plot
    cCtx.strokeStyle = "#b58900"; cCtx.beginPath();
    history.forEach((h, i) => { if(i===0) cCtx.moveTo(i, 90-h.cte*0.4); else cCtx.lineTo(i, 90-h.cte*0.4); });
    cCtx.stroke();

    // Bearing Plot
    cCtx.strokeStyle = "#2aa198"; cCtx.beginPath();
    history.forEach((h, i) => { if(i===0) cCtx.moveTo(i, 90-h.brg*20); else cCtx.lineTo(i, 90-h.brg*20); });
    cCtx.stroke();
}

function loop() { update(); draw(); requestAnimationFrame(loop); }
window.addEventListener('resize', resize);
resize(); loop();
</script>
</body>
</html>

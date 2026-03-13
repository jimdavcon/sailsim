<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Sailboat: Sidebar Chart Fix</title>
    <style>
        body, html { margin: 0; padding: 0; width: 100%; height: 100%; overflow: hidden; background: #002b36; font-family: monospace; }
        
        /* Main World Map (Takes all space not used by sidebar) */
        #world-pane { position: absolute; top: 0; left: 0; right: 240px; bottom: 0; background: #002b36; }
        
        /* Sidebar (Increased width to accommodate chart) */
        #ui-sidebar { 
            position: absolute; top: 0; right: 0; width: 240px; bottom: 0; 
            background: #073642; border-left: 1px solid #586e75; 
            padding: 15px; box-sizing: border-box; color: #859900;
            display: flex; flex-direction: column;
        }
        
        canvas { width: 100%; height: 100%; display: block; }
        
        /* Stripchart container inside sidebar */
        #chart-box { 
            width: 100%; height: 150px; background: #001e26; 
            margin-top: auto; border: 1px solid #586e75; position: relative;
        }

        .label { font-size: 10px; color: #93a1a1; text-transform: uppercase; margin: 15px 0 5px 0; }
        input { width: 100%; accent-color: #859900; margin-bottom: 10px; }
        #telemetry { font-size: 11px; color: #268bd2; line-height: 1.6; background: #00212b; padding: 10px; border-radius: 4px; }
    </style>
</head>
<body>

<div id="world-pane">
    <canvas id="simCanvas"></canvas>
</div>

<div id="ui-sidebar">
    <h3 style="margin:0 0 15px 0; color:#b58900;">INSTRUMENTS</h3>
    
    <div id="telemetry">
        MODE: <span id="mode-val">LINE</span><br>
        WP: <span id="wp-val">1</span><br>
        CTE: <span id="cte-val">0</span> ft<br>
        BRG: <span id="brg-val">0</span>°
    </div>

    <div class="label">Wind Direction</div>
    <input type="range" id="windDir" min="0" max="360" value="45">
    
    <div class="label">Wind Speed</div>
    <input type="range" id="windSpd" min="0" max="40" value="12">

    <div class="label">Stripchart</div>
    <div id="chart-box">
        <canvas id="chartCanvas"></canvas>
        <div style="position:absolute; bottom:2px; right:4px; font-size:9px; color:#586e75;">
            YEL:CTE | CYN:BRG
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
let wind = { angle: 0.78, speed: 12 * KNOTS_TO_FTS };
let waypoints = [];
let currentWPIndex = 0;
let history = [];
let boat = { x: 0, y: 0, theta: 0, omega: 0 };
let lastTargetB = 0;

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
    PX_PER_FT = Math.min(simCanvas.width, simCanvas.height) / 1800;
    
    const d = 400 * PX_PER_FT; const r = 80 * PX_PER_FT;
    waypoints = [
        {x:d, y:-d, r:r, id:"1"}, {x:d, y:d, r:r, id:"2"},
        {x:-d, y:d, r:r, id:"3"}, {x:-d, y:-d, r:r, id:"4"}
    ];
}

document.getElementById('windDir').oninput = e => wind.angle = e.target.value * Math.PI/180;
document.getElementById('windSpd').oninput = e => wind.speed = e.target.value * KNOTS_TO_FTS;

function update() {
    let pA = currentWPIndex === 0 ? waypoints[waypoints.length-1] : waypoints[currentWPIndex-1];
    let pB = waypoints[currentWPIndex];

    let dx = pB.x - pA.x, dy = pB.y - pA.y, len = Math.hypot(dx, dy);
    let ux = dx/len, uy = dy/len;
    let atd = (boat.x - pA.x)*ux + (boat.y - pA.y)*uy;
    let cte = (boat.x - pA.x)*(-uy) + (boat.y - pA.y)*ux;

    if (Math.hypot(pB.x - boat.x, pB.y - boat.y) < pB.r) {
        currentWPIndex = (currentWPIndex + 1) % waypoints.length;
        boat.omega *= 0.1;
    }

    // Steering Stability: Look-ahead point
    let tx = pA.x + (atd + 400*PX_PER_FT)*ux;
    let ty = pA.y + (atd + 400*PX_PER_FT)*uy;
    
    let rawTargetB = Math.atan2(ty - boat.y, tx - boat.x);
    // Smooth the target bearing to prevent sign-flip 360s
    let targetB = lastTargetB + norm(rawTargetB - lastTargetB);
    lastTargetB = targetB;

    let err = norm(targetB - boat.theta);
    boat.omega += (err * 4.0 - boat.omega * 6.0) * 0.016;
    boat.theta = norm(boat.theta + boat.omega * 0.016);
    
    let speed = wind.speed * 0.4 * (1.0 - Math.min(0.8, Math.abs(err)));
    boat.x += Math.cos(boat.theta) * speed;
    boat.y += Math.sin(boat.theta) * speed;

    document.getElementById('cte-val').innerText = (cte/PX_PER_FT).toFixed(0);
    document.getElementById('wp-val').innerText = pB.id;
    document.getElementById('brg-val').innerText = (norm(targetB) * 180/Math.PI).toFixed(0);

    history.push({cte: cte/PX_PER_FT, brg: norm(targetB)});
    if(history.length > chartCanvas.width) history.shift();
}

function draw() {
    // 1. World Map
    sCtx.clearRect(0,0,simCanvas.width, simCanvas.height);
    sCtx.save(); sCtx.translate(simCanvas.width/2, simCanvas.height/2);
    
    waypoints.forEach((wp, i) => {
        sCtx.strokeStyle = (i===currentWPIndex) ? "#859900" : "#073642";
        sCtx.setLineDash([5, 5]);
        sCtx.beginPath(); sCtx.arc(wp.x, wp.y, wp.r, 0, 7); sCtx.stroke();
        sCtx.setLineDash([]);
    });

    sCtx.save(); sCtx.translate(boat.x, boat.y); sCtx.rotate(boat.theta);
    sCtx.strokeStyle = "#eee8d5"; sCtx.strokeRect(-12,-6,24,12);
    sCtx.fillStyle = "#dc322f"; sCtx.fillRect(8,-2,4,4);
    sCtx.restore(); sCtx.restore();

    // 2. Stripchart in Sidebar
    cCtx.fillStyle = "#001e26"; cCtx.fillRect(0,0,chartCanvas.width, chartCanvas.height);
    cCtx.strokeStyle = "#586e75"; cCtx.beginPath(); cCtx.moveTo(0,75); cCtx.lineTo(chartCanvas.width, 75); cCtx.stroke();
    
    // Plot CTE
    cCtx.strokeStyle = "#b58900"; cCtx.beginPath();
    history.forEach((h, i) => { if(i===0) cCtx.moveTo(i, 75-h.cte*0.4); else cCtx.lineTo(i, 75-h.cte*0.4); });
    cCtx.stroke();

    // Plot Bearing
    cCtx.strokeStyle = "#2aa198"; cCtx.beginPath();
    history.forEach((h, i) => { if(i===0) cCtx.moveTo(i, 75-h.brg*20); else cCtx.lineTo(i, 75-h.brg*20); });
    cCtx.stroke();
}

function loop() { update(); draw(); requestAnimationFrame(loop); }
window.addEventListener('resize', resize);
resize(); loop();
</script>
</body>
</html>

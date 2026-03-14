<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Sailboat: 5-Waypoint Mission</title>
    <style>
        body, html { margin: 0; padding: 0; width: 100%; height: 100%; overflow: hidden; background: #002b36; font-family: monospace; }
        #world-pane { position: absolute; top: 0; left: 0; right: 240px; bottom: 0; background: #002b36; }
        #ui-sidebar { 
            position: absolute; top: 0; right: 0; width: 240px; bottom: 0; 
            background: #073642; border-left: 1px solid #586e75; 
            padding: 15px; box-sizing: border-box; color: #859900;
            display: flex; flex-direction: column;
        }
        canvas { width: 100%; height: 100%; display: block; }
        #chart-box { width: 100%; height: 150px; background: #001e26; margin-top: auto; border: 1px solid #586e75; position: relative; }
        .label { font-size: 10px; color: #93a1a1; text-transform: uppercase; margin: 15px 0 5px 0; }
        input { width: 100%; accent-color: #859900; margin-bottom: 10px; }
        #telemetry { font-size: 11px; color: #268bd2; line-height: 1.6; background: #00212b; padding: 10px; border-radius: 4px; }
    </style>
</head>
<body>

<div id="world-pane"><canvas id="simCanvas"></canvas></div>

<div id="ui-sidebar">
    <h3 style="margin:0 0 15px 0; color:#b58900;">MISSION CMD</h3>
    <div id="telemetry">
        TARGET: <span id="wp-val">WP1</span><br>
        CTE: <span id="cte-val">0</span> ft<br>
        BRG: <span id="brg-val">0</span>°
    </div>
    <div class="label">Wind Direction</div>
    <input type="range" id="windDir" min="0" max="360" value="45">
    <div class="label">Wind Speed</div>
    <input type="range" id="windSpd" min="0" max="40" value="15">
    <div class="label">Stripchart</div>
    <div id="chart-box">
        <canvas id="chartCanvas"></canvas>
        <div style="position:absolute; bottom:2px; right:4px; font-size:9px; color:#586e75;">YEL:CTE | CYN:BRG</div>
    </div>
</div>

<script>
const simCanvas = document.getElementById('simCanvas');
const sCtx = simCanvas.getContext('2d');
const chartCanvas = document.getElementById('chartCanvas');
const cCtx = chartCanvas.getContext('2d');

let PX_PER_FT;
const KNOTS_TO_FTS = 1.68781;
let wind = { angle: 0.78, speed: 15 * KNOTS_TO_FTS };
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
    PX_PER_FT = Math.min(simCanvas.width, simCanvas.height) / 2200;
    
    const d = 500 * PX_PER_FT;
    const r = 100 * PX_PER_FT;
    // 5 Waypoints: Square + Return to Origin
    waypoints = [
        {x: d,  y: -d, r: r, id: "WP1"},
        {x: d,  y: d,  r: r, id: "WP2"},
        {x: -d, y: d,  r: r, id: "WP3"},
        {x: -d, y: -d, r: r, id: "WP4"},
        {x: 0,  y: 0,  r: r, id: "ORIGIN"}
    ];
}

document.getElementById('windDir').oninput = e => wind.angle = e.target.value * Math.PI/180;
document.getElementById('windSpd').oninput = e => wind.speed = e.target.value * KNOTS_TO_FTS;

function update() {
    let prevIdx = (currentWPIndex === 0) ? waypoints.length - 1 : currentWPIndex - 1;
    let pA = waypoints[prevIdx];
    let pB = waypoints[currentWPIndex];

    let dx = pB.x - pA.x, dy = pB.y - pA.y, pathLen = Math.hypot(dx, dy);
    let ux = dx / pathLen, uy = dy / pathLen;
    let bdx = boat.x - pA.x, bdy = boat.y - pA.y;

    let atd = bdx * ux + bdy * uy;
    let cte = bdx * (-uy) + bdy * ux;

    if (Math.hypot(pB.x - boat.x, pB.y - boat.y) < pB.r) {
        currentWPIndex = (currentWPIndex + 1) % waypoints.length;
        boat.omega *= 0.1;
    }

    let tx = pA.x + (atd + 400 * PX_PER_FT) * ux;
    let ty = pA.y + (atd + 400 * PX_PER_FT) * uy;
    
    let rawTargetB = Math.atan2(ty - boat.y, tx - boat.x);
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
    sCtx.clearRect(0,0,simCanvas.width, simCanvas.height);
    sCtx.save(); sCtx.translate(simCanvas.width/2, simCanvas.height/2);
    
    waypoints.forEach((wp, i) => {
        let prev = (i === 0) ? waypoints[waypoints.length - 1] : waypoints[i-1];
        let ang = Math.atan2(wp.y - prev.y, wp.x - prev.x);
        
        // Corridors
        sCtx.strokeStyle = (i === currentWPIndex) ? "rgba(133, 153, 0, 0.4)" : "rgba(88, 110, 117, 0.1)";
        sCtx.beginPath();
        sCtx.moveTo(prev.x + wp.r * Math.cos(ang + Math.PI/2), prev.y + wp.r * Math.sin(ang + Math.PI/2));
        sCtx.lineTo(wp.x + wp.r * Math.cos(ang + Math.PI/2), wp.y + wp.r * Math.sin(ang + Math.PI/2));
        sCtx.moveTo(prev.x + wp.r * Math.cos(ang - Math.PI/2), prev.y + wp.r * Math.sin(ang - Math.PI/2));
        sCtx.lineTo(wp.x + wp.r * Math.cos(ang - Math.PI/2), wp.y + wp.r * Math.sin(ang - Math.PI/2));
        sCtx.stroke();

        // WP Circles
        sCtx.strokeStyle = (i === currentWPIndex) ? "#b58900" : "rgba(88, 110, 117, 0.3)";
        sCtx.beginPath(); sCtx.arc(wp.x, wp.y, wp.r, 0, 7); sCtx.stroke();
    });

    sCtx.save(); sCtx.translate(boat.x, boat.y); sCtx.rotate(boat.theta);
    sCtx.strokeStyle = "#eee8d5"; sCtx.strokeRect(-12,-6,24,12);
    sCtx.fillStyle = "#dc322f"; sCtx.fillRect(8,-2,4,4);
    sCtx.restore(); sCtx.restore();

    // Chart
    cCtx.fillStyle = "#001e26"; cCtx.fillRect(0,0,chartCanvas.width, chartCanvas.height);
    cCtx.strokeStyle = "#586e75"; cCtx.beginPath(); cCtx.moveTo(0,75); cCtx.lineTo(chartCanvas.width, 75); cCtx.stroke();
    cCtx.lineWidth = 2;
    cCtx.strokeStyle = "#b58900"; cCtx.beginPath();
    history.forEach((h, i) => { if(i===0) cCtx.moveTo(i, 75-h.cte*0.4); else cCtx.lineTo(i, 75-h.cte*0.4); });
    cCtx.stroke();
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

<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Sailboat: Sensor-Constrained Autonomy</title>
    <style>
        body, html { margin: 0; padding: 0; width: 100%; height: 100%; overflow: hidden; background: #002b36; font-family: monospace; }
        #world-pane { position: absolute; top: 0; left: 0; right: 260px; bottom: 0; background: #002b36; }
        #ui-sidebar { 
            position: absolute; top: 0; right: 0; width: 260px; bottom: 0; 
            background: #073642; border-left: 2px solid #586e75; 
            padding: 15px; box-sizing: border-box; color: #859900;
            display: flex; flex-direction: column;
        }
        canvas { width: 100%; height: 100%; display: block; }
        #chart-box { width: 100%; height: 160px; background: #001e26; margin-top: auto; border: 2px solid #586e75; position: relative; }
        .label { font-size: 11px; color: #93a1a1; text-transform: uppercase; margin: 15px 0 5px 0; font-weight: bold; }
        input { width: 100%; accent-color: #859900; margin-bottom: 10px; }
        #telemetry { font-size: 11px; color: #268bd2; line-height: 1.4; background: #00212b; padding: 10px; border: 1px solid #586e75; }
    </style>
</head>
<body>

<div id="world-pane"><canvas id="simCanvas"></canvas></div>

<div id="ui-sidebar">
    <h3 style="margin:0 0 10px 0; color:#b58900;">SENSOR INPUTS</h3>
    <div id="telemetry">
        POS: <span id="pos-val">0,0</span><br>
        REL WIND: <span id="rel-wind-val">0</span>° @ <span id="ws-val">15</span>kt<br>
        WING1 AOA: <span id="w1-val">0</span>°<br>
        WING2 AOA: <span id="w2-val">0</span>°
    </div>
    
    <div class="label">True Wind Dir</div>
    <input type="range" id="windDir" min="0" max="360" value="45">
    
    <div class="label">True Wind Speed</div>
    <input type="range" id="windSpd" min="0" max="40" value="15">

    <div class="label">Performance Chart</div>
    <div id="chart-box">
        <canvas id="chartCanvas"></canvas>
    </div>
</div>

<script>
const simCanvas = document.getElementById('simCanvas');
const sCtx = simCanvas.getContext('2d');
const chartCanvas = document.getElementById('chartCanvas');
const cCtx = chartCanvas.getContext('2d');

let PX_PER_FT;
const KNOTS_TO_FTS = 1.68781;
let trueWind = { angle: 0.78, speed: 15 * KNOTS_TO_FTS };
let waypoints = [];
let currentWPIndex = 0;
let history = [];

// Boat State
const boat = { 
    x: 0, y: 0, theta: 0, omega: 0, 
    sails: [{x_off: 3.5, aoa: 0}, {x_off: -3.5, aoa: 0}] 
};

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
    const d = 550 * PX_PER_FT, r = 120 * PX_PER_FT;
    waypoints = [
        {x: d,  y: -d, r: r, id: "WP1"}, {x: d,  y: d,  r: r, id: "WP2"},
        {x: -d, y: d,  r: r, id: "WP3"}, {x: -d, y: -d, r: r, id: "WP4"},
        {x: 0,  y: 0,  r: r, id: "ORIGIN"}
    ];
}

document.getElementById('windDir').oninput = e => trueWind.angle = e.target.value * Math.PI/180;
document.getElementById('windSpd').oninput = e => trueWind.speed = e.target.value * KNOTS_TO_FTS;

function update() {
    let pB = waypoints[currentWPIndex];

    // --- SENSOR INPUTS (Only these can be used for logic) ---
    // 1. Boat Position (Provided)
    let boatPos = { x: boat.x, y: boat.y };
    // 2. Relative Wind (Body-relative angle and magnitude)
    let relWindAngle = norm((trueWind.angle + Math.PI) - boat.theta);
    let relWindSpeed = trueWind.speed; // Simplified: assumes low boat speed vs wind
    // 3. Wing AOA (Set by controller, read by physics)
    
    // --- NAVIGATION LOGIC (Sensor-Limited) ---
    // Calculate vector to waypoint in world space...
    let dx = pB.x - boatPos.x;
    let dy = pB.y - boatPos.y;
    let dist = Math.hypot(dx, dy);
    
    // ...BUT transform it into a body-relative bearing immediately
    let worldBearingToWP = Math.atan2(dy, dx);
    let relativeBearingToWP = norm(worldBearingToWP - boat.theta);

    if (dist < pB.r) currentWPIndex = (currentWPIndex + 1) % waypoints.length;

    // Control rudder based ONLY on relative bearing
    // No global theta used here.
    boat.omega += (relativeBearingToWP * 4.0 - boat.omega * 6.0) * 0.016;
    boat.theta = norm(boat.theta + boat.omega * 0.016);
    
    // Control wings based ONLY on relative wind sensor
    boat.sails.forEach(s => {
        // Simple bang-bang or proportional AOA control based on relWindAngle
        s.aoa = Math.max(-0.25, Math.min(0.25, Math.sign(Math.sin(relWindAngle)) * 0.22));
    });

    // --- PHYSICS (External to Controller) ---
    let thrust = relWindSpeed * 0.45 * Math.cos(s.aoa) * (1.0 - Math.abs(relativeBearingToWP)/Math.PI);
    boat.x += Math.cos(boat.theta) * thrust;
    boat.y += Math.sin(boat.theta) * thrust;

    // Telemetry Update
    document.getElementById('pos-val').innerText = `${(boat.x/PX_PER_FT).toFixed(0)}, ${(boat.y/PX_PER_FT).toFixed(0)}`;
    document.getElementById('rel-wind-val').innerText = (relWindAngle * 180/Math.PI).toFixed(0);
    document.getElementById('w1-val').innerText = (boat.sails[0].aoa * 180/Math.PI).toFixed(0);
    
    history.push({cte: relativeBearingToWP, brg: relWindAngle});
    if(history.length > chartCanvas.width) history.shift();
}

function draw() {
    sCtx.clearRect(0,0,simCanvas.width, simCanvas.height);
    sCtx.save(); sCtx.translate(simCanvas.width/2, simCanvas.height/2);
    
    sCtx.lineWidth = 3;
    waypoints.forEach((wp, i) => {
        sCtx.strokeStyle = (i === currentWPIndex) ? "#b58900" : "#586e75";
        sCtx.beginPath(); sCtx.arc(wp.x, wp.y, wp.r, 0, 7); sCtx.stroke();
        
        // WP Wind Indicators
        sCtx.save(); sCtx.translate(wp.x, wp.y); sCtx.rotate(trueWind.angle);
        sCtx.strokeStyle = "#268bd2"; sCtx.beginPath(); sCtx.moveTo(-20,0); sCtx.lineTo(20,0); sCtx.lineTo(12,-6); sCtx.stroke();
        sCtx.restore();
    });

    // Boat Body
    sCtx.save(); sCtx.translate(boat.x, boat.y); sCtx.rotate(boat.theta);
    sCtx.strokeStyle = "#eee8d5"; sCtx.lineWidth = 3;
    sCtx.strokeRect(-14,-7,28,14);
    
    // Sails (drawn using body-relative angles)
    boat.sails.forEach(s => {
        sCtx.save(); sCtx.translate(s.x_off, 0); 
        // Sensor Input: relWindAngle + commanded s.aoa
        let relWindAngle = norm((trueWind.angle + Math.PI) - boat.theta);
        sCtx.rotate(relWindAngle + s.aoa);
        sCtx.fillStyle = "rgba(42, 161, 152, 0.9)"; sCtx.fillRect(-12, -2, 24, 4);
        sCtx.restore();
    });
    sCtx.restore(); sCtx.restore();

    // Chart
    cCtx.fillStyle = "#001e26"; cCtx.fillRect(0,0,chartCanvas.width, chartCanvas.height);
    cCtx.strokeStyle = "#2aa198"; cCtx.beginPath();
    history.forEach((h, i) => { let y = 80 - h.cte*20; if(i===0) cCtx.moveTo(i, y); else cCtx.lineTo(i, y); });
    cCtx.stroke();
}

function loop() { update(); draw(); requestAnimationFrame(loop); }
window.addEventListener('resize', resize); resize(); loop();
</script>
</body>
</html>

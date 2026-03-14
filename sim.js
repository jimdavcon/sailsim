<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Sailboat: Full Graphical Recovery</title>
    <style>
        body, html { margin: 0; padding: 0; width: 100%; height: 100%; overflow: hidden; background: #002b36; font-family: monospace; }
        #world-pane { position: absolute; top: 0; left: 0; right: 280px; bottom: 0; background: #002b36; }
        #ui-sidebar { 
            position: absolute; top: 0; right: 0; width: 280px; bottom: 0; 
            background: #073642; border-left: 3px solid #586e75; 
            padding: 15px; box-sizing: border-box; color: #859900;
            display: flex; flex-direction: column;
        }
        canvas { width: 100%; height: 100%; display: block; }
        #chart-box { width: 100%; height: 180px; background: #001e26; margin-top: auto; border: 2px solid #268bd2; position: relative; }
        .label { font-size: 11px; color: #93a1a1; text-transform: uppercase; margin: 15px 0 5px 0; font-weight: bold; }
        input { width: 100%; accent-color: #859900; margin-bottom: 10px; }
        #telemetry { font-size: 12px; color: #268bd2; line-height: 1.5; background: #00212b; padding: 12px; border: 1px solid #586e75; border-radius: 4px; }
    </style>
</head>
<body>

<div id="world-pane"><canvas id="simCanvas"></canvas></div>

<div id="ui-sidebar">
    <h3 style="margin:0 0 10px 0; color:#b58900; border-bottom: 2px solid #586e75; padding-bottom: 5px;">SENSORS & PERF</h3>
    <div id="telemetry">
        WP: <span id="wp-val">1</span> | CTE: <span id="cte-val">0</span> ft<br>
        REL WIND: <span id="rel-w-val">0</span>°<br>
        WING AOA: <span id="aoa-val">0</span>°
    </div>
    
    <div class="label">True Wind Direction</div>
    <input type="range" id="windDir" min="0" max="360" value="45">
    
    <div class="label">True Wind Speed (kts)</div>
    <input type="range" id="windSpd" min="0" max="40" value="15">

    <div class="label">Stripchart (Body-Rel Error)</div>
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
const boat = { x: 0, y: 0, theta: 0, omega: 0, sails: [{x: 4, a:0}, {x: -4, a:0}] };

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
    PX_PER_FT = Math.min(simCanvas.width, simCanvas.height) / 2400;
    const d = 550 * PX_PER_FT, r = 120 * PX_PER_FT;
    waypoints = [
        {x: d,  y: -d, r: r, id: "1"}, {x: d,  y: d,  r: r, id: "2"},
        {x: -d, y: d,  r: r, id: "3"}, {x: -d, y: -d, r: r, id: "4"},
        {x: 0,  y: 0,  r: r, id: "ORIGIN"}
    ];
}

document.getElementById('windDir').oninput = e => trueWind.angle = e.target.value * Math.PI/180;
document.getElementById('windSpd').oninput = e => trueWind.speed = e.target.value * KNOTS_TO_FTS;

function update() {
    let prevIdx = (currentWPIndex === 0) ? waypoints.length - 1 : currentWPIndex - 1;
    let pA = waypoints[prevIdx], pB = waypoints[currentWPIndex];

    // Sensor Inputs
    let relWindAngle = norm((trueWind.angle + Math.PI) - boat.theta);
    let dx = pB.x - boat.x, dy = pB.y - boat.y;
    let relativeBearingToWP = norm(Math.atan2(dy, dx) - boat.theta);
    let cte = (dx * -Math.sin(boat.theta) + dy * Math.cos(boat.theta)) / PX_PER_FT;

    if (Math.hypot(dx, dy) < pB.r) currentWPIndex = (currentWPIndex + 1) % waypoints.length;

    // Control Logic (Sensor Constrained)
    boat.omega += (relativeBearingToWP * 4.5 - boat.omega * 6.0) * 0.016;
    boat.theta = norm(boat.theta + boat.omega * 0.016);
    
    let sailAOA = Math.sign(Math.sin(relWindAngle)) * 0.22;
    boat.sails.forEach(s => s.a = relWindAngle + sailAOA);

    // Physics
    let thrust = trueWind.speed * 0.45 * (1.0 - Math.abs(relativeBearingToWP)/Math.PI);
    boat.x += Math.cos(boat.theta) * thrust;
    boat.y += Math.sin(boat.theta) * thrust;

    // Telemetry
    document.getElementById('wp-val').innerText = pB.id;
    document.getElementById('cte-val').innerText = (dy/PX_PER_FT).toFixed(0);
    document.getElementById('rel-w-val').innerText = (relWindAngle * 180/Math.PI).toFixed(0);
    document.getElementById('aoa-val').innerText = (sailAOA * 180/Math.PI).toFixed(0);
    
    history.push({err: relativeBearingToWP, wind: relWindAngle});
    if(history.length > chartCanvas.width) history.shift();
}

function draw() {
    sCtx.clearRect(0,0,simCanvas.width, simCanvas.height);
    sCtx.save(); sCtx.translate(simCanvas.width/2, simCanvas.height/2);
    
    // 1. Corridors & Waypoints (3x THICK)
    sCtx.lineWidth = 3;
    waypoints.forEach((wp, i) => {
        let prev = (i === 0) ? waypoints[waypoints.length - 1] : waypoints[i-1];
        let ang = Math.atan2(wp.y - prev.y, wp.x - prev.x);
        
        sCtx.strokeStyle = (i === currentWPIndex) ? "rgba(133, 153, 0, 0.5)" : "rgba(88, 110, 117, 0.2)";
        sCtx.beginPath();
        sCtx.moveTo(prev.x + wp.r*Math.cos(ang+1.57), prev.y + wp.r*Math.sin(ang+1.57));
        sCtx.lineTo(wp.x + wp.r*Math.cos(ang+1.57), wp.y + wp.r*Math.sin(ang+1.57));
        sCtx.moveTo(prev.x + wp.r*Math.cos(ang-1.57), prev.y + wp.r*Math.sin(ang-1.57));
        sCtx.lineTo(wp.x + wp.r*Math.cos(ang-1.57), wp.y + wp.r*Math.sin(ang-1.57));
        sCtx.stroke();

        sCtx.strokeStyle = (i === currentWPIndex) ? "#b58900" : "#586e75";
        sCtx.beginPath(); sCtx.arc(wp.x, wp.y, wp.r, 0, 7); sCtx.stroke();

        // Waypoint Wind Arrows
        sCtx.save(); sCtx.translate(wp.x, wp.y); sCtx.rotate(trueWind.angle);
        sCtx.strokeStyle = "#268bd2"; sCtx.beginPath();
        sCtx.moveTo(-25,0); sCtx.lineTo(25,0); sCtx.lineTo(15,-8); sCtx.moveTo(25,0); sCtx.lineTo(15,8);
        sCtx.stroke(); sCtx.restore();
    });

    // 2. Boat & Wing Sails
    sCtx.save(); sCtx.translate(boat.x, boat.y); sCtx.rotate(boat.theta);
    sCtx.strokeStyle = "#eee8d5"; sCtx.strokeRect(-15,-8,30,16);
    sCtx.fillStyle = "#dc322f"; sCtx.fillRect(10,-4,6,8); // Bow
    
    boat.sails.forEach(s => {
        sCtx.save(); sCtx.translate(s.x, 0); sCtx.rotate(s.a);
        sCtx.fillStyle = "rgba(42, 161, 152, 0.9)"; sCtx.fillRect(-12, -2, 24, 4);
        sCtx.restore();
    });
    sCtx.restore(); sCtx.restore();

    // 3. Stripchart
    cCtx.fillStyle = "#001e26"; cCtx.fillRect(0,0,chartCanvas.width, chartCanvas.height);
    cCtx.strokeStyle = "#586e75"; cCtx.beginPath(); cCtx.moveTo(0,90); cCtx.lineTo(chartCanvas.width, 90); cCtx.stroke();
    cCtx.lineWidth = 2;
    cCtx.strokeStyle = "#2aa198"; cCtx.beginPath(); // Relative Error (Cyan)
    history.forEach((h, i) => { let y = 90 - h.err*30; if(i===0) cCtx.moveTo(i, y); else cCtx.lineTo(i, y); });
    cCtx.stroke();
}

function loop() { update(); draw(); requestAnimationFrame(loop); }
window.onresize = resize; resize(); loop();
</script>
</body>
</html>

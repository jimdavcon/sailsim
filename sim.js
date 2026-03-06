const canvas = document.getElementById('simCanvas');
const ctx = canvas.getContext('2d');

const windSpeedSlider = document.getElementById('windSpeed');
const windSpeedLabel = document.getElementById('windSpeedLabel');
const windDirSlider = document.getElementById('windDir');
const windDirLabel = document.getElementById('windDirLabel');
const wingAngleSlider = document.getElementById('wingAngle');
const wingAngleLabel = document.getElementById('wingAngleLabel');

const speedGauge = document.getElementById('speedGauge');
const headingGauge = document.getElementById('headingGauge');

// Physical parameters for 6 ft, 20 lb hull
const m = 9.1;          // mass [kg]
const Iz = 2.5;         // yaw inertia [kg m^2]
const rhoAir = 1.225;   // air density [kg/m^3]
const rhoWater = 1025;  // water density [kg/m^3]

// Hull hydrodynamics (Option C: keel-like)
const Ah = 0.30;            // reference area [m^2]
const CLalpha_h = 4.0;
const CLmax_h   = 1.5;
const CD0_h     = 0.01;
const k_induced_h = 0.05;

// Wing parameters (6 ft × 1 ft)
const Aw = 0.56;              // wing area [m^2]
const CLalpha = 2 * Math.PI;  // lift slope
const CLmax = 1.2;            // stall limit
const CD0 = 0.02;             // profile drag
const k_induced = 0.08;       // induced drag

// Yaw damping
const Cr  = 5.0;

// Mast at CG → no direct moment arm
const leverWing = 0.0;

// Time step
const dt = 0.02;

// State
let x = canvas.width / 2;
let y = canvas.height / 2;
let psi = 0;
let u = 0;
let v = 0;
let r = 0;

// Environment defaults
let windSpeed = 0;
let windDir = 0;
let wingAngle = -Math.PI / 2;   // -90°

windSpeedLabel.textContent = "0.0 m/s";
windDirLabel.textContent = "0° (from)";
wingAngleLabel.textContent = "-90°";

windSpeedSlider.value = 0;
windDirSlider.value = 0;
wingAngleSlider.value = -90;

// Debug values (wing)
let debugAlpha = 0;
let debugLift = 0;
let debugDrag = 0;

// Trail
const trail = [];
let trailTimer = 0;
const TRAIL_LENGTH = 30;

// Total force in world frame (for drawing)
let Fwx = 0;
let Fwy = 0;

windSpeedSlider.oninput = () => {
  windSpeed = parseFloat(windSpeedSlider.value);
  windSpeedLabel.textContent = windSpeed.toFixed(1) + ' m/s';
};

windDirSlider.oninput = () => {
  const deg = parseFloat(windDirSlider.value);
  windDir = deg * Math.PI / 180;
  windDirLabel.textContent = deg.toFixed(0) + '° (from)';
};

wingAngleSlider.oninput = () => {
  const deg = parseFloat(wingAngleSlider.value);
  wingAngle = deg * Math.PI / 180;
  wingAngleLabel.textContent = deg.toFixed(0) + '°';
};

function wrapAngle(a) {
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
}

function step() {
  // World-frame velocity
  const vx = u * Math.cos(psi) - v * Math.sin(psi);
  const vy = u * Math.sin(psi) + v * Math.cos(psi);

  // Wind: windDir is direction wind is coming FROM
  const Vwx = windSpeed * Math.cos(windDir + Math.PI);
  const Vwy = windSpeed * Math.sin(windDir + Math.PI);

  // Apparent wind in world frame
  const Vax = Vwx - vx;
  const Vay = Vwy - vy;

  // Apparent wind in body frame
  const cosPsi = Math.cos(psi);
  const sinPsi = Math.sin(psi);
  const Vabx =  cosPsi * Vax + sinPsi * Vay;
  const Vaby = -sinPsi * Vax + cosPsi * Vay;

  const Va = Math.hypot(Vabx, Vaby) || 1e-6;
  const betaA = Math.atan2(Vaby, Vabx);

  // --- Wing aerodynamics ---
  const alpha = betaA - wingAngle;

  let CL = CLalpha * alpha;
  if (CL > CLmax) CL = CLmax;
  if (CL < -CLmax) CL = -CLmax;

  const CD = CD0 + k_induced * CL * CL;

  const q = 0.5 * rhoAir * Va * Va;

  const L = q * Aw * CL;
  const D = q * Aw * CD;

  const awx = Vabx / Va;
  const awy = Vaby / Va;

  const dragDirX = -awx;
  const dragDirY = -awy;

  const liftDirX = -dragDirY;
  const liftDirY =  dragDirX;

  const F_wing_x = L * liftDirX + D * dragDirX;
  const F_wing_y = L * liftDirY + D * dragDirY;

  const M_wing_z = leverWing * F_wing_y;

  // Store debug values for display
  debugAlpha = alpha;
  debugLift = L;
  debugDrag = D;

  // --- Hull hydrodynamics (Option C) ---
  const Vh = Math.hypot(u, v) || 1e-6;
  const betaH = Math.atan2(v, u);

  let CLh = CLalpha_h * betaH;
  if (CLh > CLmax_h) CLh = CLmax_h;
  if (CLh < -CLmax_h) CLh = -CLmax_h;

  const CDh = CD0_h + k_induced_h * CLh * CLh;

  const qh = 0.5 * rhoWater * Vh * Vh;

  const Lh = qh * Ah * CLh;
  const Dh = qh * Ah * CDh;

  const hwx = u / Vh;
  const hwy = v / Vh;

  const dragHDirX = -hwx;
  const dragHDirY = -hwy;

  const liftHDirX = -dragHDirY;
  const liftHDirY =  dragHDirX;

  const F_hull_x = Lh * liftHDirX + Dh * dragHDirX;
  const F_hull_y = Lh * liftHDirY + Dh * dragHDirY;

  // Hull yaw damping
  const Md = -Cr * r * Math.abs(r);

  // --- Total forces and moment in body frame ---
  const Fx = F_wing_x + F_hull_x;
  const Fy = F_wing_y + F_hull_y;
  const Mz = M_wing_z + Md;

  // Total force in world frame (for drawing)
  Fwx = Fx * Math.cos(psi) - Fy * Math.sin(psi);
  Fwy = Fx * Math.sin(psi) + Fy * Math.cos(psi);

  // Integrate
  const du = Fx / m;
  const dv = Fy / m;
  const dr = Mz / Iz;

  u += du * dt;
  v += dv * dt;
  r += dr * dt;

  psi = wrapAngle(psi + r * dt);
  x += (u * Math.cos(psi) - v * Math.sin(psi)) * dt;
  y += (u * Math.sin(psi) + v * Math.cos(psi)) * dt;

  // Wrap world
  if (x < 0) x += canvas.width;
  if (x > canvas.width) x -= canvas.width;
  if (y < 0) y += canvas.height;
  if (y > canvas.height) y -= canvas.height;

  // Gauges
  const speed = Math.hypot(u, v);
  speedGauge.textContent = speed.toFixed(2);
  headingGauge.textContent = (psi * 180 / Math.PI).toFixed(1);

  // Trail
  trailTimer += dt;
  if (trailTimer >= 1.0) {
    trailTimer = 0;
    trail.push({ x, y, speed });
    if (trail.length > TRAIL_LENGTH) trail.shift();
  }
}

function drawDebugText() {
  ctx.save();
  ctx.font = '14px monospace';

  const stallAoA = CLmax / CLalpha;
  const aoaColor = Math.abs(debugAlpha) > Math.abs(stallAoA) ? '#cc0000' : '#009900';

  ctx.fillStyle = aoaColor;
  ctx.fillText(`AoA: ${(debugAlpha * 180/Math.PI).toFixed(1)} deg`, 10, canvas.height - 60);

  ctx.fillStyle = '#000';
  ctx.fillText(`Lift: ${debugLift.toFixed(2)} N`, 10, canvas.height - 40);
  ctx.fillText(`Drag: ${debugDrag.toFixed(2)} N`, 10, canvas.height - 20);

  ctx.restore();
}

function drawGrid() {
  const spacing = 40;
  ctx.save();
  ctx.strokeStyle = 'rgba(255,255,255,0.25)';
  for (let xg = 0; xg <= canvas.width; xg += spacing) {
    ctx.beginPath(); ctx.moveTo(xg, 0); ctx.lineTo(xg, canvas.height); ctx.stroke();
  }
  for (let yg = 0; yg <= canvas.height; yg += spacing) {
    ctx.beginPath(); ctx.moveTo(0, yg); ctx.lineTo(canvas.width, yg); ctx.stroke();
  }
  ctx.restore();
}

function drawTrail() {
  if (!trail.length) return;
  let maxSpeed = Math.max(...trail.map(p => p.speed), 0.01);

  ctx.save();
  const n = trail.length;
  for (let i = 0; i < n; i++) {
    const p = trail[i];
    const age = n > 1 ? i / (n - 1) : 1;
    const alpha = 0.1 + 0.9 * age;
    const base = speedToColor(p.speed, maxSpeed);
    const rgba = base.replace("rgb", "rgba").replace(")", `,${alpha})`);
    ctx.fillStyle = rgba;
    ctx.beginPath();
    ctx.arc(p.x, p.y, 4, 0, 2 * Math.PI);
    ctx.fill();
  }
  ctx.restore();
}

function speedToColor(speed, maxSpeed) {
  const t = Math.min(speed / maxSpeed, 1);
  return `rgb(${Math.floor(255 * t)},0,0)`;
}

function drawBoat() {
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(psi);

  // Hull: symmetric airfoil shape (pointy at both ends)
  ctx.fillStyle = '#444';
  ctx.beginPath();
  ctx.moveTo(30, 0);     // front point
  ctx.lineTo(0, -8);     // upper mid
  ctx.lineTo(-30, 0);    // rear point
  ctx.lineTo(0, 8);      // lower mid
  ctx.closePath();
  ctx.fill();

  // Wing: centered on mast, pointy leading edge
  ctx.save();
  ctx.rotate(wingAngle);

  ctx.fillStyle = '#ffffff';
  ctx.beginPath();
  ctx.moveTo(0, -25);    // leading edge point
  ctx.lineTo(5, 25);     // trailing edge lower
  ctx.lineTo(-5, 25);    // trailing edge upper
  ctx.closePath();
  ctx.fill();

  ctx.restore();
  ctx.restore();
}

function drawForceVector() {
  const scale = 0.2; // pixels per Newton
  const endX = x + Fwx * scale;
  const endY = y + Fwy * scale;

  ctx.save();
  ctx.strokeStyle = '#00aa00';
  ctx.fillStyle = '#00aa00';
  ctx.lineWidth = 3;

  ctx.beginPath();
  ctx.moveTo(x, y);
  ctx.lineTo(endX, endY);
  ctx.stroke();

  const angle = Math.atan2(endY - y, endX - x);
  const ah = 10;
  ctx.beginPath();
  ctx.moveTo(endX, endY);
  ctx.lineTo(endX - ah * Math.cos(angle - Math.PI / 6),
             endY - ah * Math.sin(angle - Math.PI / 6));
  ctx.lineTo(endX - ah * Math.cos(angle + Math.PI / 6),
             endY - ah * Math.sin(angle + Math.PI / 6));
  ctx.closePath();
  ctx.fill();

  ctx.restore();
}

function drawWindVector() {
  const originX = 60, originY = 60, scale = 5;
  const Vwx = windSpeed * Math.cos(windDir + Math.PI);
  const Vwy = windSpeed * Math.sin(windDir + Math.PI);
  const endX = originX + Vwx * scale;
  const endY = originY + Vwy * scale;

  ctx.save();
  ctx.strokeStyle = '#ff0000';
  ctx.fillStyle = '#ff0000';
  ctx.lineWidth = 2;

  ctx.beginPath();
  ctx.moveTo(originX, originY);
  ctx.lineTo(endX, endY);
  ctx.stroke();

  const angle = Math.atan2(endY - originY, endX - originX);
  const ah = 8;
  ctx.beginPath();
  ctx.moveTo(endX, endY);
  ctx.lineTo(endX - ah * Math.cos(angle - Math.PI / 6),
             endY - ah * Math.sin(angle - Math.PI / 6));
  ctx.lineTo(endX - ah * Math.cos(angle + Math.PI / 6),
             endY - ah * Math.sin(angle + Math.PI / 6));
  ctx.closePath();
  ctx.fill();

  ctx.restore();
}

function render() {
  step();
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  drawGrid();
  drawTrail();
  drawBoat();
  drawForceVector();
  drawWindVector();
  drawDebugText();
  requestAnimationFrame(render);
}

render();

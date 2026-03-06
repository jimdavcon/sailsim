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

// Hull hydrodynamics (airfoil-like hull)
const Ax = 0.02;        // surge projected area [m^2]
const Ay = 0.25;        // sway projected area [m^2]
const Cdx = 0.8;
const Cdy = 1.2;
const Cr  = 5.0;        // yaw damping

// Wing parameters (6 ft × 1 ft)
const Aw = 0.56;              // wing area [m^2]
const CLalpha = 2 * Math.PI;  // lift slope
const CLmax = 1.2;            // stall limit
const CD0 = 0.02;             // profile drag
const k_induced = 0.08;       // induced drag

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

// Environment
let windSpeed = parseFloat(windSpeedSlider.value);
let windDir = parseFloat(windDirSlider.value) * Math.PI / 180;
let wingAngle = 0;

windSpeedLabel.textContent = windSpeed.toFixed(1) + ' m/s';
windDirLabel.textContent = windDirSlider.value + '° (from)';
wingAngleLabel.textContent = wingAngleSlider.value + '°';

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

// Trail
const trail = [];
let trailTimer = 0;
const TRAIL_LENGTH = 30;

// Total force in world frame (for drawing)
let Fwx = 0;
let Fwy = 0;

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

  // Wing AoA
  const alpha = betaA - wingAngle;

  // Lift coefficient with stall
  let CL = CLalpha * alpha;
  if (CL > CLmax) CL = CLmax;
  if (CL < -CLmax) CL = -CLmax;

  // Drag coefficient
  const CD = CD0 + k_induced * CL * CL;

  // Dynamic pressure
  const q = 0.5 * rhoAir * Va * Va;

  // Lift and drag magnitudes
  const L = q * Aw * CL;
  const D = q * Aw * CD;

  // Apparent wind unit vector in body frame
  const awx = Vabx / Va;
  const awy = Vaby / Va;

  // Drag direction = opposite apparent wind
  const dragDirX = -awx;
  const dragDirY = -awy;

  // Lift direction = perpendicular to drag
  const liftDirX = -dragDirY;
  const liftDirY =  dragDirX;

  // Total wing forces in body frame
  const F_wing_x = L * liftDirX + D * dragDirX;
  const F_wing_y = L * liftDirY + D * dragDirY;

  // Wing moment about CG
  const M_wing_z = leverWing * F_wing_y;

  // Hull drag
  const Fdx = -0.5 * rhoWater * Cdx * Ax * Math.abs(u) * u;
  const Fdy = -0.5 * rhoWater * Cdy * Ay * Math.abs(v) * v;

  // Yaw damping
  const Md = -Cr * r * Math.abs(r);

  // Total forces and moment in body frame
  const Fx = F_wing_x + Fdx;
  const Fy = F_wing_y + Fdy;
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

function speedToColor(speed, maxSpeed) {
  const t = Math.min(speed / maxSpeed, 1);
  return `rgb(${Math.floor(255 * t)},0,0)`;
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

function drawBoat() {
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(psi);

  // Hull
  ctx.fillStyle = '#444';
  ctx.beginPath();
  ctx.moveTo(30, 0);
  ctx.lineTo(-30, -6);
  ctx.lineTo(-30, 6);
  ctx.closePath();
  ctx.fill();

  // Wing
  ctx.save();
  ctx.rotate(wingAngle);
  ctx.fillStyle = '#ffffff';
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(0, -40);
  ctx.lineTo(5, -40);
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
  requestAnimationFrame(render);
}

render();

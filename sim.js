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
const m = 9.1;          // mass [kg] (20 lb)
const Iz = 2.5;         // yaw inertia [kg m^2] (approx for 1.83 m hull)
const rhoAir = 1.225;   // air density [kg/m^3]
const rhoWater = 1025;  // water density [kg/m^3]

// Hull / hydrodynamics for slender airfoil-like hull
const Ax = 0.02;        // surge projected area [m^2]
const Ay = 0.25;        // sway projected area [m^2]
const Cdx = 0.8;        // surge drag coefficient
const Cdy = 1.2;        // sway drag coefficient
const Cr  = 5.0;        // yaw damping coefficient (lighter boat)

// Wing parameters (6 ft tall, 1 ft chord)
const Aw = 0.56;              // wing area [m^2] (1.83 m * 0.305 m)
const CLalpha = 2 * Math.PI;  // lift slope [per rad]
const CLmax = 1.2;            // stall limit
const CD0 = 0.02;             // profile drag
const k_induced = 0.08;       // induced drag factor

// Mast at CG: no direct moment arm from wing force
const leverWing = 0.0;        // [m] (set ~0.1 if you want some direct yaw)

// Time step
const dt = 0.02;        // [s]

// State (world frame position, heading; body frame velocities)
let x = canvas.width / 2;
let y = canvas.height / 2;
let psi = 0;            // heading [rad]
let u = 0.0;            // surge [m/s]
let v = 0.0;            // sway [m/s]
let r = 0.0;            // yaw rate [rad/s]

// Environment / controls
let windSpeed = parseFloat(windSpeedSlider.value); // [m/s]
// windDir is direction wind is coming FROM, in world frame
let windDir = parseFloat(windDirSlider.value) * Math.PI / 180;
let wingAngle = 0;      // wing rotation relative to boat x-axis [rad]

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

// --- Position history (30 seconds, 1 sample per second) ---
const trail = [];
let trailTimer = 0;        // seconds since last dot
const TRAIL_LENGTH = 30;   // seconds (max entries)

// Utility: wrap angle to [-pi, pi]
function wrapAngle(a) {
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
}

function step() {
  // --- Kinematics: boat velocity in world frame ---
  const vx = u * Math.cos(psi) - v * Math.sin(psi);
  const vy = u * Math.sin(psi) + v * Math.cos(psi);

  // --- Wind model ---
  // windDir is direction wind is coming FROM.
  // So wind velocity vector points TO windDir + pi.
  const Vwx = windSpeed * Math.cos(windDir + Math.PI);
  const Vwy = windSpeed * Math.sin(windDir + Math.PI);

  // Apparent wind in world frame
  const Vax = Vwx - vx;
  const Vay = Vwy - vy;

  // Transform apparent wind to body frame
  const cosPsi = Math.cos(psi);
  const sinPsi = Math.sin(psi);
  const Vabx =  cosPsi * Vax + sinPsi * Vay;
  const Vaby = -sinPsi * Vax + cosPsi * Vay;

  const Va = Math.hypot(Vabx, Vaby) || 1e-6;
  const betaA = Math.atan2(Vaby, Vabx); // apparent wind angle in body frame

  // --- Wing aerodynamic model ---
  // Wing chord aligned with boat x-axis, plus control angle wingAngle
  const alpha = betaA - wingAngle; // angle of attack

  // Lift coefficient with stall limit
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

  // Directions in body frame
  const liftDirX = -Math.sin(betaA);
  const liftDirY =  Math.cos(betaA);

  const dragDirX = -Math.cos(betaA);
  const dragDirY = -Math.sin(betaA);

  // Total wing forces in body frame
  const F_wing_x = L * liftDirX + D * dragDirX;
  const F_wing_y = L * liftDirY + D * dragDirY;

  // Wing moment about CG (mast at CG → leverWing = 0)
  const M_wing_z = leverWing * F_wing_y;

  // --- Hydrodynamic drag (hull/keel) ---
  const Fdx = -0.5 * rhoWater * Cdx * Ax * Math.abs(u) * u;
  const Fdy = -0.5 * rhoWater * Cdy * Ay * Math.abs(v) * v;

  // --- Yaw damping (hydrodynamic) ---
  const Md = -Cr * r * Math.abs(r);

  // --- Total forces and moment in body frame ---
  const Fx = F_wing_x + Fdx;
  const Fy = F_wing_y + Fdy;
  const Mz = M_wing_z + Md;

  // --- Equations of motion (body frame) ---
  const du = Fx / m;
  const dv = Fy / m;
  const dr = Mz / Iz;

  // Semi-implicit Euler integration
  u += du * dt;
  v += dv * dt;
  r += dr * dt;

  psi = wrapAngle(psi + r * dt);
  x += (u * Math.cos(psi) - v * Math.sin(psi)) * dt;
  y += (u * Math.sin(psi) + v * Math.cos(psi)) * dt;

  // Wrap around canvas edges (world-fixed axes, torus world)
  if (x < 0) x += canvas.width;
  if (x > canvas.width) x -= canvas.width;
  if (y < 0) y += canvas.height;
  if (y > canvas.height) y -= canvas.height;

  // Update gauges
  const speed = Math.hypot(u, v);
  speedGauge.textContent = speed.toFixed(2);
  const headingDeg = wrapAngle(psi) * 180 / Math.PI;
  headingGauge.textContent = headingDeg.toFixed(1);

  // --- Update trail once per simulated second ---
  trailTimer += dt;
  if (trailTimer >= 1.0) {
    trailTimer = 0;

    const s = Math.hypot(u, v);
    trail.push({ x, y, speed: s });

    if (trail.length > TRAIL_LENGTH) {
      trail.shift();
    }
  }
}

// Map speed to color: black (0) → red (maxSpeed)
function speedToColor(speed, maxSpeed) {
  const t = Math.min(speed / maxSpeed, 1);   // normalize [0,1]
  const r = Math.floor(255 * t);
  const g = 0;
  const b = 0;
  return `rgb(${r},${g},${b})`;
}

function drawGrid() {
  const spacing = 40;
  ctx.save();
  ctx.strokeStyle = 'rgba(255,255,255,0.25)';
  ctx.lineWidth = 1;

  // Vertical lines
  for (let xg = 0; xg <= canvas.width; xg += spacing) {
    ctx.beginPath();
    ctx.moveTo(xg, 0);
    ctx.lineTo(xg, canvas.height);
    ctx.stroke();
  }

  // Horizontal lines
  for (let yg = 0; yg <= canvas.height; yg += spacing) {
    ctx.beginPath();
    ctx.moveTo(0, yg);
    ctx.lineTo(canvas.width, yg);
    ctx.stroke();
  }

  // Axes cross at canvas center
  const cx = canvas.width / 2;
  const cy = canvas.height / 2;
  ctx.strokeStyle = 'rgba(0,0,0,0.4)';
  ctx.lineWidth = 1.5;

  ctx.beginPath();
  ctx.moveTo(cx, 0);
  ctx.lineTo(cx, canvas.height);
  ctx.stroke();

  ctx.beginPath();
  ctx.moveTo(0, cy);
  ctx.lineTo(canvas.width, cy);
  ctx.stroke();

  ctx.fillStyle = '#000';
  ctx.font = '12px sans-serif';
  ctx.fillText('+x', canvas.width - 20, cy - 5);
  ctx.fillText('+y', cx + 5, 15);

  ctx.restore();
}

function drawTrail() {
  if (trail.length === 0) return;

  // Find max speed in the last 30 seconds
  let maxSpeed = 0;
  for (const p of trail) {
    if (p.speed > maxSpeed) maxSpeed = p.speed;
  }
  if (maxSpeed < 0.01) maxSpeed = 0.01;

  ctx.save();

  const n = trail.length;
  for (let i = 0; i < n; i++) {
    const p = trail[i];

    // Age factor: 0 = oldest, 1 = newest
    const age = n > 1 ? i / (n - 1) : 1;
    const alpha = 0.1 + 0.9 * age; // fade oldest

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

  // Hull (6 ft ~ 1.83 m, just a visual cue)
  ctx.fillStyle = '#444';
  ctx.beginPath();
  ctx.moveTo(30, 0);   // bow
  ctx.lineTo(-30, -6); // stern upper
  ctx.lineTo(-30, 6);  // stern lower
  ctx.closePath();
  ctx.fill();

  // Wing (vertical, drawn as a fin)
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

function drawWindVector() {
  const originX = 60;
  const originY = 60;
  const scale = 5; // pixels per m/s

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

  ctx.fillStyle = '#000';
  ctx.font = '12px sans-serif';
  ctx.fillText('Wind', originX + 10, originY - 10);

  ctx.restore();
}

function render() {
  step();

  ctx.clearRect(0, 0, canvas.width, canvas.height);

  drawGrid();
  drawTrail();
  drawBoat();
  drawWindVector();

  requestAnimationFrame(render);
}

render();

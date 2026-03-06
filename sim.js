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

// Physical parameters
const m = 9.1;
const Iz = 2.5;
const rhoAir = 1.225;
const rhoWater = 1025;

// Hull hydrodynamics (Option C)
const Ah = 0.30;
const CLalpha_h = 4.0;
const CLmax_h   = 1.5;
const CD0_h     = 0.01;
const k_induced_h = 0.05;

// Wing parameters
const Aw = 0.56;
const CLalpha = 2 * Math.PI;
const CLmax = 1.2;
const CD0 = 0.02;
const k_induced = 0.08;

// Sail positions (meters)
const halfLength = 0.915;
const x_front = +halfLength / 2;   // +0.4575 m
const x_rear  = -halfLength / 2;   // -0.4575 m

// Yaw damping
const Cr = 5.0;

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
let windSpeed = 0;
let windDir = 0;
let rearWingAngle = -Math.PI / 2;

// Debug
let debugAlpha = 0;
let debugLift = 0;
let debugDrag = 0;

// Trail
const trail = [];
let trailTimer = 0;
const TRAIL_LENGTH = 30;

// Total force in world frame
let Fwx = 0;
let Fwy = 0;

// UI handlers
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
  rearWingAngle = deg * Math.PI / 180;
  wingAngleLabel.textContent = deg.toFixed(0) + '°';
};

function wrapAngle(a) {
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
}

function apparentWindAtOffset(Vax, Vay, offsetX) {
  // Add rotational component: r × offset
  // In body frame: (0, r * offsetX)
  const Vax_local = Vax - r * offsetX * Math.sin(psi);
  const Vay_local = Vay + r * offsetX * Math.cos(psi);

  // Transform to body frame
  const cosPsi = Math.cos(psi);
  const sinPsi = Math.sin(psi);

  const Vabx =  cosPsi * Vax_local + sinPsi * Vay_local;
  const Vaby = -sinPsi * Vax_local + cosPsi * Vay_local;

  return { Vabx, Vaby };
}

function computeWingForces(Vabx, Vaby, wingAngle, area, CLalpha, CLmax, CD0, k_induced) {
  const Va = Math.hypot(Vabx, Vaby) || 1e-6;
  const betaA = Math.atan2(Vaby, Vabx);
  const alpha = betaA - wingAngle;

  let CL = CLalpha * alpha;
  if (CL > CLmax) CL = CLmax;
  if (CL < -CLmax) CL = -CLmax;

  const CD = CD0 + k_induced * CL * CL;
  const q = 0.5 * rhoAir * Va * Va;

  const L = q * area * CL;
  const D = q * area * CD;

  const awx = Vabx / Va;
  const awy = Vaby / Va;

  const dragDirX = -awx;
  const dragDirY = -awy;

  const liftDirX = -dragDirY;
  const liftDirY =  dragDirX;

  return {
    Fx: L * liftDirX + D * dragDirX,
    Fy: L * liftDirY + D * dragDirY,
    alpha,
    L,
    D
  };
}

function step() {
  // World-frame velocity
  const vx = u * Math.cos(psi) - v * Math.sin(psi);
  const vy = u * Math.sin(psi) + v * Math.cos(psi);

  // Wind (coming FROM windDir)
  const Vwx = windSpeed * Math.cos(windDir + Math.PI);
  const Vwy = windSpeed * Math.sin(windDir + Math.PI);

  // Apparent wind in world frame
  const Vax = Vwx - vx;
  const Vay = Vwy - vy;

  // --- Front sail (auto-align, no forces) ---
  const frontAW = apparentWindAtOffset(Vax, Vay, x_front);
  const frontBeta = Math.atan2(frontAW.Vaby, frontAW.Vabx);
  const frontWingAngle = frontBeta;  // auto-align

  // --- Rear sail (user-controlled) ---
  const rearAW = apparentWindAtOffset(Vax, Vay, x_rear);
  const rearForces = computeWingForces(
    rearAW.Vabx, rearAW.Vaby,
    rearWingAngle,
    Aw, CLalpha, CLmax, CD0, k_induced
  );

  debugAlpha = rearForces.alpha;
  debugLift = rearForces.L;
  debugDrag = rearForces.D;

  // Rear sail moment
  const M_rear = x_rear * rearForces.Fy;

  // --- Hull hydrodynamics ---
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

  const M_hull = -Cr * r * Math.abs(r);

  // --- Total forces ---
  const Fx = rearForces.Fx + F_hull_x;
  const Fy = rearForces.Fy + F_hull_y;
  const Mz = M_rear + M_hull;

  // Convert to world frame for drawing
  Fwx = Fx * Math.cos(psi) - Fy * Math.sin(psi);
  Fwy = Fx * Math.sin(psi) + Fy * Math.cos(psi);

  // Integrate
  u += (Fx / m) * dt;
  v += (Fy / m) * dt;
  r += (Mz / Iz) * dt;

  psi = wrapAngle(psi + r * dt);
  x += (u * Math.cos(psi) - v * Math.sin(psi)) * dt;
  y += (u * Math.sin(psi) + v * Math.cos(psi)) * dt;

  // Wrap world
  if (x < 0) x += canvas.width;
  if (x > canvas.width) x -= canvas.width;
  if (y < 0) y += canvas.height;
  if (y > canvas.height) y -= canvas.height;

  // Gauges
  speedGauge.textContent = Math.hypot(u, v).toFixed(2);
  headingGauge.textContent = (psi * 180 / Math.PI).toFixed(1);

  // Trail
  trailTimer += dt;
  if (trailTimer >= 1.0) {
    trailTimer = 0;
    trail.push({ x, y });
    if (trail.length > TRAIL_LENGTH) trail.shift();
  }

  // Store front sail angle for drawing
  step.frontWingAngle = frontWingAngle;
}
step.frontWingAngle = 0;

function drawBoat() {
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(psi);

  // Hull
  ctx.fillStyle = '#444';
  ctx.beginPath();
  ctx.moveTo(30, 0);
  ctx.lineTo(0, -8);
  ctx.lineTo(-30, 0);
  ctx.lineTo(0, 8);
  ctx.closePath();
  ctx.fill();

  // Front sail
  ctx.save();
  ctx.translate(x_front * 50, 0);
  ctx.rotate(step.frontWingAngle);
  ctx.fillStyle = '#dddddd';
  ctx.beginPath();
  ctx.moveTo(0, -20);
  ctx.lineTo(5, 20);
  ctx.lineTo(-5, 20);
  ctx.closePath();
  ctx.fill();
  ctx.restore();

  // Rear sail
  ctx.save();
  ctx.translate(x_rear * 50, 0);
  ctx.rotate(rearWingAngle);
  ctx.fillStyle = '#ffffff';
  ctx.beginPath();
  ctx.moveTo(0, -25);
  ctx.lineTo(5, 25);
  ctx.lineTo(-5, 25);
  ctx.closePath();
  ctx.fill();
  ctx.restore();

  ctx.restore();
}

function drawForceVector() {
  const scale = 0.2;
  const endX = x + Fwx * scale;
  const endY = y + Fwy * scale;

  ctx.save();
  ctx.strokeStyle = '#00aa00';
  ctx.lineWidth = 3;

  ctx.beginPath();
  ctx.moveTo(x, y);
  ctx.lineTo(endX, endY);
  ctx.stroke();

  ctx.restore();
}

function drawDebugText() {
  ctx.save();
  ctx.font = '14px monospace';

  const stallAoA = CLmax / CLalpha;
  const aoaColor = Math.abs(debugAlpha) > Math.abs(stallAoA) ? '#cc0000' : '#009900';

  ctx.fillStyle = aoaColor;
  ctx.fillText(`AoA: ${(debugAlpha * 180/Math.PI).toFixed(1)}°`, 10, canvas.height - 60);

  ctx.fillStyle = '#000';
  ctx.fillText(`Lift: ${debugLift.toFixed(2)} N`, 10, canvas.height - 40);
  ctx.fillText(`Drag: ${debugDrag.toFixed(2)} N`, 10, canvas.height - 20);

  ctx.restore();
}

function drawGrid() {
  ctx.save();
  ctx.strokeStyle = 'rgba(255,255,255,0.25)';
  for (let xg = 0; xg <= canvas.width; xg += 40) {
    ctx.beginPath(); ctx.moveTo(xg, 0); ctx.lineTo(xg, canvas.height); ctx.stroke();
  }
  for (let yg = 0; yg <= canvas.height; yg += 40) {
    ctx.beginPath(); ctx.moveTo(0, yg); ctx.lineTo(canvas.width, yg); ctx.stroke();
  }
  ctx.restore();
}

function drawTrail() {
  ctx.save();
  ctx.fillStyle = 'rgba(255,0,0,0.4)';
  for (const p of trail) {
    ctx.beginPath();
    ctx.arc(p.x, p.y, 3, 0, 2 * Math.PI);
    ctx.fill();
  }
  ctx.restore();
}

function render() {
  step();
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  drawGrid();
  drawTrail();
  drawBoat();
  drawForceVector();
  drawDebugText();
  requestAnimationFrame(render);
}

render();

// ===== Vector3 & KalmanFilter =====
class Vector3 {
  constructor(x = 0, y = 0, z = 0) { this.x = x; this.y = y; this.z = z; }
  add(v) { return new Vector3(this.x + v.x, this.y + v.y, this.z + v.z); }
  subtract(v) { return new Vector3(this.x - v.x, this.y - v.y, this.z - v.z); }
  multiplyScalar(s) { return new Vector3(this.x * s, this.y * s, this.z * s); }
  length() { return Math.sqrt(this.x ** 2 + this.y ** 2 + this.z ** 2); }
  normalize() { const len = this.length(); return len > 0 ? this.multiplyScalar(1 / len) : new Vector3(); }
  clone() { return new Vector3(this.x, this.y, this.z); }
  static zero() { return new Vector3(0, 0, 0); }
}

class KalmanFilter {
  constructor(R = 0.005, Q = 0.00005) {
    this.R = R; this.Q = Q;
    this.A = 1; this.C = 1;
    this.cov = NaN; this.x = NaN;
  }
  filter(z) {
    if (isNaN(this.x)) {
      this.x = z; this.cov = this.R;
    } else {
      const predX = this.A * this.x;
      const predCov = this.cov + this.Q;
      const K = predCov / (predCov + this.R);
      this.x = predX + K * (z - predX);
      this.cov = predCov - K * predCov;
    }
    return this.x;
  }
}

// ===== Aim System Smooth + No Shake =====
class SmoothHeadDragAim {
  constructor() {
    this.kalmanX = new KalmanFilter();
    this.kalmanY = new KalmanFilter();
    this.kalmanZ = new KalmanFilter();
    this.dragSensitivity = 5.0; // max drag
    this.prevAim = Vector3.zero();
  }

  updateHeadTarget(headPos) {
    const filtered = new Vector3(
      this.kalmanX.filter(headPos.x),
      this.kalmanY.filter(headPos.y),
      this.kalmanZ.filter(headPos.z)
    );

    const aimDelta = filtered.subtract(this.prevAim).multiplyScalar(this.dragSensitivity);
    const nextAim = this.prevAim.add(aimDelta);

    this.prevAim = nextAim;
    this.setCrosshair(nextAim);
  }

  setCrosshair(v) {
    console.log(`🎯 [SmoothDrag] Aim to Head: (${v.x.toFixed(5)}, ${v.y.toFixed(5)}, ${v.z.toFixed(5)})`);
    // GameAPI.setCrosshairTarget(v.x, v.y, v.z); // optional real API
  }

  loop(headSourceFunction) {
    const tick = () => {
      const headPos = headSourceFunction();
      this.updateHeadTarget(headPos);
      setTimeout(tick, 16); // ~60FPS
    };
    tick();
  }
}

// ===== Bone Head Source (with bindpose + transform) =====
function getBoneHeadWorldPos() {
  const pos = { x: -0.0456970781, y: -0.004478302, z: -0.0200432576 };
  const rot = { x: 0.0258174837, y: -0.08611039, z: -0.1402113, w: 0.9860321 };
  const scale = { x: 0.99999994, y: 1.00000012, z: 1.0 };
  const bind = {
    e00: -1.34559613e-13, e01: 8.881784e-14, e02: -1.0, e03: 0.487912,
    e10: -2.84512817e-6,  e11: -1.0,         e12: 8.881784e-14, e13: -2.842171e-14,
    e20: -1.0,            e21: 2.84512817e-6,e22: -1.72951931e-13, e23: 0.0,
    e30: 0.0,             e31: 0.0,          e32: 0.0,             e33: 1.0
  };

  const m = quaternionToMatrix(rot);
  const local = [
    m[0] * scale.x, m[1] * scale.y, m[2] * scale.z, pos.x,
    m[4] * scale.x, m[5] * scale.y, m[6] * scale.z, pos.y,
    m[8] * scale.x, m[9] * scale.y, m[10] * scale.z, pos.z,
    0, 0, 0, 1
  ];

  const bindMatrix = [
    bind.e00, bind.e01, bind.e02, bind.e03,
    bind.e10, bind.e11, bind.e12, bind.e13,
    bind.e20, bind.e21, bind.e22, bind.e23,
    bind.e30, bind.e31, bind.e32, bind.e33
  ];

  return multiplyMatrixVec(bindMatrix, new Vector3(local[3], local[7], local[11]));
}

// ===== Quaternion to Matrix =====
function quaternionToMatrix(q) {
  const { x, y, z, w } = q;
  return [
    1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w,     2*x*z + 2*y*w,     0,
    2*x*y + 2*z*w,     1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w,     0,
    2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y, 0,
    0,                 0,                 0,                 1
  ];
}

function multiplyMatrixVec(m, v) {
  return new Vector3(
    m[0] * v.x + m[1] * v.y + m[2] * v.z + m[3],
    m[4] * v.x + m[5] * v.y + m[6] * v.z + m[7],
    m[8] * v.x + m[9] * v.y + m[10] * v.z + m[11]
  );
}

// ===== Khởi chạy hệ thống drag ổn định vào bone_Head =====
const smoothAimSystem = new SmoothHeadDragAim();
smoothAimSystem.loop(getBoneHeadWorldPos);

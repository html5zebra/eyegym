// ===== Volleyball serve physics (SI units) =====
// World: net plane z = 0; opponent court z ∈ [-9, 0]; our court z ∈ [0, 9]; ball travels toward +z.
const VB = { M: 0.27, R: 0.105, RHO: 1.15, G: 9.81, DT: 1 / 240 };
VB.A = Math.PI * VB.R * VB.R;
// Drag crisis: ~0.47 when slow, ~0.22 when fast (transition around 17 m/s ≈ 61 km/h).
// This is why a float serve flies "flat" and then suddenly drops as it slows down.
const vbCd = v => 0.22 + 0.25 / (1 + Math.exp((v - 17) / 1.3));
function vbNoise(seed) {
  const r = (k) => { const x = Math.sin(seed * 12.9898 + k * 78.233) * 43758.5453; return x - Math.floor(x); };
  const comps = [0, 1, 2].map(k => ({ f: 0.9 + r(k) * 2.2, p: r(k + 7) * Math.PI * 2, a: 0.5 + r(k + 13) }));
  const sum = comps.reduce((a, c) => a + c.a, 0);
  return t => comps.reduce((a, c) => a + c.a * Math.sin(2 * Math.PI * c.f * t + c.p), 0) / sum;
}
function vbAccel(s, t, sv, out) {
  const vx = s[3], vy = s[4], vz = s[5];
  const v = Math.hypot(vx, vy, vz) || 1e-6;
  const q = 0.5 * VB.RHO * VB.A * v * v / VB.M;            // dynamic pressure × area / mass
  let ax = -q * vbCd(v) * vx / v, ay = -q * vbCd(v) * vy / v - VB.G, az = -q * vbCd(v) * vz / v;
  const w = sv.spin;                                        // spin vector, rad/s
  const wm = Math.hypot(w[0], w[1], w[2]);
  if (wm > 0.01) {                                          // Magnus: F ∝ ω × v
    const S = VB.R * wm / v, Cl = Math.min(0.32, 1.1 * S);
    let cx = w[1] * vz - w[2] * vy, cy = w[2] * vx - w[0] * vz, cz = w[0] * vy - w[1] * vx;
    const cm = Math.hypot(cx, cy, cz) || 1e-6;
    ax += q * Cl * cx / cm; ay += q * Cl * cy / cm; az += q * Cl * cz / cm;
  }
  if (sv.knuckle > 0) {                                     // float serve: unsteady side/vertical forces
    const amp = sv.knuckle * (1 + 1.5 * Math.exp(-Math.pow((v - 17) / 3, 2)));
    ax += q * amp * sv.nx(t); ay += q * amp * 0.6 * sv.ny(t);
  }
  out[0] = vx; out[1] = vy; out[2] = vz; out[3] = ax; out[4] = ay; out[5] = az;
}
// Integrate one serve (RK4). Returns sampled path, landing point/time, net contact.
function vbSimulate(p0, v0, sv, netH) {
  const s = [p0[0], p0[1], p0[2], v0[0], v0[1], v0[2]];
  const k1 = new Array(6), k2 = new Array(6), k3 = new Array(6), k4 = new Array(6), tmp = new Array(6);
  const pts = [[0, s[0], s[1], s[2]]];
  let t = 0, net = false, crossed = false;
  const dt = VB.DT;
  for (let i = 0; i < 2400; i++) {
    vbAccel(s, t, sv, k1);
    for (let j = 0; j < 6; j++) tmp[j] = s[j] + k1[j] * dt / 2; vbAccel(tmp, t + dt / 2, sv, k2);
    for (let j = 0; j < 6; j++) tmp[j] = s[j] + k2[j] * dt / 2; vbAccel(tmp, t + dt / 2, sv, k3);
    for (let j = 0; j < 6; j++) tmp[j] = s[j] + k3[j] * dt; vbAccel(tmp, t + dt, sv, k4);
    const prevZ = s[2], prevY = s[1];
    for (let j = 0; j < 6; j++) s[j] += dt / 6 * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]);
    t += dt;
    if (!crossed && prevZ < 0 && s[2] >= 0) {
      crossed = true;
      const yAtNet = prevY + (s[1] - prevY) * (0 - prevZ) / (s[2] - prevZ);
      if (yAtNet < netH + VB.R && Math.abs(s[0]) < 5) net = true;
    }
    if (s[1] <= VB.R) {                                     // floor contact: interpolate exact landing
      const f = (pts[pts.length - 1][2] - VB.R) / (pts[pts.length - 1][2] - s[1]);
      const prev = pts[pts.length - 1];
      const land = { t: prev[0] + f * dt, x: prev[1] + f * (s[0] - prev[1]), z: prev[3] + f * (s[2] - prev[3]) };
      pts.push([land.t, land.x, VB.R, land.z]);
      return { pts, land, net, crossed, v: [s[3], s[4], s[5]] };
    }
    pts.push([t, s[0], s[1], s[2]]);
  }
  return { pts, land: null, net, crossed, v: [s[3], s[4], s[5]] };
}
// Serve types. Speeds are at contact; tuned for recreational-to-strong amateur play.
const VB_TYPES = {
  float:   { name: 'планер',   v: [13, 19],   spin: 0,   knuckle: 0.15, h: [2.15, 2.75], zc: [-9.45, -9.05], maxApex: 4.6 },
  topspin: { name: 'крученая', v: [16.5, 24], spin: 38,  knuckle: 0,    h: [2.75, 3.15], zc: [-9.15, -8.75], maxApex: 4.8 },
  lob:     { name: 'свеча',    v: [12.5, 15.5], spin: -6, knuckle: 0,   h: [1.0, 1.2], zc: [-9.5, -9.2], high: true, maxApex: 10 },
};
// Pick a target, then solve launch elevation (bisection) and aim so the ball lands there.
function vbMakeServe(opts, rnd = Math.random) {
  const lvl = { easy: 0.88, mid: 1, hard: 1.12 }[opts.level] || 1;
  for (let attempt = 0; attempt < 60; attempt++) {
    const pool = opts.types === 'all' ? ['float', 'float', 'topspin', 'lob'] : [opts.types];
    const key = pool[Math.floor(rnd() * pool.length)], T = VB_TYPES[key];
    const out = rnd() < (opts.outShare ?? 0.15);
    let tx, tz;
    if (!out) { tx = (rnd() * 2 - 1) * 4.2; tz = 0.8 + rnd() * 8.0; }
    else if (rnd() < 0.6) { tx = (rnd() * 2 - 1) * 4; tz = 9.3 + rnd() * 1.6; }          // long
    else { tx = (rnd() < 0.5 ? -1 : 1) * (4.75 + rnd() * 1.2); tz = 2 + rnd() * 6.5; } // wide
    const p0 = [(rnd() * 2 - 1) * 3.5, T.h[0] + rnd() * (T.h[1] - T.h[0]), T.zc[0] + rnd() * (T.zc[1] - T.zc[0])];
    const speed = (T.v[0] + rnd() * (T.v[1] - T.v[0])) * lvl;
    const seed = rnd() * 1000;
    const sv = { spin: [T.spin, 0, 0], knuckle: T.knuckle, nx: vbNoise(seed), ny: vbNoise(seed + 50) };
    let az = Math.atan2(tx - p0[0], tz - p0[2]);
    const want = Math.hypot(tx - p0[0], tz - p0[2]);
    const launch = (el, a) => [speed * Math.cos(el) * Math.sin(a), speed * Math.sin(el), speed * Math.cos(el) * Math.cos(a)];
    const dist = r => r.land ? Math.hypot(r.land.x - p0[0], r.land.z - p0[2]) : 1e9;
    const noKnuckle = { ...sv, knuckle: 0 };                 // aim as if no wobble; the wobble is the surprise
    // range peaks at some elevation: find it, then bisect on the low branch (driven serves)
    // or on the high branch (lobs), where distance changes monotonically
    let eMax = -0.25, dMax = -1;
    for (let e = -0.25; e <= 1.3; e += 0.1) { const d = dist(vbSimulate(p0, launch(e, az), noKnuckle, opts.netH)); if (d < 1e8 && d > dMax) { dMax = d; eMax = e; } }
    if (dMax < want) continue;                                    // too slow to reach the target
    let lo, hi, res = null;
    if (T.high) { lo = eMax; hi = 1.35; if (dist(vbSimulate(p0, launch(hi, az), noKnuckle, opts.netH)) > want) continue; }
    else { lo = -0.25; hi = eMax; if (dist(vbSimulate(p0, launch(lo, az), noKnuckle, opts.netH)) > want) continue; }
    for (let i = 0; i < 22; i++) {
      const mid = (lo + hi) / 2;
      const short = dist(vbSimulate(p0, launch(mid, az), noKnuckle, opts.netH)) < want;
      if (T.high ? !short : short) lo = mid; else hi = mid;
    }
    const el = (lo + hi) / 2;
    res = vbSimulate(p0, launch(el, az), sv, opts.netH);
    if (!res.land || res.net || !res.crossed) continue;
    const landedOut = Math.abs(res.land.x) > 4.5 + VB.R || res.land.z > 9 + VB.R;
    const apex = Math.max(...res.pts.map(p => p[2]));
    if (apex > T.maxApex) continue;
    return { key, name: T.name, speed, kmh: Math.round(speed * 3.6), p0, el, az, sv, res, apex, out: landedOut,
             target: { x: tx, z: tz } };
  }
  return null;
}

// =====================================================================
// Scene: the hall, the server with joints, the ball, sounds, replay
// =====================================================================
const RECV = { x: 0, z: 7.2, eye: 1.45 };
const VBT = { THINK: 1.4, REVEAL: 5.6, GAP: 0.6, EYE: 0.034 };
const VB_POS = { left: -2.6, center: 0, right: 2.6 };
let VBS = null;
const vLerp = (a, b, k) => a + (b - a) * k;
const vSm = k => (k <= 0 ? 0 : k >= 1 ? 1 : k * k * (3 - 2 * k));
function vKf(track, t) {
  const n = track.length;
  if (t <= track[0][0]) return track[0][1];
  if (t >= track[n - 1][0]) return track[n - 1][1];
  for (let i = 1; i < n; i++) if (t < track[i][0]) { const a = track[i - 1], b = track[i]; return vLerp(a[1], b[1], vSm((t - a[0]) / (b[0] - a[0]))); }
  return track[n - 1][1];
}
function vbTex(w, h, draw, rx, ry) {
  const c = document.createElement('canvas'); c.width = w; c.height = h; draw(c.getContext('2d'), w, h);
  const t = new THREE.CanvasTexture(c);
  if (rx) { t.wrapS = t.wrapT = THREE.RepeatWrapping; t.repeat.set(rx, ry); }
  t.encoding = THREE.sRGBEncoding;
  t.anisotropy = 4;
  return t;
}

// ---------- A person built from capsules, with shoulders, elbows, hips and knees ----------
// Faces +z (toward our court), so the right arm is at −x.
function vbFigure(o) {
  const T = THREE, g = new T.Group();
  const M = c => new T.MeshStandardMaterial({ color: c, roughness: 0.85 });
  const mJ = M(o.jersey), mS = M(o.shorts), mSk = M(o.skin), mSh = M(0xf2f2f2), mPad = M(0x15171c), mHair = M(o.hair || 0x1a1410), mSole = M(0x2a2c31);
  mSk.roughness = 0.6;
  const limb = (len, r, mat) => { const m = new T.Mesh(new T.CapsuleGeometry(r, Math.max(0.01, len - 2 * r), 4, 10), mat); m.position.y = -len / 2; m.castShadow = true; return m; };
  const hips = new T.Group(); hips.position.y = 0.96; g.add(hips);
  const pelvis = new T.Mesh(new T.CapsuleGeometry(0.13, 0.14, 4, 12), mS); pelvis.rotation.z = Math.PI / 2; pelvis.castShadow = true; hips.add(pelvis);
  const spine = new T.Group(); hips.add(spine);
  const torso = new T.Mesh(new T.CapsuleGeometry(0.165, 0.3, 4, 14), mJ); torso.position.y = 0.3; torso.scale.set(1.18, 1, 0.72); torso.castShadow = true; spine.add(torso);
  const yoke = new T.Mesh(new T.CapsuleGeometry(0.075, 0.3, 4, 10), mJ); yoke.rotation.z = Math.PI / 2; yoke.position.y = 0.52; yoke.castShadow = true; spine.add(yoke);
  const neck = new T.Mesh(new T.CylinderGeometry(0.045, 0.05, 0.1, 10), mSk); neck.position.y = 0.6; spine.add(neck);
  const nose = new T.Mesh(new T.SphereGeometry(0.018, 8, 6), mSk); nose.position.set(0, 0.715, 0.1); spine.add(nose);
  [-1, 1].forEach(sd => { const ear = new T.Mesh(new T.SphereGeometry(0.02, 8, 6), mSk); ear.position.set(sd * 0.097, 0.72, 0); spine.add(ear); });
  const head = new T.Mesh(new T.SphereGeometry(0.105, 18, 14), mSk); head.position.y = 0.72; head.scale.set(0.92, 1.08, 1); head.castShadow = true; spine.add(head);
  const hair = new T.Mesh(new T.SphereGeometry(0.11, 18, 10, 0, Math.PI * 2, 0, Math.PI * 0.55), mHair); hair.position.set(0, 0.735, -0.012); spine.add(hair);
  const arm = side => {
    const sh = new T.Group(); sh.position.set(side * 0.215, 0.53, 0); spine.add(sh);
    sh.add(new T.Mesh(new T.SphereGeometry(0.068, 12, 10), mJ));
    sh.add(limb(0.33, 0.05, mSk));
    const sleeve = new T.Mesh(new T.CapsuleGeometry(0.058, 0.07, 4, 10), mJ); sleeve.position.y = -0.085; sleeve.castShadow = true; sh.add(sleeve);
    const elb = new T.Group(); elb.position.y = -0.33; sh.add(elb);
    const fore = new T.Mesh(new T.CylinderGeometry(0.044, 0.033, 0.27, 10), mSk); fore.position.y = -0.135; fore.castShadow = true; elb.add(fore);
    elb.add(new T.Mesh(new T.SphereGeometry(0.045, 10, 8), mSk));
    // open palm: the reference point for contact stays at the palm centre
    const hand = new T.Mesh(new T.BoxGeometry(0.085, 0.1, 0.03), mSk); hand.position.y = -0.33; hand.castShadow = true; elb.add(hand);
    const fingers = new T.Mesh(new T.BoxGeometry(0.08, 0.085, 0.022), mSk); fingers.position.y = -0.09; hand.add(fingers);
    const thumb = new T.Mesh(new T.BoxGeometry(0.025, 0.06, 0.022), mSk); thumb.position.set(-side * 0.052, -0.01, 0.005); thumb.rotation.z = -side * 0.5; hand.add(thumb);
    return { sh, elb, hand };
  };
  const leg = side => {
    const hp = new T.Group(); hp.position.set(side * 0.1, -0.03, 0); hips.add(hp);
    hp.add(limb(0.46, 0.078, mSk));
    const sho = new T.Mesh(new T.CapsuleGeometry(0.088, 0.12, 4, 10), mS); sho.position.y = -0.12; hp.add(sho);
    const kn = new T.Group(); kn.position.y = -0.46; hp.add(kn);
    const pad = new T.Mesh(new T.SphereGeometry(0.072, 10, 8), mPad); pad.position.z = 0.012; kn.add(pad);
    const shin = new T.Mesh(new T.CylinderGeometry(0.06, 0.04, 0.4, 10), mSk); shin.position.y = -0.2; shin.castShadow = true; kn.add(shin);
    const sock = new T.Mesh(new T.CylinderGeometry(0.046, 0.044, 0.1, 10), mSh); sock.position.y = -0.37; kn.add(sock);
    // sneaker: upper + toe + dark sole (the sole is the lowest point, used to stand on the floor)
    const foot = new T.Group(); foot.position.set(0, -0.455, 0.06); kn.add(foot);
    const upper = new T.Mesh(new T.BoxGeometry(0.095, 0.06, 0.2), mSh); upper.position.set(0, 0.008, -0.02); upper.castShadow = true; foot.add(upper);
    const toe = new T.Mesh(new T.SphereGeometry(0.05, 10, 8), mSh); toe.scale.set(0.95, 0.6, 1.25); toe.position.set(0, -0.004, 0.085); foot.add(toe);
    const sole = new T.Mesh(new T.BoxGeometry(0.1, 0.018, 0.27), mSole); sole.position.set(0, -0.031, 0.005); foot.add(sole);
    return { hp, kn, foot };
  };
  const R = arm(-1), L = arm(1), lr = leg(-1), ll = leg(1);
  return { g, hips, spine, shR: R.sh, elR: R.elb, handR: R.hand, shL: L.sh, elL: L.elb, handL: L.hand,
           hipR: lr.hp, knR: lr.kn, footR: lr.foot, hipL: ll.hp, knL: ll.kn, footL: ll.foot };
}
const _vbBox = new THREE.Box3();
function vbApplyPose(f, p) {
  f.g.position.set(p.x, 0, p.z);
  f.g.rotation.y = p.face || 0;
  f.spine.rotation.set(p.lean || 0, p.twist || 0, 0);
  f.shR.rotation.set(p.shR || 0, 0, p.shRz || 0); f.elR.rotation.x = p.elR || 0;
  f.shL.rotation.set(p.shL || 0, 0, p.shLz || 0); f.elL.rotation.x = p.elL || 0;
  f.hipR.rotation.x = p.hipR || 0; f.knR.rotation.x = p.knR || 0;
  f.hipL.rotation.x = p.hipL || 0; f.knL.rotation.x = p.knL || 0;
  f.g.updateMatrixWorld(true);
  const lowFoot = Math.min(_vbBox.setFromObject(f.footR).min.y, _vbBox.setFromObject(f.footL).min.y);
  f.g.position.y = -lowFoot + (p.air || 0);                 // grounded: soles exactly on the floor
  f.g.updateMatrixWorld(true);
}
const VB_READY = (x, z, face) => ({ x, z, face, lean: 0.38, shR: -0.75, elR: -0.75, shL: -0.75, elL: -0.75, shRz: 0.12, shLz: -0.12,
  hipR: -0.75, knR: 1.05, hipL: -0.75, knL: 1.05 });

// ---------- Serve technique: a different motion for each serve ----------
const VB_ARM = 0.66;
function vbServeTracks(an) {
  const T = an.T, rel = an.rel, top = an.top, jf = an.jf, lob = an.lob, cA = an.cA;
  const swingT = lob ? -0.32 : -0.2, run = an.run;
  const tr = {
    z: [[-T, -run], [an.tStart, -run], [an.tPlant, -0.12], [0, 0], [0.4, top ? 0.75 : jf ? 0.4 : 0.08]],
    lean: lob ? [[-T, 0.05], [swingT, 0.28], [0, 0.32], [0.35, 0.18]]
              : [[-T, 0.04], [swingT - 0.15, top ? -0.22 : -0.1], [0, 0.16], [0.3, top ? 0.5 : 0.14]],
    twist: lob ? [[-T, 0], [swingT, -0.35], [0, 0.1], [0.3, 0.25]]
               : [[-T, 0], [swingT - 0.12, top ? -0.7 : -0.45], [0, 0.18], [0.28, top ? 0.6 : 0.25]],
    elR: lob ? [[-T, -0.15], [0, -0.08], [0.3, -0.3]]
             : [[-T, -1.3], [rel, -1.2], [swingT - 0.1, -1.7], [0, -0.06], [0.25, top ? -0.5 : -0.12]],
    kn: top || jf ? [[-T, 0.15], [an.tPlant - 0.12, 0.3], [an.tPlant, 1.0], [-0.12, 0.3], [0, 0.55], [0.3, 0.3], [0.38, 0.9], [0.75, 0.2]]
                  : lob ? [[-T, 0.3], [swingT, 0.55], [0, 0.45], [0.4, 0.3]] : [[-T, 0.15], [-0.35, 0.4], [0, 0.12], [0.4, 0.2]],
  };
  if (lob) {         // underhand: arm swings back, then forward under the ball at the waist
    tr.shR = [[-T, -0.2], [rel, -0.3], [swingT, 1.0], [0, cA], [0.3, -1.7]];
    tr.shL = [[-T, -0.9], [rel, -0.95], [-0.06, -0.85], [0.15, 0.2], [0.4, 0.4]];
    tr.elL = [[-T, -0.6], [0, -0.55], [0.3, -0.3]];
  } else if (top) {  // power serve: own toss with the hitting hand, three-step approach, full swing
    tr.shR = [[-T, -0.95], [rel - 0.32, -0.8], [rel, -2.35], [rel + 0.35, -0.7], [an.tPlant - 0.05, 0.5], [swingT, -3.75], [0, cA], [0.26, -0.55]];
    tr.shL = [[-T, -0.95], [rel, -1.6], [rel + 0.35, -0.4], [an.tPlant - 0.05, 0.6], [swingT, -2.9], [0, -0.7], [0.3, 0.3]];
    tr.elL = [[-T, -1.3], [rel, -0.9], [an.tPlant, -0.3], [0, -0.7]];
  } else {           // float: low toss with the left hand, short punch, the arm stops at contact
    tr.shR = [[-T, -0.95], [rel, -1.1], [swingT - 0.05, -3.6], [0, cA], [0.25, cA + 0.42]];
    tr.shL = [[-T, -0.95], [rel - 0.28, -0.85], [rel, -2.45], [swingT, -2.85], [0, -0.8], [0.3, 0.1]];
    tr.elL = [[-T, -1.3], [rel, -0.1], [0, -0.6]];
  }
  an.tr = tr;
}
function vbServeAnim(sv, reach) {
  const top = sv.key === 'topspin', lob = sv.key === 'lob';
  const jump = lob ? 0 : Math.max(0, sv.p0[1] - (reach + 0.04));   // flat-foot reach + rising on the toes
  const jf = !top && !lob && jump > 0.1;
  const an = { top, lob, jf, jump, run: top ? 2.7 : jf ? 1.3 : 0.3,
    T: top ? 2.5 : jf ? 2.1 : lob ? 1.8 : 1.7, rel: top ? -1.6 : jf ? -1.1 : lob ? -0.38 : -0.62,
    cA: lob ? -0.45 : -2.95, dx: 0, dz: 0, tossHand: top ? 'R' : 'L' };
  an.tStart = an.rel + 0.12; an.tPlant = top || jf ? -0.3 : -0.12; an.steps = top ? 3 : jf ? 2 : 1;
  an.jumpy = top || jf;
  vbServeTracks(an);
  return an;
}
// Move the body, set the jump height and the arm angle until the palm meets the ball at contact.
function vbCalibrate(sv, fig) {
  const an = sv.anim, h = new THREE.Vector3();
  an.dx = sv.p0[0] + 0.215; an.dz = sv.p0[2] - 0.4;
  for (let i = 0; i < 12; i++) {
    vbApplyPose(fig, vbServePose(an, 0));
    fig.handR.getWorldPosition(h);
    const tx = sv.p0[0], ty = sv.p0[1] - (an.lob ? VB.R + 0.03 : VB.R * 0.55), tz = sv.p0[2] - (an.lob ? 0.02 : VB.R * 0.7);
    const ex = tx - h.x, ey = ty - h.y, ez = tz - h.z;
    an.dx += ex; an.dz += ez;
    if (an.jumpy) an.jump = Math.max(0.05, an.jump + ey);
    else if (ey > 0 && !an.lob) an.jump = Math.min(0.12, an.jump + ey);          // rising on the toes
    else {                                                                       // reach lower: open the arm angle
      const s = Math.sin(an.cA);
      if (Math.abs(s) > 0.05) an.cA += ey / (VB_ARM * s) * 0.8;
      an.cA = an.lob ? Math.min(0.3, Math.max(-1.7, an.cA)) : Math.min(-2.0, Math.max(-Math.PI, an.cA));
    }
    vbServeTracks(an);
  }
  vbApplyPose(fig, vbServePose(an, 0)); fig.handR.getWorldPosition(h);
  an.err = Math.hypot(sv.p0[0] - h.x, sv.p0[1] - (an.lob ? VB.R + 0.03 : VB.R * 0.55) - h.y, sv.p0[2] - (an.lob ? 0.02 : VB.R * 0.7) - h.z);
}
function vbServePose(an, t) {
  const tr = an.tr, jumpy = an.jumpy;
  let air = 0;
  if (an.jump > 0) {
    if (jumpy) { if (t > an.tPlant && t < -an.tPlant) { const u = t / an.tPlant; air = an.jump * (1 - u * u); } }  // apex at contact
    else if (t > -0.35 && t < 0.3) air = an.jump * Math.sin(Math.PI * (t + 0.35) / 0.65);                   // toes
  }
  const kn = vKf(tr.kn, t);
  let stride = 0;
  if (t > an.tStart && t < an.tPlant) stride = Math.sin(Math.PI * an.steps * (t - an.tStart) / (an.tPlant - an.tStart)) * (jumpy ? 0.62 : 0.35);
  const inAir = jumpy && t > an.tPlant && t < -an.tPlant ? Math.sin(Math.PI * (t - an.tPlant) / (-2 * an.tPlant)) : 0;
  return {
    x: an.dx, z: an.dz + vKf(tr.z, t), air, face: 0,
    lean: vKf(tr.lean, t), twist: vKf(tr.twist, t),
    shR: vKf(tr.shR, t), elR: vKf(tr.elR, t), shRz: 0.06,
    shL: vKf(tr.shL, t), elL: vKf(tr.elL, t), shLz: -0.06,
    hipR: -kn * 0.5 - stride + inAir * 0.25, knR: kn + Math.max(0, stride) * 0.9 + inAir * 0.9,
    hipL: -kn * 0.5 + stride + inAir * 0.1, knL: kn + Math.max(0, -stride) * 0.9 + inAir * 0.5,
  };
}

// ---------- The hall ----------
function vbBuildHall(scene) {
  const T = THREE, HW = 13, HL = 19, WH = 7.6;
  // dark glossy wood floor
  const wood = vbTex(512, 512, (c, w, h) => {
    c.fillStyle = '#4a3325'; c.fillRect(0, 0, w, h);
    for (let i = 0; i < 16; i++) {
      const x = i * w / 16, tone = 0.86 + ((i * 37) % 11) / 40;
      c.fillStyle = `rgba(${Math.round(92 * tone)},${Math.round(62 * tone)},${Math.round(42 * tone)},1)`; c.fillRect(x, 0, w / 16 - 1, h);
      c.fillStyle = 'rgba(20,12,8,0.55)'; c.fillRect(x + w / 16 - 1, 0, 1, h);
      for (let k = 0; k < 3; k++) { const y = ((i * 131 + k * 197) % h); c.fillRect(x, y, w / 16 - 1, 1); }
    }
  }, HW * 2 / 4, HL * 2 / 4);
  const floor = new T.Mesh(new T.PlaneGeometry(HW * 2, HL * 2), new T.MeshStandardMaterial({ map: wood, color: 0x9c8f86, roughness: 0.34, metalness: 0.0 }));
  floor.rotation.x = -Math.PI / 2; floor.receiveShadow = true; scene.add(floor);
  // floor lines
  const lineMat = c => new T.MeshBasicMaterial({ color: c });
  const strip = (w, l, x, z, mat, y) => { const m = new T.Mesh(new T.PlaneGeometry(w, l), mat); m.rotation.x = -Math.PI / 2; m.position.set(x, y, z); scene.add(m); };
  const rect = (cx, cz, w, l, th, mat, y) => {
    strip(th, l, cx - w / 2 + th / 2, cz, mat, y); strip(th, l, cx + w / 2 - th / 2, cz, mat, y);
    strip(w, th, cx, cz - l / 2 + th / 2, mat, y); strip(w, th, cx, cz + l / 2 - th / 2, mat, y);
  };
  const dim = lineMat(0xb9b6ae), yel = lineMat(0xc9a51c), white = lineMat(0xfafafa);
  [-8.2, 0, 8.2].forEach(x => { rect(x, 0, 6.1, 13.4, 0.04, dim, 0.003); strip(6.1, 0.04, x, -1.98, dim, 0.003); strip(6.1, 0.04, x, 1.98, dim, 0.003); strip(0.04, 13.4, x, 0, dim, 0.003); });
  rect(0, 0, 15, 28, 0.05, yel, 0.0035);
  rect(0, 0, 9, 18, 0.05, white, 0.005);                 // volleyball court
  strip(9, 0.05, 0, 0, white, 0.005); strip(9, 0.05, 0, -3, white, 0.005); strip(9, 0.05, 0, 3, white, 0.005);
  // walls: blue base, lime green with columns, window band, purple panels
  const wallTex = (rep) => vbTex(1024, 512, (c, w, h) => {
    const Y = m => h - m / WH * h;
    c.fillStyle = '#8cc63f'; c.fillRect(0, 0, w, h);
    c.fillStyle = '#9b7fe6'; c.fillRect(0, 0, w, Y(6.35));                       // translucent panels under the roof
    c.strokeStyle = 'rgba(255,255,255,0.75)'; c.lineWidth = 3;
    for (let x = 0; x <= w; x += w / 6) { c.beginPath(); c.moveTo(x, 0); c.lineTo(x, Y(6.35)); c.stroke(); }
    c.fillStyle = '#74ab2f'; c.fillRect(0, Y(6.35), w, 6);
    for (let i = 0; i < 4; i++) {                                                // window grilles
      const x0 = i * w / 4 + w * 0.03, ww = w / 4 - w * 0.06;
      c.fillStyle = '#3b4038'; c.fillRect(x0, Y(6.15), ww, Y(5.25) - Y(6.15));
      c.strokeStyle = '#a9b3a0'; c.lineWidth = 2;
      for (let k = 0; k <= 6; k++) { const x = x0 + k * ww / 6; c.beginPath(); c.moveTo(x, Y(6.15)); c.lineTo(x, Y(5.25)); c.stroke(); }
      c.beginPath(); c.moveTo(x0, Y(5.7)); c.lineTo(x0 + ww, Y(5.7)); c.stroke();
    }
    c.fillStyle = '#6ea52b';                                                     // columns
    for (let i = 0; i <= 4; i++) c.fillRect(i * w / 4 - 9, Y(6.35), 18, Y(1.25) - Y(6.35));
    c.fillRect(0, Y(4.15), w, 8);
    c.fillStyle = '#3f86c9'; c.fillRect(0, Y(1.25), w, h - Y(1.25));             // blue base
    c.fillStyle = '#2f6aa6'; [0.42, 0.84].forEach(m => c.fillRect(0, Y(m), w, 3));
    c.fillStyle = '#5a9bd8'; c.fillRect(0, Y(1.25), w, 4);
  }, rep, 1);
  const wall = (w, x, z, ry) => {
    const m = new T.Mesh(new T.PlaneGeometry(w, WH), new T.MeshLambertMaterial({ map: wallTex(w / 14) }));
    m.position.set(x, WH / 2, z); m.rotation.y = ry; scene.add(m);
  };
  wall(HW * 2, 0, -HL, 0); wall(HW * 2, 0, HL, Math.PI); wall(HL * 2, -HW, 0, Math.PI / 2); wall(HL * 2, HW, 0, -Math.PI / 2);
  // roof with trusses
  const roofTex = vbTex(256, 256, (c, w, h) => {
    c.fillStyle = '#2b2e33'; c.fillRect(0, 0, w, h);
    c.strokeStyle = '#4a4f57'; c.lineWidth = 2; for (let x = 0; x < w; x += 16) { c.beginPath(); c.moveTo(x, 0); c.lineTo(x, h); c.stroke(); }
    c.strokeStyle = '#1b3d2a'; c.lineWidth = 9; c.beginPath(); c.moveTo(0, h / 2); c.lineTo(w, h / 2); c.stroke();
    c.lineWidth = 4; c.beginPath(); c.moveTo(0, h / 2); c.lineTo(w / 2, 0); c.lineTo(w, h / 2); c.lineTo(w / 2, h); c.closePath(); c.stroke();
  }, 5, 7);
  const roof = new T.Mesh(new T.PlaneGeometry(HW * 2, HL * 2), new T.MeshBasicMaterial({ map: roofTex }));
  roof.rotation.x = Math.PI / 2; roof.position.y = WH + 1.4; scene.add(roof);
  // floodlights on the walls
  const glow = vbTex(128, 128, (c, w, h) => {
    const g = c.createRadialGradient(w / 2, h / 2, 0, w / 2, h / 2, w / 2);
    g.addColorStop(0, 'rgba(255,255,255,1)'); g.addColorStop(0.25, 'rgba(255,255,245,0.85)'); g.addColorStop(1, 'rgba(255,255,240,0)');
    c.fillStyle = g; c.fillRect(0, 0, w, h);
  });
  const glowMat = new T.SpriteMaterial({ map: glow, blending: T.AdditiveBlending, depthWrite: false, toneMapped: false });
  const haloMat = new T.SpriteMaterial({ map: glow, blending: T.AdditiveBlending, depthWrite: false, opacity: 0.22, toneMapped: false });
  const lamp = (x, y, z) => {
    const core = new T.Sprite(glowMat); core.position.set(x, y, z); core.scale.set(0.75, 0.75, 1); scene.add(core);
    const halo = new T.Sprite(haloMat); halo.position.set(x, y, z); halo.scale.set(3.4, 3.4, 1); scene.add(halo);
  };
  // translucent panels under the roof glow with daylight, as in the real hall
  const panelMat = new T.MeshBasicMaterial({ color: 0xb79cff, transparent: true, opacity: 0.55, toneMapped: false });
  const panel = (w, x, z, ry) => { const m = new T.Mesh(new T.PlaneGeometry(w, 1.25), panelMat); m.position.set(x, 6.98, z); m.rotation.y = ry; scene.add(m); };
  panel(HW * 2, 0, -HL + 0.03, 0); panel(HW * 2, 0, HL - 0.03, Math.PI); panel(HL * 2, -HW + 0.03, 0, Math.PI / 2); panel(HL * 2, HW - 0.03, 0, -Math.PI / 2);
  // steel trusses under the roof
  const beamMat = new T.MeshLambertMaterial({ color: 0x1f4a33 });
  for (let z = -15; z <= 15; z += 6) {
    const top = new T.Mesh(new T.BoxGeometry(HW * 2, 0.14, 0.14), beamMat); top.position.set(0, WH + 1.25, z); scene.add(top);
    const bot = new T.Mesh(new T.BoxGeometry(HW * 2, 0.12, 0.12), beamMat); bot.position.set(0, WH + 0.35, z); scene.add(bot);
    for (let x = -HW + 1; x < HW; x += 2) { const d = new T.Mesh(new T.BoxGeometry(0.07, 1.25, 0.07), beamMat); d.position.set(x, WH + 0.8, z); d.rotation.z = (Math.round(x) % 4 === 0 ? 1 : -1) * 0.75; scene.add(d); }
  }
  for (let x = -10; x <= 10; x += 2.9) lamp(x, 5.0, -HL + 0.15);
  [-12, -4, 4, 12].forEach(z => lamp(HW - 0.15, 5.0, z));   // none on the wall behind the replay camera
  return { HW, HL, WH };
}

function vbBuildScene() {
  const T = THREE;
  const renderer = new T.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
  renderer.setPixelRatio(1);
  renderer.shadowMap.enabled = true; renderer.shadowMap.type = T.PCFSoftShadowMap; renderer.shadowMap.autoUpdate = false;
  if (T.ColorManagement) T.ColorManagement.legacyMode = false;
  renderer.outputEncoding = T.sRGBEncoding; renderer.toneMapping = T.ACESFilmicToneMapping; renderer.toneMappingExposure = 0.95;
  const scene = new T.Scene(); scene.background = new T.Color(0x1d2024); scene.fog = new T.Fog(0x2b3038, 26, 80);
  scene.add(new T.HemisphereLight(0xf4f0ff, 0x5a4332, 0.48));
  scene.add(new T.AmbientLight(0xffffff, 0.1));
  [[-7, -18.4], [0, -18.4], [7, -18.4], [12.4, -8], [12.4, 6], [-12.4, -8], [-12.4, 6]].forEach(([x, z]) => {
    const pl = new T.PointLight(0xfff4de, 0.16, 60, 1.3); pl.position.set(x, 5, z); scene.add(pl);
  });
  const sun = new T.DirectionalLight(0xfff6e6, 0.8); sun.position.set(2.5, 18, 3.5); sun.castShadow = true;
  sun.shadow.mapSize.set(1024, 1024);
  Object.assign(sun.shadow.camera, { left: -9, right: 9, top: 15, bottom: -15, near: 2, far: 30 });
  sun.shadow.bias = -0.0006; sun.shadow.radius = 3;
  scene.add(sun); scene.add(sun.target);
  vbBuildHall(scene);
  // net: padded posts, top tape, mesh, antennas
  const netH = cfg.netH;
  const padTex = vbTex(16, 64, (c, w, h) => { c.fillStyle = '#f1f1f1'; c.fillRect(0, 0, w, h); c.fillStyle = '#16181c'; [8, 30, 50].forEach(y => c.fillRect(0, y, w, 6)); });
  [-5.05, 5.05].forEach(x => {
    const p = new T.Mesh(new T.CylinderGeometry(0.07, 0.07, netH + 0.12, 14), new T.MeshLambertMaterial({ map: padTex }));
    p.position.set(x, (netH + 0.12) / 2, 0); p.castShadow = true; scene.add(p);
  });
  const tape = new T.Mesh(new T.BoxGeometry(10.1, 0.07, 0.02), new T.MeshLambertMaterial({ color: 0xffffff })); tape.position.set(0, netH - 0.035, 0); scene.add(tape);
  const tape2 = new T.Mesh(new T.BoxGeometry(10.1, 0.04, 0.02), new T.MeshLambertMaterial({ color: 0xffffff })); tape2.position.set(0, netH - 0.98, 0); scene.add(tape2);
  // thin cord, 10 cm mesh: see-through, as a real net looks from the back of the court
  const netTex = vbTex(2048, 192, (c, w, h) => {
    c.clearRect(0, 0, w, h); c.strokeStyle = 'rgba(10,10,10,0.55)'; c.lineWidth = 1;
    for (let x = 0.5; x <= w; x += 20.5) { c.beginPath(); c.moveTo(x, 0); c.lineTo(x, h); c.stroke(); }
    for (let y = 0.5; y <= h; y += 19) { c.beginPath(); c.moveTo(0, y); c.lineTo(w, y); c.stroke(); }
  });
  const net = new T.Mesh(new T.PlaneGeometry(10.1, 0.9), new T.MeshBasicMaterial({ map: netTex, transparent: true, side: T.DoubleSide }));
  net.position.set(0, netH - 0.07 - 0.45, 0); scene.add(net);
  const antTex = vbTex(8, 64, (c, w, h) => { for (let i = 0; i < 8; i++) { c.fillStyle = i % 2 ? '#ffffff' : '#e53935'; c.fillRect(0, i * 8, w, 8); } });
  [-4.5, 4.5].forEach(x => { const a = new T.Mesh(new T.CylinderGeometry(0.012, 0.012, 1.8, 8), new T.MeshBasicMaterial({ map: antTex })); a.position.set(x, netH - 0.1, 0); scene.add(a); });

  // people: server + five opponents in ready position (dark kits, as in the hall)
  const kits = [{ jersey: 0x1b1d22, shorts: 0x14151a, skin: 0xb98a66 }, { jersey: 0x22304f, shorts: 0x14151a, skin: 0x8a5f43 }, { jersey: 0x1b1d22, shorts: 0x1c1c1c, skin: 0xc79b78 },
    { jersey: 0x2a2a2e, shorts: 0x14151a, skin: 0x9c6f50 }, { jersey: 0x1b1d22, shorts: 0x2a2f3a, skin: 0xd0a583 }];
  const server = vbFigure({ jersey: 0x1b1d22, shorts: 0x14151a, skin: 0xb98a66 }); scene.add(server.g);
  const opp = [[-3, -2.4], [0.2, -2.8], [3.1, -2.3], [-2.4, -6.3], [2.2, -6.6]].map(([x, z], i) => {
    const f = vbFigure(kits[i]); f.g.scale.setScalar([0.97, 1.04, 0.93, 1.0, 1.06][i]);
    vbApplyPose(f, VB_READY(x, z, 0)); f.g.traverse(m => { m.castShadow = false; }); scene.add(f.g);
    f.home = [x, z]; f.ph = i * 1.7;
    return f;
  });
  const blobTex = vbTex(64, 64, (c, w, h) => { const g = c.createRadialGradient(w / 2, h / 2, 0, w / 2, h / 2, w / 2); g.addColorStop(0, 'rgba(0,0,0,0.55)'); g.addColorStop(1, 'rgba(0,0,0,0)'); c.fillStyle = g; c.fillRect(0, 0, w, h); });
  opp.forEach(f => { const b = new T.Mesh(new T.PlaneGeometry(1.0, 0.8), new T.MeshBasicMaterial({ map: blobTex, transparent: true, depthWrite: false })); b.rotation.x = -Math.PI / 2; b.position.set(f.home[0], 0.009, f.home[1]); scene.add(b); });
  const me = vbFigure({ jersey: 0xe9edf2, shorts: 0x2d3340, skin: 0xb98a66 }); me.g.visible = false; scene.add(me.g);

  // ball: generic three-colour panel ball; the pattern makes spin (or no spin) readable
  const ballTex = vbTex(1024, 512, (c, w, h) => {
    c.fillStyle = '#f7f7f2'; c.fillRect(0, 0, w, h);
    const band = (ph, amp, col, wd) => {
      c.strokeStyle = col; c.lineWidth = wd; c.beginPath();
      for (let x = -20; x <= w + 20; x += 8) { const y = h / 2 + Math.sin(x / w * Math.PI * 2 + ph) * amp; x === -20 ? c.moveTo(x, y) : c.lineTo(x, y); }
      c.stroke();
    };
    band(0, h * 0.3, '#f2c200', h * 0.2); band(Math.PI, h * 0.3, '#1f4fbf', h * 0.2);
    band(Math.PI / 2, h * 0.3, '#f2c200', h * 0.09); band(-Math.PI / 2, h * 0.3, '#1f4fbf', h * 0.09);
    [0, Math.PI, Math.PI / 2, -Math.PI / 2].forEach((ph, i) => { [-1, 1].forEach(sg => { c.save(); c.translate(0, sg * h * (i < 2 ? 0.1 : 0.045)); band(ph, h * 0.3, 'rgba(40,40,40,0.55)', 2); c.restore(); }); });
  });
  const ball = new T.Mesh(new T.SphereGeometry(VB.R, 36, 20), new T.MeshStandardMaterial({ map: ballTex, roughness: 0.42 }));
  ball.castShadow = true; scene.add(ball);
  const ghosts = [0.2, 0.09].map(op => { const g = new T.Mesh(ball.geometry, new T.MeshBasicMaterial({ map: ballTex, transparent: true, opacity: op, depthWrite: false })); g.visible = false; scene.add(g); return g; });
  const ring = new T.Mesh(new T.RingGeometry(0.2, 0.3, 36), new T.MeshBasicMaterial({ color: 0xffd24a, side: T.DoubleSide }));
  ring.rotation.x = -Math.PI / 2; ring.visible = false; scene.add(ring);
  const path = new T.Line(new T.BufferGeometry(), new T.LineBasicMaterial({ color: 0xffd24a })); path.visible = false; path.frustumCulled = false; scene.add(path);
  const camera = new T.PerspectiveCamera(60, 1, 0.05, 120);
  const side = new T.PerspectiveCamera(50, 1, 0.5, 120); side.position.set(-25, 5.2, 0.8); side.lookAt(0, 2.4, 0.8);
  // standing reach of the model: arm straight up, flat feet
  vbApplyPose(server, { x: 0, z: 0, shR: -Math.PI, elR: 0 });
  const hp = new T.Vector3(); server.handR.getWorldPosition(hp);
  return { renderer, scene, camera, side, ball, ghosts, ring, path, server, opp, me, sun, N: 0, netH, reach: hp.y };
}

// ---------- Sounds (generated) ----------
let vbNoiseBuf = null;
function vbBurst(t, type, freq, vol, dur) {
  if (!audio) return;
  try {
    if (!vbNoiseBuf) { vbNoiseBuf = audio.createBuffer(1, audio.sampleRate, audio.sampleRate); const d = vbNoiseBuf.getChannelData(0); for (let i = 0; i < d.length; i++) d[i] = Math.random() * 2 - 1; }
    const s = audio.createBufferSource(), f = audio.createBiquadFilter(), g = audio.createGain();
    s.buffer = vbNoiseBuf; f.type = type; f.frequency.value = freq;
    g.gain.setValueAtTime(vol, t); g.gain.exponentialRampToValueAtTime(0.0001, t + dur);
    s.connect(f); f.connect(g); g.connect(audio.destination); s.start(t); s.stop(t + dur + 0.02);
  } catch (e) {}
}
function vbThump(t, f0, f1, vol, dur) {
  if (!audio) return;
  try {
    const o = audio.createOscillator(), g = audio.createGain();
    o.frequency.setValueAtTime(f0, t); o.frequency.exponentialRampToValueAtTime(f1, t + dur * 0.6);
    g.gain.setValueAtTime(vol, t); g.gain.exponentialRampToValueAtTime(0.0001, t + dur);
    o.connect(g); g.connect(audio.destination); o.start(t); o.stop(t + dur + 0.02);
  } catch (e) {}
}
const vbSfx = {
  whistle() {
    if (!audio) return;
    try {
      const t = audio.currentTime, o = audio.createOscillator(), lfo = audio.createOscillator(), lg = audio.createGain(), g = audio.createGain();
      o.type = 'square'; o.frequency.value = 2350; lfo.frequency.value = 38; lg.gain.value = 180; lfo.connect(lg); lg.connect(o.frequency);
      g.gain.setValueAtTime(0.0001, t); g.gain.linearRampToValueAtTime(0.09, t + 0.02); g.gain.setValueAtTime(0.09, t + 0.33); g.gain.exponentialRampToValueAtTime(0.0001, t + 0.42);
      o.connect(g); g.connect(audio.destination); o.start(t); lfo.start(t); o.stop(t + 0.45); lfo.stop(t + 0.45);
    } catch (e) {}
  },
  hit(key, speed) {            // float: dull thud; power serve: sharp slap; lob: soft
    if (!audio) return;
    const t = audio.currentTime, k = Math.min(1, speed / 24);
    if (key === 'topspin') { vbBurst(t, 'bandpass', 2100, 0.75 * k + 0.2, 0.09); vbThump(t, 210, 90, 0.5, 0.1); }
    else if (key === 'float') { vbBurst(t, 'lowpass', 900, 0.55, 0.07); vbThump(t, 170, 80, 0.6, 0.09); }
    else { vbBurst(t, 'lowpass', 600, 0.3, 0.08); vbThump(t, 140, 70, 0.4, 0.1); }
  },
  bounce() { if (!audio) return; const t = audio.currentTime; vbThump(t, 120, 55, 0.7, 0.16); vbBurst(t, 'lowpass', 500, 0.25, 0.08); },
};

// ---------- Rendering ----------
function vbRenderEye(f, cam, eye) {
  const V = VBS, dpr = cv.width / W, N = Math.round(2 * f.b * dpr);
  if (V.N !== N) { V.renderer.setSize(N, N, false); V.N = N; }
  V.renderer.render(V.scene, cam);
  ctx.save();
  ctx.drawImage(V.renderer.domElement, f.cx - f.b, f.cy - f.b, 2 * f.b, 2 * f.b);
  const A = eyeAlpha(eye);
  if (A < 1) { ctx.fillStyle = `rgba(0,0,0,${1 - A})`; ctx.fillRect(f.cx - f.b, f.cy - f.b, 2 * f.b, 2 * f.b); }
  ctx.restore();
}
function vbSetEyeCam(f, ex) {
  const V = VBS, Fp = FZ.FOCAL_MM * cfg.pxmm;
  // "real": true angles through the lens (correct size and stereo); "wide": more of the hall, slightly minified
  V.camera.fov = cfg.vbFov === 'wide' ? 82 : 2 * Math.atan(f.b / Fp) * 180 / Math.PI;
  V.camera.aspect = 1; V.camera.updateProjectionMatrix();
  V.camera.position.set(RECV.x + ex, RECV.eye, RECV.z);   // parallel cameras → zero disparity at infinity
  V.camera.rotation.set(0, 0, 0);
  return V.camera;
}
function vbPosAt(sv, t) {
  const pts = sv.res.pts, tl = sv.res.land.t;
  if (t <= 0) return [sv.p0[0], sv.p0[1], sv.p0[2]];
  if (t <= tl) {
    let lo = 0, hi = pts.length - 1;
    while (hi - lo > 1) { const m = (lo + hi) >> 1; if (pts[m][0] <= t) lo = m; else hi = m; }
    const a = pts[lo], b = pts[hi], k = b[0] > a[0] ? (t - a[0]) / (b[0] - a[0]) : 0;
    return [a[1] + (b[1] - a[1]) * k, a[2] + (b[2] - a[2]) * k, a[3] + (b[3] - a[3]) * k];
  }
  const v = sv.res.v, dt = t - tl, vy = Math.abs(v[1]) * 0.5;   // one damped bounce, for the eye only
  return [sv.res.land.x + v[0] * 0.6 * dt, Math.max(VB.R, VB.R + vy * dt - 0.5 * VB.G * dt * dt), sv.res.land.z + v[2] * 0.6 * dt];
}
function vbDescribe(sv) {
  const L = sv.res.land, dz = L.z - RECV.z, dx = L.x - RECV.x;
  const f = v => Math.abs(v).toFixed(1).replace('.', ',');
  let where;
  if (sv.out) where = (L.z > 9 + VB.R ? 'аут, длинная' : 'аут, в сторону') + ' — пропусти';
  else {
    const d = dz < -0.4 ? `ближе на ${f(dz)} м` : dz > 0.4 ? `дальше на ${f(dz)} м` : 'прямо на тебя';
    const s2 = dx < -0.4 ? `, левее на ${f(dx)} м` : dx > 0.4 ? `, правее на ${f(dx)} м` : '';
    where = d + s2;
  }
  const pace = sv.kmh < 55 ? 'медленная' : sv.kmh < 70 ? 'средняя' : 'быстрая';
  const kind = sv.key === 'float' && sv.anim && sv.anim.jf ? 'планер в прыжке' : sv.key === 'topspin' ? 'силовая в прыжке' : sv.name;
  return { where, what: `${kind} · ${sv.kmh} км/ч · ${pace}` };
}
// reveal overlay: text + small top view (the 3D side replay is underneath)
function vbDrawReveal(f, sv) {
  const L = sv.res.land, d = vbDescribe(sv), fs = Math.max(12, f.b * 0.085);
  ctx.save();
  ctx.fillStyle = 'rgba(8,10,14,0.72)';
  ctx.fillRect(f.cx - f.b, f.cy - f.b, 2 * f.b, f.b * 0.3); ctx.fillRect(f.cx - f.b, f.cy + f.b * 0.74, 2 * f.b, f.b * 0.26);
  ctx.fillStyle = '#ffffff'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  ctx.font = `700 ${fs}px -apple-system, system-ui, sans-serif`; ctx.fillText(d.where, f.cx, f.cy - f.b * 0.85);
  ctx.font = `500 ${fs * 0.85}px -apple-system, system-ui, sans-serif`; ctx.fillStyle = '#cfd6e2'; ctx.fillText(d.what, f.cx, f.cy + f.b * 0.87);
  // top view inset
  const size = f.b * 0.58, x0 = f.cx - f.b * 0.94, y0 = f.cy - f.b * 0.66;
  const X = x => x0 + (x + 6.5) / 13 * size, Y = z => y0 + (z + 0.5) / 12 * size;
  ctx.fillStyle = 'rgba(8,10,14,0.8)'; ctx.fillRect(x0 - 4, y0 - 4, size + 8, size + 8);
  ctx.fillStyle = '#5a3d2b'; ctx.fillRect(X(-4.5), Y(0), X(4.5) - X(-4.5), Y(9) - Y(0));
  ctx.strokeStyle = '#f2f2f2'; ctx.lineWidth = 1.5; ctx.strokeRect(X(-4.5), Y(0), X(4.5) - X(-4.5), Y(9) - Y(0));
  ctx.beginPath(); ctx.moveTo(X(-4.5), Y(3)); ctx.lineTo(X(4.5), Y(3)); ctx.stroke();
  ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(X(-5.2), Y(0)); ctx.lineTo(X(5.2), Y(0)); ctx.stroke();
  ctx.setLineDash([4, 4]); ctx.strokeStyle = 'rgba(255,210,74,0.85)'; ctx.lineWidth = 1.5; ctx.beginPath();
  sv.res.pts.filter(q => q[3] >= 0).forEach((q, i) => (i ? ctx.lineTo(X(q[1]), Y(q[3])) : ctx.moveTo(X(q[1]), Y(q[3]))));
  ctx.stroke(); ctx.setLineDash([]);
  ctx.fillStyle = '#ffffff'; ctx.beginPath(); ctx.arc(X(RECV.x), Y(RECV.z), Math.max(3, f.b * 0.028), 0, Math.PI * 2); ctx.fill();
  const r = Math.max(4, f.b * 0.04), lx = X(L.x), ly = Y(L.z);
  ctx.strokeStyle = sv.out ? '#ff5a4f' : '#ffd24a'; ctx.lineWidth = 3;
  ctx.beginPath(); ctx.moveTo(lx - r, ly - r); ctx.lineTo(lx + r, ly + r); ctx.moveTo(lx + r, ly - r); ctx.lineTo(lx - r, ly + r); ctx.stroke();
  ctx.restore();
}

// ---------- One serve ----------
function vbNewServe(now) {
  RECV.x = VB_POS[cfg.vbPos] ?? 0;
  const V = VBS;
  let sv = null;
  for (let i = 0; i < 8; i++) {           // keep only serves where the palm meets the ball exactly
    const c = vbMakeServe({ level: cfg.vbLevel, types: cfg.vbTypes, netH: cfg.netH, outShare: 0.12 });
    if (!c) continue;
    c.anim = vbServeAnim(c, V.reach); vbCalibrate(c, V.server);
    sv = c; if (c.anim.err < 0.05) break;
  }
  // where the toss leaves the hand
  vbApplyPose(V.server, vbServePose(sv.anim, sv.anim.rel)); V.server.g.updateMatrixWorld(true);
  const hp = new THREE.Vector3(); (sv.anim.tossHand === 'R' ? V.server.handR : V.server.handL).getWorldPosition(hp);
  sv.tossFrom = [hp.x, hp.y + 0.13, hp.z + 0.04];
  const pos = new Float32Array(sv.res.pts.length * 3);
  sv.res.pts.forEach((q, i) => { pos[i * 3] = q[1]; pos[i * 3 + 1] = q[2]; pos[i * 3 + 2] = q[3]; });
  V.path.geometry.dispose(); V.path.geometry = new THREE.BufferGeometry(); V.path.geometry.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  V.path.visible = false; V.ring.visible = false; V.me.g.visible = false;
  Object.assign(S.vb, { serve: sv, phase: 'pre', t0: now, logged: false, thud: false, hit: false });
  vbSfx.whistle();
}
function vbBallPre(sv, tRel, V) {       // ball before contact: in the hand, then the toss
  const an = sv.anim;
  if (tRel < an.rel) {
    const hp = new THREE.Vector3(); V.server.g.updateMatrixWorld(true);
    (an.tossHand === 'R' ? V.server.handR : V.server.handL).getWorldPosition(hp);
    return [hp.x, hp.y + 0.13, hp.z + 0.04];
  }
  const D = -an.rel, u = (tRel - an.rel) / D, A = sv.tossFrom, B = sv.p0;
  const vy0 = (B[1] - A[1] + 0.5 * VB.G * D * D) / D, tt = tRel - an.rel;
  return [A[0] + (B[0] - A[0]) * u, A[1] + vy0 * tt - 0.5 * VB.G * tt * tt, A[2] + (B[2] - A[2]) * u];
}
function volleyFrame(now) {
  const F = eyeFrames();
  if (!window.THREE) {
    [F.left, F.right].forEach(f => { drawFrame(f, 'rgba(230,230,230,0.6)'); drawText(f, 'Не загрузился 3D-движок.\nПроверь интернет\nи перезапусти.', Math.max(13, f.b * 0.09)); });
    return;
  }
  if (!VBS || VBS.netH !== cfg.netH) VBS = vbBuildScene();
  if (!S.vb) { S.vb = { n: 0, lastT: now }; vbNewServe(now); }
  const vb = S.vb, sv = vb.serve, V = VBS, an = sv.anim;
  const dt = Math.min(0.1, (now - vb.lastT) / 1000); vb.lastT = now;
  const t = (now - vb.t0) / 1000, tl = sv.res.land.t, occl = cfg.vbOccl;
  let pos, visible = true, label = '', tRel, replay = false, speed = 0;

  if (vb.phase === 'pre') {
    tRel = -an.T + t;
    if (tRel >= 0) { vb.phase = 'flight'; vb.t0 = now; tRel = 0; }
    vbApplyPose(V.server, vbServePose(an, tRel));
    pos = vbBallPre(sv, tRel, V);
    if (an.top && tRel > an.rel) V.ball.rotation.x += 14 * dt;      // the power-serve toss already spins
  }
  if (vb.phase === 'flight') {
    const tf = (now - vb.t0) / 1000;
    if (!vb.hit) { vb.hit = true; vbSfx.hit(sv.key, sv.speed); }
    vbApplyPose(V.server, vbServePose(an, tf));
    pos = vbPosAt(sv, tf);
    visible = occl >= 1 || tf <= occl * tl;
    speed = sv.speed;
    if (sv.sv.spin[0]) V.ball.rotation.x += sv.sv.spin[0] * dt;      // a float serve does not rotate — a cue
    if (!vb.thud && tf >= tl && visible) { vb.thud = true; vbSfx.bounce(); }
    if (tf >= tl + (occl < 1 ? 0.05 : 0.55)) { vb.phase = occl < 1 ? 'think' : 'reveal'; vb.t0 = now; }
  } else if (vb.phase === 'think') {
    pos = vbPosAt(sv, tl); visible = false; label = 'куда?';
    vbApplyPose(V.server, vbServePose(an, 1));
    if (t >= VBT.THINK) { vb.phase = 'reveal'; vb.t0 = now; }
  } else if (vb.phase === 'reveal') {
    replay = true;
    const rt = Math.min(tl, t / 2.4 * tl);                           // slow-motion flight along the full arc
    pos = vbPosAt(sv, rt);
    vbApplyPose(V.server, vbServePose(an, Math.min(0.5, rt)));
    if (sv.sv.spin[0]) V.ball.rotation.x += sv.sv.spin[0] * dt * 0.4;
    if (!vb.logged) {
      vb.logged = true; vb.n++;
      const d = vbDescribe(sv);
      (S.vbLog = S.vbLog || []).push(`${S.vbLog.length + 1}. ${d.what} → ${d.where}`);
    }
    if (t >= VBT.REVEAL) { vb.phase = 'gap'; vb.t0 = now; }
  } else if (vb.phase === 'gap') {
    pos = vbPosAt(sv, tl); visible = false;
    if (t >= VBT.GAP) { vbNewServe(now); return volleyFrame(now); }
  }

  V.opp.forEach(f => {
    const w = Math.sin(now / 900 + f.ph), w2 = Math.sin(now / 1300 + f.ph * 2);
    const p = VB_READY(f.home[0] + w * 0.04, f.home[1], 0);
    p.lean += w2 * 0.04; p.twist = w * 0.06; p.knR += w * 0.06; p.knL -= w * 0.06; p.shR += w2 * 0.05; p.shL += w2 * 0.05;
    vbApplyPose(f, p);
  });
  V.ball.visible = visible; V.ball.castShadow = !!cfg.vbShadow;
  V.ball.position.set(pos[0], pos[1], pos[2]);
  // slight motion smear on fast serves, as the eye sees it
  V.ghosts.forEach((g, i) => {
    const on = visible && vb.phase === 'flight' && speed > 15 && pos[1] > VB.R + 0.02;
    g.visible = on;
    if (on) { const q = vbPosAt(sv, Math.max(0, (now - vb.t0) / 1000 - 0.012 * (i + 1))); g.position.set(q[0], q[1], q[2]); g.rotation.copy(V.ball.rotation); }
  });
  V.ring.visible = replay; V.path.visible = replay; V.me.g.visible = replay;
  if (replay) { V.ring.position.set(sv.res.land.x, 0.008, sv.res.land.z); vbApplyPose(V.me, VB_READY(RECV.x, RECV.z, Math.PI)); }
  V.renderer.shadowMap.needsUpdate = true;

  if (replay) {                                // same side view for both eyes
    vbRenderEye(F.left, V.side, 'left'); V.renderer.shadowMap.needsUpdate = false; vbRenderEye(F.right, V.side, 'right');
    vbDrawReveal(F.left, sv); vbDrawReveal(F.right, sv);
  } else {
    vbRenderEye(F.left, vbSetEyeCam(F.left, -VBT.EYE), 'left');
    vbRenderEye(F.right, vbSetEyeCam(F.right, VBT.EYE), 'right');
  }
  drawFrame(F.left, 'rgba(230,230,230,0.7)', false); drawFrame(F.right, 'rgba(230,230,230,0.7)', false);
  if (label) { drawLabel(F.left, label); drawLabel(F.right, label); }
}
function vbShowLogImpl() {
  if (!S.vbLog || !S.vbLog.length) return;
  const box = document.getElementById('extra');
  box.appendChild(el('p', { className: 'hint' }, 'Подачи по порядку — сравни с тем, что говорил про себя:'));
  const ul = el('ul', { className: 'hist' });
  S.vbLog.forEach(x => ul.appendChild(el('li', {}, x)));
  box.appendChild(ul);
}
window.vbShowLogImpl = vbShowLogImpl;
// debug: render one moment of a serve from any camera (used for testing the animation)
function vbDebugShot(key, tRel, view, size) {
  if (!VBS) VBS = vbBuildScene();
  const V = VBS; let sv = null;
  for (let i = 0; i < 60 && !sv; i++) { const c = vbMakeServe({ level: 'mid', types: key, netH: cfg.netH, outShare: 0 }); if (c && (view.jf == null || (vbServeAnim(c, V.reach).jf === view.jf))) sv = c; }
  sv.anim = vbServeAnim(sv, V.reach); vbCalibrate(sv, V.server);
  vbApplyPose(V.server, vbServePose(sv.anim, sv.anim.rel)); V.server.g.updateMatrixWorld(true);
  const hp = new THREE.Vector3(); (sv.anim.tossHand === 'R' ? V.server.handR : V.server.handL).getWorldPosition(hp); sv.tossFrom = [hp.x, hp.y + 0.13, hp.z + 0.04];
  const out = [];
  (Array.isArray(tRel) ? tRel : [tRel]).forEach(t => {
    vbApplyPose(V.server, vbServePose(sv.anim, t));
    const p = t < 0 ? vbBallPre(sv, t, V) : vbPosAt(sv, t);
    V.ball.visible = true; V.ball.position.set(p[0], p[1], p[2]); V.ghosts.forEach(g => (g.visible = false));
    const cam = new THREE.PerspectiveCamera(view.fov || 40, 1, 0.1, 100);
    const sx = V.server.g.position.x, sz = V.server.g.position.z;
    cam.position.set(sx + view.dx, view.h, sz + view.dz); cam.lookAt(sx + (view.lx || 0), view.ly || 1.5, sz + (view.lz || 0));
    V.renderer.setSize(size, size, false); V.N = 0; V.renderer.shadowMap.needsUpdate = true; V.renderer.render(V.scene, cam);
    out.push(V.renderer.domElement.toDataURL('image/jpeg', 0.85));
  });
  return { imgs: out, info: { key, jf: sv.anim.jf, jump: +sv.anim.jump.toFixed(2), kmh: sv.kmh, T: sv.anim.T, rel: sv.anim.rel, contactY: +sv.p0[1].toFixed(2), err: +sv.anim.err.toFixed(3) } };
}

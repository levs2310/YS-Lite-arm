// 六轴机械臂视图：URDF + 关节滑块 + 键盘 + 拖动目标球做逆解
// 模型不动；相机/灯光/网格按模型尺寸自适应；Y 轴向上。

import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import { DragControls } from 'three/examples/jsm/controls/DragControls.js'
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js'
import URDFLoader from 'urdf-loader'

const URDF_URL = 'URDF/robot.urdf'
const VIEW_AZIMUTH = -195 * Math.PI / 180   // 初始视角绕竖直轴（净 -195°）
const JOG_DEG_PER_SEC = 90

// ---------------- 场景 ----------------
const renderer = new THREE.WebGLRenderer({ antialias: true })
renderer.setPixelRatio(Math.min(devicePixelRatio, 2))
renderer.setSize(innerWidth, innerHeight)
renderer.shadowMap.enabled = true
renderer.shadowMap.type = THREE.PCFSoftShadowMap
renderer.setClearColor(0x11161c, 1)
document.getElementById('app').appendChild(renderer.domElement)

const scene = new THREE.Scene()
scene.background = new THREE.Color(0x11161c)

const camera = new THREE.PerspectiveCamera(45, innerWidth / innerHeight, 0.1, 50000)
camera.up.set(0, 1, 0)
camera.position.set(300, 260, 380)

const controls = new OrbitControls(camera, renderer.domElement)
controls.enableDamping = true
controls.dampingFactor = 0.08

scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x20262c, 1.15))
const sun = new THREE.DirectionalLight(0xffffff, 1.6)
sun.castShadow = true
scene.add(sun)

let ground = new THREE.Mesh(
  new THREE.PlaneGeometry(10, 10),
  new THREE.ShadowMaterial({ opacity: 0.3 })
)
ground.rotation.x = -Math.PI / 2
ground.receiveShadow = true
scene.add(ground)

let grid = new THREE.GridHelper(10, 20, 0x2b3a47, 0x1d2831)
scene.add(grid)

// 目标球
const targetBall = new THREE.Mesh(
  new THREE.SphereGeometry(1, 24, 18),
  new THREE.MeshBasicMaterial({ color: 0xff8a4c, transparent: true, opacity: 0.75 })
)
targetBall.visible = false
scene.add(targetBall)

// ---------------- 状态 ----------------
const BODY_MAT = new THREE.MeshStandardMaterial({
  color: 0xf2f4f6, metalness: 0.08, roughness: 0.55,
})
let robot = null
let actuated = []
let eeLink = null
let eePoint = null
let unitToMm = 1

// ---------------- URDF ----------------
const loader = new URDFLoader()
loader.parseVisual = true
loader.parseCollision = false
loader.loadMeshCb = (path, manager, done) => {
  new STLLoader(manager).load(path, (geo) => {
    geo.computeVertexNormals()
    const mesh = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({
      color: 0xf2f4f6, metalness: 0.08, roughness: 0.55,
    }))
    mesh.castShadow = true
    mesh.receiveShadow = true
    done(mesh)
  }, undefined, (err) => { console.warn('mesh fail', path, err); done(new THREE.Object3D()) })
}

function load() {
  fetch(URDF_URL + '?t=' + Date.now())
    .then((r) => { if (!r.ok) throw new Error('HTTP ' + r.status); return r.text() })
    .then((txt) => {
      const base = URDF_URL.slice(0, URDF_URL.lastIndexOf('/') + 1)
      robot = loader.parse(txt, base)
      scene.add(robot)
      robot.traverse((o) => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true } })

      // 可动关节（按 URDF 顺序）
      actuated = Object.values(robot.joints || {})
        .filter((j) => ['revolute', 'continuous', 'prismatic'].includes(j.jointType))
        .map((j) => {
          const isSl = j.jointType === 'prismatic'
          let lo = j.limit?.lower ?? (isSl ? -0.1 : -Math.PI)
          let hi = j.limit?.upper ?? (isSl ? 0.1 : Math.PI)
          if (!isFinite(lo) || !isFinite(hi) || Math.abs(hi - lo) < 1e-6) {
            lo = isSl ? -0.1 : -Math.PI
            hi = isSl ? 0.1 : Math.PI
          }
          return { joint: j, name: j.name, min: lo, max: hi, prismatic: isSl }
        })

      // 末端 link
      let best = null, depth = -1
      robot.traverse((o) => {
        if (o.isURDFLink) {
          let d = 0, p = o
          while (p && p !== robot) { d++; p = p.parent }
          if (d > depth) { depth = d; best = o }
        }
      })
      eeLink = best
      eePoint = null

      buildSliders()
      // 等网格加载完再定工具点
      setTimeout(() => {
        // ★ urdf-loader 在网格回调里会把材质换成默认白，必须在加载完成后覆盖
        robot.traverse((o) => {
          if (o.isMesh) {
            o.material = BODY_MAT
            o.castShadow = true
            o.receiveShadow = true
          }
        })
        computeUnit()
        setupEndPoint()
        fitScene()
        updateTargetBall()
        refreshEEReadout()
        console.log('[urdf] 已加载：%d 关节，末端 %s，工具点 %s',
          actuated.length, eeLink?.name,
          eePoint ? eePoint.getWorldPosition(new THREE.Vector3()).toArray().map((v) => v.toFixed(1)).join(', ') : '-')
      }, 700)
    })
    .catch((e) => console.error('加载 URDF 失败：', e))
}

function computeUnit() {
  robot.updateMatrixWorld(true)
  const box = new THREE.Box3().setFromObject(robot)
  if (box.isEmpty()) { unitToMm = 1; return }
  const size = box.getSize(new THREE.Vector3())
  const m = Math.max(size.x, size.y, size.z)
  unitToMm = m < 10 ? 1000 : 1     // 米建模 → ×1000；毫米建模 → ×1
}

// 末端工具点 = 末端 link 在 +X 方向最外侧「端面」的中心（不是某个角点）
function setupEndPoint() {
  if (!eeLink) return
  eePoint = new THREE.Object3D()
  eePoint.name = 'ee_point'
  eeLink.updateMatrixWorld(true)

  const inv = new THREE.Matrix4().copy(eeLink.matrixWorld).invert()
  const meshes = []
  eeLink.traverse((o) => {
    if (o.isMesh && o.geometry && o.geometry.attributes.position) {
      meshes.push({ mesh: o, pos: o.geometry.attributes.position,
                    toLocal: new THREE.Matrix4().multiplyMatrices(inv, o.matrixWorld) })
    }
  })
  if (!meshes.length) { eeLink.add(eePoint); return }

  const v = new THREE.Vector3()
  // 第一遍：最外侧的 X
  let maxX = -Infinity
  for (const m of meshes) {
    for (let i = 0; i < m.pos.count; i++) {
      v.fromBufferAttribute(m.pos, i).applyMatrix4(m.toLocal)
      if (v.x > maxX) maxX = v.x
    }
  }
  if (!isFinite(maxX)) maxX = 0

  // 第二遍：取最外端面（X 差在容差内）所有顶点的中心
  const eps = Math.max(0.05, Math.abs(maxX) * 0.005)
  const acc = new THREE.Vector3()
  let cnt = 0
  for (const m of meshes) {
    for (let i = 0; i < m.pos.count; i++) {
      v.fromBufferAttribute(m.pos, i).applyMatrix4(m.toLocal)
      if (maxX - v.x <= eps) { acc.add(v); cnt++ }
    }
  }
  const tip = cnt ? acc.divideScalar(cnt) : new THREE.Vector3(maxX, 0, 0)
  eePoint.position.copy(tip)
  eeLink.add(eePoint)
  eePoint.updateMatrixWorld(true)
  console.log('[ee] 端面中心（link 局部）=', tip.toArray().map((n) => n.toFixed(2)).join(', '),
    '｜ 世界 =', eePoint.getWorldPosition(new THREE.Vector3()).toArray()
      .map((n) => n.toFixed(2)).join(', '))
}

function eeWorld() {
  const o = eePoint || eeLink
  return o.getWorldPosition(new THREE.Vector3())
}

// ---------------- 界面 ----------------
function buildSliders() {
  const box = document.getElementById('joints')
  box.innerHTML = ''
  actuated.forEach((a, i) => {
    const lo = a.prismatic ? a.min * 1000 : (a.min * 180) / Math.PI
    const hi = a.prismatic ? a.max * 1000 : (a.max * 180) / Math.PI
    const cur = a.prismatic ? (a.joint.jointValue?.[0] ?? 0) * 1000
      : ((a.joint.jointValue?.[0] ?? 0) * 180) / Math.PI
    const unit = a.prismatic ? 'mm' : '°'
    const div = document.createElement('div')
    div.className = 'joint'
    div.innerHTML =
      `<div class="h"><b>${i + 1}. ${a.name}</b><span data-v>${cur.toFixed(1)}${unit}</span></div>
       <input type="range" min="${lo}" max="${hi}" step="0.5" value="${cur}">`
    const sl = div.querySelector('input')
    const out = div.querySelector('[data-v]')
    sl.addEventListener('input', () => {
      const val = parseFloat(sl.value)
      a.joint.setJointValue(a.prismatic ? val / 1000 : (val * Math.PI) / 180)
      out.textContent = val.toFixed(1) + unit
      updateTargetBall()
      refreshEEReadout()
      markUserInput()
    })
    a.slider = sl; a.out = out; a.unit = unit; a.isPrismatic = a.prismatic
    box.appendChild(div)
  })
}

function syncSliders() {
  actuated.forEach((a) => {
    if (!a.slider) return
    const v = a.prismatic ? (a.joint.jointValue?.[0] ?? 0) * 1000
      : ((a.joint.jointValue?.[0] ?? 0) * 180) / Math.PI
    a.slider.value = v
    a.out.textContent = v.toFixed(1) + a.unit
  })
}

// ---------------- 键盘 ----------------
const KEYMAP = {
  a: ['joint1_yaw', +1], d: ['joint1_yaw', -1],
  w: ['joint2_arm', +1], s: ['joint2_arm', -1],
  i: ['joint3_fore', +1], k: ['joint3_fore', -1],
  j: ['joint4_roll', +1], l: ['joint4_roll', -1],
  h: ['joint5_wrist', +1], f: ['joint5_wrist', -1],
  z: ['joint6_roll', +1], x: ['joint6_roll', -1],
}
const keysDown = new Set()

window.addEventListener('keydown', (e) => {
  const k = (e.key || '').toLowerCase()
  if (KEYMAP[k]) { keysDown.add(k); e.preventDefault() }
})
window.addEventListener('keyup', (e) => keysDown.delete((e.key || '').toLowerCase()))
window.addEventListener('blur', () => keysDown.clear())

function applyKeys(dt) {
  if (!keysDown.size) return
  const step = ((JOG_DEG_PER_SEC * Math.PI) / 180) * dt
  let moved = false
  keysDown.forEach((k) => {
    const [name, sgn] = KEYMAP[k]
    const a = actuated.find((x) => x.name === name)
    if (!a || a.prismatic) return
    const cur = a.joint.jointValue?.[0] ?? 0
    const nv = Math.min(a.max, Math.max(a.min, cur + sgn * step))
    if (Math.abs(nv - cur) > 1e-9) { a.joint.setJointValue(nv); moved = true }
  })
  if (moved) { syncSliders(); updateTargetBall(); refreshEEReadout(); markUserInput() }
}

// ---------------- 逆解（位置，阻尼最小二乘） ----------------
function solveIK(targetWorld, opts = {}) {
  const n = actuated.length
  if (!n || !eeLink) return { ok: false, err: Infinity, q: [] }
  const clampQ = (qq) => qq.map((v, i) => Math.min(actuated[i].max, Math.max(actuated[i].min, v)))
  let q = clampQ(opts.seed ? opts.seed.slice() : actuated.map((a) => a.joint.jointValue?.[0] ?? 0))
  const setQ = (qq) => {
    actuated.forEach((a, i) => a.joint.setJointValue(Math.min(a.max, Math.max(a.min, qq[i]))))
    robot.updateMatrixWorld(true)
  }
  const fk = (qq) => { setQ(qq); return eeWorld() }

  let lambda = 0.08
  let err = targetWorld.clone().sub(fk(q)).length()
  let best = { err, q: q.slice() }
  const tol = 0.3 / unitToMm

  for (let it = 0; it < (opts.iters || 90) && err > tol; it++) {
    const p0 = fk(q)
    const e = targetWorld.clone().sub(p0)
    const h = 1e-4
    const J = []
    for (let i = 0; i < n; i++) {
      const qp = q.slice(); qp[i] += h
      J.push(fk(qp).clone().sub(p0).divideScalar(h))
    }
    const A = new THREE.Matrix3()
    const a9 = new Array(9).fill(0)
    for (let r = 0; r < 3; r++) {
      for (let c = 0; c < 3; c++) {
        let ssum = 0
        for (let k = 0; k < n; k++) ssum += J[k].getComponent(r) * J[k].getComponent(c)
        a9[r * 3 + c] = ssum + (r === c ? lambda : 0)
      }
    }
    A.set(a9[0], a9[1], a9[2], a9[3], a9[4], a9[5], a9[6], a9[7], a9[8])
    let Ainv
    try { Ainv = A.clone().invert() } catch (e2) { break }
    const ae = e.clone().applyMatrix3(Ainv)
    const dq = J.map((col) => col.dot(ae))
    let mx = 0
    dq.forEach((v) => { mx = Math.max(mx, Math.abs(v)) })
    const scale = mx > 0.25 ? 0.25 / mx : 1
    const qn = clampQ(q.map((v, i) => v + dq[i] * scale))

    const en = targetWorld.clone().sub(fk(qn)).length()
    if (en < err) {
      q = qn; err = en; lambda = Math.max(0.002, lambda * 0.6)
      if (en < best.err) best = { err: en, q: q.slice() }
    } else {
      lambda = Math.min(4, lambda * 2.2)
    }
  }
  if (!opts.noApply) setQ(best.q)
  return { ok: best.err < 1.0 / unitToMm, err: best.err, q: best.q }
}

// 固定当前末端位置，换一组关节角（多解搜索），并以匀速过渡过去
const JOINT_MOVE_DEG_PER_SEC = 90
let jointAnim = null

function moveToJoints(targetQ, speedDeg = JOINT_MOVE_DEG_PER_SEC) {
  const from = actuated.map((a) => a.joint.jointValue?.[0] ?? 0)
  let maxd = 0
  for (let i = 0; i < from.length; i++) maxd = Math.max(maxd, Math.abs(targetQ[i] - from[i]))
  if (maxd < 1e-6) { jointAnim = null; return 0 }
  const dur = (maxd / ((speedDeg * Math.PI) / 180)) * 1000
  jointAnim = { from, to: targetQ.slice(), t0: performance.now(), dur: Math.max(80, dur) }
  return dur
}

function stepJointAnim(now) {
  if (!jointAnim) return
  const k = Math.min(1, (now - jointAnim.t0) / jointAnim.dur)
  actuated.forEach((a, i) => {
    a.joint.setJointValue(jointAnim.from[i] + (jointAnim.to[i] - jointAnim.from[i]) * k)
  })
  syncSliders()
  updateTargetBall()
  refreshEEReadout()
  if (k >= 1) jointAnim = null
}

function alternatePose() {
  if (!eePoint) return
  const target = targetBall.visible ? targetBall.position.clone() : eeWorld()
  const cur = actuated.map((a) => a.joint.jointValue?.[0] ?? 0)
  let best = null
  for (let k = 0; k < 18; k++) {
    const seed = cur.map((v, i) => {
      const a = actuated[i]
      const span = a.max - a.min
      return Math.min(a.max, Math.max(a.min, v + (Math.random() - 0.5) * span * 0.7))
    })
    const r = solveIK(target, { seed, iters: 90, noApply: true })
    if (!r.ok) continue
    const diff = r.q.reduce((s2, v, i) => s2 + Math.abs(v - cur[i]), 0)
    if (diff < 0.35) continue
    if (!best || diff > best.diff) best = { q: r.q, diff, err: r.err }
  }
  const msg = document.getElementById('ikMsg')
  if (!best) {
    msg.textContent = '没找到明显不同的姿态（可能已贴限位）'
    return
  }
  // 回到当前姿态，再匀速过渡到新姿态
  actuated.forEach((a, i) => a.joint.setJointValue(cur[i]))
  robot.updateMatrixWorld(true)
  const dur = moveToJoints(best.q, JOINT_MOVE_DEG_PER_SEC)
  msg.textContent = `正在过渡到新姿态：最大关节行程 ${(best.diff * 180 / Math.PI / 6).toFixed(0)}°，` +
    `用时约 ${(dur / 1000).toFixed(1)} s（末端保持不动）`
}

// ---------------- 目标球拖动 ----------------
let dragControls = null
function setupDrag() {
  dragControls = new DragControls([targetBall], camera, renderer.domElement)
  dragControls.addEventListener('dragstart', () => { controls.enabled = false })
  dragControls.addEventListener('dragend', () => {
    controls.enabled = true
    refreshEEReadout()
  })
  let last = 0
  dragControls.addEventListener('drag', () => {
    const now = performance.now()
    if (now - last < 60) return
    last = now
    solveIK(targetBall.position)
    syncSliders()
  })
}

function updateTargetBall() {
  if (!eePoint) return
  const p = eeWorld()
  targetBall.position.copy(p)
  const r = camera.position.distanceTo(controls.target) * 0.012
  targetBall.scale.setScalar(Math.max(r, 1e-4))
  targetBall.visible = true
}

// ---------------- 取景 ----------------
function fitScene() {
  robot.updateMatrixWorld(true)
  const box = new THREE.Box3().setFromObject(robot)
  if (box.isEmpty()) return
  const c = box.getCenter(new THREE.Vector3())
  const s = box.getSize(new THREE.Vector3())
  const r = Math.max(s.x, s.y, s.z) * 0.5 || 1

  // 灯光
  const lh = new THREE.Vector3(1, 0, 1).normalize().multiplyScalar(r * 2.6)
  lh.applyAxisAngle(new THREE.Vector3(0, 1, 0), VIEW_AZIMUTH)
  sun.position.copy(c).add(new THREE.Vector3(0, r * 3.2, 0)).add(lh)
  const sc = sun.shadow.camera
  sc.left = -r * 2.5; sc.right = r * 2.5; sc.top = r * 2.5; sc.bottom = -r * 2.5
  sc.near = r * 0.05; sc.far = r * 14
  sc.updateProjectionMatrix()

  // 地面 / 网格
  scene.remove(grid); grid.geometry.dispose(); grid.material.dispose()
  grid = new THREE.GridHelper(r * 8, 30, 0x2b3a47, 0x1d2831)
  grid.position.set(c.x, box.min.y, c.z)
  scene.add(grid)

  scene.remove(ground); ground.geometry.dispose()
  ground = new THREE.Mesh(new THREE.PlaneGeometry(r * 16, r * 16),
    new THREE.ShadowMaterial({ opacity: 0.3 }))
  ground.rotation.x = -Math.PI / 2
  ground.receiveShadow = true
  ground.position.copy(grid.position)
  scene.add(ground)

  // 相机
  controls.target.copy(c)
  const dist = (r / Math.tan((camera.fov * Math.PI) / 360)) * 1.8
  const hDir = new THREE.Vector3(1, 0, 1).normalize()
  const off = hDir.clone().multiplyScalar(dist * 0.78).add(new THREE.Vector3(0, dist * 0.5, 0))
  off.applyAxisAngle(new THREE.Vector3(0, 1, 0), VIEW_AZIMUTH)
  camera.position.copy(c).add(off)
  camera.near = Math.max(r / 500, 0.01)
  camera.far = dist * 30
  camera.updateProjectionMatrix()
  controls.update()
}

// ---------------- 交互 ----------------
const $ = (id) => document.getElementById(id)

// 末端坐标 → mm（世界单位按 unitToMm 换算）
function eeMm() {
  return eeWorld().multiplyScalar(unitToMm)
}

function refreshEEReadout() {
  if (!eePoint) return
  const p = eeMm()
  $('eeNow').textContent = `${p.x.toFixed(1)},  ${p.y.toFixed(1)},  ${p.z.toFixed(1)}`
}

$('btnZero').addEventListener('click', () => {
  actuated.forEach((a) => a.joint.setJointValue(0))
  syncSliders()
  updateTargetBall()
  refreshEEReadout()
  markUserInput()
})


$('btnAltPose').addEventListener('click', () => {
  if (simLinked) return          // 与真机联动时不允许用逆解直接驱动真机
  markUserInput()
  alternatePose()
})

let _last = performance.now()
let _eeTick = 0
function animate(now) {
  requestAnimationFrame(animate)
  const t = now ?? performance.now()
  const dt = Math.min(0.1, (t - _last) / 1000)
  _last = t
  if (keysDown.size) jointAnim = null      // 手动操作时打断自动过渡
  stepJointAnim(t)
  applyKeys(dt)
  controls.update()
  renderer.render(scene, camera)

  // 末端坐标实时刷新（≈10Hz，避免每帧改 DOM）
  if (t - _eeTick > 100) {
    _eeTick = t
    refreshEEReadout()
  }
}

addEventListener('resize', () => {
  camera.aspect = innerWidth / innerHeight
  camera.updateProjectionMatrix()
  renderer.setSize(innerWidth, innerHeight)
})

// ---------------- 上位机联动（WebSocket） ----------------
// 上位机 → 页面：{"type":"motor_angles","payload":{"angles":[6 个值]}}
// 页面 → 上位机：{"type":"sim_angles","payload":{"angles":[6 个值],"t":秒}}
// 1-5 为关节角(度)；第 6 路是 0-400 的夹爪开合量，按关节行程等比映射，来回一致。
const SIM_WS_PORT = 8765
const SIM_UPLINK_MS = 100
const SIM_UPLINK_HOLD_MS = 1200     // 用户停手后仍回传的时长
let simWs = null
let simRetryMs = 1000
let simRetryTimer = null
// 初值必须是 -Infinity：若用 0，页面刚打开的前 1.2 秒会满足「刚操作过」的条件，
// 于是连上就上行一帧全 0，双向模式下会把机械臂命令回零。
let simLastUserInput = -Infinity
let simLinked = false          // 上位机是否处于联动状态（非「关闭」）
let simModeLabel = ''
// 运行状态探针，便于排查联动问题（控制台可查看 window.__simDebug）
const simDebug = { open: 0, msg: 0, mode: 0, connected: false, lastType: '' }
window.__simDebug = simDebug

// 联动中禁用会直接驱动真机的按钮（逆解姿态），避免一次点击让六轴同时大动
function applySimMode(mode, label) {
  simLinked = !!mode && mode !== 'off'
  simModeLabel = simLinked ? (label || '') : ''
  const btn = $('btnAltPose')
  if (btn) {
    btn.disabled = simLinked
    btn.title = simLinked
      ? '与真机联动中（' + simModeLabel + '），逆解会同时驱动真机，已禁用'
      : ''
  }
  if (simWs && simWs.readyState === WebSocket.OPEN) {
    setSimStatus('上位机：已连接' + (simLinked ? ' · ' + simModeLabel : ' · 未联动'), '#7ee081')
  }
}

function markUserInput() { simLastUserInput = performance.now() }

function setSimStatus(text, color) {
  const el = $('simStatus')
  if (!el) return
  el.textContent = text
  if (color) el.style.color = color
}

function scheduleSimRetry() {
  if (simRetryTimer) return
  simRetryTimer = setTimeout(() => {
    simRetryTimer = null
    connectSim()
  }, simRetryMs)
  simRetryMs = Math.min(simRetryMs * 2, 8000)
}

function connectSim() {
  const host = location.hostname || '127.0.0.1'
  const url = 'ws://' + host + ':' + SIM_WS_PORT

  // 建新连接前先把旧连接的毁掉并摘掉回调：ws.close() 是异步的，
  // 旧 socket 的 onclose 稍后会触发，会把刚建立的新连接状态改回「未连接」。
  if (simWs) {
    const old = simWs
    old.onopen = old.onmessage = old.onerror = old.onclose = null
    try { old.close() } catch (e) { /* 已断开就忽略 */ }
    simWs = null
  }

  let sock
  try {
    sock = new WebSocket(url)
  } catch (e) {
    scheduleSimRetry()
    return
  }
  simWs = sock

  sock.onopen = () => {
    if (simWs !== sock) return
    simRetryMs = 1000
    simDebug.open++
    simDebug.connected = true
    setSimStatus('上位机：已连接', '#7ee081')
    console.log('[WS] 已连接上位机 ' + url)
  }
  sock.onmessage = (ev) => {
    if (simWs !== sock) return
    let msg
    try { msg = JSON.parse(ev.data) } catch (e) { return }
    simDebug.msg++
    simDebug.lastType = (msg && msg.type) || ''
    if (msg && msg.type === 'sim_mode' && msg.payload) {
      simDebug.mode++
      applySimMode(msg.payload.mode, msg.payload.label)
      return
    }
    if (msg && msg.type === 'motor_angles' && msg.payload && Array.isArray(msg.payload.angles)) {
      applyMotorAngles(msg.payload.angles)
    }
  }
  sock.onclose = () => {
    if (simWs !== sock) return       // 旧连接的关闭事件，忽略
    simWs = null
    simDebug.connected = false
    applySimMode('off')
    setSimStatus('上位机：未连接', '#7d8b99')
    scheduleSimRetry()
  }
  sock.onerror = () => { try { sock.close() } catch (e) { /* 交给 onclose 重连 */ } }
}

// 第 6 路：0-400 的开合量 ↔ 关节行程，等比互映
function gripperToJoint(v) {
  const a = actuated[5]
  if (!a) return 0
  const frac = Math.max(0, Math.min(1, v / 400))
  return a.min + frac * (a.max - a.min)
}

function jointToGripper(a) {
  const cur = a.joint.jointValue?.[0] ?? 0
  const span = a.max - a.min
  return span ? ((cur - a.min) / span) * 400 : 0
}

function applyMotorAngles(angles) {
  if (!actuated.length) return
  // 用户刚拖动的这段时间里不接受下发，否则 10Hz 的推送会把刚拖到的角度立刻压回去，
  // 页面就再也拖不动了。本地输入优先，停手 300ms 后恢复跟随。
  if (performance.now() - simLastUserInput < 300) return
  let changed = false
  for (let i = 0; i < 5 && i < angles.length; i++) {
    const a = actuated[i]
    const v = Number(angles[i])
    if (!a || !isFinite(v)) continue
    const target = Math.min(a.max, Math.max(a.min, (v * Math.PI) / 180))
    if (Math.abs(target - (a.joint.jointValue?.[0] ?? 0)) < 1e-4) continue
    a.joint.setJointValue(target)
    changed = true
  }
  if (angles.length > 5 && actuated[5]) {
    const a = actuated[5]
    const target = Math.min(a.max, Math.max(a.min, gripperToJoint(Number(angles[5]) || 0)))
    if (Math.abs(target - (a.joint.jointValue?.[0] ?? 0)) >= 1e-4) {
      a.joint.setJointValue(target)
      changed = true
    }
  }
  if (changed) { syncSliders(); updateTargetBall(); refreshEEReadout() }
}

function currentSimAngles() {
  const out = []
  for (let i = 0; i < 5; i++) {
    const a = actuated[i]
    out.push(a ? ((a.joint.jointValue?.[0] ?? 0) * 180) / Math.PI : 0)
  }
  out.push(actuated[5] ? jointToGripper(actuated[5]) : 0)
  return out
}

// 只在用户刚操作过时上行，避免与上位机的下发互相灌
setInterval(() => {
  if (!simWs || simWs.readyState !== WebSocket.OPEN) return
  if (performance.now() - simLastUserInput > SIM_UPLINK_HOLD_MS) return
  try {
    simWs.send(JSON.stringify({
      type: 'sim_angles',
      payload: { angles: currentSimAngles(), t: Date.now() / 1000 },
    }))
  } catch (e) { /* 下一帧重试 */ }
}, SIM_UPLINK_MS)

connectSim()

load()
setupDrag()
animate()

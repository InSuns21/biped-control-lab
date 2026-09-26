import * as THREE from "../../vendor/three.module.js";

const BODY_COLORS = {
  body: 0xb7bec8,
  head: 0xdbe1e8,
  nozzle: 0x657180,
  com: 0x49a078,
  water: 0x4db7e5,
  thrust: 0xf1c84b,
  gravity: 0xe25b5b,
  torque: 0xa66de0,
  target: 0x88919c,
};

function vectorLength(v) {
  return Math.hypot(v[0], v[1], v[2]);
}

function direction3(v, fallback = [0, 1, 0]) {
  const n = vectorLength(v);
  const source = n > 1e-12 ? v : fallback;
  const d = new THREE.Vector3(source[0], source[1], source[2]);
  return d.normalize();
}

function makeLine(color, opacity = 1) {
  const geometry = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(),
    new THREE.Vector3(0, 1, 0),
  ]);
  const material = new THREE.LineBasicMaterial({
    color,
    transparent: opacity < 1,
    opacity,
  });
  return new THREE.Line(geometry, material);
}

export function createShowerScene(canvas) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;

  const styles = getComputedStyle(document.documentElement);
  const panelColor = styles.getPropertyValue("--panel").trim() || "#ffffff";
  const lineColor = styles.getPropertyValue("--line").trim() || "#d9dee7";

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(panelColor);

  const camera = new THREE.PerspectiveCamera(42, 1, 0.01, 10);
  const cameraTarget = new THREE.Vector3(0, 0.22, 0);

  scene.add(new THREE.HemisphereLight(0xffffff, 0x5f6875, 1.45));
  const keyLight = new THREE.DirectionalLight(0xffffff, 2.1);
  keyLight.position.set(1.5, 2.2, 1.3);
  scene.add(keyLight);

  const grid = new THREE.GridHelper(1.8, 18, lineColor, lineColor);
  grid.material.transparent = true;
  grid.material.opacity = 0.42;
  scene.add(grid);

  const pivot = new THREE.Mesh(
    new THREE.SphereGeometry(0.028, 18, 12),
    new THREE.MeshStandardMaterial({ color: BODY_COLORS.nozzle }),
  );
  scene.add(pivot);

  const targetAxis = makeLine(BODY_COLORS.target, 0.55);
  targetAxis.geometry.setFromPoints([
    new THREE.Vector3(0, 0, 0),
    new THREE.Vector3(0, 0.53, 0),
  ]);
  scene.add(targetAxis);

  const bodyRoot = new THREE.Group();
  scene.add(bodyRoot);

  const handle = new THREE.Mesh(
    new THREE.CylinderGeometry(0.035, 0.048, 0.28, 28),
    new THREE.MeshStandardMaterial({
      color: BODY_COLORS.body,
      metalness: 0.28,
      roughness: 0.48,
    }),
  );
  handle.position.y = 0.17;
  bodyRoot.add(handle);

  const head = new THREE.Mesh(
    new THREE.CylinderGeometry(0.11, 0.105, 0.065, 36),
    new THREE.MeshStandardMaterial({
      color: BODY_COLORS.head,
      metalness: 0.2,
      roughness: 0.42,
    }),
  );
  head.position.y = 0.335;
  bodyRoot.add(head);

  const nozzlePlate = new THREE.Mesh(
    new THREE.CylinderGeometry(0.092, 0.092, 0.012, 36),
    new THREE.MeshStandardMaterial({
      color: BODY_COLORS.nozzle,
      metalness: 0.08,
      roughness: 0.6,
    }),
  );
  nozzlePlate.position.y = 0.374;
  bodyRoot.add(nozzlePlate);

  const comMarker = new THREE.Mesh(
    new THREE.SphereGeometry(0.018, 18, 12),
    new THREE.MeshStandardMaterial({ color: BODY_COLORS.com }),
  );
  comMarker.position.set(0, 0.12, 0);
  bodyRoot.add(comMarker);

  const waterStream = makeLine(BODY_COLORS.water, 0.9);
  bodyRoot.add(waterStream);

  const thrustArrow = new THREE.ArrowHelper(
    new THREE.Vector3(0, 1, 0),
    new THREE.Vector3(0, 0.38, 0),
    0.32,
    BODY_COLORS.thrust,
    0.075,
    0.038,
  );
  bodyRoot.add(thrustArrow);

  const torqueArrow = new THREE.ArrowHelper(
    new THREE.Vector3(0, 0, 1),
    new THREE.Vector3(0, 0.02, 0),
    0.18,
    BODY_COLORS.torque,
    0.055,
    0.03,
  );
  bodyRoot.add(torqueArrow);

  const gravityArrow = new THREE.ArrowHelper(
    new THREE.Vector3(0, -1, 0),
    new THREE.Vector3(),
    0.3,
    BODY_COLORS.gravity,
    0.07,
    0.035,
  );
  scene.add(gravityArrow);

  const nozzleLocal = new THREE.Vector3(0, 0.38, 0);
  const comLocal = new THREE.Vector3(0, 0.12, 0);

  function setCameraView(view) {
    const positions = {
      iso: [1.05, 0.72, 1.2],
      front: [0, 0.42, 1.55],
      side: [1.55, 0.42, 0],
      top: [0.001, 1.75, 0.001],
    };
    const p = positions[view] ?? positions.iso;
    camera.position.set(p[0], p[1], p[2]);
    camera.up.set(0, 1, 0);
    if (view === "top") camera.up.set(0, 0, -1);
    camera.lookAt(cameraTarget);
  }

  function resizeIfNeeded() {
    const width = Math.max(1, canvas.clientWidth);
    const height = Math.max(1, canvas.clientHeight);
    const pixelRatio = renderer.getPixelRatio();
    const renderWidth = Math.floor(width * pixelRatio);
    const renderHeight = Math.floor(height * pixelRatio);
    if (canvas.width !== renderWidth || canvas.height !== renderHeight) {
      renderer.setSize(width, height, false);
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
    }
  }

  function updateBodyQuaternion(q) {
    bodyRoot.quaternion.set(q[1], q[2], q[3], q[0]).normalize();
  }

  function updateVectors(diagnostics) {
    const force = diagnostics.jetForceBodyN;
    const thrustN = diagnostics.thrustN;
    const forceDirection = direction3(force);
    thrustArrow.setDirection(forceDirection);
    thrustArrow.setLength(
      thrustN > 1e-6 ? 0.14 + 0.34 * Math.min(thrustN, 1.2) : 0.001,
      0.075,
      0.038,
    );
    thrustArrow.visible = thrustN > 1e-6;

    const waterDirection = forceDirection.clone().multiplyScalar(-1);
    const streamEnd = nozzleLocal.clone().add(
      waterDirection.multiplyScalar(thrustN > 1e-6 ? 0.62 : 0.001),
    );
    const streamPosition = waterStream.geometry.getAttribute("position");
    streamPosition.setXYZ(0, nozzleLocal.x, nozzleLocal.y, nozzleLocal.z);
    streamPosition.setXYZ(1, streamEnd.x, streamEnd.y, streamEnd.z);
    streamPosition.needsUpdate = true;
    waterStream.visible = thrustN > 1e-6;

    const torque = diagnostics.jetTorqueBodyNm;
    const torqueMagnitude = vectorLength(torque);
    torqueArrow.setDirection(direction3(torque, [0, 0, 1]));
    torqueArrow.setLength(
      torqueMagnitude > 1e-7
        ? 0.11 + 1.35 * Math.min(torqueMagnitude, 0.16)
        : 0.001,
      0.055,
      0.03,
    );
    torqueArrow.visible = torqueMagnitude > 1e-7;

    const comWorld = bodyRoot.localToWorld(comLocal.clone());
    gravityArrow.position.copy(comWorld);
    gravityArrow.setDirection(new THREE.Vector3(0, -1, 0));
    gravityArrow.setLength(0.28, 0.07, 0.035);
  }

  function render(state, diagnostics) {
    resizeIfNeeded();
    updateBodyQuaternion(state.qBodyToWorld);
    if (diagnostics) updateVectors(diagnostics);
    renderer.render(scene, camera);
  }

  setCameraView("iso");

  return {
    render,
    setCameraView,
  };
}

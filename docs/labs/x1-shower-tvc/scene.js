import * as THREE from "../../vendor/three.module.js";

const COLORS = {
  body: 0xb7bec8,
  head: 0xdbe1e8,
  nozzle: 0x657180,
  com: 0x49a078,
  water: 0x4db7e5,
  reaction: 0xf1c84b,
  gravity: 0xe25b5b,
  waterTorque: 0xa66de0,
  holdTorque: 0x49a078,
  target: 0x88919c,
  hose: 0x7f8b99,
};

function vectorLength(v) {
  return Math.hypot(v[0], v[1], v[2]);
}

function direction3(v, fallback = [0, 1, 0]) {
  const n = vectorLength(v);
  const source = n > 1e-12 ? v : fallback;
  return new THREE.Vector3(source[0], source[1], source[2]).normalize();
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

function setLineEndpoints(line, a, b) {
  const position = line.geometry.getAttribute("position");
  position.setXYZ(0, a.x, a.y, a.z);
  position.setXYZ(1, b.x, b.y, b.z);
  position.needsUpdate = true;
}

function addCylinderBetween(group, a, b, radius, color) {
  const start = new THREE.Vector3(...a);
  const end = new THREE.Vector3(...b);
  const delta = end.clone().sub(start);
  const length = delta.length();
  const mesh = new THREE.Mesh(
    new THREE.CylinderGeometry(radius, radius, length, 24),
    new THREE.MeshStandardMaterial({
      color,
      metalness: 0.18,
      roughness: 0.55,
    }),
  );
  mesh.position.copy(start).add(end).multiplyScalar(0.5);
  mesh.quaternion.setFromUnitVectors(
    new THREE.Vector3(0, 1, 0),
    delta.normalize(),
  );
  group.add(mesh);
  return mesh;
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
  const cameraTarget = new THREE.Vector3(0, -0.2, 0);

  scene.add(new THREE.HemisphereLight(0xffffff, 0x5f6875, 1.45));
  const keyLight = new THREE.DirectionalLight(0xffffff, 2.1);
  keyLight.position.set(1.5, 1.8, 1.3);
  scene.add(keyLight);

  const grid = new THREE.GridHelper(1.8, 18, lineColor, lineColor);
  grid.position.y = -0.62;
  grid.material.transparent = true;
  grid.material.opacity = 0.36;
  scene.add(grid);

  const joint = new THREE.Mesh(
    new THREE.SphereGeometry(0.03, 18, 12),
    new THREE.MeshStandardMaterial({ color: COLORS.nozzle }),
  );
  scene.add(joint);

  const targetDown = makeLine(COLORS.target, 0.6);
  setLineEndpoints(
    targetDown,
    new THREE.Vector3(0, 0, 0),
    new THREE.Vector3(0, -0.68, 0),
  );
  scene.add(targetDown);

  const heldHoseLine = makeLine(COLORS.hose, 0.95);
  scene.add(heldHoseLine);

  const handMarker = new THREE.Mesh(
    new THREE.SphereGeometry(0.038, 18, 12),
    new THREE.MeshStandardMaterial({ color: COLORS.hose }),
  );
  scene.add(handMarker);

  const bodyRoot = new THREE.Group();
  scene.add(bodyRoot);

  addCylinderBetween(
    bodyRoot,
    [0, -0.015, 0],
    [0, -0.255, 0],
    0.035,
    COLORS.body,
  );
  addCylinderBetween(
    bodyRoot,
    [0, -0.255, 0],
    [0.045, -0.31, 0.02],
    0.03,
    COLORS.body,
  );

  const head = new THREE.Mesh(
    new THREE.CylinderGeometry(0.11, 0.105, 0.065, 36),
    new THREE.MeshStandardMaterial({
      color: COLORS.head,
      metalness: 0.2,
      roughness: 0.42,
    }),
  );
  head.position.set(0.045, -0.325, 0.02);
  bodyRoot.add(head);

  const nozzlePlate = new THREE.Mesh(
    new THREE.CylinderGeometry(0.092, 0.092, 0.012, 36),
    new THREE.MeshStandardMaterial({
      color: COLORS.nozzle,
      metalness: 0.08,
      roughness: 0.6,
    }),
  );
  nozzlePlate.position.set(0.045, -0.363, 0.02);
  bodyRoot.add(nozzlePlate);

  const comMarker = new THREE.Mesh(
    new THREE.SphereGeometry(0.019, 18, 12),
    new THREE.MeshStandardMaterial({ color: COLORS.com }),
  );
  comMarker.position.set(0, -0.17, 0);
  bodyRoot.add(comMarker);

  const waterStream = makeLine(COLORS.water, 0.92);
  bodyRoot.add(waterStream);

  const reactionArrow = new THREE.ArrowHelper(
    new THREE.Vector3(0, 1, 0),
    new THREE.Vector3(0.045, -0.33, 0.02),
    0.32,
    COLORS.reaction,
    0.075,
    0.038,
  );
  bodyRoot.add(reactionArrow);

  const waterTorqueArrow = new THREE.ArrowHelper(
    new THREE.Vector3(0, 0, 1),
    new THREE.Vector3(0, -0.04, 0),
    0.18,
    COLORS.waterTorque,
    0.055,
    0.03,
  );
  bodyRoot.add(waterTorqueArrow);

  const holdTorqueArrow = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(0, 0.035, 0),
    0.18,
    COLORS.holdTorque,
    0.055,
    0.03,
  );
  bodyRoot.add(holdTorqueArrow);

  const gravityArrow = new THREE.ArrowHelper(
    new THREE.Vector3(0, -1, 0),
    new THREE.Vector3(),
    0.3,
    COLORS.gravity,
    0.07,
    0.035,
  );
  scene.add(gravityArrow);

  const nozzleLocal = new THREE.Vector3(0.045, -0.37, 0.02);
  const comLocal = new THREE.Vector3(0, -0.17, 0);

  function setCameraView(view) {
    const positions = {
      iso: [1.05, 0.35, 1.2],
      front: [0, -0.18, 1.55],
      side: [1.55, -0.18, 0],
      top: [0.001, 1.45, 0.001],
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
    const thrustN = diagnostics.thrustN;
    const reaction = diagnostics.waterReactionForceBodyN;
    const reactionDirection = direction3(reaction);

    reactionArrow.setDirection(reactionDirection);
    reactionArrow.setLength(
      thrustN > 1e-6 ? 0.14 + 0.34 * Math.min(thrustN, 1.2) : 0.001,
      0.075,
      0.038,
    );
    reactionArrow.visible = thrustN > 1e-6;

    const jetDirection = direction3(
      diagnostics.waterJetDirectionBody,
      [0, -1, 0],
    );
    const streamEnd = nozzleLocal.clone().add(
      jetDirection.multiplyScalar(thrustN > 1e-6 ? 0.62 : 0.001),
    );
    setLineEndpoints(waterStream, nozzleLocal, streamEnd);
    waterStream.visible = thrustN > 1e-6;

    const waterTorque = diagnostics.waterReactionTorqueBodyNm;
    const waterTorqueMagnitude = vectorLength(waterTorque);
    waterTorqueArrow.setDirection(direction3(waterTorque, [0, 0, 1]));
    waterTorqueArrow.setLength(
      waterTorqueMagnitude > 1e-7
        ? 0.11 + 2.0 * Math.min(waterTorqueMagnitude, 0.12)
        : 0.001,
      0.055,
      0.03,
    );
    waterTorqueArrow.visible = waterTorqueMagnitude > 1e-7;

    const holdTorque = diagnostics.hoseHoldingTorqueBodyNm;
    const holdTorqueMagnitude = vectorLength(holdTorque);
    holdTorqueArrow.setDirection(direction3(holdTorque, [1, 0, 0]));
    holdTorqueArrow.setLength(
      holdTorqueMagnitude > 1e-7
        ? 0.11 + 2.0 * Math.min(holdTorqueMagnitude, 0.12)
        : 0.001,
      0.055,
      0.03,
    );
    holdTorqueArrow.visible = holdTorqueMagnitude > 1e-7;

    const comWorld = bodyRoot.localToWorld(comLocal.clone());
    gravityArrow.position.copy(comWorld);
    gravityArrow.setDirection(new THREE.Vector3(0, -1, 0));
    gravityArrow.setLength(0.28, 0.07, 0.035);

    const holdDirection = direction3(diagnostics.holdDirectionWorld);
    const handWorld = holdDirection.multiplyScalar(0.30);
    setLineEndpoints(
      heldHoseLine,
      new THREE.Vector3(0, 0, 0),
      handWorld,
    );
    handMarker.position.copy(handWorld);
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

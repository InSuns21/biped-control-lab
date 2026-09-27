import * as THREE from "../../vendor/three.module.js";

export const DEFAULT_GAME3D_LAYOUT = Object.freeze({
  handHeightM: 1.68,
  physicsPlaneZM: 0,
  waterLengthM: 0.55,
});

export const GAME3D_CAMERA_VIEWS = Object.freeze({
  game: Object.freeze({
    position: Object.freeze([1.55, 1.20, 2.65]),
    target: Object.freeze([0, 0.82, 0]),
    fovDeg: 43,
  }),
  front: Object.freeze({
    position: Object.freeze([0, 0.90, 3.0]),
    target: Object.freeze([0, 0.82, 0]),
    fovDeg: 43,
  }),
  close: Object.freeze({
    position: Object.freeze([1.08, 0.90, 1.95]),
    target: Object.freeze([0, 0.78, 0]),
    fovDeg: 55,
  }),
});

export function mapRodPointToGame3D(
  point,
  layout = DEFAULT_GAME3D_LAYOUT,
) {
  return [
    point[0],
    layout.handHeightM - point[1],
    layout.physicsPlaneZM,
  ];
}

export function mapRodVectorToGame3D(vector) {
  return [vector[0], -vector[1], 0];
}

function vec3(values) {
  return new THREE.Vector3(values[0], values[1], values[2]);
}

function setCylinderBetween(
  mesh,
  start,
  end,
  baseLength = 1,
) {
  const delta = end.clone().sub(start);
  const length = Math.max(1e-6, delta.length());
  mesh.position.copy(start).add(end).multiplyScalar(0.5);
  mesh.quaternion.setFromUnitVectors(
    new THREE.Vector3(0, 1, 0),
    delta.clone().normalize(),
  );
  mesh.scale.set(1, length / baseLength, 1);
}

function makeCylinder(radius, color, {
  metalness = 0,
  roughness = 0.6,
  radialSegments = 18,
} = {}) {
  return new THREE.Mesh(
    new THREE.CylinderGeometry(
      radius,
      radius,
      1,
      radialSegments,
    ),
    new THREE.MeshStandardMaterial({
      color,
      metalness,
      roughness,
    }),
  );
}

function makeLine(color, opacity = 1) {
  const geometry = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(),
    new THREE.Vector3(0, -1, 0),
  ]);
  const material = new THREE.LineBasicMaterial({
    color,
    transparent: opacity < 1,
    opacity,
  });
  return new THREE.Line(geometry, material);
}

function setLinePoints(line, points) {
  line.geometry.dispose();
  line.geometry = new THREE.BufferGeometry().setFromPoints(points);
}

function resizeRenderer(renderer, camera, canvas) {
  const width = Math.max(1, Math.floor(canvas.clientWidth));
  const height = Math.max(1, Math.floor(canvas.clientHeight));
  const ratio = renderer.getPixelRatio();
  const rw = Math.floor(width * ratio);
  const rh = Math.floor(height * ratio);

  if (canvas.width !== rw || canvas.height !== rh) {
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
  }
}

export function createGame3DView(
  canvas,
  {
    segmentCapacity = 24,
    layout = DEFAULT_GAME3D_LAYOUT,
  } = {},
) {
  const renderer = new THREE.WebGLRenderer({
    canvas,
    antialias: true,
    alpha: false,
  });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;

  const styles = getComputedStyle(document.documentElement);
  const panelColor =
    styles.getPropertyValue("--panel").trim() || "#ffffff";
  const lineColor =
    styles.getPropertyValue("--line").trim() || "#d9dee7";

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(panelColor);

  const camera = new THREE.PerspectiveCamera(43, 1, 0.02, 12);
  const cameraTarget = new THREE.Vector3(0, 0.82, 0);

  scene.add(new THREE.HemisphereLight(
    0xffffff,
    0x69717d,
    1.45,
  ));
  const key = new THREE.DirectionalLight(0xffffff, 2.3);
  key.position.set(1.7, 2.8, 2.2);
  key.castShadow = true;
  scene.add(key);

  const fill = new THREE.DirectionalLight(0xdde8ff, 0.8);
  fill.position.set(-2, 1.4, 1.3);
  scene.add(fill);

  const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(3.4, 3.0),
    new THREE.MeshStandardMaterial({
      color: 0xdce3e8,
      roughness: 0.8,
    }),
  );
  floor.rotation.x = -Math.PI / 2;
  floor.position.set(0, 0, 0.15);
  floor.receiveShadow = true;
  scene.add(floor);

  const backWall = new THREE.Mesh(
    new THREE.PlaneGeometry(3.4, 2.35),
    new THREE.MeshStandardMaterial({
      color: 0xe9edf1,
      roughness: 0.88,
    }),
  );
  backWall.position.set(0, 1.15, -0.72);
  backWall.receiveShadow = true;
  scene.add(backWall);

  const wallGrid = new THREE.GridHelper(
    3.2,
    16,
    lineColor,
    lineColor,
  );
  wallGrid.rotation.x = Math.PI / 2;
  wallGrid.position.set(0, 1.15, -0.715);
  wallGrid.material.transparent = true;
  wallGrid.material.opacity = 0.18;
  scene.add(wallGrid);

  const floorGrid = new THREE.GridHelper(
    3.0,
    15,
    lineColor,
    lineColor,
  );
  floorGrid.position.y = 0.002;
  floorGrid.material.transparent = true;
  floorGrid.material.opacity = 0.22;
  scene.add(floorGrid);

  const hoseMaterial = new THREE.MeshStandardMaterial({
    color: 0x6f7c88,
    roughness: 0.62,
    metalness: 0.05,
  });
  const hoseSegments = [];
  for (let i = 0; i < segmentCapacity; i += 1) {
    const mesh = new THREE.Mesh(
      new THREE.CylinderGeometry(0.018, 0.018, 1, 16),
      hoseMaterial,
    );
    mesh.castShadow = true;
    mesh.visible = false;
    scene.add(mesh);
    hoseSegments.push(mesh);
  }

  const jointMaterial = new THREE.MeshStandardMaterial({
    color: 0x84909b,
    roughness: 0.55,
  });
  const jointMarkers = [];
  for (let i = 0; i <= segmentCapacity; i += 1) {
    const marker = new THREE.Mesh(
      new THREE.SphereGeometry(0.022, 12, 9),
      jointMaterial,
    );
    marker.visible = false;
    scene.add(marker);
    jointMarkers.push(marker);
  }

  const hand = new THREE.Group();
  const handGrip = makeCylinder(0.035, 0x49a078, {
    roughness: 0.5,
  });
  handGrip.castShadow = true;
  hand.add(handGrip);
  scene.add(hand);

  const targetHand = new THREE.Group();
  const targetGrip = makeCylinder(0.026, 0x2f8f83, {
    roughness: 0.55,
  });
  const targetMaterial = targetGrip.material;
  targetMaterial.transparent = true;
  targetMaterial.opacity = 0.48;
  targetGrip.material = targetMaterial;
  targetHand.add(targetGrip);
  scene.add(targetHand);

  const headNeck = makeCylinder(0.026, 0xaeb7c1, {
    metalness: 0.35,
    roughness: 0.34,
  });
  headNeck.castShadow = true;
  scene.add(headNeck);

  const headDisc = new THREE.Mesh(
    new THREE.CylinderGeometry(0.085, 0.085, 0.045, 32),
    new THREE.MeshStandardMaterial({
      color: 0xd6dde4,
      metalness: 0.45,
      roughness: 0.28,
    }),
  );
  headDisc.castShadow = true;
  scene.add(headDisc);

  const nozzleDisc = new THREE.Mesh(
    new THREE.CylinderGeometry(0.070, 0.070, 0.012, 32),
    new THREE.MeshStandardMaterial({
      color: 0x596675,
      metalness: 0.18,
      roughness: 0.52,
    }),
  );
  scene.add(nozzleDisc);

  const waterLine = makeLine(0x4db7e5, 0.92);
  scene.add(waterLine);

  const equilibriumLine = makeLine(0x7f8b99, 0.46);
  equilibriumLine.material.transparent = true;
  equilibriumLine.material.opacity = 0.5;
  scene.add(equilibriumLine);

  const targetRing = new THREE.Mesh(
    new THREE.TorusGeometry(0.075, 0.007, 12, 40),
    new THREE.MeshBasicMaterial({
      color: 0x2f8f83,
      transparent: true,
      opacity: 0.75,
    }),
  );
  targetRing.position.z = -0.015;
  scene.add(targetRing);

  const waterImpactMarker = new THREE.Mesh(
    new THREE.RingGeometry(0.025, 0.040, 28),
    new THREE.MeshBasicMaterial({
      color: 0x4db7e5,
      side: THREE.DoubleSide,
      transparent: true,
      opacity: 0.8,
    }),
  );
  waterImpactMarker.visible = false;
  scene.add(waterImpactMarker);

  const aimTargetGroup = new THREE.Group();
  aimTargetGroup.renderOrder = 40;

  const aimTargetFill = new THREE.Mesh(
    new THREE.CircleGeometry(1, 64),
    new THREE.MeshBasicMaterial({
      color: 0x2f8f83,
      transparent: true,
      opacity: 0.22,
      side: THREE.DoubleSide,
      depthTest: false,
      depthWrite: false,
    }),
  );
  aimTargetFill.renderOrder = 40;

  const aimTargetRing = new THREE.Mesh(
    new THREE.RingGeometry(0.72, 1, 64),
    new THREE.MeshBasicMaterial({
      color: 0x2f8f83,
      transparent: true,
      opacity: 1,
      side: THREE.DoubleSide,
      depthTest: false,
      depthWrite: false,
    }),
  );
  aimTargetRing.renderOrder = 41;

  const aimTargetOuterRing = new THREE.Mesh(
    new THREE.RingGeometry(1.12, 1.24, 64),
    new THREE.MeshBasicMaterial({
      color: 0xffffff,
      transparent: true,
      opacity: 0.92,
      side: THREE.DoubleSide,
      depthTest: false,
      depthWrite: false,
    }),
  );
  aimTargetOuterRing.renderOrder = 42;

  const aimCrossMaterial = new THREE.MeshBasicMaterial({
    color: 0xffffff,
    transparent: true,
    opacity: 0.95,
    side: THREE.DoubleSide,
    depthTest: false,
    depthWrite: false,
  });
  const aimCrossHorizontal = new THREE.Mesh(
    new THREE.PlaneGeometry(2.70, 0.055),
    aimCrossMaterial,
  );
  const aimCrossVertical = new THREE.Mesh(
    new THREE.PlaneGeometry(0.055, 2.70),
    aimCrossMaterial,
  );
  aimCrossHorizontal.renderOrder = 43;
  aimCrossVertical.renderOrder = 43;

  const aimCenterDot = new THREE.Mesh(
    new THREE.CircleGeometry(0.09, 24),
    new THREE.MeshBasicMaterial({
      color: 0xffffff,
      transparent: true,
      opacity: 1,
      side: THREE.DoubleSide,
      depthTest: false,
      depthWrite: false,
    }),
  );
  aimCenterDot.renderOrder = 44;

  aimTargetGroup.add(
    aimTargetFill,
    aimTargetRing,
    aimTargetOuterRing,
    aimCrossHorizontal,
    aimCrossVertical,
    aimCenterDot,
  );
  aimTargetGroup.visible = false;
  scene.add(aimTargetGroup);

  const aimClosestMarker = new THREE.Mesh(
    new THREE.RingGeometry(0.014, 0.025, 28),
    new THREE.MeshBasicMaterial({
      color: 0xd95c5c,
      side: THREE.DoubleSide,
      transparent: true,
      opacity: 1,
      depthTest: false,
      depthWrite: false,
    }),
  );
  aimClosestMarker.renderOrder = 45;
  aimClosestMarker.visible = false;
  scene.add(aimClosestMarker);

  const reactionArrow = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(),
    0.12,
    0xd68a00,
    0.035,
    0.02,
  );
  scene.add(reactionArrow);

  function setCameraView(view) {
    const config = GAME3D_CAMERA_VIEWS[view]
      ?? GAME3D_CAMERA_VIEWS.game;
    camera.position.set(...config.position);
    cameraTarget.set(...config.target);
    camera.fov = config.fovDeg ?? 43;
    camera.up.set(0, 1, 0);
    camera.lookAt(cameraTarget);
    camera.updateProjectionMatrix();
  }

  function updateRod(nodes) {
    const mapped = nodes.map(
      (point) => vec3(mapRodPointToGame3D(point, layout)),
    );
    const segmentCount = mapped.length - 1;

    for (let i = 0; i < hoseSegments.length; i += 1) {
      const visible = i < segmentCount;
      hoseSegments[i].visible = visible;
      if (visible) {
        setCylinderBetween(
          hoseSegments[i],
          mapped[i],
          mapped[i + 1],
        );
      }
    }

    for (let i = 0; i < jointMarkers.length; i += 1) {
      const visible = i < mapped.length;
      jointMarkers[i].visible = visible;
      if (visible) jointMarkers[i].position.copy(mapped[i]);
    }

    return mapped;
  }

  function updateHand(group, boundary, {
    opacity = 1,
  } = {}) {
    const base = vec3(
      mapRodPointToGame3D(
        [boundary.lateralPositionM ?? 0, 0],
        layout,
      ),
    );
    const angle = boundary.angleRad ?? 0;
    const direction = new THREE.Vector3(
      Math.sin(angle),
      -Math.cos(angle),
      0,
    );
    const end = base.clone().add(
      direction.multiplyScalar(0.17),
    );
    const grip = group.children[0];
    setCylinderBetween(grip, base, end);
    grip.material.opacity = opacity;
    group.visible = true;
  }

  function updateHead(
    currentKinematics,
    reaction,
  ) {
    const tip = vec3(
      mapRodPointToGame3D(currentKinematics.tip, layout),
    );
    const nozzleRod = [
      currentKinematics.tip[0] + reaction.nozzleOffsetWorldM[0],
      currentKinematics.tip[1] + reaction.nozzleOffsetWorldM[1],
    ];
    const nozzle = vec3(
      mapRodPointToGame3D(nozzleRod, layout),
    );

    setCylinderBetween(headNeck, tip, nozzle);

    const outlet = vec3(
      mapRodVectorToGame3D(reaction.outletDirection),
    ).normalize();

    headDisc.position.copy(nozzle);
    headDisc.quaternion.setFromUnitVectors(
      new THREE.Vector3(0, 1, 0),
      outlet,
    );
    nozzleDisc.position.copy(
      nozzle.clone().add(outlet.clone().multiplyScalar(0.027)),
    );
    nozzleDisc.quaternion.copy(headDisc.quaternion);

    const waterEnd = nozzle.clone().add(
      outlet.clone().multiplyScalar(layout.waterLengthM),
    );
    setLinePoints(waterLine, [nozzle, waterEnd]);

    const reactionVector = vec3(
      mapRodVectorToGame3D(reaction.forceXYN),
    );
    const reactionMagnitude = reactionVector.length();
    if (reactionMagnitude > 1e-7) {
      reactionArrow.visible = true;
      reactionArrow.position.copy(nozzle);
      reactionArrow.setDirection(reactionVector.normalize());
      reactionArrow.setLength(
        0.08 + 0.07 * Math.min(reactionMagnitude, 4),
        0.035,
        0.02,
      );
    } else {
      reactionArrow.visible = false;
    }

    return { tip, nozzle, waterEnd };
  }

  function updateEquilibrium(equilibriumKinematics) {
    const points = equilibriumKinematics.nodes.map(
      (point) => vec3(mapRodPointToGame3D(point, layout)),
    );
    setLinePoints(equilibriumLine, points);
    const target = vec3(
      mapRodPointToGame3D(equilibriumKinematics.tip, layout),
    );
    targetRing.position.set(target.x, target.y, -0.02);
  }

  function updateAimTarget(aimTarget, aimSample) {
    if (!aimTarget?.center || !(aimTarget.radiusM > 0)) {
      aimTargetGroup.visible = false;
      aimClosestMarker.visible = false;
      return;
    }

    const targetPosition = vec3(
      mapRodPointToGame3D(aimTarget.center, layout),
    );
    aimTargetGroup.position.copy(targetPosition);
    // The bullseye is a gameplay marker, not a physical plate. Keep it
    // screen-facing so an oblique game camera cannot reduce it to a sliver.
    aimTargetGroup.quaternion.copy(camera.quaternion);
    aimTargetGroup.scale.set(
      aimTarget.radiusM,
      aimTarget.radiusM,
      aimTarget.radiusM,
    );
    aimTargetGroup.visible = true;

    const hit = Boolean(aimSample?.hit);
    const color = hit ? 0x42d989 : 0x00b8d9;
    waterLine.material.color.setHex(hit ? 0x42d989 : 0x4db7e5);
    waterLine.material.opacity = hit ? 1 : 0.92;
    aimTargetFill.material.color.setHex(color);
    aimTargetFill.material.opacity = hit ? 0.28 : 0.14;
    aimTargetRing.material.color.setHex(color);
    aimTargetRing.material.opacity = hit ? 1 : 0.92;

    if (aimSample?.closestPoint) {
      aimClosestMarker.position.copy(
        vec3(
          mapRodPointToGame3D(
            aimSample.closestPoint,
            layout,
          ),
        ),
      );
      aimClosestMarker.quaternion.copy(camera.quaternion);
      aimClosestMarker.material.color.setHex(
        hit ? 0x42d989 : 0xff5a5f,
      );
      aimClosestMarker.visible = true;
    } else {
      aimClosestMarker.visible = false;
    }
  }

  function render({
    currentKinematics,
    equilibriumKinematics,
    reaction,
    handBoundary,
    handTarget,
    aimTarget = null,
    aimSample = null,
  }) {
    resizeRenderer(renderer, camera, canvas);
    updateRod(currentKinematics.nodes);
    updateHand(hand, handBoundary);
    updateHand(targetHand, {
      lateralPositionM: handTarget?.lateralPositionM ?? 0,
      angleRad: handTarget?.angleRad ?? 0,
    }, {
      opacity: 0.42,
    });
    updateHead(currentKinematics, reaction);
    updateEquilibrium(equilibriumKinematics);
    updateAimTarget(aimTarget, aimSample);
    if (aimTargetGroup.visible) {
      aimTargetGroup.quaternion.copy(camera.quaternion);
    }
    if (aimClosestMarker.visible) {
      aimClosestMarker.quaternion.copy(camera.quaternion);
    }
    renderer.render(scene, camera);
  }

  function dispose() {
    renderer.dispose();
  }

  setCameraView("game");

  return {
    render,
    setCameraView,
    dispose,
  };
}

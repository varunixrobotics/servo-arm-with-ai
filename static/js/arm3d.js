/**
 * =============================================================================
 * S.A.M. (Servo Arm Manipulator) - 3D Digital Twin (Three.js)
 * =============================================================================
 * Precisely models the user's 5-axis 3D-printed robotic arm:
 * - White Cylindrical Base Pedestal & Flange
 * - Royal Blue Rotating Base Turret (Joint 1: Pan)
 * - Royal Blue Shoulder Linkage (Joint 2: Shoulder Pitch)
 * - Royal Blue Forearm Linkage (Joint 3: Elbow Pitch)
 * - White Wrist Plate & Interlocking Gear Mechanism (Joint 4: Wrist Pitch)
 * - 3-Claw Articulated Gripper with Blue Fingers (Joint 5: Gripper)
 * =============================================================================
 */

class Arm3DVisualizer {
    constructor(containerId) {
        this.container = document.getElementById(containerId);
        if (!this.container) return;

        this.width = this.container.clientWidth || 480;
        this.height = this.container.clientHeight || 420;

        // Current Joint Angles (degrees: Base, Shoulder, Elbow, Wrist Pitch, Wrist Roll, Gripper)
        this.angles = [90, 90, 90, 90, 90, 100];

        // Materials matching user's arm photo:
        // Royal Blue Plastic: #0066FF / #0052cc
        this.matBlue = new THREE.MeshStandardMaterial({
            color: 0x0066ff,
            roughness: 0.35,
            metalness: 0.15,
            flatShading: false
        });
        // Matte White Plastic: #f4f6fa
        this.matWhite = new THREE.MeshStandardMaterial({
            color: 0xf4f6fa,
            roughness: 0.45,
            metalness: 0.05
        });
        // Dark accents (screws / bearings)
        this.matMetal = new THREE.MeshStandardMaterial({
            color: 0x7a8290,
            roughness: 0.2,
            metalness: 0.8
        });
        // Gripper accent blue
        this.matClaw = new THREE.MeshStandardMaterial({
            color: 0x004ecc,
            roughness: 0.3,
            metalness: 0.2
        });

        this.initScene();
        this.buildRobotArm();
        this.animate = this.animate.bind(this);
        this.animate();

        window.addEventListener('resize', () => this.onWindowResize());
    }

    initScene() {
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x0a0e17);

        // Grid floor
        const grid = new THREE.GridHelper(40, 40, 0x00e5ff, 0x1b2434);
        grid.position.y = -0.05;
        this.scene.add(grid);

        // Lighting
        const ambient = new THREE.AmbientLight(0xffffff, 0.65);
        this.scene.add(ambient);

        const spotLight = new THREE.DirectionalLight(0xffffff, 0.9);
        spotLight.position.set(15, 25, 20);
        spotLight.castShadow = true;
        this.scene.add(spotLight);

        const rimLight = new THREE.DirectionalLight(0x00c6ff, 0.5);
        rimLight.position.set(-15, 10, -15);
        this.scene.add(rimLight);

        // Camera
        this.camera = new THREE.PerspectiveCamera(42, this.width / this.height, 0.1, 1000);
        this.camera.position.set(22, 18, 25);

        // Renderer
        this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
        this.renderer.setSize(this.width, this.height);
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        this.renderer.shadowMap.enabled = true;
        this.container.appendChild(this.renderer.domElement);

        // Orbit Controls
        if (window.THREE && window.THREE.OrbitControls) {
            this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
            this.controls.enableDamping = true;
            this.controls.dampingFactor = 0.05;
            this.controls.target.set(0, 7, 0);
            this.controls.maxPolarAngle = Math.PI / 2 + 0.1;
        }
    }

    buildRobotArm() {
        this.rootGroup = new THREE.Group();
        this.scene.add(this.rootGroup);

        // 1. Stationary Base (White Flange & Cylinder)
        const baseFlangeGeo = new THREE.CylinderGeometry(4.2, 4.4, 0.3, 32);
        const baseFlange = new THREE.Mesh(baseFlangeGeo, this.matWhite);
        baseFlange.position.y = 0.15;
        this.rootGroup.add(baseFlange);

        const baseCylinderGeo = new THREE.CylinderGeometry(3.2, 3.2, 3.2, 32);
        const baseCylinder = new THREE.Mesh(baseCylinderGeo, this.matWhite);
        baseCylinder.position.y = 1.9;
        this.rootGroup.add(baseCylinder);

        // 2. Joint 1: Base Turret (Pan Rotation - Yaw)
        this.j1_Turret = new THREE.Group();
        this.j1_Turret.position.set(0, 3.5, 0);
        this.rootGroup.add(this.j1_Turret);

        const turretPlateGeo = new THREE.CylinderGeometry(3.3, 3.3, 0.6, 32);
        const turretPlate = new THREE.Mesh(turretPlateGeo, this.matBlue);
        turretPlate.position.y = 0.3;
        this.j1_Turret.add(turretPlate);

        // Shoulder Hinge Bracket (Blue side ears)
        const shoulderBracketGeo = new THREE.CylinderGeometry(1.6, 1.6, 2.8, 24);
        const shoulderBracket = new THREE.Mesh(shoulderBracketGeo, this.matBlue);
        shoulderBracket.rotation.z = Math.PI / 2;
        shoulderBracket.position.set(0, 1.6, 0);
        this.j1_Turret.add(shoulderBracket);

        // 3. Joint 2: Shoulder (Pitch)
        this.j2_Shoulder = new THREE.Group();
        this.j2_Shoulder.position.set(0, 1.6, 0);
        this.j1_Turret.add(this.j2_Shoulder);

        // Upper Arm Link (Blue Chamfered Arm with cutouts)
        const upperArmGeo = new THREE.BoxGeometry(1.8, 7.5, 1.4);
        const upperArm = new THREE.Mesh(upperArmGeo, this.matBlue);
        upperArm.position.set(0, 3.75, 0);
        this.j2_Shoulder.add(upperArm);

        // Joint pivot covers
        const jointPivotGeo = new THREE.CylinderGeometry(1.4, 1.4, 1.6, 24);
        const j2Cap = new THREE.Mesh(jointPivotGeo, this.matBlue);
        j2Cap.rotation.z = Math.PI / 2;
        this.j2_Shoulder.add(j2Cap);

        // 4. Joint 3: Elbow (Pitch)
        this.j3_Elbow = new THREE.Group();
        this.j3_Elbow.position.set(0, 7.5, 0);
        this.j2_Shoulder.add(this.j3_Elbow);

        // Elbow Pivot
        const elbowPivotGeo = new THREE.CylinderGeometry(1.3, 1.3, 1.5, 24);
        const j3Cap = new THREE.Mesh(elbowPivotGeo, this.matBlue);
        j3Cap.rotation.z = Math.PI / 2;
        this.j3_Elbow.add(j3Cap);

        // Forearm Link (Blue)
        const forearmGeo = new THREE.BoxGeometry(1.6, 6.5, 1.2);
        const forearm = new THREE.Mesh(forearmGeo, this.matBlue);
        forearm.position.set(0, 3.25, 0);
        this.j3_Elbow.add(forearm);

        // 5. Joint 4: Wrist Pitch
        this.j4_WristPitch = new THREE.Group();
        this.j4_WristPitch.position.set(0, 6.5, 0);
        this.j3_Elbow.add(this.j4_WristPitch);

        // White Wrist Bracket (Matches user's white 3D printed linkage)
        const wristMountGeo = new THREE.BoxGeometry(2.0, 1.4, 1.5);
        const wristMount = new THREE.Mesh(wristMountGeo, this.matWhite);
        wristMount.position.set(0, 0.7, 0);
        this.j4_WristPitch.add(wristMount);

        // 6. Joint 5: Wrist Roll (Rotation around wrist axis)
        this.j5_WristRoll = new THREE.Group();
        this.j5_WristRoll.position.set(0, 1.4, 0);
        this.j4_WristPitch.add(this.j5_WristRoll);

        // Dual White Interlocking Gears
        const gearGeo = new THREE.CylinderGeometry(0.7, 0.7, 0.35, 16);
        const gear1 = new THREE.Mesh(gearGeo, this.matWhite);
        gear1.rotation.x = Math.PI / 2;
        gear1.position.set(-0.6, 0.3, 0.5);
        this.j5_WristRoll.add(gear1);

        const gear2 = new THREE.Mesh(gearGeo, this.matWhite);
        gear2.rotation.x = Math.PI / 2;
        gear2.position.set(0.6, 0.3, 0.5);
        this.j5_WristRoll.add(gear2);

        // 7. Joint 6: Gripper Claws (Articulated Multi-Finger Gripper)
        this.j6_Gripper = new THREE.Group();
        this.j6_Gripper.position.set(0, 0.4, 0);
        this.j5_WristRoll.add(this.j6_Gripper);

        // Claw Left (2 Blue fingers linked)
        this.clawLeft = new THREE.Group();
        this.clawLeft.position.set(-0.7, 0.2, 0);
        this.j6_Gripper.add(this.clawLeft);

        const fingerGeo1 = new THREE.BoxGeometry(0.35, 2.6, 0.4);
        const fingerL1 = new THREE.Mesh(fingerGeo1, this.matClaw);
        fingerL1.position.set(-0.2, 1.3, -0.35);
        fingerL1.rotation.z = -0.15;
        this.clawLeft.add(fingerL1);

        const fingerL2 = new THREE.Mesh(fingerGeo1, this.matClaw);
        fingerL2.position.set(-0.2, 1.3, 0.35);
        fingerL2.rotation.z = -0.15;
        this.clawLeft.add(fingerL2);

        // Claw Right (Opposing Blue finger)
        this.clawRight = new THREE.Group();
        this.clawRight.position.set(0.7, 0.2, 0);
        this.j6_Gripper.add(this.clawRight);

        const fingerR = new THREE.Mesh(fingerGeo1, this.matClaw);
        fingerR.position.set(0.2, 1.3, 0);
        fingerR.rotation.z = 0.15;
        this.clawRight.add(fingerR);

        // 4 White Visual Servoing Dots on Gripper
        const dotGeo = new THREE.SphereGeometry(0.12, 12, 12);
        const dotMat = new THREE.MeshBasicMaterial({ color: 0xffffff });
        const dotPositions = [
            [-0.7, 0.4, -0.5],
            [0.7, 0.4, -0.5],
            [-0.7, 0.4, 0.5],
            [0.7, 0.4, 0.5]
        ];
        dotPositions.forEach(pos => {
            const dot = new THREE.Mesh(dotGeo, dotMat);
            dot.position.set(...pos);
            this.j6_Gripper.add(dot);
        });

        // Apply initial angles
        this.setAngles(this.angles);
    }

    setAngles(angles) {
        if (!angles || angles.length < 5) return;
        this.angles = [...angles];

        // Convert angles from Servo degrees (0-180) to 3D Kinematics radians
        // J1: Base Pan (90 deg = Center / Forward)
        const j1_rad = THREE.MathUtils.degToRad(this.angles[0] - 90);
        this.j1_Turret.rotation.y = -j1_rad;

        // J2: Shoulder (90 deg = Upright)
        const j2_rad = THREE.MathUtils.degToRad(this.angles[1] - 90);
        this.j2_Shoulder.rotation.z = j2_rad;

        // J3: Elbow (90 deg = Right Angle)
        const j3_rad = THREE.MathUtils.degToRad(this.angles[2] - 90);
        this.j3_Elbow.rotation.z = j3_rad;

        // J4: Wrist Pitch (90 deg = In-line)
        const j4_rad = THREE.MathUtils.degToRad(this.angles[3] - 90);
        this.j4_WristPitch.rotation.z = j4_rad;

        // J5: Wrist Roll (90 deg = Neutral rotation)
        const j5_deg = this.angles.length >= 6 ? this.angles[4] : 90;
        const j5_rad = THREE.MathUtils.degToRad(j5_deg - 90);
        this.j5_WristRoll.rotation.y = j5_rad;

        // J6: Gripper (20 = Closed, 100+ = Open)
        const gripAngle = this.angles.length >= 6 ? this.angles[5] : this.angles[4];
        const gripFactor = THREE.MathUtils.clamp((gripAngle - 20) / 80, 0, 1);
        const clawAngle = THREE.MathUtils.degToRad(10 + gripFactor * 32);
        this.clawLeft.rotation.z = -clawAngle;
        this.clawRight.rotation.z = clawAngle;
    }

    animate() {
        requestAnimationFrame(this.animate);
        if (this.controls) {
            this.controls.update();
        }
        this.renderer.render(this.scene, this.camera);
    }

    onWindowResize() {
        if (!this.container) return;
        this.width = this.container.clientWidth;
        this.height = this.container.clientHeight;
        this.camera.aspect = this.width / this.height;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(this.width, this.height);
    }
}

window.Arm3DVisualizer = Arm3DVisualizer;

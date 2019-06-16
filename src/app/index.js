const math = require("mathjs");
const p5 = require("p5");
const THREE = require("three");
const { KinematicTree } = require("./kinematic_tree");
const { OrbitControls } = require("three/examples/jsm/controls/OrbitControls.js");
const { PerspectiveCursor } = require("./perspective_cursor");
const { UrdfLoader } = require("./urdf_loader");

/**
 * A closure that provides the basic "App" for displaying a kinematic tree.
 * @param {p5} p 
 */
let sketch = (p) => {
    /**@type {Document} */
    let xml_urdf;
    /**@type {THREE.Scene} */
    let scene;
    /**@type {THREE.PerspectiveCamera} */
    let camera;
    /**@type {OrbitControls} */
    let controls;
    /**@type {PerspectiveCursor} */
    let poker;
    /**@type {string} */
    let poked_name;
    /**@type {string} */
    let local_poke_point;
    /**@type {THREE.Raycaster} */
    let raycaster;
    /**@type {THREE.WebGLRenderer} */
    let renderer;
    /**@type {KinematicTree} */
    let kinematic_tree;

    p.preload = () => {
        const xml_prefix = "xml/kuka_iiwa/";
        let xml_filename = xml_prefix + "model.urdf";
        let xhr = new XMLHttpRequest();
        xhr.open("GET", xml_filename, /*async*/ false);
        xhr.onerror = function () { console.log("Failed to load: " + xml_filename); }
        xhr.onload = function () {
            // Create a new scene.
            scene = new THREE.Scene();
            scene.background = new THREE.Color("white");
            kinematic_tree = UrdfLoader.createKinematicTree(xhr.responseXML, scene, xml_prefix);
        }
        xhr.send();
    }
    /**
     * Setup the p5js rendering context.
     */
    p.setup = () => {
        p.noCanvas()
        // Setup the renderer.
        renderer = new THREE.WebGLRenderer();
        renderer.setSize(0.9 * p.windowWidth, 0.9 * p.windowHeight);
        document.body.appendChild(renderer.domElement);
        // Create a new camera.
        camera = new THREE.PerspectiveCamera(75, p.windowWidth / p.windowHeight, 0.001, 100.0);
        // Attach some controls.
        controls = new OrbitControls(camera, renderer.domElement);
        controls.dampingFactor = 0.2;
        controls.addEventListener("change", render);
        // Setup the raycaster.
        raycaster = new THREE.Raycaster();
        poker = new PerspectiveCursor(scene, 10);
        poker.setVisibility(false);
        // Parse the kinematic tree.
        let tip_frame_name = Array.from(kinematic_tree.frames.keys()).pop();
        console.log("Position of " + tip_frame_name + ": " + kinematic_tree.getWorldPose(tip_frame_name).getTranslation().format(4));
        // Finish the geometry setup.
        webGlInit();
        // Hold some items in global scope so they are accessible from the console.
        window.kinematic_tree = kinematic_tree;
        console.log("Finished setup.");
    }

    /**
     * Setup the p5js draw loop.
     */
    p.draw = () => {
        controls.update();
        render();
    }

    /**
     * Handle key-press events.
     */
    p.keyPressed = () => {
        if (p.keyCode === p.SHIFT) {
            updateRaycaster();
            let intersectingBodies = raycaster.intersectObjects(scene.children, true);
            for (let body of intersectingBodies) {
                if (body.object.type === "Mesh") {
                    poker.setVisibility(true);
                    poked_name = body.object.parent.name;
                    local_poke_point = kinematic_tree.getWorldPose(poked_name).inv().transform(math.matrix(body.point.toArray()));
                    console.log("locally poked: " + local_poke_point.format());
                    console.log("selected: " + poked_name);
                    break;
                }
            }
        }
    }

    p.keyReleased = () => {
        if (p.keyCode === p.SHIFT) {
            poker.setVisibility(false);
        }
    }

    /**
     * Set the target and source for the raycasting.
     */
    function updateRaycaster() {
        const top = renderer.domElement.offsetTop;
        const left = renderer.domElement.offsetLeft;
        const height = renderer.domElement.height;
        const width = renderer.domElement.width
        let mouse = new THREE.Vector2(2 * (p.winMouseX - left) / width - 1, -2 * (p.winMouseY - top) / height + 1);
        raycaster.setFromCamera(mouse, camera);
        if (poked_name) {
            let global_poke_point = kinematic_tree.getWorldPose(poked_name).transform(local_poke_point);
            poker.setSpaceOrigin(new THREE.Vector3(global_poke_point.get([0]), global_poke_point.get([1]), global_poke_point.get([2])));
        }
        poker.setSpacePoint(mouse, camera);
    }

    /**
     * This is where the user specific drawing occurs.
     */
    function render() {
        if (p.keyIsDown(p.SHIFT)) {
            updateRaycaster();
            if (kinematic_tree.frames.has(poked_name) && poker.visible) {
                let jacobian = kinematic_tree.getJacobian(poked_name);
                let force = math.zeros(6, 1);
                force.set([0, 0], poker.cursor.direction.x);
                force.set([1, 0], poker.cursor.direction.y);
                force.set([2, 0], poker.cursor.direction.z);
                let torque = math.multiply(math.transpose(jacobian), force);
                let angles = kinematic_tree.getCoordinates();
                angles = math.add(angles, math.squeeze(torque));
                kinematic_tree.setCoordinates(angles.toArray());
            }
        }

        renderer.render(scene, camera);
    }

    /**
     * This is where the user specific render setup occurs.
     */
    function webGlInit() {
        // Setup the camera
        let camera_distance = 1.5
        camera.position.x = camera_distance;
        camera.position.y = camera_distance;
        camera.position.z = camera_distance;
        camera.lookAt(new THREE.Vector3(0.0, 0.0, 0.0));
        camera.up = new THREE.Vector3(0.0, 0.0, 1.0);
        // Add some diffuse lighting.
        const skyColor = 0xB1E1FF;  // light blue
        const groundColor = 0xB97A20;  // brownish orange
        const intensity = 0.5;
        const hemisphereLight = new THREE.HemisphereLight(skyColor, groundColor, intensity);
        hemisphereLight.position.copy(new THREE.Vector3(0, 0, 1));
        scene.add(hemisphereLight);
        const directionalColor = 0xFFFFFF;
        const directionalLight = new THREE.DirectionalLight(directionalColor, intensity);
        directionalLight.position.set(0, 0, 10 * camera_distance);
        directionalLight.target.position.set(0, 0, 0);
        scene.add(directionalLight);
        scene.add(directionalLight.target);
        // Add an origin frame.
        let axes_scale = 0.02;
        let axes = new THREE.AxesHelper(axes_scale);
        scene.add(axes);

        console.log("Finished webGlInit.");
    }
};

p5_sketch = new p5(sketch);

const { KinematicTree } = require("./kinematic_tree");
const { Pose } = require("./pose");
const math = require('mathjs');
const THREE = require("three");

class CoordinateFrame {
    /**
     * @param {THREE.Scene} current_scene The scene to attach this frame to.
     * @param {number} [scale] The size of the coordinate frame.
     */
    constructor(current_scene, scale = 0.01) {
        /**
         * @property {THREE.AxesHelper} axes The underlying axes object.
         */
        this.axes = new THREE.AxesHelper(scale);
        current_scene.add(this.axes);
    }

    /**
     * @param {Pose} world_pose_local 
     */
    setWorldTransform(world_pose_local) {
        this.axes.matrix.identity();
        let m = new THREE.Matrix4();
        m.fromArray(math.transpose(world_pose_local.homogeneous_transform).reshape([16]).toArray());
        this.axes.applyMatrix(m);
        this.axes.matrixAutoUpdate = false;
    }
}

/**A frame that implements a rotation primitive for a DOF about the x-axis.
 * @extends {KinematicTree}
 */
class VisualTree extends KinematicTree {
    constructor(scene) {
        super();
        /** @property {THREE.Scene} scene The scene to attach frames to. */
        this.scene = scene;
        /** @property {Map<string, THREE.Object3D>} visuals The frames of the kinematic tree. */
        this.visuals = new Map();
        /** @property {number} frameSize The size of the visual coordinate frames. */
        this.frameSize = 0.05;
    }

    /**
     * Add a visual feature to an existing frame.
     * @param {string} frameName The name of the frame to attach the visual object to.
     * @param {THREE.Object3D} frameObject The Threejs renderable object to attach.
     * @param {Pose} [frame_pose_object] The transform of the visual feature relative to the frame.
     */
    addVisual(frameName, frameObject, frame_pose_object = new Pose()) {
        if (this.frames.has(frameName)) {
            this.visuals.set(frameName, new CoordinateFrame(this.scene, this.frameSize));
            if (frameObject !== null) {
                this.visuals.get(frameName).axes.attach(frameObject);
                frameObject.matrix.identity();
                let m = new THREE.Matrix4();
                m.fromArray(math.transpose(frame_pose_object.homogeneous_transform).reshape([16]).toArray());
                frameObject.applyMatrix(m);
            }
        }
    }

    /**@override */
    update() {
        super.update();
        for (let [frameName, visual] of this.visuals.entries()) {
            const frame = this.frames.get(frameName);
            visual.setWorldTransform(frame.world_pose_child);
        }
    }
}

exports.VisualTree = VisualTree;
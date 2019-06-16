const { Frame } = require("./frame");
const { Pose } = require("./pose");
const math = require('mathjs');

class KinematicTree {
    constructor() {
        /** @property {Frame} spatial The inertial (root) frame for the tree. */
        this.spatial = new Frame(null, new Pose());
        /** @property {Map<string, Frame>} frames The frames of the kinematic tree. */
        this.frames = new Map();
        /** @property {Array<Frame>} coordinates The set of frames that make up the degrees-of-freedom. */
        this.coordinates = [];
    }
    /**
     * Updates the kinematics and cached frames.
     */
    update() {
        this.spatial.update();
    }
    /**
     * Adds a frame to the kinematic tree.
     * @param {string} name The name of the frame to add.
     * @param {function(Frame):Frame} frame_constructor A function that will take a parent and return a new frame.
     * @param {string} [parent_name] The name of the frame to give the frame_constructor, defaults to root.
     */
    addFrame(name, frame_constructor, parent_name = null) {
        const parent = parent_name === null ? this.spatial : this.frames.get(parent_name);
        this.frames.set(name, frame_constructor(parent));
        if (!this.frames.get(name).is_fixed) {
            this.coordinates.push(this.frames.get(name));
        }
    }
    /**
     * Set the position for each coordinate from an array.
     * @param {Array<number>} vector Coordinate positions as indexed.
     */
    setCoordinates(vector) {
        for (let [index, frame] of this.coordinates.entries()) {
            frame.q = vector[index];
        }
        this.update();
    }
    /**
     * Get the current value for the positions as an array.
     * @returns {Array<number>} The coordinate positions as indexed.
     */
    getCoordinates() {
        let coords = [];
        for (let frame of this.coordinates) {
            coords.push(frame.q);
        }
        return coords;
    }
    /**
     * Get the world_pose_name pose in the current configuration.
     * @param {string} name The name of the frame.
     * @returns {Pose} A copy of the world_pose_name frame.
     */
    getWorldPose(name) {
        return this.frames.get(name).world_pose_child.clone();
    }
    /**
     * Get the jacobian for the specfied frame mapping, x_dot = J * q_dot, in world coordinates.
     * @param {string} name The name of the frame.
     * @param {Array<number>} point A point to take the Jacobian about, defaults to origin.
     * @returns {math.Matrix} The 6xN jacobian for the given frame.
     */
    getJacobian(name, point = []) {
        let body_frame = this.frames.get(name);
        let jacobian = [];
        for (let frame of this.coordinates) {
            jacobian.push(body_frame.parPoseQ(frame).unhat().toArray());
        }
        if (point.length === 3) {
            for (let col of jacobian) {
                let relative_velocity = math.cross(col.slice(0, 3), point);
                let partial_velocity = math.add(col.slice(0, 3), relative_velocity);
                col.splice(0, 3, ...partial_velocity);
            }
        }
        return math.transpose(math.matrix(jacobian));
    }
    /**
     * Get the jacobian for the specfied frame mapping, x_dot = J * q_dot, in body coordinates.
     * @param {string} name The name of the frame.
     * @returns {math.Matrix} The 6xN jacobian for the given frame.
     */
    getBodyJacobian(name) {
        let body_frame = this.frames.get(name);
        let child_pose_world = body_frame.world_pose_child.inv();
        let jacobian = [];
        for (let frame of this.coordinates) {
            jacobian.push(child_pose_world.multiply(body_frame.parPoseQ(frame)).unhat().toArray());
        }
        return math.transpose(math.matrix(jacobian));
    }
}
exports.KinematicTree = KinematicTree;

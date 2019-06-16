const { Twist } = require("./twist");
const { Pose } = require("./pose");
const math = require('mathjs');

/**
 * Describes the spatial kinematics for tree-like chains. This base class is useful for a fixed frame.
 */
class Frame {
    /**
     * @param {Frame} parent The parent frame defining the basis.
     * @param {Pose} parent_pose_child The relative transform between this frame and its parent.
     */
    constructor(parent, parent_pose_child) {
        /**
         * @property {Frame} parent The parent frame defining the basis.
         */
        this.parent = parent;
        /**
         * @property {Frame[]} children A list of frames.
         */
        this.children = [];
        /**
         * @property {bool} is_fixed Defines if this frame is considered to be fixed.
         */
        this.is_fixed = true;
        /**
         * @property {Pose} world_pose_child The local transform between this frame and the world.
         */
        this.world_pose_child = new Pose();
        /**
         * @property {Pose} parent_pose_child The local transform between this frame and its parent.
         */
        this.parent_pose_child = parent_pose_child.clone();
        /**
         * @property {Twist} parent_dif1_child First partial w.r.t. this frame.
         */
        this.parent_dif1_child = new Twist();
        /**
         * @property {Twist} parent_dif2_child Second partial w.r.t. this frame.
         */
        this.parent_dif2_child = new Twist();
        /**
         * @property {Pose} child_pose_parent Cache of the inverse for parent_pose_child.
         */
        this.child_pose_parent = parent_pose_child.inv();
        /**
         * @property {Twist} child_dif1_parent Cache of the inverse for parent_dif1_child.
         */
        this.child_dif1_parent = new Twist();
        /**
         * @property {Twist} child_dif2_parent Cache of the inverse for parent_dif2_child.
         */
        this.child_dif2_parent = new Twist();
        /**
         * @property {Twist} parent_twist_child The local twist velocity for this frame.
         */
        this.parent_twist_child = new Twist();
        /**
         * @property {Twist} world_twist_child The global velocity of this frame.
         */
        this.world_twist_child = new Twist();
        /**
         * @property {number} q The generalized coordinate for this frame.
         */
        this.q = 0;
        /**
         * @property {number} q_dot The generalized speed for this frame.
         */
        this.q_dot = 0;
        // Attach to its parent frame.
        if (this.parent !== null) {
            this.parent.children.push(this);
        }
    }
    /**
     * Update all cached matrices and their children. For best affect only call this function from the "Spatial Frame" (the most parent frame).
     */
    update() {
        this.updateLocal();
        if (this.parent === null) {
            this.world_pose_child = this.parent_pose_child.clone();
            this.world_twist_child = new Twist();
        }
        else {
            this.world_pose_child = this.parent.world_pose_child.multiply(this.parent_pose_child);
            let world_twist_parent = this.child_pose_parent.multiply(this.parent.world_twist_child.multiply(this.parent_pose_child));
            if (this.is_fixed) {
                this.world_twist_child = world_twist_parent;
            }
            else {
                this.world_twist_child = world_twist_parent.add(this.parent_twist_child.scale(this.q_dot));
            }
        }
        for (let child of this.children) {
            child.update();
        }
    }
    /**
     * Update the local cache. Does nothing for a fixed frame.
     */
    updateLocal() { }
    /**
     * First partial of pose w.r.t. to q in global coordinates.
     * @param {Frame} i_frame The i-th frame to take the partial with respect to.
     * @returns {Twist} The result of diff(P, q_i).
     */
    parPoseQ(i_frame) {
        if (this.parent === null) {
            return new Twist();
        }
        else {
            if (i_frame === this) {
                return this.parent.world_pose_child.multiply(this.parent_dif1_child);
            }
            else {
                return this.parent.parPoseQ(i_frame).multiply(this.parent_pose_child);
            }
        }
    }
    /**
     * Second partial of pose w.r.t. to q in global coordinates.
     * @param {Frame} i_frame The i-th frame to take the partial with respect to.
     * @param {Frame} j_frame The j-th frame to take the partial with respect to.
     * @returns {Twist} The result of diff2(P, q_i, q_j).
     */
    par2PoseQQ(i_frame, j_frame) {
        if (this.parent === null) {
            return new Twist();
        }
        else {
            if (i_frame === this && j_frame === this) {
                return this.parent.world_pose_child.multiply(this.parent_dif2_child);
            }
            else if (i_frame === this && j_frame !== this) {
                return this.parent.parPoseQ(j_frame).multiply(this.parent_dif1_child);
            }
            else if (i_frame !== this && j_frame === this) {
                return this.parent.parPoseQ(i_frame).multiply(this.parent_dif1_child);
            }
            else {
                return this.parent.par2PoseQQ(i_frame, j_frame).multiply(this.parent_pose_child);
            }
        }
    }
    /**
     * First partial of twist w.r.t. to q in global coordinates.
     * @param {Frame} i_frame The i-th frame to take the partial with respect to.
     * @returns {Twist} The result of diff(V, q_i).
     */
    parTwistQ(i_frame) {
        if (this.parent === null) {
            return new Twist();
        }
        else {
            if (i_frame === this) {
                const left = this.child_dif1_parent.multiply(this.parent.world_twist_child.multiply(this.parent_pose_child));
                const right = this.child_pose_parent.multiply(this.parent.world_twist_child.multiply(this.parent_dif1_child));
                return math.add(left, right);
            }
            else {
                return this.child_pose_parent.multiply(this.parent.parTwistQ(i_frame).multiply(this.parent_pose_child));
            }
        }
    }
    /**
     * Second partial of twist w.r.t. to q in global coordinates.
     * @param {Frame} i_frame The i-th frame to take the partial with respect to.
     * @param {Frame} j_frame The j-th frame to take the partial with respect to.
     * @returns {Twist} The result of diff2(V, q_i, q_j).
     */
    par2TwistQQ(i_frame, j_frame) {
        if (this.parent === null) {
            return new Twist();
        }
        else {
            if (i_frame === this && j_frame === this) {
                const left = this.child_dif2_parent.multiply(this.parent.world_twist_child.multiply(parent_pose_child));
                const mixed = this.child_dif1_parent.multiply(this.parent.world_twist_child.multiply(parent_dif1_child));
                const right = this.child_pose_parent.multiply(this.parent.world_twist_child.multiply(parent_dif2_child));
                return left.add(right.add(mixed.scale(2.0)));
                return math.add(math.add(left, right).multiply(2.0, mixed));
            }
            else if (i_frame === this && j_frame !== this) {
                const left = this.child_dif1_parent.multiply(this.parent.parTwistQ(j_frame).multiply(parent_pose_child));
                const right = this.child_pose_parent.multiply(this.parent.parTwistQ(j_frame).multiply(parent_dif1_child));
                return left.add(right);
            }
            else if (i_frame !== this && j_frame === this) {
                const left = this.child_dif1_parent.multiply(this.parent.parTwistQ(i_frame).multiply(parent_pose_child));
                const right = this.child_pose_parent.multiply(this.parent.parTwistQ(i_frame).multiply(parent_dif1_child));
                return left.add(right);
            }
            else {
                return this.child_pose_parent.multiply(this.parent.par2TwistQQ(i_frame, j_frame).multiply(parent_pose_child));
            }
        }
    }
    /**
     * First partial of twist w.r.t. to q_dot in global coordinates.
     * @param {Frame} i_frame The i-th frame to take the partial with respect to.
     * @returns {Twist} The result of diff(V, dq_i).
     */
    parTwistQdot(i_frame) {
        if (this.parent === null) {
            return new Twist();
        }
        else {
            if (i_frame === this) {
                return this.parent_twist_child.clone();
            }
            else {
                return this.child_pose_parent.multiply(this.parent.parTwistQdot(i_frame).multiply(this.parent_pose_child));
            }
        }
    }
    /**
     * Second partial of twist w.r.t. to q_dot in global coordinates.
     * @param {Frame} i_frame The i-th frame to take the partial with respect to.
     * @param {Frame} j_frame The j-th frame to take the partial with respect to.
     * @returns {Twist} The result of diff2(V, qd_i, qd_j). Always zero.
     */
    par2TwistQdotQdot(i_frame, j_frame) {
        return new Twist();
    }
    /**
     * Mixed partial of twist w.r.t. to q and q_dot in global coordinates.
     * @param {Frame} q_frame The q frame to take the partial with respect to.
     * @param {Frame} q_dot_frame The q_dot frame to take the partial with respect to.
     * @returns {Twist} The result of diff2(V, q_i, qd_j).
     */
    par2TwistQQdot(q_frame, q_dot_frame) {
        if (this.parent === null) {
            return new Twist();
        }
        else {
            if (q_frame === this) {
                const left = this.child_dif1_parent.multiply(this.parent.parTwistQdot(qdot_frame).multiply(parent_pose_child));
                const right = this.child_pose_parent.multiply(this.parent.parTwistQdot(qdot_frame).multiply(parent_dif1_child));
                return math.add(left, right);
            }
            else if (q_dot_frame === this) {
                return new Twist();
            }
            else {
                return this.child_pose_parent.multiply(this.parent.par2TwistQQdot(q_frame, q_dot_frame).multiply(parent_pose_child));
            }
        }
    }
}

/**
 * A static builder for creating a frame from a translation vector.
 * @param {Frame} parent The frame to set as the basis.
 * @param {number[]} translation The relative translation between this frame and the parent in the parent's coordinates.
 * @returns {Frame} A new fixed frame.
 */
function createFrameFromTranslation(parent, translation) {
    let parent_pose_child = new Pose();
    parent_pose_child.setTranslation(translation);
    return new Frame(parent, parent_pose_child);
}

/** @const */
exports.Frame = Frame;
exports.createFrameFromTranslation = createFrameFromTranslation;


const { Frame } = require("./frame");
const { Pose } = require("./pose");
const math = require('mathjs');

/**A frame that implements a rotation primitive for a DOF about the x-axis.
 * @extends {Frame}
 */
class JointRotationX extends Frame {
    /**
     * @param {Frame} parent The parent frame defining the basis.
     * @param {number} [initial_q] The initial position for the joint.
     * @param {number} [initial_q_dot] The initial velocity for the joint.
     */
    constructor(parent, initial_q = 0.0, initial_q_dot = 0.0) {
        super(parent, new Pose());
        this.q = initial_q;
        this.q_dot = initial_q_dot;
        this.is_fixed = false;
    }
    /** @override */
    updateLocal() {
        const cq = math.cos(this.q);
        const sq = math.sin(this.q);
        this.parent_pose_child.set([1, 1], cq);
        this.parent_pose_child.set([1, 2], -sq);
        this.parent_pose_child.set([2, 1], sq);
        this.parent_pose_child.set([2, 2], cq);

        this.parent_dif1_child.set([1, 1], -sq);
        this.parent_dif1_child.set([1, 2], -cq);
        this.parent_dif1_child.set([2, 1], cq);
        this.parent_dif1_child.set([2, 2], -sq);

        this.parent_dif2_child.set([1, 1], -cq);
        this.parent_dif2_child.set([1, 2], sq);
        this.parent_dif2_child.set([2, 1], -sq);
        this.parent_dif2_child.set([2, 2], -cq);

        this.child_pose_parent.set([1, 1], cq);
        this.child_pose_parent.set([1, 2], sq);
        this.child_pose_parent.set([2, 1], -sq);
        this.child_pose_parent.set([2, 2], cq);

        this.child_dif1_parent.set([1, 1], -sq);
        this.child_dif1_parent.set([1, 2], cq);
        this.child_dif1_parent.set([2, 1], -cq);
        this.child_dif1_parent.set([2, 2], -sq);

        this.child_dif2_parent.set([1, 1], -cq);
        this.child_dif2_parent.set([1, 2], -sq);
        this.child_dif2_parent.set([2, 1], sq);
        this.child_dif2_parent.set([2, 2], -cq);
        this.parent_twist_child = this.child_pose_parent.multiply(this.parent_dif1_child);
    }
}

/**A frame that implements a rotation primitive for a DOF about the y-axis.
 * @extends {Frame}
 */
class JointRotationY extends Frame {
    /**
     * @param {Frame} parent The parent frame defining the basis.
     * @param {number} [initial_q] The initial position for the joint.
     * @param {number} [initial_q_dot] The initial velocity for the joint.
     */
    constructor(parent, initial_q = 0.0, initial_q_dot = 0.0) {
        super(parent, new Pose());
        this.q = initial_q;
        this.q_dot = initial_q_dot;
        this.is_fixed = false;
    }

    /** @override */
    updateLocal() {
        const cq = math.cos(this.q);
        const sq = math.sin(this.q);
        this.parent_pose_child.set([0, 0], cq);
        this.parent_pose_child.set([0, 2], sq);
        this.parent_pose_child.set([2, 0], -sq);
        this.parent_pose_child.set([2, 2], cq);

        this.parent_dif1_child.set([0, 0], -sq);
        this.parent_dif1_child.set([0, 2], cq);
        this.parent_dif1_child.set([2, 0], -cq);
        this.parent_dif1_child.set([2, 2], -sq);

        this.parent_dif2_child.set([0, 0], -cq);
        this.parent_dif2_child.set([0, 2], -sq);
        this.parent_dif2_child.set([2, 0], sq);
        this.parent_dif2_child.set([2, 2], -cq);

        this.child_pose_parent.set([0, 0], cq);
        this.child_pose_parent.set([0, 2], -sq);
        this.child_pose_parent.set([2, 0], sq);
        this.child_pose_parent.set([2, 2], cq);

        this.child_dif1_parent.set([0, 0], -sq);
        this.child_dif1_parent.set([0, 2], -cq);
        this.child_dif1_parent.set([2, 0], cq);
        this.child_dif1_parent.set([2, 2], -sq);

        this.child_dif2_parent.set([0, 0], -cq);
        this.child_dif2_parent.set([0, 2], sq);
        this.child_dif2_parent.set([2, 0], -sq);
        this.child_dif2_parent.set([2, 2], -cq);

        this.parent_twist_child = this.child_pose_parent.multiply(this.parent_dif1_child);
    }
}

/**A frame that implements a rotation primitive for a DOF about the z-axis.
 * @extends {Frame}
 */
class JointRotationZ extends Frame {
    /**
     * @param {Frame} parent The parent frame defining the basis.
     * @param {number} [initial_q] The initial position for the joint.
     * @param {number} [initial_q_dot] The initial velocity for the joint.
     */
    constructor(parent, initial_q = 0.0, initial_q_dot = 0.0) {
        super(parent, new Pose());
        this.q = initial_q;
        this.q_dot = initial_q_dot;
        this.is_fixed = false;
    }

    /** @override */
    updateLocal() {
        const cq = math.cos(this.q);
        const sq = math.sin(this.q);
        this.parent_pose_child.set([0, 0], cq);
        this.parent_pose_child.set([0, 1], -sq);
        this.parent_pose_child.set([1, 0], sq);
        this.parent_pose_child.set([1, 1], cq);

        this.parent_dif1_child.set([0, 0], -sq);
        this.parent_dif1_child.set([0, 1], -cq);
        this.parent_dif1_child.set([1, 0], cq);
        this.parent_dif1_child.set([1, 1], -sq);

        this.parent_dif2_child.set([0, 0], -cq);
        this.parent_dif2_child.set([0, 1], sq);
        this.parent_dif2_child.set([1, 0], -sq);
        this.parent_dif2_child.set([1, 1], -cq);

        this.child_pose_parent.set([0, 0], cq);
        this.child_pose_parent.set([0, 1], sq);
        this.child_pose_parent.set([1, 0], -sq);
        this.child_pose_parent.set([1, 1], cq);

        this.child_dif1_parent.set([0, 0], -sq);
        this.child_dif1_parent.set([0, 1], cq);
        this.child_dif1_parent.set([1, 0], -cq);
        this.child_dif1_parent.set([1, 1], -sq);

        this.child_dif2_parent.set([0, 0], -cq);
        this.child_dif2_parent.set([0, 1], -sq);
        this.child_dif2_parent.set([1, 0], sq);
        this.child_dif2_parent.set([1, 1], -cq);

        this.parent_twist_child = this.child_pose_parent.multiply(this.parent_dif1_child);
    }
}

/**A frame that implements a translation primitive for a DOF along the x-axis.
 * @extends {Frame}
 */
class JointTranslationX extends Frame {
    /**
     * @param {Frame} parent The parent frame defining the basis.
     * @param {number} [initial_q] The initial position for the joint.
     * @param {number} [initial_q_dot] The initial velocity for the joint.
     */
    constructor(parent, initial_q = 0.0, initial_q_dot = 0.0) {
        super(parent, new Pose());
        this.q = initial_q;
        this.q_dot = initial_q_dot;
        this.is_fixed = false;
    }

    /** @override */
    updateLocal() {
        this.parent_pose_child.set([0, 3], this.q);

        this.parent_dif1_child.set([0, 3], 1.0);

        this.child_pose_parent.set([0, 3], -this.q);

        this.child_dif1_parent.set([0, 3], -1.0);;

        this.parent_twist_child.set([0, 3], 1.0);
    }
}

/**A frame that implements a translation primitive for a DOF along the y-axis.
 * @extends {Frame}
 */
class JointTranslationY extends Frame {
    /**
     * @param {Frame} parent The parent frame defining the basis.
     * @param {number} [initial_q] The initial position for the joint.
     * @param {number} [initial_q_dot] The initial velocity for the joint.
     */
    constructor(parent, initial_q = 0.0, initial_q_dot = 0.0) {
        super(parent, new Pose());
        this.q = initial_q;
        this.q_dot = initial_q_dot;
        this.is_fixed = false;
    }

    /** @override */
    updateLocal() {
        this.parent_pose_child.set([1, 3], this.q);

        this.parent_dif1_child.set([1, 3], 1.0);

        this.child_pose_parent.set([1, 3], -this.q);

        this.child_dif1_parent.set([1, 3], -1.0);;

        this.parent_twist_child.set([1, 3], 1.0);
    }
}

/**A frame that implements a translation primitive for a DOF along the z-axis.
 * @extends {Frame}
 */
class JointTranslationZ extends Frame {
    /**
     * @param {Frame} parent The parent frame defining the basis.
     * @param {number} [initial_q] The initial position for the joint.
     * @param {number} [initial_q_dot] The initial velocity for the joint.
     */
    constructor(parent, initial_q = 0.0, initial_q_dot = 0.0) {
        super(parent, new Pose());
        this.q = initial_q;
        this.q_dot = initial_q_dot;
        this.is_fixed = false;
    }

    /** @override */
    updateLocal() {
        this.parent_pose_child.set([2, 3], this.q);

        this.parent_dif1_child.set([2, 3], 1.0);

        this.child_pose_parent.set([2, 3], -this.q);

        this.child_dif1_parent.set([2, 3], -1.0);;

        this.parent_twist_child.set([2, 3], 1.0);
    }
}

exports.JointRotationX = JointRotationX;
exports.JointRotationY = JointRotationY;
exports.JointRotationZ = JointRotationZ;
exports.JointTranslationX = JointTranslationX;
exports.JointTranslationY = JointTranslationY;
exports.JointTranslationZ = JointTranslationZ;

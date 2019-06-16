const { Frame } = require("./frame");
const math = require('mathjs');

/**Contains the inertial information and kinematics of a rigid-body.
 * @extends {Frame}
 */
class Body extends Frame {
    /**
     * @param {Frame} parent The parent frame to attach the inertia to.
     * @param {math.Matrix} parent_pose_child The relative transform between this frame and its parent.
     * @param {number} mass The mass of the body.
     * @param {number} i_xx The principle inertia about the x-axis of the body-frame.
     * @param {number} i_yy The principle inertia about the y-axis of the body-frame.
     * @param {number} i_zz The principle inertia about the z-axis of the body-frame.
     */
    constructor(parent, parent_pose_child, mass, i_xx, i_yy, i_zz) {
        super(parent, parent_pose_child);
        /**
         * @property {math.Matrix} inertia The [m, m, m, i_xx, i_yy, i_zz] principle inertia at the CoM.
         */
        this.inertia = math.matrix([mass, mass, mass, i_xx, i_yy, i_zz]);
    }
    /**
     * Calculate the current position of the center-of-mass in the global frame.
     * @returns {math.Matrix} An [x, y, z, 1] vector of world_xyz_com.
     */
    position() {
        return this.world_pose_child.getLocation();
    }
    /**
     * First partial of position w.r.t. to q in global coordinates.
     * @param {Frame} i_frame The i-th frame to take the partial with respect to.
     * @returns {math.Matrix} The result of diff(P, q_i).
     */
    parPositionQ(i_frame) {
        return this.parPoseQ(i_frame).getLocation();
    }
    /**
     * Second partial of position w.r.t. to q in global coordinates.
     * @param {Frame} i_frame The i-th frame to take the partial with respect to.
     * @param {Frame} j_frame The j-th frame to take the partial with respect to.
     * @returns {math.Matrix} The result of diff2(P, q_i, q_j).
     */
    par2PositionQQ(i_frame, j_frame) {
        return this.par2PoseQQ(i_frame, j_frame).getLocation();
    }
    /**
     * Calculate the current velocity of the center-of-mass in the global frame.
     * @returns {math.Matrix} An twist vector, V, of world_xyz_com.
     */
    velocity() {
        return this.world_twist_child.unhat();
    }
    /**
     * First partial of velocity w.r.t. to q in global coordinates.
     * @param {Frame} i_frame The i-th frame to take the partial with respect to.
     * @returns {math.Matrix} The twist vector result of diff(V, q_i).
     */
    parVelocityQ(i_frame) {
        return this.parTwistQ(i_frame).unhat();
    }
    /**
     * Second partial of velocity w.r.t. to q in global coordinates.
     * @param {Frame} i_frame The i-th frame to take the partial with respect to.
     * @param {Frame} j_frame The j-th frame to take the partial with respect to.
     * @returns {math.Matrix} The twist vector result of diff2(V, q_i, q_j).
     */
    par2VelocityQQ(i_frame, j_frame) {
        return this.par2TwistQQ(i_frame, j_frame).unhat();
    }
    /**
     * First partial of velocity w.r.t. to q_dot in global coordinates.
     * @param {Frame} i_frame The i-th frame to take the partial with respect to.
     * @returns {math.Matrix} The twist vector result of diff(V, dq_i).
     */
    parVelocityQdot(i_frame) {
        return this.parTwistQdot(i_frame).unhat();
    }
    /**
     * Second partial of velocity w.r.t. to q_dot in global coordinates.
     * @param {Frame} i_frame The i-th frame to take the partial with respect to.
     * @param {Frame} j_frame The j-th frame to take the partial with respect to.
     * @returns {math.Matrix} The twist vector result of diff2(V, qd_i, qd_j). Always zero.
     */
    par2VelocityQdotQdot(i_frame, j_frame) {
        return this.par2TwistQdotQdot(i_frame, j_frame).unhat();
    }
    /**
     * Mixed partial of velocity w.r.t. to q and q_dot in global coordinates.
     * @param {Frame} q_frame The q frame to take the partial with respect to.
     * @param {Frame} q_dot_frame The q_dot frame to take the partial with respect to.
     * @returns {math.Matrix} The twist vector result of diff2(V, q_i, qd_j).
     */
    par2VelocityQQdot(q_frame, q_dot_frame) {
        return this.par2TwistQQdot(q_frame, q_dot_frame).unhat();
    }
}

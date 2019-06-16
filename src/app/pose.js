const math = require('mathjs');

/**
 * Stores basic transform information and methods for operating on spatial transforms. Used as a potential shim for improving performance in the future.
 */
class Pose {
    /**
     * @param {math.Matrix} [transform] A 4x4 matrix defining the translation/rotation of this Pose.
     */
    constructor(transform = math.identity(4)) {
        /**
         * The internal representation of the pose.
         * @type {math.Matrix}
         */
        this.homogeneous_transform = transform.clone();
    }
    /**
     * Make a copy of the current pose.
     * @returns {Pose} A copy of the pose.
     */
    clone() { return new Pose(this.homogeneous_transform); }
    /**
     * Directly set the value of the underlying homogeneous transform.
     * @param {number[]} index An array of indices [row, col].
     * @param {number} value The new value.
     */
    set(index, value) {
        this.homogeneous_transform.set(index, value);
    }
    /**
     * Directly read an element of the underlying homogeneous transform.
     * @param {number[]} index An array of indices [row, col].
     * @returns {number} The selected value.
     */
    get(index) {
        return this.homogeneous_transform.get(index);
    }
    /**
     * Multiplies the current pose from the right with the supplied pose, i.e. this_pose * child_pose.
     * @param {Pose} child_pose A pose to multiply from the right.
     * @returns {Pose} A new pose that is the product of the pose multiplication.
     */
    multiply(child_pose) {
        let result = child_pose.clone();
        result.homogeneous_transform = math.multiply(this.homogeneous_transform, child_pose.homogeneous_transform);
        return result;
    }
    /**
     * Multiplies the current pose from the right with the supplied matrix, i.e. this_pose * child_matrix.
     * @param {math.Matrix} matrix A matrix to multiply from the right.
     * @returns {math.Matrix} A new matrix that is the product of multiplication.
     */
    transform(child_matrix) {
        if (child_matrix.size()[0] === 3) {
            return math.add(math.multiply(this.getRotationMatrix(), child_matrix), this.getTranslation());
        }
        return math.multiply(this.homogeneous_transform, child_matrix);
    }
    /**
     * Inverts the current pose.
     * @returns {Pose} A new pose resulting from the inversion.
     */
    inv() {
        let pose_inv = this.clone();
        let rotation_inv = math.transpose(this.getRotationMatrix());
        let translation = this.getTranslation();
        pose_inv.setRotationMatrix(rotation_inv);
        pose_inv.setTranslation(math.multiply(-1.0, math.multiply(rotation_inv, translation)));
        return pose_inv;
    }
    /**
     * Adds a vector to the translation part of the homogoneous transform.
     * @param {math.Matrix | number[]} translation A vector to add to this pose.
     * @returns {Pose} A new pose that has been translated.
     */
    translate(translation) {
        let result = this.clone();
        result.setTranslation(math.add(this.getTranslation(), math.squeeze(math.matrix(translation))));
        return result;
    }

    /**
     * Rotate the current pose about its x-axis, this_pose * rotation.
     * @param {float} angle An angle to rotate by in radians.
     * @returns {Pose} A new pose that has been rotated.
     */
    rotateByX(angle) {
        let rotation = new Pose();
        const cq = math.cos(angle);
        const sq = math.sin(angle);
        rotation.set([1, 1], cq);
        rotation.set([1, 2], -sq);
        rotation.set([2, 1], sq);
        rotation.set([2, 2], cq);
        return this.multiply(rotation);
    }

    /**
     * Rotate the current pose about its y-axis, this_pose * rotation.
     * @param {float} angle An angle to rotate by in radians.
     * @returns {Pose} A new pose that has been rotated.
     */
    rotateByY(angle) {
        let rotation = new Pose();
        const cq = math.cos(angle);
        const sq = math.sin(angle);
        rotation.set([0, 0], cq);
        rotation.set([0, 2], sq);
        rotation.set([2, 0], -sq);
        rotation.set([2, 2], cq);
        return this.multiply(rotation);
    }

    /**
     * Rotate the current pose about its z-axis, this_pose * rotation.
     * @param {float} angle An angle to rotate by in radians.
     * @returns {Pose} A new pose that has been rotated.
     */
    rotateByZ(angle) {
        let rotation = new Pose();
        const cq = math.cos(angle);
        const sq = math.sin(angle);
        rotation.set([0, 0], cq);
        rotation.set([0, 1], -sq);
        rotation.set([1, 0], sq);
        rotation.set([1, 1], cq);
        return this.multiply(rotation);
    }

    /**
     * Performs the similarity transform, i.e. new_basis^T * this_pose * new_basis.
     * @param {Pose} new_basis A pose that will become the new basis.
     * @returns {Pose} A new pose having the changed basis.
     */
    changeBasis(new_basis) {
        let result = this.clone();
        result.homogeneous_transform = new_basis.inv().multiply(this.multiply(new_basis)).homogeneous_transform;
        return result;
    }
    /**
     * Extract the fourth column of the homogeneous transform.
     * @returns {math.Matrix} The vector [x, y, z, 1].
     */
    getLocation() {
        return math.squeeze(this.homogeneous_transform.subset(math.index(math.range(0, 4), 3)));
    }
    /**
     * Extract the translation from the homogeneous transform.
     * @returns {math.Matrix} The vector [x, y, z].
     */
    getTranslation() {
        return math.squeeze(this.homogeneous_transform.subset(math.index(math.range(0, 3), 3)));
    }
    /**
     * Set the translation component.
     * @param {number[]} translation An array of [x, y, z].
     */
    setTranslation(translation) {
        let vector = math.matrix(translation);
        if (vector.size()[0] === 3) {
            this.homogeneous_transform.subset(math.index(math.range(0, 3), 3), vector);
        }
        else {
            this.homogeneous_transform.subset(math.index(math.range(0, 3), 3), vector.transpose());
        }
    }
    /**
     * Extract the rotation matrix from the homogeneous transform.
     * @returns {math.Matrix} The 3x3 rotation matrix.
     */
    getRotationMatrix() {
        return this.homogeneous_transform.subset(math.index(math.range(0, 3), math.range(0, 3)));
    }
    /**
     * Set the rotation component.
     * @param {math.Matrix} rotation An array of [x, y, z].
     */
    setRotationMatrix(rotation) {
        this.homogeneous_transform.subset(math.index(math.range(0, 3), math.range(0, 3)), rotation);
    }
}

/**
 * Create a pose, whose z-axis is defined by the supplied vector.
 * @param {math.Matrix} axis The vector that will become the z-axis.
 * @returns {Pose} A new pose whose z-axis is the supplied vector.
 */
function createPoseFromAxis(axis) {
    let pose = new Pose();
    const EPSILON = 1.0e-12;
    const axis_norm = math.norm(axis);
    if (axis_norm > EPSILON) {
        const z_axis = math.divide(axis, axis_norm);
        let x_axis = math.matrix([1.0, 0.0, 0.0]);
        let y_axis = math.cross(z_axis, x_axis);
        if (math.abs(math.norm(y_axis)) > EPSILON) {
            x_axis = math.cross(y_axis, z_axis);
        } else {
            y_axis = math.matrix([0.0, 1.0, 0.0]);
            x_axis = math.cross(y_axis, z_axis);
            y_axis = math.cross(z_axis, x_axis);
        }
        pose.homogeneous_transform.subset(math.index(math.range(0, 3), 0), x_axis);
        pose.homogeneous_transform.subset(math.index(math.range(0, 3), 1), y_axis);
        pose.homogeneous_transform.subset(math.index(math.range(0, 3), 2), z_axis);
    }
    return pose;
}

/**
 * Create a pose whose homogoneous transform is identity.
 * @returns {Pose} A new identity pose.
 */
function createPoseIdentity() {
    return new Pose();
}
exports.Pose = Pose;
exports.createPoseFromAxis = createPoseFromAxis;
exports.createPoseIdentity = createPoseIdentity;

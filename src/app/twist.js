const { Pose } = require("./pose");
const math = require('mathjs');
/**
 * A differential varient of the pose.
 * @extends {Pose}
 */
class Twist extends Pose {
    /**
     * @param {number[]} [twist] An array of twist velocities [tx, ty, tz, rx, ry, rz].
     */
    constructor(twist = [0, 0, 0, 0, 0, 0]) {
        super(math.zeros(4, 4));
        this.hat(twist);
    }
    /** @override */
    clone() { return new Twist(this.unhat().toArray()); }
    /** @override */
    multiply(child_pose) {
        let result = this.clone();
        result.homogeneous_transform = math.multiply(this.homogeneous_transform, child_pose.homogeneous_transform);
        return result;
    }
    /**
     * Adds the current twist from the right with the supplied twist, i.e. this_twist + child_twist.
     * @param {Twist} child_twist A twist to add from the right.
     * @returns {Twist} A new twist that is the sum of the twist addition.
     */
    add(child_twist) {
        let result = this.clone();
        result.homogeneous_transform = math.add(this.homogeneous_transform, child_twist.homogeneous_transform);
        return result;
    }
    /**
     * Subtracts the current twist from the right with the supplied twist, i.e. this_twist - child_twist.
     * @param {Twist} child_twist A twist to subtract from the right.
     * @returns {Twist} A new twist that is the result of the twist subtraction.
     */
    subtract(child_twist) {
        let result = this.clone();
        result.homogeneous_transform = math.subtract(this.homogeneous_transform, child_twist.homogeneous_transform);
        return result;
    }
    /**
     * Scales the current twist from the right with the supplied value, i.e. this_twist * value.
     * @param {number} value A value to scale the twist.
     * @returns {Twist} A new twist that is the result of scaling.
     */
    scale(value) {
        let result = this.clone();
        result.homogeneous_transform = math.multiply(this.homogeneous_transform, value);
        return result;
    }
    /**
     * Sets the provided twist values into the appropriate slots of the homogeneous transform.
     * @param {number[]} twist An array of twist velocities [tx, ty, tz, rx, ry, rz].
     */
    hat(twist) {
        this.homogeneous_transform.set([0, 3], twist[0]);
        this.homogeneous_transform.set([1, 3], twist[1]);
        this.homogeneous_transform.set([2, 3], twist[2]);
        this.homogeneous_transform.set([1, 2], -twist[3]);
        this.homogeneous_transform.set([0, 2], twist[4]);
        this.homogeneous_transform.set([0, 1], -twist[5]);
        this.homogeneous_transform.set([2, 1], twist[3]);
        this.homogeneous_transform.set([2, 0], -twist[4]);
        this.homogeneous_transform.set([1, 0], twist[5]);
    }
    /**
     * Extract the twist from the homogeneous transform.
     * @returns {math.Matrix} The twist vector [tx, ty, tz, rx, ry, rz].
     */
    unhat() {
        return math.matrix([
            this.homogeneous_transform.get([0, 3]),
            this.homogeneous_transform.get([1, 3]),
            this.homogeneous_transform.get([2, 3]),
            this.homogeneous_transform.get([2, 1]),
            this.homogeneous_transform.get([0, 2]),
            this.homogeneous_transform.get([1, 0])
        ]);
    }
}

exports.Twist = Twist;

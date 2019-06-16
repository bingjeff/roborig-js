const { PokeArrow } = require("./poke_arrow");
const math = require("mathjs");
const THREE = require("three");
class PerspectiveCursor {
    /**
     * @param {THREE.Scene} current_scene The scene to attach the cursor.
     * @param {number} scale The relative scale to apply to the arrow.
     */
    constructor(current_scene, scale = 1.0) {
        /** @property {PokeArrow} cursor An arrow corresponding to the current 3D mouse location. */
        this.cursor = new PokeArrow(current_scene);
        /** @property {Array<THREE.Vector3>} dagger The points corresponding to the cardinal axes. */
        this.dagger = [
            new THREE.Vector3(1, 0, 0),
            new THREE.Vector3(0, 1, 0),
            new THREE.Vector3(0, 0, 1),
            new THREE.Vector3(-1, 0, 0),
            new THREE.Vector3(0, -1, 0),
            new THREE.Vector3(0, 0, -1)
        ];
        /** @property {boolean} visible The visibility state of the cursor. */
        this.visible = true;
        /** @property {number} scale The visibility state of the cursor. */
        this.scale = scale;
        /** @type {Array<THREE.Vector3>} The cardinal directions of the dagger. */
        this.cardinals_ = [
            new THREE.Vector3(1, 0, 0),
            new THREE.Vector3(0, 1, 0),
            new THREE.Vector3(0, 0, 1),
            new THREE.Vector3(-1, 0, 0),
            new THREE.Vector3(0, -1, 0),
            new THREE.Vector3(0, 0, -1)
        ];
        this.setSpaceOrigin(new THREE.Vector3());
    }
    /**
     * @param {THREE.Vector3} origin The origin of the current cursor in 3D space.
     */
    setSpaceOrigin(origin) {
        this.cursor.setOrigin(origin);
        for (let a = 0; a < this.dagger.length; a++) {
            this.dagger[a].addVectors(origin, this.cardinals_[a].clone().multiplyScalar(0.01));
        }
    }
    /**
     * Calculate an approximate point in space relative to the origin.
     * @param {THREE.Vector2} mousePoint Normalized screen coordinates [-1, 1].
     * @param {THREE.PerspectiveCamera} camera Current perspective camera.
     */
    setSpacePoint(mousePoint, camera) {
        let mouseOrigin = this.cursor.tail.clone().project(camera);
        let mouseVector = new THREE.Vector2(mousePoint.x - mouseOrigin.x, mousePoint.y - mouseOrigin.y);
        let projectedPairs = [];
        for (let a = 0; a < this.cardinals_.length; a++) {
            let point3 = this.dagger[a].clone().project(camera);
            let point2 = new THREE.Vector2(point3.x - mouseOrigin.x, point3.y - mouseOrigin.y);
            projectedPairs.push({ "point3": this.cardinals_[a], "angle": point2.angle() });
        }
        projectedPairs.sort((a, b) => a.angle - b.angle);
        const firstPair = projectedPairs[0];
        const lastPair = projectedPairs[projectedPairs.length - 1];
        projectedPairs.push({ "point3": firstPair.point3, "angle": firstPair.angle + 2 * math.pi });
        projectedPairs.unshift({ "point3": lastPair.point3, "angle": lastPair.angle - 2 * math.pi });
        const mouseAngle = mouseVector.angle();
        let interval;
        for (interval = 1; interval < projectedPairs.length; interval++) {
            if (mouseAngle < projectedPairs[interval].angle) {
                break;
            }
        }
        const ratio = (mouseAngle - projectedPairs[interval - 1].angle) / (projectedPairs[interval].angle - projectedPairs[interval - 1].angle);
        const direction = new THREE.Vector3().lerpVectors(projectedPairs[interval - 1].point3, projectedPairs[interval].point3, ratio).normalize();
        this.cursor.setColor(new THREE.Color().setRGB(math.abs(direction.x), math.abs(direction.y), math.abs(direction.z)));
        let scale = this.scale * mouseVector.length() * mouseOrigin.z / camera.getFocalLength();
        this.cursor.setDirection(direction, scale);
    }
    /**
     * @param {boolean} visible Turn on and off the rendering of the cursor.
     */
    setVisibility(visible) {
        this.visible = visible;
        this.cursor.setVisibility(visible);
    }
}
exports.PerspectiveCursor = PerspectiveCursor;

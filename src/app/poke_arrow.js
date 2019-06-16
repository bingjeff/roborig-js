const THREE = require("three");
class PokeArrow {
    /**
     * @param {THREE.Scene} current_scene The scene to attach the arrow.
     */
    constructor(current_scene) {
        /**
         * @property {THREE.Vector3} tip The start of the line.
         */
        this.tip = new THREE.Vector3();
        /**
         * @property {THREE.Vector3} tail The end of the line.
         */
        this.tail = new THREE.Vector3();
        /**
         * @property {THREE.Vector3} direction The arrow vector.
         */
        this.direction = new THREE.Vector3();
        let geometry = new THREE.Geometry();
        geometry.vertices.push(this.tip, this.tail);
        let material = new THREE.LineBasicMaterial({
            color: 0xff0000
        });
        /**
         * @property {THREE.Line} arrow The line.
         */
        this.arrow = new THREE.Line(geometry, material);
        current_scene.add(this.arrow);
    }
    /**
     * Sets the location of the tail in global coordinates.
     * @param {THREE.Vector3} origin New desired origin.
     */
    setOrigin(origin) {
        this.tail.copy(origin);
        this.arrow.geometry.verticesNeedUpdate = true;
    }
    /**
     * Set the pointing direction for the arrow.
     * @param {THREE.Vector3} direction The unit normal direction to point the arrow in global coordinates.
     * @param {number} scale The length of the vector.
     */
    setDirection(direction, scale = 1.0) {
        this.direction.copy(direction);
        this.direction.setLength(scale);
        this.tip.addVectors(this.tail, this.direction);
        this.arrow.geometry.verticesNeedUpdate = true;
    }
    /**
     * @param {boolean} visible
     */
    setVisibility(visible) {
        this.arrow.visible = visible;
    }
    /**
     * @param {THREE.Color} color The new color of the arrow.
     */
    setColor(color) {
        this.arrow.material.color.set(color);
        this.arrow.material.needsUpdate = true;
    }
}
exports.PokeArrow = PokeArrow;

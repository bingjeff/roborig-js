const { createPoseFromAxis } = require("./pose");
const { createPoseIdentity } = require("./pose");
const { Frame } = require("./frame");
const { JointRotationZ } = require("./joint");
const { JointTranslationZ } = require("./joint");
const { KinematicTree } = require("./kinematic_tree");
const { MtlObjBridge } = require("three/examples/jsm/loaders/bridge/MtlObjBridge");
const { MTLLoader } = require("three/examples/jsm/loaders/MTLLoader");
const { OBJLoader2 } = require("three/examples/jsm/loaders/OBJLoader2");
const { VisualTree } = require("./visual_tree");
const math = require("mathjs");

class UrdfLoader {
	static parseName(element) {
		return element.getAttribute("name");
	}
	static parseValue(element, attribute) {
		let value = 0;
		if (element) {
			let string = element.getAttribute(attribute);
			if (string) {
				value = string;
			}
		}
		return value;
	}
	static parseTagValue(element, tag, attribute) {
		let value = 0;
		if (element) {
			return this.parseValue(element.getElementsByTagName(tag)[0], attribute);
		}
		return value;
	}
	static parseTriplet(element, attribute) {
		let value = math.zeros(3);
		if (element) {
			let string = element.getAttribute(attribute);
			if (string) {
				string = string.split(" ");
				if (string.length > 2) {
					value.set([0], Number(string[0]));
					value.set([1], Number(string[1]));
					value.set([2], Number(string[2]));
				}
			}
		}
		return value;
	}
	static parseTagTriplet(element, tag, attribute) {
		let value = 0;
		if (element) {
			return this.parseTriplet(element.getElementsByTagName(tag)[0], attribute);
		}
		return value;
	}
	static addJointFrame(tree, element, child_name, parent_name = null) {
		const frame_name = child_name + "_f";
		const joint_type = this.parseValue(element, "type");
		const joint_axis = this.parseTagTriplet(element, "axis", "xyz");
		const frame_xyz = this.parseTagTriplet(element, "origin", "xyz");
		const frame_rpy = this.parseTagTriplet(element, "origin", "rpy");
		const parent_pose_child = createPoseIdentity().rotateByZ(frame_rpy.get([2])).rotateByY(frame_rpy.get([1])).rotateByX(frame_rpy.get([0])).translate(frame_xyz).multiply(createPoseFromAxis(joint_axis));
		switch (joint_type) {
			case "revolute":
			case "continuous":
				tree.addFrame(frame_name, parent => new Frame(parent, parent_pose_child), parent_name);
				tree.addFrame(child_name, parent => new JointRotationZ(parent), frame_name);
				break;
			case "prismatic":
				tree.addFrame(frame_name, parent => new Frame(parent, parent_pose_child), parent_name);
				tree.addFrame(child_name, parent => new JointTranslationZ(parent), frame_name);
				break;
			case "floating":
			case "planar":
			default:
				this.warn("Setting joint " + child_name + " as a fixed joint.");
			case "fixed":
				tree.addFrame(child_name, parent => new Frame(parent, parent_pose_child), parent_name);
				break;
		}
	}
	static addVisualObject(tree, element, parent_name, path_prefix = "") {
		const frame_xyz = this.parseTagTriplet(element, "origin", "xyz");
		const frame_rpy = this.parseTagTriplet(element, "origin", "rpy");
		const parent_pose_child = createPoseIdentity().rotateByZ(frame_rpy.get([2])).rotateByY(frame_rpy.get([1])).rotateByX(frame_rpy.get([0])).translate(frame_xyz);
		const geometry_element = element.getElementsByTagName("geometry")[0];
		const meshName = this.parseTagValue(geometry_element, "mesh", "filename")
		const meshType = meshName.slice(-3);
		const meshPath = path_prefix + meshName.slice(0, -4);

		switch (meshType) {
			case "obj":
				let objLoader = new OBJLoader2();
				let mtlLoader = new MTLLoader();
				let callbackOnLoad = function (object3d) {
					object3d.name = parent_name;
					tree.addVisual(parent_name, object3d, parent_pose_child);
					tree.update();
					console.log('Loading model complete: ' + parent_name);
				};
				let onLoadMtl = function (mtlParseResult) {
					objLoader.setModelName(parent_name);
					objLoader.setLogging(false, false);
					objLoader.addMaterials(MtlObjBridge.addMaterialsFromMtlLoader(mtlParseResult));
					objLoader.load(meshPath + ".obj", callbackOnLoad, null, null, null);
				};
				mtlLoader.load(meshPath + ".mtl", onLoadMtl);
				break;
			default:
				tree.addVisual(parent_name, null);
		}
	}
    /**
     * Create a kinematic tree object from an XML DOM made from the URDF.
     * @param {Document} xml_dom An xml document containing the URDF.
	 * @param {THREE.Scene} [current_scene] Optional scene to load geometry into.
     */
	static createKinematicTree(xml_dom, current_scene = null, mesh_prefix = "") {
		// Create a new kinematic tree.
		let tree;
		if (current_scene !== null) {
			tree = new VisualTree(current_scene);
		} else {
			tree = new KinematicTree();
		}
		// Get pointers to the joint URDF elements.
		let joints = xml_dom.getElementsByTagName("joint");
		let link_map = new Map();
		// Walk through joints and build link map.
		for (let joint of Array.from(joints)) {
			const joint_name = this.parseName(joint);
			if (joint_name) {
				const child_name = this.parseTagValue(joint, "child", "link");
				link_map.set(child_name, joint);
			}
			else {
				this.warn("Skipping bad joint tag.");
			}
		}
		// Find the root joints.
		let root_names = [];
		for (let [child_name, joint] of link_map) {
			const parent_name = this.parseTagValue(joint, "parent", "link");
			if (!link_map.has(parent_name)) {
				root_names.push(child_name);
			}
		}
		// Attach the root frames.
		for (let root_name of root_names) {
			let joint = link_map.get(root_name);
			this.addJointFrame(tree, joint, root_name);
			link_map.delete(root_name);
		}
		// Attach the child frames.
		const size_of_joints = link_map.size;
		let num_loops = 0;
		while (link_map.size > 0 && num_loops < size_of_joints) {
			for (let [child_name, joint] of link_map) {
				let parent_name = this.parseTagValue(joint, "parent", "link");
				if (tree.frames.has(parent_name)) {
					this.addJointFrame(tree, joint, child_name, parent_name);
					link_map.delete(child_name);
				}
			}
			num_loops += 1;
		}
		if (link_map.size > 0) {
			this.warn("Failed to parse: " + link_map.keys());
		}
		// Parse visual information.
		if (current_scene !== null) {
			// Get pointers to the link URDF elements.
			let links = xml_dom.getElementsByTagName("link");
			// Walk through joints and build link map.
			for (let link of Array.from(links)) {
				const link_name = this.parseName(link);
				if (link_name) {
					const visual_tag = link.getElementsByTagName("visual")[0];
					this.addVisualObject(tree, visual_tag, link_name, mesh_prefix);
				}
				else {
					this.warn("Skipping bad link tag.");
				}
			}
		}
		tree.update();
		return tree;
	}
	// Helper function to display errors
	warn(warning_string) {
		console.log(warning_string);
	}
}
exports.UrdfLoader = UrdfLoader;

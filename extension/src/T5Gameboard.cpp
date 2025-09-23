#include <T5Gameboard.h>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/packed_scene.hpp>
#include <godot_cpp/classes/resource_loader.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/property_info.hpp>
#include <godot_cpp/variant/callable.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using godot::Callable;
using godot::ClassDB;
using godot::D_METHOD;
using godot::Engine;
using godot::Object;
using godot::PackedScene;
using godot::PropertyInfo;
using godot::ResourceLoader;
using godot::SceneTree;
using godot::UtilityFunctions;
using godot::Variant;
using godot::Vector3;
using godot::Basis;

const char le_name[] = "LE";
const char xe_name[] = "XE";
const char raised_xe_name[] = "Raised XE";

void T5Gameboard::set_content_scale(float scale) {
	content_scale = scale;
	set_board_state();
}

float T5Gameboard::get_content_scale() const {
	return content_scale;
}

void T5Gameboard::set_gameboard_type(String gb_type) {
	if (gb_type == le_name) {
		gameboard_type = TiltFiveXRInterface::LE_GAMEBOARD;
	} else if (gb_type == xe_name) {
		gameboard_type = TiltFiveXRInterface::XE_GAMEBOARD;
	} else if (gb_type == raised_xe_name) {
		gameboard_type = TiltFiveXRInterface::XE_RAISED_GAMEBOARD;
	} else {
		gameboard_type = TiltFiveXRInterface::NO_GAMEBOARD_SET;
	}
	set_board_state();
}

String T5Gameboard::get_gameboard_type() const {
	switch (gameboard_type) {
		case TiltFiveXRInterface::LE_GAMEBOARD:
			return le_name;
		case TiltFiveXRInterface::XE_GAMEBOARD:
			return xe_name;
		case TiltFiveXRInterface::XE_RAISED_GAMEBOARD:
			return raised_xe_name;
		default:
			break;
	}
	return "Unknown";
}

void T5Gameboard::set_gameboard_orientation(Vector3 eulerAngles) {
	gameboard_basis_eulers = eulerAngles;
	set_board_state();
}

Vector3 T5Gameboard::get_gameboard_orientation() const {
	return gameboard_basis_eulers;
}

void T5Gameboard::set_show_at_runtime(bool show) {
	show_at_runtime = show;
	set_board_state();
}

bool T5Gameboard::get_show_at_runtime() const {
	return show_at_runtime;
}

void T5Gameboard::set_layer_mask(uint32_t p_mask) {
	layers = p_mask;
	set_board_state();
}

uint32_t T5Gameboard::get_layer_mask() const {
	return layers;
}

void T5Gameboard::set_layer_mask_value(int p_layer_number, bool p_value) {
	ERR_FAIL_COND_MSG(p_layer_number < 1, "Render layer number must be between 1 and 20 inclusive.");
	ERR_FAIL_COND_MSG(p_layer_number > 20, "Render layer number must be between 1 and 20 inclusive.");
	uint32_t mask = get_layer_mask();
	if (p_value) {
		mask |= 1 << (p_layer_number - 1);
	} else {
		mask &= ~(1 << (p_layer_number - 1));
	}
	set_layer_mask(mask);
}

bool T5Gameboard::get_layer_mask_value(int p_layer_number) const {
	ERR_FAIL_COND_V_MSG(p_layer_number < 1, false, "Render layer number must be between 1 and 20 inclusive.");
	ERR_FAIL_COND_V_MSG(p_layer_number > 20, false, "Render layer number must be between 1 and 20 inclusive.");
	return layers & (1 << (p_layer_number - 1));
}

T5Gameboard::T5Gameboard() {
}

T5Gameboard::~T5Gameboard() {
}

Node3D* T5Gameboard::add_scene(String path) {
	Ref<PackedScene> scene = ResourceLoader::get_singleton()->load(path, "PackedScene");
	ERR_FAIL_COND_V_MSG(scene.is_null(), nullptr, godot::vformat("Failed to load: %s", path));

	auto node = scene->instantiate();
	auto node_3d = Object::cast_to<Node3D>(node);
	if (!node_3d) {
		node->queue_free();
		ERR_FAIL_V_MSG(nullptr, godot::vformat("Not Node3D: %s", path));
	}
	if (node_3d->get_child_count() == 0 || node->get_child(0)->get_class() != "MeshInstance3D") {
		node->queue_free();
		ERR_FAIL_V_MSG(nullptr, godot::vformat("Not Node3D and MeshInstance3D: %s", path));
	}

	add_child(node_3d);
	return node_3d;
}

void configure_board_mesh(Node3D* node_ptr, bool visible, float content_scale, uint32_t layers, std::optional<Transform3D> gameboard_transform) {
	if (node_ptr) {
		node_ptr->set_scale(Vector3(content_scale, content_scale, content_scale));
		node_ptr->set_visible(visible);
		auto mesh_inst = Object::cast_to<VisualInstance3D>(node_ptr->get_child(0));
		mesh_inst->set_layer_mask(layers);
		if(gameboard_transform.has_value())
		mesh_inst->set_transform(*gameboard_transform);
	}
}

void T5Gameboard::set_board_state() {
	T5_GameboardExtents gameboard_extents(gameboard_type);

	Transform3D gameboard_transform;
	Basis gameboard_basis = Basis::from_euler(gameboard_basis_eulers);
	gameboard_transform.set_basis(gameboard_basis);

	bool show_board = show_at_runtime || Engine::get_singleton()->is_editor_hint();
	
	// Show the stage only when the gameboard pose is different enough for it not to be visually busy/confusing.
	float stage_visibility_angular_threshold_radians = godot::Math::deg_to_rad(STAGE_VISIBILITY_ANGULAR_THRESHOLD_DEGREES);
	float stage_angular_displacement_radians = godot::Basis().get_rotation_quaternion().angle_to(gameboard_basis.get_rotation_quaternion());
	bool gameboard_is_tilted = stage_angular_displacement_radians >= stage_visibility_angular_threshold_radians;
	bool show_stage = gameboard_is_tilted && show_board;

	// Determine the lowest corner of the board in stage space after applying the user-provided rotation.
	std::optional<T5_GameboardExtents::Corner> lowest_corner = {};
	real_t lowest_observed_elevation;

	for (T5_GameboardExtents::Corner corner : T5_GameboardExtents::CORNERS) {
		Vector3 corner_position_BOARD = gameboard_extents.get_corner_position(corner);
		// Project corner coords onto the XZ plane before transforming them.
		// This ensures that the XE-Raised board's far corners' vertical displacement is still detected correctly despite their inherent elevation.
		Vector3 corner_position_STAGE = gameboard_basis.xform(Vector3(corner_position_BOARD.x, 0, corner_position_BOARD.z));

		if (!lowest_corner.has_value() || corner_position_STAGE.y < lowest_observed_elevation) {
			lowest_corner = corner;
			lowest_observed_elevation = corner_position_STAGE.y;
		}
	}

	// With the lowest corner determined, translate the gameboard through space until the lowest corner overlaps with the equivalent stage corner.
	Vector3 matched_stage_corner_pos = gameboard_extents.get_corner_position(*lowest_corner);
	Vector3 gameboard_stage_offset = matched_stage_corner_pos - gameboard_basis.xform(matched_stage_corner_pos);
	gameboard_transform.origin += gameboard_stage_offset;

	configure_board_mesh(le_stage_node, show_stage && gameboard_type == TiltFiveXRInterface::LE_GAMEBOARD, content_scale, layers, {});
	configure_board_mesh(xe_stage_node, show_stage && gameboard_type == TiltFiveXRInterface::XE_GAMEBOARD, content_scale, layers, {});
	configure_board_mesh(raised_xe_stage_node, show_stage && gameboard_type == TiltFiveXRInterface::XE_RAISED_GAMEBOARD, content_scale, layers, {});
	
	configure_board_mesh(le_gameboard_node, show_board && gameboard_type == TiltFiveXRInterface::LE_GAMEBOARD, content_scale, layers, gameboard_transform);
	configure_board_mesh(xe_gameboard_node, show_board && gameboard_type == TiltFiveXRInterface::XE_GAMEBOARD, content_scale, layers, gameboard_transform);
	configure_board_mesh(raised_xe_gameboard_node, show_board && gameboard_type == TiltFiveXRInterface::XE_RAISED_GAMEBOARD, content_scale, layers, gameboard_transform);
}

void T5Gameboard::_ready() {
	le_stage_node = add_scene("res://addons/tiltfive/assets/T5_stage_border.glb");
	xe_stage_node = add_scene("res://addons/tiltfive/assets/T5_stage_border_XE.glb");
	raised_xe_stage_node = add_scene("res://addons/tiltfive/assets/T5_stage_border_XE_raised.glb");

	le_gameboard_node = add_scene("res://addons/tiltfive/assets/T5_border.glb");
	xe_gameboard_node = add_scene("res://addons/tiltfive/assets/T5_border_XE.glb");
	raised_xe_gameboard_node = add_scene("res://addons/tiltfive/assets/T5_border_XE_raised.glb");

	set_board_state();
}

void T5Gameboard::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_content_scale", "scale"), &T5Gameboard::set_content_scale);
	ClassDB::bind_method(D_METHOD("get_content_scale"), &T5Gameboard::get_content_scale);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "content_scale", godot::PROPERTY_HINT_RANGE, "0.001, 1000.0, or_greater, hide_slider"), "set_content_scale", "get_content_scale");

	ClassDB::bind_method(D_METHOD("set_gameboard_type", "gameboard_type"), &T5Gameboard::set_gameboard_type);
	ClassDB::bind_method(D_METHOD("get_gameboard_type"), &T5Gameboard::get_gameboard_type);

	ClassDB::bind_method(D_METHOD("set_gameboard_orientation", "orientation"), &T5Gameboard::set_gameboard_orientation);
	ClassDB::bind_method(D_METHOD("get_gameboard_orientation"), &T5Gameboard::get_gameboard_orientation);
	ADD_PROPERTY(
		PropertyInfo(Variant::VECTOR3, "gameboard_orientation", godot::PROPERTY_HINT_RANGE, "-360,360,0.05,or_less,or_greater,radians"),
		"set_gameboard_orientation", "get_gameboard_orientation");

	auto hint = godot::vformat("%s,%s,%s", le_name, xe_name, raised_xe_name);
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "gameboard_type", godot::PROPERTY_HINT_ENUM, hint), "set_gameboard_type", "get_gameboard_type");

	ClassDB::bind_method(D_METHOD("set_show_at_runtime", "show"), &T5Gameboard::set_show_at_runtime);
	ClassDB::bind_method(D_METHOD("get_show_at_runtime"), &T5Gameboard::get_show_at_runtime);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "show_at_runtime", godot::PROPERTY_HINT_NONE), "set_show_at_runtime", "get_show_at_runtime");

	ClassDB::bind_method(D_METHOD("set_layer_mask", "mask"), &T5Gameboard::set_layer_mask);
	ClassDB::bind_method(D_METHOD("get_layer_mask"), &T5Gameboard::get_layer_mask);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "layers", godot::PROPERTY_HINT_LAYERS_3D_RENDER), "set_layer_mask", "get_layer_mask");

	ClassDB::bind_method(D_METHOD("set_layer_mask_value", "layer_number", "value"), &T5Gameboard::set_layer_mask_value);
	ClassDB::bind_method(D_METHOD("get_layer_mask_value", "layer_number"), &T5Gameboard::get_layer_mask_value);
};
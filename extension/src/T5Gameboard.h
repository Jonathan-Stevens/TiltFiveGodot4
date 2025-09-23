#ifndef T5_GAMEBOARD_H
#define T5_GAMEBOARD_H

#include <TiltFiveXRInterface.h>
#include <godot_cpp/classes/packed_data_container.hpp>
#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/classes/visual_instance3d.hpp>
#include <godot_cpp/classes/xr_pose.hpp>
#include <godot_cpp/classes/xr_positional_tracker.hpp>
#include <godot_cpp/variant/string_name.hpp>

using godot::Ref;
using godot::String;
using godot::StringName;
using godot::VisualInstance3D;

class T5Gameboard : public Node3D {
	GDCLASS(T5Gameboard, Node3D);

public:
	
	struct T5_GameboardExtents {
	public:
		real_t viewable_extent_right;
		real_t viewable_extent_left;
		real_t viewable_extent_near;
		real_t viewable_extent_far;
		real_t viewable_extent_up;

		inline static const T5_GameboardSize SIZE_LE		{0.35, 0.35, 0.35,    0.35, 0.0};
		inline static const T5_GameboardSize SIZE_XE		{0.35, 0.35, 0.61667, 0.35, 0.0};
		inline static const T5_GameboardSize SIZE_XE_RAISED	{0.35, 0.35, 0.55321, 0.35, 0.15321};

		// It's a bit surprising that the Godot C++ Vector3 API doesn't provide constants
		// for up/down/left/right/forward/back, at least as of Godot 4.5
		inline static const Vector3 NEAR	{ 0,  0,  1};
		inline static const Vector3 FAR		{ 0,  0, -1};
		/*
		inline static const Vector3 BACK	{ 0,  0,  1};
		inline static const Vector3 FORWARD	{ 0,  0, -1};
		*/

		inline static const Vector3 LEFT	{-1,  0,  0};
		inline static const Vector3 RIGHT	{ 1,  0,  0};

		inline static const Vector3 UP		{ 0,  1,  0};
		inline static const Vector3 DOWN	{ 0, -1,  0};

		enum Corner { NearLeft, NearRight, FarLeft, FarRight };

		inline static const std::array<Corner, 4> CORNERS {Corner::NearLeft, Corner::NearRight, Corner::FarLeft, Corner::FarRight };

	public:
		T5_GameboardExtents(T5_GameboardSize gameboardSize) :
			viewable_extent_right(gameboardSize.viewableExtentPositiveX),
			viewable_extent_left(gameboardSize.viewableExtentNegativeX),
			viewable_extent_near(gameboardSize.viewableExtentNegativeY),	// T5 <--> Godot: Swap sign of Y axis
			viewable_extent_far(gameboardSize.viewableExtentPositiveY),		// T5 <--> Godot: Swap sign of Y axis
			viewable_extent_up(gameboardSize.viewableExtentPositiveZ) {
		}

		T5_GameboardExtents(TiltFiveXRInterface::GameBoardType gameboard_type) {
			// TODO: Obtain gameboard extents from T5 NDK, falling back on these hardcoded corner coords if that's not possible.
			T5_GameboardSize gameboard_size = SIZE_LE;
			switch (gameboard_type) {
				case TiltFiveXRInterface::GameBoardType::XE_GAMEBOARD:
					gameboard_size = SIZE_XE;
					break;
				case TiltFiveXRInterface::GameBoardType::XE_RAISED_GAMEBOARD:
					gameboard_size = SIZE_XE_RAISED;
					break;
			}
			viewable_extent_right = gameboard_size.viewableExtentPositiveX;
			viewable_extent_left = gameboard_size.viewableExtentNegativeX;
			viewable_extent_near = gameboard_size.viewableExtentNegativeY;	// T5 <--> Godot: Swap sign of Y axis
			viewable_extent_far = gameboard_size.viewableExtentPositiveY;	// T5 <--> Godot: Swap sign of Y axis
			viewable_extent_up = gameboard_size.viewableExtentPositiveZ;
		}

		Vector3 get_corner_position(Corner corner) {
			switch (corner) {
				case Corner::NearLeft:
					return	LEFT * viewable_extent_left +
							Vector3() +		// The near corners don't have a height component
							NEAR * viewable_extent_near;
				case Corner::NearRight:
					return	RIGHT * viewable_extent_right +
							Vector3() +		// The near corners don't have a height component
							NEAR * viewable_extent_near;
				case Corner::FarLeft:
					return	LEFT * viewable_extent_left +
							UP * viewable_extent_up +
							FAR * viewable_extent_far;
				case Corner::FarRight:
					return	RIGHT * viewable_extent_right +
							UP * viewable_extent_up +
							FAR * viewable_extent_far;
			}
			return Vector3();
		}

		real_t get_origin_offset_z() {
			real_t span_z = viewable_extent_near + viewable_extent_far;
			return (span_z / 2) - viewable_extent_far;
		}

		Vector3 get_geometric_center() {
			return FAR * get_origin_offset_z();
		}
	};

	void _ready() override;

	void set_content_scale(float scale);
	float get_content_scale() const;

	void set_gameboard_type(String gbtype);
	String get_gameboard_type() const;

	void set_gameboard_orientation(Vector3 eulerAngles);
	Vector3 get_gameboard_orientation() const;

	void set_show_at_runtime(bool show);
	bool get_show_at_runtime() const;

	void set_layer_mask(uint32_t p_mask);
	uint32_t get_layer_mask() const;

	void set_layer_mask_value(int p_layer_number, bool p_enable);
	bool get_layer_mask_value(int p_layer_number) const;

	T5Gameboard();
	~T5Gameboard();

protected:
	static void _bind_methods();

private:
	Node3D* add_scene(String path);
	void set_board_state();

	float content_scale = 1.0;
	TiltFiveXRInterface::GameBoardType gameboard_type = TiltFiveXRInterface::GameBoardType::LE_GAMEBOARD;
	
	Vector3 gameboard_basis_eulers;
	bool show_at_runtime = false;
	uint32_t layers = 1;

	inline static const real_t STAGE_VISIBILITY_ANGULAR_THRESHOLD_DEGREES = 6.5;	// TODO: JSTEVENS@T5 - Worth exposing this to user?

	Node3D* le_stage_node = nullptr;
	Node3D* xe_stage_node = nullptr;
	Node3D* raised_xe_stage_node = nullptr;

	Node3D* le_gameboard_node = nullptr;
	Node3D* xe_gameboard_node = nullptr;
	Node3D* raised_xe_gameboard_node = nullptr;
};

#endif // T5_GAMEBOARD_H
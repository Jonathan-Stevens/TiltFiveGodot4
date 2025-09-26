using Godot;
using System;

public partial class T5XRRig : SubViewport
{
	T5OriginCS origin;
	T5CameraCS camera;
	T5ControllerCS leftWand;
	T5ControllerCS rightWand;

	public string GlassesID { get; set; } 
	public T5Def.GameboardType GameboardType { get; set; }
	public Aabb GameboardSize { get; set; }
	public T5OriginCS Origin {  get { return origin; } }
	public T5CameraCS Camera{ get { return camera; } }

	/// <summary>
	/// Obtains the first wand associated with the <see cref="T5XRRig"/>.
	/// See also: <seealso cref="RightWand"/>
	/// </summary>
	public T5ControllerCS Wand { get { return rightWand; } }

	/// <summary>
	/// Obtains the first wand associated with the <see cref="T5XRRig"/>.
	/// This wand is typically held in the user's right hand.
	/// See also: <seealso cref="Wand"/>
	/// </summary>
	public T5ControllerCS RightWand { get => rightWand; }

	/// <summary>
	/// Obtains the second wand associated with the <see cref="T5XRRig"/>.
	/// This wand is typically held in the user's left hand.
	/// </summary>
	public T5ControllerCS LeftWand { get { return leftWand; } }

	public T5ControllerCS Wand { get { return wand; } }
	
	// Returns the friendly name of the glasses defined in the Tilt Five control panel
	public string GlassesName { 
		get {
			var t5Interface = GetNode<T5Interface>("/root/T5Interface");
			return t5Interface?.GetGlassesName(GlassesID) ?? "";
		}
	}

	// Called when the node enters the scene tree for the first time.
	public override void _EnterTree()
	{
		base._EnterTree();

		origin = GetNode<T5OriginCS>("Origin");
		camera = GetNode<T5CameraCS>("Origin/Camera");
		rightWand = GetNode<T5ControllerCS>("Origin/Wand_1");
		leftWand = GetNode<T5ControllerCS>("Origin/Wand_2");
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
		if(rightWand != null) rightWand.Visible = rightWand.getHasTrackingData();
		if(leftWand != null) leftWand.Visible = leftWand.getHasTrackingData();
	}
}

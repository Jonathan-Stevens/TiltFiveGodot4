/*
 * Copyright (C) 2020-2023 Tilt Five, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

/// \file
/// \brief C common types for the Tilt Five™ API

#ifndef __cplusplus
#include <stdbool.h>
#include <stdint.h>
#else
#include <cstdint>
#endif

//////////////////////////////////////////////////////////
////                  Common Types                    ////
//////////////////////////////////////////////////////////

/// \defgroup C_Common_Types Tilt Five™ Common Types
/// Common types for use with C and C++ interface
/// \{

/// \brief The maximum number of characters allowed for string values
#define T5_MAX_STRING_PARAM_LEN (260)

/// \brief The minimum width required for camera image buffers
#define T5_MIN_CAM_IMAGE_BUFFER_WIDTH (768)

/// \brief The minimum height required for camera image buffers
#define T5_MIN_CAM_IMAGE_BUFFER_HEIGHT (600)

/// \brief The maximum number of gameboards that will be reported by t5GetGameboardPoses
#define T5_MAX_GAMEBOARD_POSES (1)

/// \brief 2D vector
typedef struct {
    float x;
    float y;
} T5_Vec2;

/// \brief An integer cartesian offset
typedef struct {
    uint32_t x;
    uint32_t y;
} T5_Offset;

/// \brief An integer width and height
typedef struct {
    uint32_t width;
    uint32_t height;
} T5_Extent;

/// \brief A rectangle in integer cartesian coordinates
typedef struct {
    T5_Offset offset;
    T5_Extent extent;
} T5_Rect;

/// \brief An integer cartesian offset
typedef struct {
    float x;
    float y;
} T5_Offsetf;

/// \brief An integer width and height
typedef struct {
    float width;
    float height;
} T5_Extentf;

/// \brief A rectangle in floating point cartesian coordinates
typedef struct {
    T5_Offsetf offset;
    T5_Extentf extent;
} T5_Rectf;

/// \brief 3D vector
typedef struct {
    float x;
    float y;
    float z;
} T5_Vec3;

/// \brief Quaternion
typedef struct {
    float w;
    float x;
    float y;
    float z;
} T5_Quat;

/// \brief Matrix order
typedef enum {
    kT5_MatrixOrder_RowMajor    = 1,
    kT5_MatrixOrder_ColumnMajor = 2,
} T5_MatrixOrder;

/// \brief Z Depth range
typedef enum {
    kT5_DepthRange_MinusOneToOne = 1,
    kT5_DepthRange_ZeroToOne     = 2,
} T5_DepthRange;

/// \brief Handedness of a cartesian coordinate system
///
/// For an explanation of coordinate system handedness, refer to
///  https://learn.microsoft.com/en-us/windows/win32/direct3d9/coordinate-systems
typedef enum {
    kT5_CartesianCoordinateHandedness_Left  = 1,
    kT5_CartesianCoordinateHandedness_Right = 2,
} T5_CartesianCoordinateHandedness;

/// \brief Opaque handle used with system-wide functions
///
/// Obtained from t5GetContext().<br/>
/// Release with t5ReleaseContext().
typedef struct T5_ContextImpl* T5_Context;

/// \brief Opaque handle used with glasses
///
/// Obtained from ::t5CreateGlasses().<br/>
/// Released with ::t5DestroyGlasses().
typedef struct T5_GlassesImpl* T5_Glasses;

/// \brief Opaque handle used with wands
///
/// Obtained from t5ListWandsForGlasses().<br/>
/// Release not currently required.
typedef uint8_t T5_WandHandle;

/// \brief Graphics API types
typedef enum {
    /// \brief No graphics API (for clients that don't send frames)
    kT5_GraphicsApi_None = 1,

    /// \brief OpenGL
    kT5_GraphicsApi_GL = 2,

    /// \brief Direct3D 11 (Windows Only)
    kT5_GraphicsApi_D3D11 = 3,

    /// \brief Vulkan
    kT5_GraphicsApi_Vulkan = 4,

    /// \brief Direct3D 12 (Windows Only)
    kT5_GraphicsApi_D3D12 = 5,

    /// When using the T5_TextureInfo formats below, The `vci`, `isUpsideDown`,
    /// `isSrgb`, `texWidth_PIX`, `texHeight_PIX` fields of T5_FrameInfo are ignored.
    /// The leftTexHandle and rightTexHandle fields of T5_FrameInfo must point to
    /// valid T5_TextureInfo structs.

    /// \brief OpenGL (With T5_TextureInfo textures)
    ///
    /// T5_GraphicsContextGL is unused.
    kT5_GraphicsApi_GL_TI = 12,

    /// \brief Direct3D 11 (With T5_TextureInfo textures) (Windows Only)
    kT5_GraphicsApi_D3D11_TI = 13,

    /// \brief Vulkan (With T5_TextureInfo textures)
    ///
    /// The `textureMode` field of T5_GraphicsContextVulkan is unused.
    kT5_GraphicsApi_Vulkan_TI = 14,

    /// \brief Direct3D 12 (With T5_TextureInfo textures) (Windows Only)
    kT5_GraphicsApi_D3D12_TI = 15,
} T5_GraphicsApi;

/// \brief Used by ::T5_GraphicsContextGL to specify interpretation of texture handles
/// Note that this is **not** used by ::kT5_GraphicsApi_GL_TI
typedef enum {
    /// \brief Treat ::T5_FrameInfo.leftTexHandle and ::T5_FrameInfo.rightTexHandle as a pair of
    /// GL_TEXTURE_2D.
    kT5_GraphicsApi_GL_TextureMode_Pair = 1,

    /// \brief Treat ::T5_FrameInfo.leftTexHandle as a GL_TEXTURE_2D_ARRAY.
    /// ::T5_FrameInfo.rightTexHandle is unused.
    ///
    /// Left/Right array index should be specified in ::T5_GraphicsContextGL::leftEyeArrayIndex
    /// and ::T5_GraphicsContextGL::rightEyeArrayIndex
    kT5_GraphicsApi_GL_TextureMode_Array = 2,
} T5_GraphicsApi_GL_TextureMode;

/// \brief Context for use with ::kT5_GraphicsApi_GL
/// Note that this is **not** used by ::kT5_GraphicsApi_GL_TI
typedef struct {
    /// \brief Specify the interpretation of the texture handles in ::T5_FrameInfo
    T5_GraphicsApi_GL_TextureMode textureMode;

    /// \brief In kT5_GraphicsApi_GL_TextureMode_Array, specify the array index of the left eye
    uint32_t leftEyeArrayIndex;

    /// \brief In kT5_GraphicsApi_GL_TextureMode_Array, specify the array index of the right eye
    uint32_t rightEyeArrayIndex;
} T5_GraphicsContextGL;

/// \brief Used by ::T5_GraphicsApi_Vulkan_TextureMode to specify interpretation of texture handles
/// Note that this is **not** used by ::kT5_GraphicsApi_Vulkan_TI
typedef enum {
    /// \brief Treat ::T5_FrameInfo.leftTexHandle and ::T5_FrameInfo.rightTexHandle as a pair of
    /// pointers to VkImage handles.
    kT5_GraphicsApi_Vulkan_TextureMode_Image = 1,

    /// \brief Treat ::T5_FrameInfo.leftTexHandle and ::T5_FrameInfo.rightTexHandle as a pair of
    /// pointers to VkImageView handles.
    kT5_GraphicsApi_Vulkan_TextureMode_ImageView = 2,
} T5_GraphicsApi_Vulkan_TextureMode;

typedef struct {
    /// \brief VkInstance handle
    void* instance;

    /// \brief VkPhysicalDevice handle
    void* physicalDevice;

    /// \brief VkDevice handle
    void* device;

    /// \brief VkQueue handle
    ///
    /// Requirements:
    ///  * Queue family must support VK_QUEUE_COMPUTE_BIT
    void* queue;

    /// \brief The queue family index used to create the queue
    uint32_t queueFamilyIndex;

    /// \brief Specify the interpretation of the texture handles in ::T5_FrameInfo
    /// Note that this is **not** used by ::kT5_GraphicsApi_Vulkan_TI
    T5_GraphicsApi_Vulkan_TextureMode textureMode;
} T5_GraphicsContextVulkan;

typedef struct {
    /// \brief A pointer to a ID3D12Device
    void* device;

    /// \brief A pointer to a ID3D12CommandQueue
    void* queue;
} T5_GraphicsContextD3D12;

/// \brief Possible gameboard types
typedef enum {
    /// \brief No gameboard
    kT5_GameboardType_None = 1,

    /// \brief An LE gameboard
    kT5_GameboardType_LE = 2,

    /// \brief An XE gameboard, flap laid flat
    kT5_GameboardType_XE = 3,

    /// \brief An XE gameboard, flap raised at an angle on the kickstand
    kT5_GameboardType_XE_Raised = 4,
} T5_GameboardType;

/// \brief Physical dimensions of a gameboard.
typedef struct {
    /// \brief The distance in meters from the gameboard origin to the edge of the viewable area in
    /// the positive X direction.
    float viewableExtentPositiveX;

    /// \brief The distance in meters from the gameboard origin to the edge of the viewable area in
    /// the negative X direction.
    float viewableExtentNegativeX;

    /// \brief The distance in meters from the gameboard origin to the edge of the viewable area in
    /// the positive Y direction.
    float viewableExtentPositiveY;

    /// \brief The distance in meters from the gameboard origin to the edge of the viewable area in
    /// the negative Y direction.
    float viewableExtentNegativeY;

    /// \brief The distance in meters above the gameboard origin that the viewable area extends in
    /// the positive Z direction.
    float viewableExtentPositiveZ;
} T5_GameboardSize;

/// \brief Client provided information for use with t5CreateGlasses()
typedef struct {
    /// \brief The application ID.
    const char* applicationId;

    /// \brief The application version.
    const char* applicationVersion;

    /// \brief The SDK type.
    ///
    /// Should be set to 0x00 unless otherwise instructed by T5 staff.
    uint8_t sdkType;

    /// \brief RESERVED: Must be set to 0
    uint64_t reserved;
} T5_ClientInfo;

/// \brief Glasses connection state
typedef enum {
    /// \brief Glasses are connected for exclusive use
    kT5_ConnectionState_ExclusiveConnection = 1,

    /// \brief Glasses are reserved for exclusive use
    kT5_ConnectionState_ExclusiveReservation = 2,

    /// \brief Glasses have not been exclusively connected or reserved
    kT5_ConnectionState_NotExclusivelyConnected = 3,

    /// \brief Glasses were previously exclusively connected, but the device has disconnected
    kT5_ConnectionState_Disconnected = 4,
} T5_ConnectionState;

/// \brief Glasses pose usage indicator
typedef enum {
    /// \brief The pose will be used to render images to be presented on the glasses.
    ///
    /// Querying a glasses pose for this usage will return a pose prediction intended to account for
    /// the render and presentation latency. The predicted pose is prone to include errors, and the
    /// rendered images may appear very jittery if they are displayed on a device other than the
    /// glasses. When displayed via the glasses, the on-glasses image stabilization compensates for
    /// this prediction error, so the image should not appear jittery.
    kT5_GlassesPoseUsage_GlassesPresentation = 1,

    /// \brief The pose will be used to render images to be presented on a device other than the
    /// glasses, such at the host system's primary display.
    ///
    /// Querying a glasses pose for this usage will return a pose with less noise than that intended
    /// for presentation via the glasses.
    kT5_GlassesPoseUsage_SpectatorPresentation = 2,
} T5_GlassesPoseUsage;

/// \brief Glasses pose information to be retrieved with t5GetGlassesPose()
///
/// The pose describes the relationship between two reference frames: one defined in terms of the
/// glasses, and the other in terms of the stage. Both reference frames are right-handed. The
/// glasses reference frame, abbreviated as GLS, has its origin at the midpoint between the
/// effective optical position of the projectors. It is oriented such that +X points to the right,
/// +Y points up, and +Z points backward for someone wearing the glasses. The STAGE reference
/// is oriented such that +X points to the right, +Y points forward, and +Z points up with respect
/// to gravity. The origin of the stage reference frame is located gravitationally coplanar with
/// the lowest corner of the given gameboard and below the point equidistant from the three
/// gameboard sides nearest to the T5 logo (i.e. the side on which the logo appears and the two
/// adjacent sides). This places the stage origin below the center of the square LE gameboard,
/// and off-center along the longer dimension of the rectangular XE gameboard.
typedef struct {
    /// \brief The timestamp of the pose.
    uint64_t timestampNanos;

    /// \brief The position of the origin of the GLS (glasses) frame relative to the STAGE
    /// frame.
    T5_Vec3 posGLS_STAGE;

    /// \brief The rotation that transforms points in the STAGE frame orientation to the
    /// GLS (glasses) frame orientation.
    T5_Quat rotToGLS_STAGE;

    /// \brief The type of gameboard visible for this pose
    T5_GameboardType gameboardType;
} T5_GlassesPose;

/// \brief Gameboard pose information to be retrieved by enumerating the output of
/// t5GetGameboardPoses()
///
/// The pose describes the relationship between two reference frames: one defined in terms of the
/// stage, and the other in terms of the gameboard. Both reference frames are right-handed.
/// The STAGE reference frame is oriented such that, when standing on the side of the physical
/// gameboard with the T5 logo, +X points to the right, +Y points forward, and +Z points
/// gravitationally up. The origin of the STAGE reference frame is located at the
/// point equidistant from the three gameboard sides nearest to the T5 logo (i.e. the side on which
/// the logo appears and the two adjacent sides). This places the gameboard origin in the center of
/// the square LE gameboard, and off-center along the longer dimension of the rectangular XE
/// gameboard. The gameboard reference frame, BOARD, is oriented such that +X points to the right,
/// +Y points forward, and +Z points normal to and away from the reflective plane of the gameboard.
/// The origin of BOARD space is located at the point equidistant from the three gameboard sides
/// nearest to the T5 logo (i.e. the side on which the logo appears and the two adjacent sides).
/// The BOARD space is identical to STAGE space when the physical gameboard is flat wrt gravity.
/// When the physical gameboard is tilted up at an angle, STAGE space z remains gravity aligned
/// where BOARD space z remains aligned with the normal vector to the reflective gameboard plane.
typedef struct {
    /// \brief The timestamp of the pose.
    uint64_t timestampNanos;

    /// \brief The position of the origin of the BOARD (gameboard) frame relative to the STAGE
    /// frame.
    T5_Vec3 posBOARD_STAGE;

    /// \brief The rotation that transforms points in the STAGE frame orientation to the
    /// BOARD (gameboard) frame orientation.
    T5_Quat rotToBOARD_STAGE;

    /// \brief The type of gameboard represented by this pose
    T5_GameboardType gameboardType;

    /// \brief The id of this gameboard pose
    uint64_t id;
} T5_GameboardPose;

/// \brief Camera stream configuration
typedef struct {
    /// \brief The index of the camera to be modified.
    uint8_t cameraIndex;

    /// \brief Enable or disable the camera stream. True = enabled
    bool enabled;
} T5_CameraStreamConfig;

/// Render information to be used with t5SendFrameToGlasses()
typedef struct {
    /// \brief Texture handle for the left image.
    ///
    /// The meaning of the handle will depend on the current graphics API.
    ///
    /// \see aboutGraphicsApi for further details.
    void* leftTexHandle;

    /// \brief Texture handle for the right image.
    ///
    /// The meaning of the handle will depend on the current graphics API.
    ///
    /// \see aboutGraphicsApi for further details.
    void* rightTexHandle;

    /// \brief Width of the textures pointed to by leftTexHandle and rightTexHandle.
    uint16_t texWidth_PIX;

    /// \brief Height of the textures pointed to by leftTexHandle and rightTexHandle.
    uint16_t texHeight_PIX;

    /// \brief True if the texture is srgb. This is only relevant for the OpenGL graphics API.
    bool isSrgb;

    /// \brief True if the image is 'upside down'.
    bool isUpsideDown;

    /// \brief The image rectangle in the normalized (z=1) image space of the virtual cameras.
    struct {
        float startX_VCI;
        float startY_VCI;
        float width_VCI;
        float height_VCI;
    } vci;

    /// \brief The rotation from STAGE to VC, the virtual camera reference frame for the left eye.
    T5_Quat rotToLVC_STAGE;

    /// \brief The position of VC, the virtual camera reference frame, relative to STAGE for the
    /// left eye.
    T5_Vec3 posLVC_STAGE;

    /// \brief The rotation from STAGE to VC, the virtual camera reference frame for the right eye
    T5_Quat rotToRVC_STAGE;

    /// \brief The position of VC, the virtual camera reference frame, relative to STAGE for the
    /// right eye.
    T5_Vec3 posRVC_STAGE;
} T5_FrameInfo;

typedef struct {
    struct {
        /// \brief Texture handle.
        ///
        /// The meaning of the handle will depend on the current graphics API.
        ///
        /// \see aboutGraphicsApi for further details.
        void* texture;

        /// \brief The dimensions of the texture
        ///
        /// This may not be the same as the actual displayed image as
        /// the subImage struct defines which part of the image is used.
        T5_Extent dimensions;

        /// \brief The array size of array based textures
        uint16_t arraySize;
    } texture;

    struct {
        /// \brief A rectangle defining which part of the texture is used.
        ///
        /// This must be specified even if the full image is used.
        T5_Rect rect;

        /// \brief For array textures, the index of the array to use.
        ///
        /// Ignored for non-array textures
        uint16_t arrayIndex;
    } subImage;

    /// \brief The graphics API specific format used to create this texture
    ///
    /// E.g. VK_FORMAT_R8G8B8A8_SRGB, GL_SRGB8_ALPHA8, DXGI_FORMAT_R8G8B8A8_UNORM_SRGB
    /// This is always a concrete format (IE Not DXGI_FORMAT_R8G8B8A8_TYPELESS)
    int64_t format;

    /// \brief The image rectangle in the normalized (z=1) image space of the virtual cameras.
    T5_Rectf vci;
} T5_TextureInfo;

/// \brief Camera Frame information to be retrieved with t5GetFilledCamImageBuffer()
typedef struct {
    /// \brief The width of the image in the image buffer. Empty buffers should set these parameters
    /// to 0.
    uint16_t imageWidth;

    /// \brief The height of the image in the image buffer. Empty buffers should set these
    /// parameters to 0.
    uint16_t imageHeight;

    /// \brief The stride of the image in the image buffer. Empty buffers should set these
    /// parameters to 0.
    uint16_t imageStride;

    /// \brief The index of the desired camera. 0 for tangible tracking camera, 1 for head tracking
    /// camera.
    uint8_t cameraIndex;

    /// \brief The illumination mode for incoming frames. 0 for unknown frame. 1 for Light frames. 2
    /// for dark frame.
    uint8_t illuminationMode;

    /// \brief The total size of the provided image buffer. Must be at least
    /// T5_MIN_CAM_IMAGE_BUFFER_WIDTH * T5_MIN_CAM_IMAGE_BUFFER_HEIGHT.
    uint32_t bufferSize;

    /// \brief The image buffer being filled by the Tilt Five service.
    uint8_t* pixelData;

    /// \brief The position of the camera relative to the STAGE.
    T5_Vec3 posCAM_STAGE;

    /// \brief The rotation of the camera relative to the STAGE.
    T5_Quat rotToCAM_STAGE;
} T5_CamImage;

typedef struct {
    /// \brief The position of the pixel in PIX space. This space is defined as the warped image
    /// space that comes as camera frame returned by either the head tracking or tangible tracking
    /// camera. +X, +Y point right and down, respectively, relative to the glasses.
    T5_Vec2 pixelCoord_PIX;

    /// \brief The dewarped position of the pixel in IMG space. This space is defined as the
    /// dewarped imaged space at z=1 relative to the camera lens, a dewarped camera frame with
    /// respect to either the head tracking or tangible tracking camera. +X, +Y point right and
    /// down, respectively, relative to the glasses.
    T5_Vec2 pixelCoord_IMG;

    /// \brief The index of the camera the pixel coordinate came from. 0 for tangible tracking
    /// camera, 1 for head tracking camera.
    uint8_t cameraIndex;
} T5_PixelDewarp;

/// \brief Wand stream configuration
typedef struct {
    /// \brief Enable or disable the entire stream. True = enabled
    bool enabled;

} T5_WandStreamConfig;

/// \brief Wand stream event type
typedef enum {
    /// \brief Wand connected
    kT5_WandStreamEventType_Connect = 1,

    /// \brief Wand disconnected
    kT5_WandStreamEventType_Disconnect = 2,

    /// \brief Stream has desynchronized
    kT5_WandStreamEventType_Desync = 3,

    /// \brief Wand report (Pose, Buttons, Trigger, Stick, Battery)
    kT5_WandStreamEventType_Report = 4,
} T5_WandStreamEventType;

/// \brief Wand hand
typedef enum {
    /// \brief Hand unknown
    kT5_Hand_Unknown = 0,

    /// \brief Left hand
    kT5_Hand_Left = 1,

    /// \brief Right hand
    kT5_Hand_Right = 2,
} T5_Hand;

/// \brief Contains wand related information (Pose, Buttons, Trigger, Stick, Battery)
typedef struct {
    /// \brief The timestamp of the wand event in nanoseconds
    uint64_t timestampNanos;

    /// \brief Validity of analog parameters. True = valid
    bool analogValid;

    /// \brief Validity of battery parameters. True = valid
    bool batteryValid;

    /// \brief Validity of button parameters. True = valid
    bool buttonsValid;

    /// \brief Validity of pose parameters. True = valid
    bool poseValid;

    /// \brief Trigger - Analog, Range [0.0 - 1.0], 1.0 = Fully depressed
    float trigger;

    /// \brief Stick (X/Y) - Analog, Range [-1.0 - 1.0], 0 = Centered, 1.0 = Top/Right
    T5_Vec2 stick;

    // TODO(khunt) : Determine units
    /// Battery
    uint8_t battery;

    /// \brief Buttons state. True = Pressed
    struct {
        bool t5;
        bool one;
        bool two;
        bool three;
        bool a;
        bool b;
        bool x;
        bool y;
    } buttons;

    /// \brief WND/STAGE rotation unit quaternion
    ///
    /// The rotation unit quaternion that takes points from the STAGE
    /// reference frame to the WND (wand) reference frame orientation.
    T5_Quat rotToWND_STAGE;

    /// \brief Position (Aim Point) - Vector3f
    T5_Vec3 posAim_STAGE;

    /// \brief Position (Fingertips) - Vector3f
    T5_Vec3 posFingertips_STAGE;

    /// \brief Position (Grip) - Vector3f
    T5_Vec3 posGrip_STAGE;

    /// \brief Wand hand
    T5_Hand hand;
} T5_WandReport;

/// \brief Represents an event from the wand stream
typedef struct {
    /// \brief Opaque identifier for the wand
    T5_WandHandle wandId;

    /// \brief Type of event
    T5_WandStreamEventType type;

    /// \brief The timestamp of the wand event in nanoseconds
    uint64_t timestampNanos;

    /// \brief Report (Valid if type = ::kT5_WandStreamEventType_Report)
    T5_WandReport report;
} T5_WandStreamEvent;

/// \brief Possible parameters that can be retrieved for glasses
typedef enum {
    /// \brief <a href="https://en.wikipedia.org/wiki/Pupillary_distance">Interpupillary
    /// distance</a> - Float, millimeters
    kT5_ParamGlasses_Float_IPD = 1,

    /// User-facing name of the glasses - UTF8
    kT5_ParamGlasses_UTF8_FriendlyName = 6,
} T5_ParamGlasses;

/// \brief Possible parameters that can be retrieved with \ref sys_getParam.
typedef enum {
    /// \brief Version of the service software - UTF8
    kT5_ParamSys_UTF8_Service_Version = 1,

    /// \brief Non-zero if the control panel requires user interaction
    /// (E.g. Important firmware update) - Integer, boolean
    kT5_ParamSys_Integer_CPL_AttRequired = 2,
} T5_ParamSys;

/// \brief Projection parameters
typedef struct {
    /// \brief 4×4 Perspective Projection Matrix
    double matrix[16];

    /// \brief Field of View (Y Axis in Degrees)
    double fieldOfView;

    /// \brief Aspect Ratio
    double aspectRatio;

    /// \brief Framebuffer Width
    uint16_t framebufferWidth;

    /// \brief Framebuffer Height
    uint16_t framebufferHeight;
} T5_ProjectionInfo;

/// \}

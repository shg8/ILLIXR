//
// Created by steven on 4/20/22.
//

#ifndef ILLIXR_HVVR_CONFIG_H
#define ILLIXR_HVVR_CONFIG_H

#include "vector_math.h"

#define ENABLE_DEPTH_OF_FIELD 0
#define ENABLE_FOVEATED 0
#define ENABLE_VSYNC 1
#define ENABLE_WIDE_FOV 0

enum ModelviewerScene {
    scene_home = 0,
    scene_bunny,
    scene_conference,
    scene_sponza,
    scene_bistro_interior,
    SceneCount
};
// which scene to load? Can be overwritten in the command line
static ModelviewerScene gSceneSelect = scene_sponza;

using hvvr::vector3;
struct SceneSpecification {
    vector3 defaultCameraOrigin;
    float defaultCameraYaw;
    float defaultCameraPitch;
    float scale;
    std::string filename;
};
static SceneSpecification gSceneSpecs[ModelviewerScene::SceneCount] = {
        {vector3(1.0f, 3.0f, -1.5f), -(float)M_PI * .7f, (float)M_PI * -.05f, 1.0f, "oculus_home.bin"}, // Oculus Home
        {vector3(-0.253644f, 0.577575f, 1.081316f), -0.162111f, -0.453079f, 1.0f, "bunny.bin"},         // Stanford Bunny
        {vector3(10.091616f, 4.139270f, 1.230567f), -5.378105f, -0.398078f, 1.0f, "conference.bin"},    // Conference Room
        {vector3(4.198845f, 6.105420f, -0.400903f), -4.704108f, -0.200078f, .01f, "sponza.bin"},        // Crytek Sponza
        {vector3(2.0f, 2.0f, -0.5f), -(float)M_PI * .5f, (float)M_PI * -.05f, 1.0f, "bistro.bin"}       // Amazon Bistro
};

struct CameraSettings {
    float lensRadius = (ENABLE_DEPTH_OF_FIELD == 1) ? 0.0015f : 0.0f;
    float focalDistance = 0.3f;
    bool foveatedCamera = (ENABLE_FOVEATED == 1);
};

struct CameraControl {
    vector3 pos = {};
    float yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
    hvvr::transform toTransform() const {
        return hvvr::transform(pos, hvvr::quaternion::fromEulerAngles(yaw, pitch, 0), 1.0f);
    }
};

#endif //ILLIXR_HVVR_CONFIG_H

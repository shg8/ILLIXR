/**
 * Copyright (c) 2017-present, Facebook, Inc. and its affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 */

#include "model_import.h"
#include "raycaster.h"

#include <cstring>
#include <iostream>

namespace model_import {

bool compareExtension(const char* a, const char* b) {
    if (strcmp(a, b) == 0)
        return true;
    return false;
}

bool load(const char* path, Model& model) {
    const char* extension = strrchr(path, '.');
    if (extension == nullptr)
        return false;
    extension++;

    if (compareExtension(extension, "bin"))
    {
        return loadBin(path, model);
    }

#if MODEL_IMPORT_ENABLE_FBX
    // attempt to load it with the FBX SDK
    return loadFbx(path, model);
#else
    return false;
#endif
}

bool createObjects(hvvr::Raycaster& raycaster, Model& model) {
    std::cout << "Loading texture" << std::endl;
    for (const Texture& texture : model.textures) {
        hvvr::Texture* object = raycaster.createTexture(texture.tex);
        if (object == nullptr)
            return false;
    }

    std::cout << "Loading light source" << std::endl;
    for (const hvvr::LightUnion& light : model.lights) {
        hvvr::Light* object = raycaster.createLight(light);
        if (object == nullptr)
            return false;
    }

    std::cout << "Loading mesh" << std::endl;
    for (const Mesh& mesh : model.meshes) {
        hvvr::Model* object = raycaster.createModel(mesh.data);
        if (object == nullptr)
            return false;

        object->setTransform(mesh.transform);
    }

    std::cout << "Model loaded in raycaster" << std::endl;
    return true;
}

} // namespace model_import

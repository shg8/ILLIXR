#pragma once

/**
 * Copyright (c) 2017-present, Facebook, Inc. and its affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 */

#include "gpu_buffer.h"
#include "gpu_camera.h"
#include "gpu_samples.cuh"
#include "vector_math.h"


namespace hvvr {

// For conveniently passing four planes by value to the world space transformation kernel.
struct FourPlanes {
    Plane data[4];
};

void ComputeEyeSpaceFrusta(const GPUBuffer<DirectionalBeam>& dirSamples,
                           GPUBuffer<SimpleRayFrustum>& tileFrusta,
                           GPUBuffer<SimpleRayFrustum>& blockFrusta);

} // namespace hvvr

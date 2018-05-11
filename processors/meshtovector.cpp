/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2018 Inviwo Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/

#include <modules/itk/processors/meshtovector.h>
#include <inviwo/core/datastructures/geometry/basicmesh.h>
#include <inviwo/core/datastructures/geometry/simplemesh.h>
#include <inviwo/core/datastructures/buffer/buffer.h>
#include <inviwo/core/datastructures/buffer/bufferram.h>
#include <inviwo/core/datastructures/buffer/bufferramprecision.h>
#include <inviwo/core/util/stdextensions.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo MeshToVector::processorInfo_{
    "org.inviwo.MeshToVector",  // Class identifier
    "Mesh To Vector",           // Display name
    "Undefined",                // Category
    CodeState::Experimental,    // Code state
    Tags::None,                 // Tags
};
const ProcessorInfo MeshToVector::getProcessorInfo() const { return processorInfo_; }

MeshToVector::MeshToVector() : Processor(), mesh_("mesh"), points_("points") {

    addPort(mesh_);
    addPort(points_);
}

void MeshToVector::process() {

    auto points = [this]() {
        if (auto dm = std::dynamic_pointer_cast<const BasicMesh>(mesh_.getData())) {
            return std::make_shared<std::vector<vec3>>(
                dm->getVertices()->getRAMRepresentation()->getDataContainer());

        } else if (auto sm = std::dynamic_pointer_cast<const SimpleMesh>(mesh_.getData())) {
            return std::make_shared<std::vector<vec3>>(
                sm->getVertexList()->getRAMRepresentation()->getDataContainer());
        } else {
            auto b = mesh_.getData()->getBuffer(0);
            if(auto b3 = dynamic_cast<const Buffer<vec3>*>(b)){
                return std::make_shared<std::vector<vec3>>(b3->getRAMRepresentation()->getDataContainer());
            }

            throw inviwo::Exception("Not a simple or basic mesh", IvwContext);
        }
    }();

    auto m = mesh_.getData()->getWorldMatrix() * mesh_.getData()->getModelMatrix();

    *points = util::transform(*points,[&](const vec3 &p){
        auto P = m * vec4(p,1);
        return vec3(P)/P.w;
    });

    points_.setData(points);
}

}  // namespace inviwo

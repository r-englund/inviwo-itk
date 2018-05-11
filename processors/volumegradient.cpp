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

#include <modules/itk/processors/volumegradient.h>

#include <modules/itk/utils/itktypes.h>

#include "itkGradientImageFilter.h"

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo ITKVolumeGradient::processorInfo_{
    "org.inviwo.ITKVolumeGradient",  // Class identifier
    "ITK Volume Gradient",           // Display name
    "ITK",                           // Category
    CodeState::Experimental,         // Code state
    "ITK",                           // Tags
};
const ProcessorInfo ITKVolumeGradient::getProcessorInfo() const { return processorInfo_; }

ITKVolumeGradient::ITKVolumeGradient() : Processor() {
    addPort(inport_);
    addPort(outport_);
}

void ITKVolumeGradient::process() {

    inport_.getData()->getRepresentation<VolumeRAM>()->dispatch<void, dispatching::filter::Scalars>(
        [&](auto ram) {
            auto img = itkutil::fromInviwo(*ram,*inport_.getData());

            using ImgPtr = decltype(img);

            auto gradFilter = itk::GradientImageFilter<ImgPtr::ObjectType>::New();
            gradFilter->SetInput(img);
            gradFilter->Update();
            auto vol = itkutil::fromITK(*gradFilter->GetOutput());
            vol->dataMap_.dataRange = dvec2(-1,1);
            outport_.setData(vol);

        });
}

}  // namespace inviwo

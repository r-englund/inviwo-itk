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


#include <modules/itk/processors/itkimagelowpass.h>

#include <modules/itk/utils/itktypes.h>
#include <modules/itk/utils/itklowpass.h>

namespace inviwo {

/*

    template <typename Format>
    struct Filter
        : std::integral_constant<bool,
        Format::comp == 1 && (
            (Format::numtype == NumericType::Float && Format::typesize != 2)
            )
        > {
    };
    //*/

//*
template <typename Format>
struct Filter : std::integral_constant<bool, !std::is_same<f16,typename Format::primitive>::value > {};
//*/
// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo ITKImageLowpass::processorInfo_{
    "org.inviwo.ITKImageLowpass",  // Class identifier
    "ITKImage Lowpass",            // Display name
    "Undefined",                   // Category
    CodeState::Experimental,       // Code state
    Tags::None,                    // Tags
};
const ProcessorInfo ITKImageLowpass::getProcessorInfo() const { return processorInfo_; }

ITKImageLowpass::ITKImageLowpass() : Processor() {
    addPort(inport_);
    addPort(outport_);

    addProperty(var_);
}

void ITKImageLowpass::process() {
    inport_.getData()->getColorLayer()->getRepresentation<LayerRAM>()->dispatch<void,Filter>(
        [&](auto ram) {
            auto img = itkutil::fromInviwo(*ram);
            // auto itk = itkutil::fromITK(*img);
            auto res = itkutil::lowpass(*img, var_.get());
            outport_.setData(itkutil::fromITK(*res));

        });
    //
}

}  // namespace inviwo

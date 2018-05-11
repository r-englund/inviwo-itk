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

#include <modules/itk/processors/itktest.h>

#include <inviwo/core/datastructures/volume/volume.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <inviwo/core/datastructures/volume/volumeramprecision.h>

#include <modules/itk/utils/itktypes.h>

#include "itkImage.h"
#include "itkImportImageFilter.h"
#include "itkAddImageFilter.h"
#include "itkSubtractImageFilter.h"
#include "itkImageRegionIterator.h"
#include "itkImageRegionIteratorWithIndex.h"
#include "itkAbsoluteValueDifferenceImageFilter.h"
#include "itkDemonsRegistrationFilter.h"
#include "itkGradientImageFilter.h"
#include "itkGradientRecursiveGaussianImageFilter.h"
#include "itkSymmetricForcesDemonsRegistrationFilter.h"
#include "itkCastImageFilter.h"
#include "itkWarpImageFilter.h"
//#include "itkLinearInterpilateImageFunction.h"

//#include "itkImageFileWriter.h"

namespace inviwo {
// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo ITKTest::processorInfo_{
    "org.inviwo.ITKTest",     // Class identifier
    "ITKTest",                // Display name
    "ITK",                    // Category
    CodeState::Experimental,  // Code state
    Tags::None,               // Tags
};
const ProcessorInfo ITKTest::getProcessorInfo() const { return processorInfo_; }

template <typename Format>
struct Filter
    : std::integral_constant<bool,
                             Format::comp == 1 && (
                                 Format::numtype != NumericType::Float ||
                                                                  (Format::numtype == NumericType::Float && Format::typesize != 2))
                                 > {
};

ITKTest::ITKTest() : Processor() {

    addPort(volumeA_);
    addPort(volumeB_);
    addPort(volumeOut_);

    addProperty(numOfIterations_);
    addProperty(useSymDemon_);
    addProperty(setInitDisplacementField_);
}

void ITKTest::process() {
    auto dims = volumeA_.getData()->getDimensions();
    if (dims != volumeB_.getData()->getDimensions()) {
        throw inviwo::Exception("Volumes need to be of same size", IvwContext);
    }
    if (volumeA_.getData()->getDataFormat() != volumeB_.getData()->getDataFormat()) {
        throw inviwo::Exception("Volumes need to have the same format", IvwContext);
    }

    std::shared_ptr<Volume> outVol;

    auto keepAlive = volumeB_.getData();

    volumeA_.getData()->getRepresentation<VolumeRAM>()->dispatch<void, Filter>([&](auto ramA) {
        using VolType = util::PrecsionType<decltype(ramA)>;
        using T = util::PrecsionValueType<decltype(ramA)>;
        using C = typename util::value_type<T>::type;

        // ITK Types
        using Img = itk::Image<C, 3>;
        using ImportFilter = itk::ImportImageFilter<C, 3>;
        using Vec3 = itk::Vector<float, 3>;
        using u8Vec3 = itk::Vector<unsigned char, 3>;
        using Vec3Img = itk::Image<Vec3, 3>;
        using Demon = itk::DemonsRegistrationFilter<Img, Img, Vec3Img>;
        using SymDemon = itk::SymmetricForcesDemonsRegistrationFilter<Img, Img, Vec3Img>;

        using Warp = itk::WarpImageFilter<Img, Img, Vec3Img>;
        using LinFunc = itk::LinearInterpolateImageFunction<Img, double>;

        auto ramB = static_cast<decltype(ramA)>(volumeB_.getData()->getRepresentation<VolumeRAM>());

        auto imgA = itkutil::fromInviwo(*ramA, *volumeA_.getData());
        auto imgB = itkutil::fromInviwo(*ramB, *volumeB_.getData());



        auto warp = Warp::New();
        auto interpolator = LinFunc::New();

        auto run = [&](auto reg) {
            reg->SetFixedImage(imgA);
            reg->SetMovingImage(imgB);
            if (setInitDisplacementField_) {
                using Grad = itk::GradientImageFilter<Img>;
                using Cast =
                    itk::CastImageFilter<Grad::OutputImageType, Demon::DisplacementFieldType>;

                auto gradFilter = Grad::New();
                gradFilter->SetInput(imgA);
                gradFilter->Update();
                auto caster = Cast::New();
                caster->SetInput(gradFilter->GetOutput());
                caster->Update();

                reg->SetInitialDisplacementField(caster->GetOutput());
            }
            reg->SetNumberOfIterations(numOfIterations_.get());

            reg->SetStandardDeviations(1.0);
            reg->Update();

            warp->SetInput(imgB);
            warp->SetInterpolator(interpolator);
            warp->SetOutputSpacing(imgA->GetSpacing());
            warp->SetOutputOrigin(imgA->GetOrigin());
            warp->SetOutputDirection(imgA->GetDirection());
            warp->SetDisplacementField(reg->GetOutput());
            warp->Update();
            //return imgA;

            return warp->GetOutput();
            // return reg->GetOutput();*/
        };

        if (useSymDemon_) {
            auto reg = SymDemon::New();
            outVol = itkutil::fromITK(*run(reg));
        } else {
            auto reg = Demon::New();
            outVol = itkutil::fromITK(*run(reg));
        }

    });

    volumeOut_.setData(outVol);
    outVol->copyMetaDataFrom(*volumeA_.getData());
    outVol->dataMap_.dataRange = dvec2(-1, 1);
    outVol->dataMap_ = volumeA_.getData()->dataMap_;

    outVol->setWorldMatrix(volumeA_.getData()->getWorldMatrix());
    outVol->setModelMatrix(volumeA_.getData()->getModelMatrix());
}

}  // namespace inviwo

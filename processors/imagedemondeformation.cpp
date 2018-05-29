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

#include <modules/itk/processors/imagedemondeformation.h>


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

namespace inviwo {


    template <typename Format>
    struct Filter
        : std::integral_constant<bool,
        Format::comp == 1 && (
            Format::numtype != NumericType::Float ||
            (Format::numtype == NumericType::Float && Format::typesize != 2))
        > {
    };

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo ImageDemonDeformation::processorInfo_{
    "org.inviwo.ImageDemonDeformation",      // Class identifier
    "Image Demon Deformation",                // Display name
    "Undefined",              // Category
    CodeState::Experimental,  // Code state
    Tags::None,               // Tags
};
const ProcessorInfo ImageDemonDeformation::getProcessorInfo() const { return processorInfo_; }

ImageDemonDeformation::ImageDemonDeformation()
    : Processor()
{

    addPort(fixedImage_);
    addPort(movingImage_);
    addPort(outport_);
    addPort(deformationField_);

    addProperty(numOfIterations_);
    addProperty(useSymDemon_);
    addProperty(setInitDisplacementField_);
}

void ImageDemonDeformation::process() {


    auto movingImg = movingImage_.getData();
    auto fixedImg = fixedImage_.getData();

    auto dims = fixedImg->getDimensions();
    if (dims != movingImg->getDimensions()) {
        throw inviwo::Exception("Images need to be of same size", IvwContext);
    }
    if (fixedImg->getDataFormat() != movingImg->getDataFormat()) {
        throw inviwo::Exception("Images need to have the same format", IvwContext);
    }


    movingImg->getColorLayer()->getRepresentation<LayerRAM>()->dispatch<void, Filter>([&](auto ramA) {

        using ImgType = util::PrecsionType<decltype(ramA)>;
        using T = util::PrecsionValueType<decltype(ramA)>;
        using C = typename util::value_type<T>::type;

        // ITK Types
        using Img = itk::Image<C, 2>;
        using ImportFilter = itk::ImportImageFilter<C, 2>;
        using Vec2 = itk::Vector<float, 2>;
        using CoVec2 = itk::CovariantVector<float, 2>;
        using u8Vec2 = itk::Vector<unsigned char, 2>;
        using Vec2Img = itk::Image<Vec2, 2>;
        using CoVec2Img = itk::Image<CoVec2, 2>;
        using Demon = itk::DemonsRegistrationFilter<Img, Img, CoVec2Img>;
        using SymDemon = itk::SymmetricForcesDemonsRegistrationFilter<Img, Img, CoVec2Img>;

        using Warp = itk::WarpImageFilter<Img, Img, Vec2Img>;
        using LinFunc = itk::LinearInterpolateImageFunction<Img, double>;




        auto ramB = static_cast<decltype(ramA)>(fixedImg->getColorLayer()->getRepresentation<LayerRAM>());

        auto mImg = itkutil::fromInviwo(*ramA);
        auto fImg = itkutil::fromInviwo(*ramB);

        auto warp = Warp::New();
        auto interpolator = LinFunc::New();

        auto run = [&](auto reg) {
            reg->SetFixedImage(fImg);
            reg->SetMovingImage(mImg);
            if (setInitDisplacementField_) {
                using Grad = itk::GradientImageFilter<Img>;
                using Cast =
                    itk::CastImageFilter<Grad::OutputImageType, Demon::DisplacementFieldType>;

                auto gradFilter = Grad::New();
                gradFilter->SetInput(fImg);
                gradFilter->Update();
                auto caster = Cast::New();
                caster->SetInput(gradFilter->GetOutput());
                caster->Update();

                //reg->SetInitialDisplacementField(caster->GetOutput());
            }
            reg->SetNumberOfIterations(numOfIterations_.get());

            reg->SetStandardDeviations(1.0);
            reg->Update();

            warp->SetInput(mImg);
            warp->SetInterpolator(interpolator);
            warp->SetOutputSpacing(fImg->GetSpacing());
            warp->SetOutputOrigin(fImg->GetOrigin());
            warp->SetOutputDirection(fImg->GetDirection());
            //warp->SetDisplacementField(reg->GetOutput());
            warp->Update();

            outport_.setData(itkutil::fromITK(*warp->GetOutput()));
            deformationField_.setData(itkutil::fromITK(*reg->GetOutput()));
        };


        if (useSymDemon_) {
            run(SymDemon::New());
        } else {
            run(Demon::New());
        }


    });



    // outport_.setData(myImage);
}

}  // namespace inviwo

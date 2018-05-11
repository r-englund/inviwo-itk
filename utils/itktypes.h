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

#ifndef IVW_ITKTYPES_H
#define IVW_ITKTYPES_H

#include <modules/itk/itkmoduledefine.h>
#include <inviwo/core/common/inviwo.h>

#include <inviwo/core/datastructures/volume/volume.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <inviwo/core/datastructures/volume/volumeramprecision.h>


#include <inviwo/core/datastructures/image/image.h>
#include <inviwo/core/datastructures/image/layer.h>
#include <inviwo/core/datastructures/image/layerram.h>
#include <inviwo/core/datastructures/image/layerramprecision.h>

#include "itkImage.h"
#include "itkImportImageFilter.h"
#include "itkImageRegionIteratorWithIndex.h"

#pragma optimize("", off)

namespace inviwo {

namespace util {

template <typename T, unsigned D>
struct extent<itk::Vector<T, D>, 0> : public std::integral_constant<std::size_t, D> {};
template <typename T, unsigned D>
struct extent<itk::CovariantVector<T, D>, 0> : public std::integral_constant<std::size_t, D> {};
}  // namespace util

namespace itkutil {

template <typename T, typename VT = typename util::value_type<T>::type,
          unsigned Dim = util::extent<T>::value == 0 ? 1 : util::extent<T>::value,
          typename PixelType = std::conditional_t<Dim==1,VT,itk::Vector<VT,Dim>> ,
          typename IMG = itk::Image<PixelType, 2>, typename IMGP = IMG::Pointer,
          typename Import = itk::ImportImageFilter<PixelType, 2> >
IMGP fromInviwo(const LayerRAMPrecision<T> &ram) {

    auto dims = ram.getDimensions();

    Import::SizeType size;
    size[0] = dims.x;
    size[1] = dims.y;

    Import::IndexType start;
    start.Fill(0);

    Import::RegionType region;
    region.SetIndex(start);
    region.SetSize(size);

    Import::Pointer importFilterA = Import::New();
    importFilterA->SetRegion(region);

    const bool passOwnership = false;
    auto data = const_cast<PixelType*>( reinterpret_cast<const PixelType *>(ram.getDataTyped()));
    //auto data = const_cast<VT *>(ram.getDataTyped());
    importFilterA->SetImportPointer(data, size[0] * size[1], passOwnership);
    importFilterA->Update();
    return importFilterA->GetOutput();
}

template <typename T, typename VT = typename util::value_type<T>::type,
          unsigned Dim = util::extent<VT>::value == 0 ? 1 : util::extent<VT>::value,
          typename IMG = itk::Image<VT, 3>, typename IMGP = IMG::Pointer,
          typename Import = itk::ImportImageFilter<VT, 3> >
IMGP fromInviwo(const VolumeRAMPrecision<T> &ram, const Volume &owner) {

    auto dims = ram.getDimensions();

    Import::SizeType size;
    size[0] = dims.x;
    size[1] = dims.y;
    size[2] = dims.z;

    Import::IndexType start;
    start.Fill(0);

    Import::RegionType region;
    region.SetIndex(start);
    region.SetSize(size);

    Import::Pointer importFilterA = Import::New();
    importFilterA->SetRegion(region);

    auto offset = owner.getOffset();
    auto m = owner.getModelMatrix();

    Import::DirectionType dir;
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
            dir(row, col) = m[col][row] / dims[col];
        }
    }

    const itk::SpacePrecisionType origin[3] = {offset.x, offset.y, offset.z};
    const itk::SpacePrecisionType spacing[3] = {
        glm::length(m[0]) / dims.x, glm::length(m[1]) / dims.y, glm::length(m[2]) / dims.z};
    importFilterA->SetOrigin(origin);
    importFilterA->SetSpacing(spacing);
    importFilterA->SetDirection(dir);

    const unsigned int numPixels = size[0] * size[1] * size[2];

    const bool passOwnership = false;

    auto data = const_cast<VT *>(ram.getDataTyped());
    importFilterA->SetImportPointer(data, numPixels, passOwnership);

    importFilterA->Update();

    return importFilterA->GetOutput();
}

template <typename Pixel>
std::shared_ptr<Image> fromITK(itk::Image<Pixel, 2> &img) {
    using ItkImage = itk::Image<Pixel, 2>;
    using T = itk::NumericTraits<Pixel>::ValueType;
    const static unsigned E = util::extent<Pixel>::value;
    const static unsigned D = E == 0 ? 1 : E;
    using VT = Vector<D, T>;
    using NT = itk::NumericTraits<Pixel>;

    auto region = img.GetLargestPossibleRegion();
    auto size = region.GetSize();
    size2_t dims{size[0], size[1]};

    auto outImg = std::make_shared<Image>(dims, DataFormat<VT>::get());
    auto outData = static_cast<VT *>(outImg->getColorLayer()->getEditableRepresentation<LayerRAM>()->getData());

    size_t i = 0;
    util::IndexMapper2D index(dims);

    itk::ImageRegionIteratorWithIndex<ItkImage> imageIterator(&img, region);

    while (!imageIterator.IsAtEnd()) {
        auto pos = imageIterator.GetIndex();

        auto pixel = imageIterator.Get();
        outData[index(pos[0], pos[1])] = *reinterpret_cast<VT *>(&pixel);
        ++imageIterator;
    }

    return outImg;
}

// template <typename Image>
template <typename Pixel>
std::shared_ptr<Volume> fromITK(itk::Image<Pixel, 3> &vol) {
    using Image = itk::Image<Pixel, 3>;
    using T = itk::NumericTraits<Pixel>::ValueType;
    const static unsigned E = util::extent<Pixel>::value;
    const static unsigned D = E == 0 ? 1 : E;
    using VT = Vector<D, T>;
    using NT = itk::NumericTraits<Pixel>;

    auto region = vol.GetLargestPossibleRegion();
    auto size = region.GetSize();
    size3_t dims{size[0], size[1], size[2]};

    auto outvol = std::make_shared<Volume>(dims, DataFormat<VT>::get());
    auto outData = static_cast<VT *>(outvol->getEditableRepresentation<VolumeRAM>()->getData());

    size_t i = 0;
    util::IndexMapper3D index(dims);

    itk::ImageRegionIteratorWithIndex<Image> imageIterator(&vol, region);

    while (!imageIterator.IsAtEnd()) {
        auto pos = imageIterator.GetIndex();

        auto pixel = imageIterator.Get();
        outData[index(pos[0], pos[1], pos[2])] = *reinterpret_cast<VT *>(&pixel);
        ++imageIterator;
    }

    auto origin = vol.GetOrigin();
    auto dir = vol.GetDirection();
    mat3 basis;
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
            basis[col][row] = dir(row, col) * dims[col];
        }
    }

    outvol->setOffset(vec3{origin[0], origin[1], origin[2]});
    outvol->setBasis(basis);
    return outvol;
}

}  // namespace itkutil

}  // namespace inviwo

#endif  // IVW_ITKTYPES_H

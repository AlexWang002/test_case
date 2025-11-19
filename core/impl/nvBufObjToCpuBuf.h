/*******************************************************************************
 * \addtogroup impl
 * \{
 * \headerfile nvBufObjToCpuBuf.h "nvBufObjToCpuBuf.h"
 * \brief
 * \version 0.1
 * \date 2025-09-26
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-09-26 | Init version |
 *
 ******************************************************************************/
#ifndef I_NVBUFOBJTOCPUBUF_H
#define I_NVBUFOBJTOCPUBUF_H

/******************************************************************************/
/*                     Include dependant library headers                      */
/******************************************************************************/
#include <cstdint>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "CameraAccess/ImageCommon.hpp"
#include "nvscibuf.h"

/******************************************************************************/
/*                          Definition of namespace                           */
/******************************************************************************/
namespace dimw::gpuBuf {

/******************************************************************************/
/*                  Using namespace, type or template alias                   */
/******************************************************************************/

/******************************************************************************/
/*                       Declaration of constant data                         */
/******************************************************************************/
#define _MAX_NUM_SURFACES_ (3U)

/******************************************************************************/
/*        Definition of exported types (typedef, enum, struct, union)         */
/******************************************************************************/
struct BufUtilSurfParams {
    float heightFactor[_MAX_NUM_SURFACES_];
    float widthFactor[_MAX_NUM_SURFACES_];
    uint32_t numSurfaces;
};

struct BufferAttrs {
    NvSciBufType bufType;
    uint64_t size;
    uint32_t planeCount;
    NvSciBufAttrValImageLayoutType layout;
    uint32_t planeWidths[_MAX_NUM_SURFACES_];
    uint32_t planeHeights[_MAX_NUM_SURFACES_];
    uint32_t planePitches[_MAX_NUM_SURFACES_];
    uint32_t planeBitsPerPixels[_MAX_NUM_SURFACES_];
    uint32_t planeAlignedHeights[_MAX_NUM_SURFACES_];
    uint64_t planeAlignedSizes[_MAX_NUM_SURFACES_];
    uint8_t planeChannelCounts[_MAX_NUM_SURFACES_];
    uint64_t planeOffsets[_MAX_NUM_SURFACES_];
    uint64_t topPadding[_MAX_NUM_SURFACES_];
    uint64_t bottomPadding[_MAX_NUM_SURFACES_];
    bool needSwCacheCoherency;
    NvSciBufAttrValColorFmt planeColorFormats[_MAX_NUM_SURFACES_];
};

/******************************************************************************/
/*                   Definition of classes or templates                       */
/******************************************************************************/

/******************************************************************************/
/*                     Declaration of exported variables                      */
/******************************************************************************/

/******************************************************************************/
/*                Declaration of exported function prototypes                 */
/******************************************************************************/
dimw::cameraipcclient::ClientErrorCode nvBufObjToCpuBuffer(NvSciBufObj& bufObj, uint8_t*& cpuBuf, uint32_t& dataSize);

} // namespace dimw::gpuBuf

/** \} impl */
#endif /* I_NVBUFOBJTOCPUBUF_H */

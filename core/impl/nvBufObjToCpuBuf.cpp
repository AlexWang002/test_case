/*******************************************************************************
 * \addtogroup impl
 * \{
 * \file nvBufObjToCpuBuf.cpp
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
 * | 0.2 | 2025-10-14 | Increase the buffer size to 1081088 to accept EBD-MIPI data |
 *
 ******************************************************************************/

/******************************************************************************/
/*                     Include dependant library headers                      */
/******************************************************************************/
#include <array>
#include <iostream>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "NvSIPLClient.hpp"
#include "NvSIPLCommon.hpp"
#include "nvBufObjToCpuBuf.h"
#include "nvscierror.h"
#include "rs_new_logger.h"
#include "decoder_emx.h"

/******************************************************************************/
/*                          Definition of namespace                           */
/******************************************************************************/
namespace dimw::gpuBuf {

/******************************************************************************/
/*                  Using namespace, type or template alias                   */
/******************************************************************************/
using namespace dimw::cameraipcclient;

/******************************************************************************/
/*                   Definition of exported constant data                     */
/******************************************************************************/

/******************************************************************************/
/*                     Definition of local constant data                      */
/******************************************************************************/
BufUtilSurfParams BufSurfParamsTable_Default = {
    .heightFactor = {1.0f, 0.0f, 0.0f},
    .widthFactor = {1.0f, 0.0f, 0.0f},
    .numSurfaces = 1U,
};

// Note : The buffer size is calculated as follows:
//       1920 + 1920 * 562 + 128 = 1081088
//       (EBD) + (mipi data) + (padding)
static constexpr uint32_t CPU_BUFFER_SIZE  {EMX_EMBED_MIPI_DATA_LEN + 128U};
static constexpr uint32_t MAX_BUFFER_COUNT {9U};

std::array<std::uint8_t, CPU_BUFFER_SIZE> m_imageData[MAX_BUFFER_COUNT];


/******************************************************************************/
/*              Definition of local types (enum, struct, union)               */
/******************************************************************************/

/******************************************************************************/
/*                     Declaration of exported variables                      */
/******************************************************************************/

/******************************************************************************/
/*                 Declaration or Definition of local variables               */
/******************************************************************************/

/******************************************************************************/
/*                       Definition of local functions                        */
/******************************************************************************/
/**
 * \brief Get the Bytes Per Pixel from Bits Per Pixel.
 *
 * \param buffBits Bits Per Pixel.
 * \param buffBytesVal Bytes Per Pixel.
 * \return true Success.
 * \return false Failed.
 */
inline bool GetBpp(uint32_t buffBits, uint32_t* buffBytesVal) {
    if (buffBytesVal == NULL) {
        return false;
    } else if (8U != buffBits) {
        LogError("Invalid planeBitsPerPixels {}.", buffBits);
        return false;
    }

    *buffBytesVal = 1U;
    return true;
}

/**
 * \brief Get the Buffer Parameters.
 *
 * \param buffAttrs Buffer Attributes.
 * \param xScale X Scale.
 * \param yScale Y Scale.
 * \param bytesPerPixel Bytes Per Pixel.
 * \param numSurfacesVal Number of Surfaces.
 * \param isPackedYUV Is Packed YUV.
 * \return nvsipl::SIPLStatus Status.
 */
nvsipl::SIPLStatus GetBuffParams(BufferAttrs buffAttrs, float** xScale, float** yScale, uint32_t** bytesPerPixel,
                                 uint32_t* numSurfacesVal, bool* isPackedYUV) {
    uint32_t numSurfaces = 1U;
    uint32_t* bytesPerPixelPtr = nullptr;
    float *xScalePtr = nullptr, *yScalePtr = nullptr;
    static uint32_t bpp[6] = {1U, 0U, 0U}; // Initializing default array for Bytes Per Pixels

    if ((1U == buffAttrs.planeCount) && ((buffAttrs.planeColorFormats[0] < NvSciColor_U8V8) ||
                                         (buffAttrs.planeColorFormats[0] == NvSciColor_X4Bayer12RGGB_RJ) ||
                                         ((buffAttrs.planeColorFormats[0] >= NvSciColor_X6Bayer10BGGI_RGGI) &&
                                          (buffAttrs.planeColorFormats[0] <= NvSciColor_Bayer16IGGR_IGGB)))) {
        // RAW
        if (!GetBpp(buffAttrs.planeBitsPerPixels[0], &bpp[0])) {
            return nvsipl::NVSIPL_STATUS_ERROR;
        }
        bytesPerPixelPtr = &bpp[0];
        xScalePtr = &BufSurfParamsTable_Default.widthFactor[0];
        yScalePtr = &BufSurfParamsTable_Default.heightFactor[0];
        numSurfaces = BufSurfParamsTable_Default.numSurfaces;
    } else {
        LogError("SDK GetBuffParams plane format is NOT 'RAW'.");

        return nvsipl::NVSIPL_STATUS_NOT_SUPPORTED;
    }

    if (xScale) {
        *xScale = xScalePtr;
    }
    if (yScale) {
        *yScale = yScalePtr;
    }
    if (bytesPerPixel) {
        *bytesPerPixel = bytesPerPixelPtr;
    }
    if (numSurfacesVal) {
        *numSurfacesVal = numSurfaces;
    }

    return nvsipl::NVSIPL_STATUS_OK;
}

/**
 * \brief Populate Buffer Attributes.
 *
 * \param sciBufObj NvSciBufObj.
 * \param bufAttrs Buffer Attributes.
 * \return nvsipl::SIPLStatus Status.
 */
nvsipl::SIPLStatus PopulateBufAttr(const NvSciBufObj& sciBufObj, BufferAttrs& bufAttrs) {
    NvSciError err = NvSciError_Success;
    NvSciBufAttrList bufAttrList;

    NvSciBufAttrKeyValuePair imgAttrs[] = {
        {NvSciBufImageAttrKey_Size, NULL, 0},                     //0
        {NvSciBufImageAttrKey_Layout, NULL, 0},                   //1
        {NvSciBufImageAttrKey_PlaneCount, NULL, 0},               //2
        {NvSciBufImageAttrKey_PlaneWidth, NULL, 0},               //3
        {NvSciBufImageAttrKey_PlaneHeight, NULL, 0},              //4
        {NvSciBufImageAttrKey_PlanePitch, NULL, 0},               //5
        {NvSciBufImageAttrKey_PlaneBitsPerPixel, NULL, 0},        //6
        {NvSciBufImageAttrKey_PlaneAlignedHeight, NULL, 0},       //7
        {NvSciBufImageAttrKey_PlaneAlignedSize, NULL, 0},         //8
        {NvSciBufImageAttrKey_PlaneChannelCount, NULL, 0},        //9
        {NvSciBufImageAttrKey_PlaneOffset, NULL, 0},              //10
        {NvSciBufImageAttrKey_PlaneColorFormat, NULL, 0},         //11
        {NvSciBufImageAttrKey_TopPadding, NULL, 0},               //12
        {NvSciBufImageAttrKey_BottomPadding, NULL, 0},            //13
        {NvSciBufGeneralAttrKey_CpuNeedSwCacheCoherency, NULL, 0} //14
    };

    err = NvSciBufObjGetAttrList(sciBufObj, &bufAttrList);
    if ((err) != NvSciError_Success) {
        return nvsipl::NVSIPL_STATUS_ERROR;
    }
    err = NvSciBufAttrListGetAttrs(bufAttrList, imgAttrs, sizeof(imgAttrs) / sizeof(imgAttrs[0]));
    if ((err) != NvSciError_Success) {
        return nvsipl::NVSIPL_STATUS_ERROR;
    }

    if (imgAttrs[0].len != 0) {
        bufAttrs.size = *(static_cast<const uint64_t*>(imgAttrs[0].value));
    } else {
        LogError("Skipped populating size as length of attribute was 0.");
    }

    if (imgAttrs[1].len != 0) {
        bufAttrs.layout = *(static_cast<const NvSciBufAttrValImageLayoutType*>(imgAttrs[1].value));
    } else {
        LogError("Skipped populating layout as length of attribute was 0.");
    }

    if (imgAttrs[2].len != 0) {
        bufAttrs.planeCount = *(static_cast<const uint32_t*>(imgAttrs[2].value));
    } else {
        LogError("Skipped populating planeCount as length of attribute was 0.");
    }

    if (imgAttrs[14].len != 0) {
        bufAttrs.needSwCacheCoherency = *(static_cast<const bool*>(imgAttrs[14].value));
    } else {
        LogError("Skipped populating needSwCacheCoherency as length of attribute was 0.");
    }

    if (imgAttrs[3].len != 0) {
        memcpy(bufAttrs.planeWidths, static_cast<const uint32_t*>(imgAttrs[3].value),
               bufAttrs.planeCount * sizeof(bufAttrs.planeWidths[0]));
    } else {
        LogError("Skipped populating planeWidths as length of attribute was 0.");
    }

    if (imgAttrs[4].len != 0) {
        memcpy(bufAttrs.planeHeights, static_cast<const uint32_t*>(imgAttrs[4].value),
               bufAttrs.planeCount * sizeof(bufAttrs.planeHeights[0]));
    } else {
        LogError("Skipped populating planeHeights as length of attribute was 0.");
    }

    if (imgAttrs[5].len != 0) {
        memcpy(bufAttrs.planePitches, static_cast<const uint32_t*>(imgAttrs[5].value),
               bufAttrs.planeCount * sizeof(bufAttrs.planePitches[0]));
    } else {
        LogError("Skipped populating planePitches as length of attribute was 0.");
    }

    if (imgAttrs[6].len != 0) {
        memcpy(bufAttrs.planeBitsPerPixels, static_cast<const uint32_t*>(imgAttrs[6].value),
               bufAttrs.planeCount * sizeof(bufAttrs.planeBitsPerPixels[0]));
    } else {
        LogError("Skipped populating planeBitsPerPixels as length of attribute was 0.");
    }

    if (imgAttrs[7].len != 0) {
        memcpy(bufAttrs.planeAlignedHeights, static_cast<const uint32_t*>(imgAttrs[7].value),
               bufAttrs.planeCount * sizeof(bufAttrs.planeAlignedHeights[0]));
    } else {
        LogError("Skipped populating planeAlignedHeights as length of attribute was 0.");
    }

    if (imgAttrs[8].len != 0) {
        memcpy(bufAttrs.planeAlignedSizes, static_cast<const uint64_t*>(imgAttrs[8].value),
               bufAttrs.planeCount * sizeof(bufAttrs.planeAlignedSizes[0]));
    } else {
        LogError("Skipped populating planeAlignedSizes as length of attribute was 0.");
    }

    if (imgAttrs[9].len != 0) {
        memcpy(bufAttrs.planeChannelCounts, static_cast<const uint8_t*>(imgAttrs[9].value),
               bufAttrs.planeCount * sizeof(bufAttrs.planeChannelCounts[0]));
    } else {
        LogError("Skipped populating planeChannelCounts as length of attribute was 0.");
    }

    if (imgAttrs[10].len != 0) {
        memcpy(bufAttrs.planeOffsets, static_cast<const uint64_t*>(imgAttrs[10].value),
               bufAttrs.planeCount * sizeof(bufAttrs.planeOffsets[0]));
    } else {
        LogError("Skipped populating planeOffsets as length of attribute was 0.");
    }

    if (imgAttrs[11].len != 0) {
        memcpy(bufAttrs.planeColorFormats, static_cast<const NvSciBufAttrValColorFmt*>(imgAttrs[11].value),
               bufAttrs.planeCount * sizeof(NvSciBufAttrValColorFmt));
    } else {
        LogError("Skipped populating planeColorFormats as length of attribute was 0.");
    }

    if (imgAttrs[12].len != 0) {
        memcpy(bufAttrs.topPadding, static_cast<const uint64_t*>(imgAttrs[12].value),
               bufAttrs.planeCount * sizeof(bufAttrs.topPadding[0]));
    } else {
        LogError("Skipped populating topPadding as length of attribute was 0.");
    }

    if (imgAttrs[13].len != 0) {
        memcpy(bufAttrs.bottomPadding, static_cast<const uint64_t*>(imgAttrs[13].value),
               bufAttrs.planeCount * sizeof(bufAttrs.bottomPadding[0]));
    } else {
        LogError("Skipped populating bottomPadding as length of attribute was 0.");
    }

    return nvsipl::NVSIPL_STATUS_OK;
}

/******************************************************************************/
/*                      Definition of exported functions                      */
/******************************************************************************/
/**
 * \brief Convert NvSciBufObj to CPU Buffer.
 *
 * \param bufObj Nvidia SCI Buffer Object.
 * \param cpuBuf The reference to pointer of CPU Buffer.
 * \param dataSize Data Size.
 * \return ClientErrorCode Error Code.
 */
ClientErrorCode nvBufObjToCpuBuffer(NvSciBufObj& bufObj, uint8_t*& cpuBuf, uint32_t& dataSize) {
    // Write Buffer
    NvSciError sciErr;
    nvsipl::SIPLStatus status;
    BufferAttrs bufAttrs;
    static uint8_t buffer_index{0U};

    status = PopulateBufAttr(bufObj, bufAttrs);

    if (status != nvsipl::NVSIPL_STATUS_OK) {
        LogError("nvBufObjToCpuBuffer: PopulateBufAttr failed.");
        return ClientErrorCode::CLIENT_GETFRAME_FAILD;
    }
    uint32_t numSurfaces = -1U;
    float *xScalePtr = nullptr, *yScalePtr = nullptr;
    uint32_t* bytesPerPixelPtr = nullptr;
    bool isPackedYUV = false;

    status = GetBuffParams(bufAttrs, &xScalePtr, &yScalePtr, &bytesPerPixelPtr, &numSurfaces, &isPackedYUV);

    if (status != nvsipl::NVSIPL_STATUS_OK) {
        LogError("nvBufObjToCpuBuffer: GetBuffParams failed.");
        return ClientErrorCode::CLIENT_GETFRAME_FAILD;
    }

    if (isPackedYUV) {
        /* no YUV */
    } else {
        uint32_t pBuffPitches[_MAX_NUM_SURFACES_] = {0U};
        uint8_t* pBuff[_MAX_NUM_SURFACES_] = {0U};
        uint32_t size[_MAX_NUM_SURFACES_] = {0U};
        uint32_t imageSize = 0U;
        uint32_t height = bufAttrs.planeHeights[0];
        uint32_t width = bufAttrs.planeWidths[0];

        size[0] = width * xScalePtr[0] * height * yScalePtr[0] * bytesPerPixelPtr[0];
        imageSize += size[0];
        pBuffPitches[0] = (uint32_t)((float)width * xScalePtr[0]) * bytesPerPixelPtr[0];

        if ((EMX_MIPI_DATA_LEN != imageSize) && (EMX_EMBED_MIPI_DATA_LEN != imageSize)) {
            LogError("[nvBufObjToCpuBuffer]: error iamge size, imageSize = {}, expected size {} or {}",
                        imageSize, EMX_MIPI_DATA_LEN, EMX_EMBED_MIPI_DATA_LEN);
            return ClientErrorCode::CLIENT_GETFRAME_FAILD;
        }
        pBuff[0] = m_imageData[buffer_index].data();

        if ((bufAttrs.needSwCacheCoherency)) {
            sciErr = NvSciBufObjFlushCpuCacheRange(bufObj, 0U, bufAttrs.planePitches[0] * height);

            if ((sciErr) != NvSciError_Success) {
                LogError("[nvBufObjToCpuBuffer]: NvSciBufObjFlushCpuCacheRange Failed.");
                return ClientErrorCode::CLIENT_GETFRAME_FAILD;
            }
        }
        sciErr = NvSciBufObjGetPixels(bufObj, nullptr, (void**)pBuff, size, pBuffPitches);

        if ((sciErr) != NvSciError_Success) {
            LogError("[nvBufObjToCpuBuffer]: NvSciBufObjGetPixels Failed.");
            return ClientErrorCode::CLIENT_GETFRAME_FAILD;
        }
        cpuBuf = m_imageData[buffer_index].data();
        dataSize = imageSize;
        buffer_index = (buffer_index + 1) % MAX_BUFFER_COUNT;

        if (EMX_EMBED_MIPI_DATA_LEN == imageSize) {
            // If have the 1920 bytes EBD data before MIPI data, Only get the MIPI data;
            cpuBuf += EMX_GMSL_WIDTH;
            dataSize -= EMX_GMSL_WIDTH;
        }
    }

    return ClientErrorCode::CLIENT_SUCCESS;
}

} // namespace dimw::gpuBuf

/* \}  impl */
#include "udImage.h"
#include "udFile.h"
#include "udMath.h"
#include "udCompression.h"
#include "udStringUtil.h"
#include "udThread.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

// ****************************************************************************
// Author: Dave Pevreal, February 2019
udResult udImage_Load(udImage **ppImage, const char *pFilename)
{
  udResult result;
  void *pMem = nullptr;
  int64_t fileLen;

  UD_ERROR_CHECK(udFile_Load(pFilename, &pMem, &fileLen));
  UD_ERROR_CHECK(udImage_LoadFromMemory(ppImage, pMem, (size_t)fileLen));

epilogue:
  udFree(pMem);
  return result;
}

// ****************************************************************************
// Author: Dave Pevreal, February 2019
udResult udImage_LoadFromMemory(udImage **ppImage, const void *pMemory, size_t length)
{
  udResult result;
  udImage *pImage = nullptr;
  const stbi_uc *pSTBIImage = nullptr;
  int w, h, sc; // Some plain integers for call to 3rd party API

  UD_ERROR_NULL(ppImage, udR_InvalidParameter_);
  UD_ERROR_NULL(pMemory, udR_InvalidParameter_);
  UD_ERROR_IF(length == 0, udR_InvalidParameter_);

  pSTBIImage = stbi_load_from_memory((const stbi_uc *)pMemory, (int)length, &w, &h, &sc, 4);
  UD_ERROR_NULL(pSTBIImage, udR_ImageLoadFailure);

  pImage = (udImage *)udAllocFlags(sizeof(udImage), udAF_Zero);
  UD_ERROR_NULL(pImage, udR_MemoryAllocationFailure);
  pImage->width = (uint32_t)w;
  pImage->height = (uint32_t)h;
  pImage->sourceChannels = (uint16_t)sc;

  // Duplicate stbi image data
  pImage->pImageData = (uint32_t *)udMemDup(pSTBIImage, w * h * 4, 0, udAF_None);
  UD_ERROR_NULL(pImage->pImageData, udR_MemoryAllocationFailure);
  stbi_image_free((void *)pSTBIImage);
  pSTBIImage = nullptr;

  *ppImage = pImage;
  pImage = nullptr;
  result = udR_Success;

epilogue:
  if (pSTBIImage)
  {
    stbi_image_free((void *)pSTBIImage);
    pSTBIImage = nullptr;
  }
  if (pImage)
    udImage_Destroy(&pImage);

  return result;
}

struct stbiWriteContext
{
  udFile *pFile;
  udResult *pResult;
  uint32_t *pBytesWritten;
};

// ----------------------------------------------------------------------------
// Author: Dave Pevreal, February 2019
static void stbiWriteCallback(void *_pContext, void *pData, int size)
{
  stbiWriteContext *pContext = (stbiWriteContext *)_pContext;
  // Only attempt to write if there isn't a previous error
  if (*pContext->pResult == udR_Success)
    *pContext->pResult = udFile_Write(pContext->pFile, pData, size);
  if (pContext->pBytesWritten && *pContext->pResult == udR_Success)
    *pContext->pBytesWritten += size;
}

// ----------------------------------------------------------------------------
// Author: Dave Pevreal, February 2019
static void stbiWriteWrapper(stbiWriteContext *pContext, uint32_t width, uint32_t height, const void *pRGB, udImageSaveType saveType)
{
  if (pContext->pResult)
    *pContext->pResult = udR_Success;

  if (pContext->pBytesWritten)
    *pContext->pBytesWritten = 0;

  switch (saveType)
  {
    case udIST_PNG:
      if (stbi_write_png_to_func(stbiWriteCallback, pContext, width, height, 3, pRGB, 0) == 0)
        *pContext->pResult = udR_Failure_;
      break;

    case udIST_JPG:
      if (stbi_write_jpg_to_func(stbiWriteCallback, pContext, width, height, 3, pRGB, 80) == 0) // quality value of 80
        *pContext->pResult = udR_Failure_;
      break;

    case udIST_Max:
      *pContext->pResult = udR_InvalidParameter_;
  }
}

// ****************************************************************************
// Author: Dave Pevreal, February 2019
udResult udImage_Save(const udImage *pImage, const char *pFilename, uint32_t *pSaveSize, udImageSaveType saveType)
{
  udResult result;
  void *pRGB = nullptr; // Sadly, stbi has no support for writing 24-bit from a 32-bit source, so we must shrink
  uint8_t *pSource, *pDest; // For shrinking
  uint32_t count;
  stbiWriteContext writeContext;

  UD_ERROR_NULL(pImage, udR_InvalidParameter_);
  UD_ERROR_NULL(pFilename, udR_InvalidParameter_);
  UD_ERROR_IF(saveType >= udIST_Max, udR_InvalidParameter_);
  count = pImage->width * pImage->height;
  pRGB = udAlloc(count * 3);
  UD_ERROR_NULL(pRGB, udR_MemoryAllocationFailure);
  pSource = (uint8_t *)pImage->pImageData;
  pDest = (uint8_t*)pRGB;
  while (count--)
  {
    *pDest++ = pSource[0];
    *pDest++ = pSource[1];
    *pDest++ = pSource[2];
    pSource += 4;
  }

  writeContext.pResult = &result;
  writeContext.pBytesWritten = pSaveSize;
  UD_ERROR_CHECK(udFile_Open(&writeContext.pFile, pFilename, udFOF_Create | udFOF_Write));
  stbiWriteWrapper(&writeContext, pImage->width, pImage->height, pRGB, saveType);
  udFile_Close(&writeContext.pFile);

epilogue:
  udFree(pRGB);
  return result;
}

// ****************************************************************************
// Author: Dave Pevreal, February 2019
uint32_t udImage_Sample(udImage *pImage, float u, float v, udImageSampleFlags flags)
{
  if (pImage->pImageData == nullptr)
    return 0;

  if (flags & udISF_Clamp)
  {
    u = udClamp(u, 0.f, 1.f);
    v = udClamp(v, 0.f, 1.f);
  }

  u =  u * pImage->width;
  if (flags & udISF_TopLeft)
    v = v * pImage->height;
  else
    v = -v * pImage->height;

  while (u < 0.0f || u >= pImage->width)
    u = u - pImage->width * udFloor((u / pImage->width));
  while (v < 0.0f || v >= pImage->height)
    v = v - pImage->height * udFloor((v / pImage->height));

  int x = (int)u;
  int y = (int)v;
  if (flags & udISF_Filter)
  {
    int u1 = (int)(u * 256) & 0xff; // Get most most significant bits of PRECISION
    int v1 = (int)(v * 256) & 0xff; // Get most most significant bits of PRECISION
    int u0 = 256 - u1;
    int v0 = 256 - v1;

    int x0 = udClamp(x + 0, 0, (int)pImage->width - 1);
    int x1 = udClamp(x + 1, 0, (int)pImage->width - 1);
    int y0 = udClamp(y + 0, 0, (int)pImage->height - 1);
    int y1 = udClamp(y + 1, 0, (int)pImage->height - 1);

    int a = u0 * v0;
    int b = u1 * v0;
    int c = u0 * v1;
    int d = u1 * v1;

    uint32_t c0 = (pImage->pImageData[x0 + y0 * pImage->width]);
    uint32_t c1 = (pImage->pImageData[x1 + y0 * pImage->width]);
    uint32_t c2 = (pImage->pImageData[x0 + y1 * pImage->width]);
    uint32_t c3 = (pImage->pImageData[x1 + y1 * pImage->width]);

    uint32_t bfB = 0x00ff0000 & (((c0 >> 16)      * a) + ((c1 >> 16)      * b) + ((c2 >> 16)      * c) + ((c3 >> 16)      * d));
    uint32_t bfG = 0xff000000 & (((c0 & 0x00ff00) * a) + ((c1 & 0x00ff00) * b) + ((c2 & 0x00ff00) * c) + ((c3 & 0x00ff00) * d));
    uint32_t bfR = 0x00ff0000 & (((c0 & 0x0000ff) * a) + ((c1 & 0x0000ff) * b) + ((c2 & 0x0000ff) * c) + ((c3 & 0x0000ff) * d));

    if (flags & udISF_ABGR)
      return 0xff000000 | bfB | ((bfG | bfR) >> 16);
    else
      return 0xff000000 | bfR | ((bfG | bfB) >> 16);
  }
  else
  {
    uint32_t c = (pImage->pImageData[x + y * pImage->width]); // STBI returns colors as ABGR

    if (flags & udISF_ABGR)
      return c;
    else
      return (c & 0xff00ff00) | ((c & 0xff) << 16) | ((c >> 16) & 0xff);
  }
}

// ****************************************************************************
// Author: Dave Pevreal, February 2019
void udImage_Destroy(udImage **ppImage)
{
  if (ppImage && *ppImage)
  {
    udImage *pImage = *ppImage;
    *ppImage = nullptr;
    udFree(pImage->pImageData);
    udFree(pImage);
  }
}

// ****************************************************************************
// Author: Dave Pevreal, March 2020
udResult udImageStreaming_Save(const udImage *pImage, udImageStreamingOnDisk **ppOnDisk, uint32_t *pSaveSize, const char *pNameDesc)
{
  udResult result;
  udImageStreamingOnDisk *pOnDisk = nullptr;
  uint32_t saveSize = (uint32_t)sizeof(udImageStreamingOnDisk);
  uint16_t mipCount = 0;
  uint32_t mipW, mipH;
  uint8_t *p24BitData = nullptr;
  uint8_t *pOut, *pIn;

  UD_ERROR_NULL(pImage, udR_InvalidParameter_);
  UD_ERROR_NULL(pSaveSize, udR_InvalidParameter_);

  mipW = pImage->width;
  mipH = pImage->height;
  do
  {
    saveSize += mipW * mipH * 3;
    mipW = udMax(1, mipW >> 1);
    mipH = udMax(1, mipH >> 1);
    ++mipCount;
  } while (mipW > 1 && mipH > 1);

  if (ppOnDisk)
  {
    pOnDisk = (udImageStreamingOnDisk *)udAlloc(saveSize);
    UD_ERROR_NULL(pOnDisk, udR_MemoryAllocationFailure);
    memset(pOnDisk, 0, sizeof(udImageStreamingOnDisk));
    pOnDisk->fourcc = udImageStreaming::Fourcc;
    pOnDisk->width = pImage->width;
    pOnDisk->height = pImage->height;
    pOnDisk->mipCount = mipCount;
    udStrcpy(pOnDisk->name, pNameDesc);
    pOnDisk->offsetToMip0 = (uint16_t)sizeof(udImageStreamingOnDisk);

    // Make a 24-bit copy of the source image
    p24BitData = udAllocType(uint8_t, pImage->width * pImage->height * 3, udAF_None);
    UD_ERROR_NULL(p24BitData, udR_MemoryAllocationFailure);
    pOut = p24BitData;
    pIn = (uint8_t*)(pImage->pImageData);
    for (uint32_t y = 0; y < pImage->height; ++y)
    {
      for (uint32_t x = 0; x < pImage->width; ++x)
      {
        *pOut++ = *pIn++;
        *pOut++ = *pIn++;
        *pOut++ = *pIn++;
        ++pIn; // Skip alpha
      }
    }

    pOut = ((uint8_t *)pOnDisk) + pOnDisk->offsetToMip0;
    mipW = pImage->width;
    mipH = pImage->height;
    for (uint16_t mip = 0; mip < mipCount; ++mip)
    {
      uint32_t cellCountX = (mipW + udImageStreamingOnDisk::TileSize - 1) / udImageStreamingOnDisk::TileSize;
      uint32_t cellCountY = (mipH + udImageStreamingOnDisk::TileSize - 1) / udImageStreamingOnDisk::TileSize;
      for (uint32_t cellY = 0; cellY < cellCountY; ++cellY)
      {
        for (uint32_t cellX = 0; cellX < cellCountX; ++cellX)
        {
          //udDebugPrintf("Writing cell %d,%d at offset %llx\n", cellX, cellY, pOut - ((uint8_t *)pOnDisk));
          uint32_t cellW = udMin(udImageStreamingOnDisk::TileSize, mipW - (cellX * udImageStreamingOnDisk::TileSize));
          uint32_t cellH = udMin(udImageStreamingOnDisk::TileSize, mipH - (cellY * udImageStreamingOnDisk::TileSize));
          for (uint32_t y = 0; y < cellH; ++y)
          {
            pIn = p24BitData + ((cellY * udImageStreamingOnDisk::TileSize + y) * (mipW * 3)) + (cellX * udImageStreamingOnDisk::TileSize * 3);
            for (uint32_t x = 0; x < cellW; ++x)
            {
              *pOut++ = *pIn++;
              *pOut++ = *pIn++;
              *pOut++ = *pIn++;
            }
          }
        }
      }

      // Now generate the next mip down
      uint8_t *pO = p24BitData; // Overwrite the data as we go by processing left to right, top to bottom order
      for (uint32_t y = 0; y < mipH; y += 2)
      {
        uint8_t *pA = p24BitData + (y * mipW * 3);
        uint8_t *pB = pA + (mipW * 3); // Line immediately under pA's line
        for (uint32_t x = 0; x < mipW; x += 2)
        {
          // Loop for each of the components r,g,b
          for (uint32_t c = 0; c < 3; ++c)
            *pO++ = (unsigned(pA[c + 0]) + unsigned(pA[c + 3]) + unsigned(pB[c + 0]) + unsigned(pB[c + 3])) >> 2;
          // Source A and B pointers advance by 2 pixels (6 bytes)
          pA += 3 * 2;
          pB += 3 * 2;
        }
      }
      mipW = udMax(1, mipW >> 1);
      mipH = udMax(1, mipH >> 1);
    }

    *ppOnDisk = pOnDisk;
  }

  *pSaveSize = saveSize;
  *ppOnDisk = pOnDisk;
  pOnDisk = nullptr;
  result = udR_Success;

epilogue:
  udFree(p24BitData);
  udFree(pOnDisk);
  return result;
}

// ****************************************************************************
// Author: Dave Pevreal, March 2020
udResult udImageStreaming_Load(udImageStreaming **ppImage, udFile *pFile, int64_t offset)
{
  udResult result;
  udImageStreaming *pImage = nullptr;

  UD_ERROR_NULL(ppImage, udR_InvalidParameter_);
  UD_ERROR_CHECK(udImageStreaming_Reserve(&pImage, pFile, offset));
  UD_ERROR_CHECK(udImageStreaming_LoadCell(pImage, (uint32_t)-1));
  *ppImage = pImage;
  pImage = nullptr;
  result = udR_Success;

epilogue:
  if (pImage)
    udImageStreaming_Destroy(&pImage);
  return result;
}

// ****************************************************************************
// Author: Dave Pevreal, March 2020
udResult udImageStreaming_Reserve(udImageStreaming **ppImage, udFile *pFile, int64_t offset)
{
  udResult result;
  udImageStreaming *pImage = nullptr;

  UD_ERROR_NULL(ppImage, udR_InvalidParameter_);
  UD_ERROR_NULL(pFile, udR_InvalidParameter_);

  pImage = udAllocType(udImageStreaming, 1, udAF_Zero);
  UD_ERROR_NULL(pImage, udR_MemoryAllocationFailure);
  pImage->pFile = pFile;
  pImage->baseOffset = offset;
  pImage->pLock = udCreateMutex();

  *ppImage = pImage;
  pImage = nullptr;
  result = udR_Success;

epilogue:
  if (pImage)
    udImageStreaming_Destroy(&pImage);
  return result;
}

// ****************************************************************************
// Author: Dave Pevreal, March 2020
uint32_t udImageStreaming_Sample(udImageStreaming *pImage, float u, float v, udImageSampleFlags flags, uint16_t mipLevel)
{
  udResult result;
  uint32_t texel = 0; // Default value returned if no data available
  udImageStreaming::Mip *pMip = nullptr;
  uint8_t *p;

  // First sample may trigger the initial load, but can't do error checking here so an error returns zero color
  if (pImage->fourcc == 0 && udImageStreaming_LoadCell(pImage, (uint32_t)-1) != udR_Success)
  {
    udDebugPrintf("Error loading texture header\n");
    return texel;
  }

  if (flags & udISF_Clamp)
  {
    u = udClamp(u, 0.f, 1.f);
    v = udClamp(v, 0.f, 1.f);
  }

  pMip = &pImage->mips[udClamp((int)mipLevel, 0, pImage->mipCount - 1)];
  u = u * pMip->width;
  if (flags & udISF_TopLeft)
    v = v * pMip->height;
  else
    v = -v * pMip->height;

  while (u < 0.0f || u >= pMip->width)
    u = u - pMip->width * udFloor((u / pMip->width));
  while (v < 0.0f || v >= pMip->height)
    v = v - pMip->height * udFloor((v / pMip->height));

  uint32_t x = (uint32_t)u;
  uint32_t y = (uint32_t)v;
  uint32_t cellX = x / udImageStreaming::TileSize;
  uint32_t cellY = y / udImageStreaming::TileSize;
  uint32_t cellIndex = cellY * pMip->gridW + cellX;
  uint32_t cellWidth = udMin(udImageStreaming::TileSize, pMip->width - (cellX * udImageStreaming::TileSize));
  if (!pMip->ppCellImage || !pMip->ppCellImage[cellIndex])
  {
    texel = mipLevel | (cellX << 8) | (cellY << 16);
    if (flags & udISF_NoStream)
      UD_ERROR_SET(udR_Success);
    else
      UD_ERROR_CHECK(udImageStreaming_LoadCell(pImage, texel));
  }
  x &= (udImageStreaming::TileSize - 1);
  y &= (udImageStreaming::TileSize - 1);
  p = pMip->ppCellImage[cellIndex] + (y * cellWidth + x) * 3;
  if (flags & udISF_ABGR)
    texel = p[0] | (p[1] << 8) | (p[2] << 16) | 0xff000000;
  else
    texel = p[2] | (p[1] << 8) | (p[0] << 16) | 0xff000000;

epilogue:
  return texel;
}

// ****************************************************************************
// Author: Dave Pevreal, March 2020
udResult udImageStreaming_LoadCell(udImageStreaming *pImage, uint32_t cellIndexData)
{
  udResult result;
  udMutex *pLocked = nullptr;
  uint8_t *pCellMem = nullptr;

  UD_ERROR_NULL(pImage, udR_InvalidParameter_);

  if (pImage->fourcc == 0)
  {
    pLocked = udLockMutex(pImage->pLock);
    if (pImage->fourcc == 0)
    {
      UD_ERROR_CHECK(udFile_Read(pImage->pFile, pImage, sizeof(udImageStreamingOnDisk), pImage->baseOffset, udFSW_SeekSet));
      UD_ERROR_IF(pImage->fourcc != udImageStreamingOnDisk::Fourcc, udR_ObjectTypeMismatch);
      UD_ERROR_IF(pImage->mipCount > udImageStreamingOnDisk::MaxMipLevels, udR_CorruptData);

      for (uint16_t mip = 0; mip < pImage->mipCount; ++mip)
      {
        if (!mip)
        {
          pImage->mips[mip].offset = pImage->baseOffset + pImage->offsetToMip0;
          pImage->mips[mip].width = pImage->width;
          pImage->mips[mip].height = pImage->height;
        }
        else
        {
          pImage->mips[mip].offset = pImage->mips[mip - 1].offset + (pImage->mips[mip - 1].width * pImage->mips[mip - 1].height * 3);
          pImage->mips[mip].width = udMax(1, pImage->mips[mip - 1].width >> 1);
          pImage->mips[mip].height = udMax(1, pImage->mips[mip - 1].height >> 1);
        }

        pImage->mips[mip].gridW = (uint16_t)(pImage->mips[mip].width + udImageStreamingOnDisk::TileSize - 1) / udImageStreamingOnDisk::TileSize;
        pImage->mips[mip].gridH = (uint16_t)(pImage->mips[mip].height + udImageStreamingOnDisk::TileSize - 1) / udImageStreamingOnDisk::TileSize;
      }
    }
  }

  if (cellIndexData != (uint32_t)-1)
  {
    uint32_t mipLevel = (cellIndexData >> 0) & 0xff;
    uint32_t cellX = (cellIndexData >> 8) & 0xff;
    uint32_t cellY = (cellIndexData >> 16) & 0xff;

    UD_ERROR_IF(mipLevel >= pImage->mipCount, udR_InvalidParameter_);
    udImageStreaming::Mip *pMip = &pImage->mips[udClamp((int)mipLevel, 0, pImage->mipCount - 1)];
    UD_ERROR_IF(cellX >= pMip->gridW, udR_InvalidParameter_);
    UD_ERROR_IF(cellY >= pMip->gridH, udR_InvalidParameter_);

    uint32_t cellIndex = cellY * pMip->gridW + cellX;
    uint32_t cellWidth = udMin(udImageStreaming::TileSize, pMip->width - (cellX * udImageStreaming::TileSize));
    if (!pMip->ppCellImage || !pMip->ppCellImage[cellIndex])
    {
      if (!pLocked)
        pLocked = udLockMutex(pImage->pLock);
      if (!pMip->ppCellImage || !pMip->ppCellImage[cellIndex]) // Second check after the lock
      {
        if (!pMip->ppCellImage)
        {
          pMip->ppCellImage = udAllocType(uint8_t *, pMip->gridW * pMip->gridH, udAF_Zero);
          UD_ERROR_NULL(pMip->ppCellImage, udR_MemoryAllocationFailure);
        }
        uint32_t cellHeight = udMin(udImageStreaming::TileSize, pMip->height - (cellY * udImageStreaming::TileSize));
        uint32_t cellSizeBytes = cellWidth * cellHeight * 3;
        uint32_t cellOffset = (cellY * udImageStreaming::TileSize * pMip->width * 3) + (cellX * udImageStreaming::TileSize * udImageStreaming::TileSize * 3);
        // Read into locally allocated block
        pCellMem = udAllocType(uint8_t, cellSizeBytes, udAF_None);
        UD_ERROR_NULL(pCellMem, udR_MemoryAllocationFailure);
        UD_ERROR_CHECK(udFile_Read(pImage->pFile, pCellMem, cellSizeBytes, pMip->offset + cellOffset, udFSW_SeekSet));
        // Assign the pointer after reading to ensure another thread doesn't access the memory before the read is complete
        udInterlockedExchangePointer(&pMip->ppCellImage[cellIndex], pCellMem);
        pCellMem = nullptr;
      }
      udReleaseMutex(pLocked);
      pLocked = nullptr;
    }
  }

  result = udR_Success;

epilogue:
  udFree(pCellMem);
  if (pLocked)
    udReleaseMutex(pLocked);
  return result;
}

// ****************************************************************************
// Author: Dave Pevreal, March 2020
udResult udImageStreaming_SaveAs(udImageStreaming *pImage, const char *pFilename, uint32_t *pSaveSize, udImageSaveType saveType)
{
  udResult result;
  uint8_t *pRGB = nullptr; // Need to make a 24-bit copy for stbi
  stbiWriteContext writeContext;

  UD_ERROR_NULL(pImage, udR_InvalidParameter_);
  UD_ERROR_NULL(pFilename, udR_InvalidParameter_);
  UD_ERROR_IF(saveType >= udIST_Max, udR_InvalidParameter_);

  // Ensure the header has been loaded by doing an initial dummy sample
  udImageStreaming_Sample(pImage, 0.f, 0.f);

  // Make a 24-bit copy to avoid PNG saving 32-bit when all alpha values are 0xff (currently anyway)
  // This isn't optimal, but for simplicity we just use the Sample function to handle streaming
  pRGB = (uint8_t *)udAlloc(pImage->width * pImage->height * 3);
  UD_ERROR_NULL(pRGB, udR_MemoryAllocationFailure);
  for (uint32_t y = 0; y < pImage->height; ++y)
  {
    for (uint32_t x = 0; x < pImage->width; ++x)
    {
      uint32_t c = udImageStreaming_Sample(pImage, x / (float)pImage->width, y / (float)pImage->height, udISF_ABGR | udISF_TopLeft);
      memcpy(&pRGB[(y * pImage->width + x) * 3], &c, 3);
    }
  }

  writeContext.pResult = &result;
  writeContext.pBytesWritten = pSaveSize;
  UD_ERROR_CHECK(udFile_Open(&writeContext.pFile, pFilename, udFOF_Create | udFOF_Write));
  stbiWriteWrapper(&writeContext, pImage->width, pImage->height, pRGB, saveType);
  udFile_Close(&writeContext.pFile);

epilogue:
  udFree(pRGB);
  return result;
}

// ****************************************************************************
// Author: Dave Pevreal, March 2020
void udImageStreaming_Destroy(udImageStreaming **ppImage)
{
  if (ppImage && *ppImage)
  {
    udImageStreaming *pImage = *ppImage;
    *ppImage = nullptr;
    for (uint16_t i = 0; i < pImage->mipCount; ++i)
    {
      if (pImage->mips[i].ppCellImage)
      {
        for (uint16_t j = 0; j < (pImage->mips[i].gridW * pImage->mips[i].gridH); ++j)
          udFree(const_cast<uint8_t*&>(pImage->mips[i].ppCellImage[j]));
        udFree(pImage->mips[i].ppCellImage);
      }
    }
    udDestroyMutex(&pImage->pLock);
    pImage->pFile = nullptr;
    udFree(pImage);
  }
}

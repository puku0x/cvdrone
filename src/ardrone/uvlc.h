#ifndef __HEADER_UVLC__
#define __HEADER_UVLC__

//#region Copyright Notice

//Copyright © 2007-2011, PARROT SA, all rights reserved. 

//DISCLAIMER 
//The APIs is provided by PARROT and contributors "AS IS" and any express or implied warranties, including, but not limited to, the implied warranties of merchantability 
//and fitness for a particular purpose are disclaimed. In no event shall PARROT and contributors be liable for any direct, indirect, incidental, special, exemplary, or 
//consequential damages (including, but not limited to, procurement of substitute goods or services; loss of use, data, or profits; or business interruption) however 
//caused and on any theory of liability, whether in contract, strict liability, or tort (including negligence or otherwise) arising in any way out of the use of this 
//software, even if advised of the possibility of such damage. 

//Author            : Daniel Schmidt
//Publishing date   : 2010-01-06 
//based on work by  : Wilke Jansoone

//Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions
//are met:
//- Redistributions of source code must retain the above copyright notice, this list of conditions, the disclaimer and the original author of the source code.
//- Neither the name of the PixVillage Team, nor the names of its contributors may be used to endorse or promote products derived from this software without 
//specific prior written permission.

////#endregion

////#region Imports

#include <inttypes.h>

namespace UVLC {
    const int BLOCK_WIDTH = 8;
    const int CIF_WIDTH   = 88;
    const int CIG_HEIGHT  = 72;
    const int VGA_WIDTH   = 160;
    const int VGA_HEIGHT  = 120;
    const int CIF         = 1;
    const int QVGA        = 2;
    const int TABLE_QUANTIZATION_MODE = 31;
    const int16_t ZIGZAG_POSITIONS[] = { 0, 1, 8, 16, 9, 2, 3, 10, 17, 24, 32, 25, 18, 11, 4, 5, 12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6, 7, 14, 21, 28, 35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51, 58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63, };
    const int16_t QUANTIZER_VALUES[] = { 3, 5, 7, 9, 11, 13, 15, 17, 5, 7, 9, 11, 13, 15, 17, 19, 7, 9, 11, 13, 15, 17, 19, 21, 9, 11, 13, 15, 17, 19, 21, 23, 11, 13, 15, 17, 19, 21, 23, 25, 13, 15, 17, 19, 21, 23, 25, 27, 15, 17, 19, 21, 23, 25, 27, 29, 17, 19, 21, 23, 25, 27, 29, 31 };
    const uint8_t CLZLUT[] = { 8, 7, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    class MacroBlock {
    public:
        int16_t *DataBlocks[6];
        MacroBlock(void);
        ~MacroBlock(void);
    };

    class ImageSlice {
    public:
        int Count;
        MacroBlock *MacroBlocks;
        ImageSlice(int);
        ~ImageSlice(void);
    };

    MacroBlock::MacroBlock(void) {
        for (int i = 0; i < 6; i++) this->DataBlocks[i] = new int16_t[64];
    }

    MacroBlock::~MacroBlock(void) {
        for (int i = 0; i < 6; i++) delete [] this->DataBlocks[i];
    }

    ImageSlice::ImageSlice(int macroBlockCount) {
        this->Count = macroBlockCount;
        this->MacroBlocks = new MacroBlock[macroBlockCount];
    }

    ImageSlice::~ImageSlice(void) {
        delete [] this->MacroBlocks;
    }

    uint32_t PeekStreamData(uint8_t *stream, int stream_size, int streamIndex, int streamField, int streamFieldBitIndex, int count)
    {
        uint32_t data = 0;
        uint32_t _streamField = (uint32_t)streamField;
        int _streamFieldBitIndex = streamFieldBitIndex;

        while (count > (32 - _streamFieldBitIndex) && streamIndex < (stream_size >> 2)) {
            data = (data << (int)(32 - _streamFieldBitIndex)) | (_streamField >> _streamFieldBitIndex);
            count -= 32 - _streamFieldBitIndex;
            _streamField = ((stream[streamIndex * 4] & 0xFF) | ((stream[streamIndex * 4 + 1] & 0xFF) << 8) | ((stream[streamIndex * 4 + 2] & 0xFF) << 16) | ((stream[streamIndex * 4 + 3] & 0xFF) << 24));
            _streamFieldBitIndex = 0;
        }

        if (count > 0) data = (data << count) | (_streamField >> (32 - count));
        return data;
    }

    int ReadStreamData(uint8_t *stream, int stream_size, int *streamIndex, int *streamField, int *streamFieldBitIndex, int count)
    {
        int data = 0;
        while (count > (32 - *streamFieldBitIndex)) {
            data = data << (32 - *streamFieldBitIndex) | ((uint32_t)(*streamField) >> *streamFieldBitIndex);
            count -= 32 - *streamFieldBitIndex;
            *streamField = ((stream[*streamIndex * 4] & 0xFF) | ((stream[*streamIndex * 4 + 1] & 0xFF) << 8) | ((stream[*streamIndex * 4 + 2] & 0xFF) << 16) | ((stream[*streamIndex * 4 + 3] & 0xFF) << 24));
            *streamFieldBitIndex = 0;
            *streamIndex += 1;
        }
        if (count > 0) {
            data = (data << count) | ((uint32_t)(*streamField) >> (32 - count));
            *streamField <<= count;
            *streamFieldBitIndex += count;
        }
        return data;
    }

    void AlignStreamData(int *streamField, int *streamFieldBitIndex)
    {
        int alignedLength;
        int actualLength = *streamFieldBitIndex;

        if (actualLength > 0) {
            alignedLength = (actualLength & ~7);
            if (alignedLength != actualLength) {
                alignedLength += 0x08;
                *streamField <<= (alignedLength - actualLength);
                *streamFieldBitIndex = alignedLength;
            }
        }
    }

    bool DecodeFieldBytes(uint8_t *stream, int stream_size, int *streamIndex, int *streamField, int *streamFieldBitIndex, int *run, int *level)
    {
        bool last = false;
        int streamLength, temp;
        uint32_t streamCode = PeekStreamData(stream, stream_size, *streamIndex, *streamField, *streamFieldBitIndex, 32);
        int zeroCount = CLZLUT[streamCode >> 24];
        if (zeroCount == 8) {
            zeroCount += CLZLUT[(streamCode >> 16) & 0xFF];
            if (zeroCount == 16) {
                zeroCount += CLZLUT[(streamCode >> 8) & 0xFF];
                if (zeroCount == 24) {
                    zeroCount += CLZLUT[streamCode & 0xFF];
                }
            }
        }

        if (zeroCount > 1) {
            temp = (streamCode << (zeroCount + 1)) >> (32 - (zeroCount - 1));
            streamCode <<= 2*zeroCount;
            streamLength = 2*zeroCount;
            *run = temp + (1 << (zeroCount - 1));
        }
        else {
            streamCode <<= (zeroCount + 1);
            streamLength = zeroCount + 1;
            *run = zeroCount;
        }

        zeroCount = CLZLUT[streamCode >> 24];
        if (zeroCount == 8) {
            zeroCount += CLZLUT[(streamCode >> 16) & 0xFF];
            if (zeroCount == 16) {
                zeroCount += CLZLUT[(streamCode >> 8) & 0xFF];
                if (zeroCount == 24) {
                    zeroCount += CLZLUT[streamCode & 0xFF];
                }
            }
        }

        if (zeroCount == 1) {
            streamCode <<= 2;
            streamLength += 2;
            last = true;
        }
        else {
            if (zeroCount == 0) {
                streamLength += 2;
                streamCode = (streamCode << 1) >> 31;
                temp = (streamCode >> 1) + 1;
            }
            else {
                streamLength += 2*zeroCount + 1;
                streamCode = (streamCode << (zeroCount + 1)) >> (32 - zeroCount);
                temp = streamCode >> 1;
                temp += (int)(1 << (zeroCount - 1));
            }

            int sign = streamCode & 1;

            *level = (sign == 1) ? -temp : temp;
            last = false;
        }

        ReadStreamData(stream, stream_size, streamIndex, streamField, streamFieldBitIndex, streamLength);
        return last;
    }

    void GetBlockBytes(uint8_t *stream, int stream_size, int16_t *dataBlockBuffer, int dataBlockBufferLength, int *streamIndex, int *streamField, int *streamFieldBitIndex, int quantizerMode, bool acCoefficientsAvailable)
    {
        bool last = false;
        int run, level;
        int zigZagPosition = 0;
        int matrixPosition = 0;

        ZeroMemory(dataBlockBuffer, dataBlockBufferLength*sizeof(int16_t));

        int dcCoefficientTemp = ReadStreamData(stream, stream_size, streamIndex, streamField, streamFieldBitIndex, 10);

        if (quantizerMode == TABLE_QUANTIZATION_MODE) {
            dataBlockBuffer[0] = (int16_t)(dcCoefficientTemp * QUANTIZER_VALUES[0]);

            if (acCoefficientsAvailable) {
                last = DecodeFieldBytes(stream, stream_size, streamIndex, streamField, streamFieldBitIndex, &run, &level);

                while (!last) {
                    zigZagPosition += run + 1;
                    matrixPosition = ZIGZAG_POSITIONS[zigZagPosition];
                    level *= QUANTIZER_VALUES[matrixPosition];
                    dataBlockBuffer[matrixPosition] = (int16_t)level;
                    last = DecodeFieldBytes(stream, stream_size, streamIndex, streamField, streamFieldBitIndex, &run, &level);
                }
            }
        }
        else {
            // Currently not implemented.
            uint16_t exception = -1;
        }
    }

    void InverseTransform(int16_t *src, int16_t *dst)
    {
        const int FIX_0_298631336 = 2446;
        const int FIX_0_390180644 = 3196;
        const int FIX_0_541196100 = 4433;
        const int FIX_0_765366865 = 6270;
        const int FIX_0_899976223 = 7373;
        const int FIX_1_175875602 = 9633;
        const int FIX_1_501321110 = 12299;
        const int FIX_1_847759065 = 15137;
        const int FIX_1_961570560 = 16069;
        const int FIX_2_053119869 = 16819;
        const int FIX_2_562915447 = 20995;
        const int FIX_3_072711026 = 25172;
        const int BITS = 13;
        const int PASS1_BITS = 1;
        const int F1 = BITS - PASS1_BITS - 1;
        const int F2 = BITS - PASS1_BITS;
        const int F3 = BITS + PASS1_BITS + 3;
        int z1, z2, z3, z4, z5;
        int tmp0, tmp1, tmp2, tmp3;
        int tmp10, tmp11, tmp12, tmp13;
        int pointer;
        int workSpace[64];

        for (pointer = 0; pointer < 8; pointer++) {
            if (src[pointer + 8] == 0 && src[pointer + 16] == 0 && src[pointer + 24] == 0 && src[pointer + 32] == 0 && src[pointer + 40] == 0 && src[pointer + 48] == 0 && src[pointer + 56] == 0) {
                int dcValue = src[pointer] << PASS1_BITS;
                workSpace[pointer +  0] = dcValue;
                workSpace[pointer +  8] = dcValue;
                workSpace[pointer + 16] = dcValue;
                workSpace[pointer + 24] = dcValue;
                workSpace[pointer + 32] = dcValue;
                workSpace[pointer + 40] = dcValue;
                workSpace[pointer + 48] = dcValue;
                workSpace[pointer + 56] = dcValue;
            }
            else {
                z2 = src[pointer + 16];
                z3 = src[pointer + 48];

                z1 = (z2 + z3) * FIX_0_541196100;
                tmp2 = z1 + z3 * -FIX_1_847759065;
                tmp3 = z1 + z2 * FIX_0_765366865;

                z2 = src[pointer];
                z3 = src[pointer + 32];

                tmp0 = (z2 + z3) << BITS;
                tmp1 = (z2 - z3) << BITS;

                tmp10 = tmp0 + tmp3;
                tmp13 = tmp0 - tmp3;
                tmp11 = tmp1 + tmp2;
                tmp12 = tmp1 - tmp2;

                tmp0 = src[pointer + 56];
                tmp1 = src[pointer + 40];
                tmp2 = src[pointer + 24];
                tmp3 = src[pointer + 8];

                z1 = tmp0 + tmp3;
                z2 = tmp1 + tmp2;
                z3 = tmp0 + tmp2;
                z4 = tmp1 + tmp3;
                z5 = (z3 + z4) * FIX_1_175875602;

                tmp0 = tmp0 * FIX_0_298631336;
                tmp1 = tmp1 * FIX_2_053119869;
                tmp2 = tmp2 * FIX_3_072711026;
                tmp3 = tmp3 * FIX_1_501321110;
                z1 = z1 * -FIX_0_899976223;
                z2 = z2 * -FIX_2_562915447;
                z3 = z3 * -FIX_1_961570560;
                z4 = z4 * -FIX_0_390180644;

                z3 += z5;
                z4 += z5;

                tmp0 += z1 + z3;
                tmp1 += z2 + z4;
                tmp2 += z2 + z3;
                tmp3 += z1 + z4;

                workSpace[pointer +  0] = ((tmp10 + tmp3 + (1 << F1)) >> F2);
                workSpace[pointer + 56] = ((tmp10 - tmp3 + (1 << F1)) >> F2);
                workSpace[pointer +  8] = ((tmp11 + tmp2 + (1 << F1)) >> F2);
                workSpace[pointer + 48] = ((tmp11 - tmp2 + (1 << F1)) >> F2);
                workSpace[pointer + 16] = ((tmp12 + tmp1 + (1 << F1)) >> F2);
                workSpace[pointer + 40] = ((tmp12 - tmp1 + (1 << F1)) >> F2);
                workSpace[pointer + 24] = ((tmp13 + tmp0 + (1 << F1)) >> F2);
                workSpace[pointer + 32] = ((tmp13 - tmp0 + (1 << F1)) >> F2);
            }
        }

        for (pointer = 0; pointer < 64; pointer += 8) {
            z2 = workSpace[pointer + 2];
            z3 = workSpace[pointer + 6];

            z1 = (z2 + z3) * FIX_0_541196100;
            tmp2 = z1 + z3 * -FIX_1_847759065;
            tmp3 = z1 + z2 * FIX_0_765366865;

            z1 = workSpace[pointer];
            z2 = workSpace[pointer + 4];

            tmp0 = (z1 + z2) << BITS;
            tmp1 = (z1 - z2) << BITS;

            tmp10 = tmp0 + tmp3;
            tmp13 = tmp0 - tmp3;
            tmp11 = tmp1 + tmp2;
            tmp12 = tmp1 - tmp2;

            tmp3 = workSpace[pointer + 1];
            tmp2 = workSpace[pointer + 3];
            tmp1 = workSpace[pointer + 5];
            tmp0 = workSpace[pointer + 7];

            z1 = (tmp0 + tmp3) * -FIX_0_899976223;
            z2 = (tmp1 + tmp2) * -FIX_2_562915447;
            z3 = tmp0 + tmp2;
            z4 = tmp1 + tmp3;

            z5 = (z3 + z4) * FIX_1_175875602;

            z3 = (z3 * -FIX_1_961570560) + z5;
            z4 = (z4 * -FIX_0_390180644) + z5;

            tmp0 = (tmp0 * FIX_0_298631336) + z1 + z3;
            tmp1 = (tmp1 * FIX_2_053119869) + z2 + z4;
            tmp2 = (tmp2 * FIX_3_072711026) + z2 + z3;
            tmp3 = (tmp3 * FIX_1_501321110) + z1 + z4;

            dst[pointer + 0] = (int16_t)((tmp10 + tmp3) >> F3);
            dst[pointer + 1] = (int16_t)((tmp11 + tmp2) >> F3);
            dst[pointer + 2] = (int16_t)((tmp12 + tmp1) >> F3);
            dst[pointer + 3] = (int16_t)((tmp13 + tmp0) >> F3);
            dst[pointer + 4] = (int16_t)((tmp13 - tmp0) >> F3);
            dst[pointer + 5] = (int16_t)((tmp12 - tmp1) >> F3);
            dst[pointer + 6] = (int16_t)((tmp11 - tmp2) >> F3);
            dst[pointer + 7] = (int16_t)((tmp10 - tmp3) >> F3);
        }
    }

    inline int Saturate5(int x)
    {
        if (x < 0) x = 0;
        x >>= 11;
        return (x > 0x1F) ? 0x1F : x;
    }

    inline int Saturate6(int x)
    {
        if (x < 0) x = 0;
        x >>= 10;
        return x > 0x3F ? 0x3F : x;
    }

    void ComposeImageSlice(ImageSlice *imageSlice, int sliceIndex, uint16_t *javaPixelData, int width, int height)
    {
        int pixelDataQuadrantOffsets[] = {0, BLOCK_WIDTH, width * BLOCK_WIDTH, (width * BLOCK_WIDTH) + BLOCK_WIDTH};
        int imageDataOffset = (sliceIndex - 1) * width * 16;
        const int cromaQuadrantOffsets[] = {0, 4, 32, 36};

        for (int i = 0; i < imageSlice->Count; i++) {
            MacroBlock *macroBlock = &(imageSlice->MacroBlocks[i]);

            for (int verticalStep = 0; verticalStep < BLOCK_WIDTH / 2; verticalStep++) {
                int chromaOffset = verticalStep * BLOCK_WIDTH;
                int lumaElementIndex1 = verticalStep * BLOCK_WIDTH * 2;
                int lumaElementIndex2 = lumaElementIndex1 + BLOCK_WIDTH;
                int dataIndex1 = imageDataOffset + (2 * verticalStep * width);
                int dataIndex2 = dataIndex1 + width;

                for (int horizontalStep = 0; horizontalStep < BLOCK_WIDTH / 2; horizontalStep++) {
                    for (int quadrant = 0; quadrant < 4; quadrant++) {
                        int chromaIndex = chromaOffset + cromaQuadrantOffsets[quadrant] + horizontalStep;
                        int chromaBlueValue = macroBlock->DataBlocks[4][chromaIndex];
                        int chromaRedValue = macroBlock->DataBlocks[5][chromaIndex];

                        int u = chromaBlueValue - 128;
                        int ug = 88 * u;
                        int ub = 454 * u;

                        int v = chromaRedValue - 128;
                        int vg = 183 * v;
                        int vr = 359 * v;

                        for (int pixel = 0; pixel < 2; pixel++) {
                            int r, g, b;
                            int deltaIndex = 2 * horizontalStep + pixel;
                            int lumaElementValue1 = macroBlock->DataBlocks[quadrant][lumaElementIndex1 + deltaIndex] << 8;
                            int lumaElementValue2 = macroBlock->DataBlocks[quadrant][lumaElementIndex2 + deltaIndex] << 8;
                            r = Saturate5(lumaElementValue1 + vr);
                            g = Saturate6(lumaElementValue1 - ug - vg);
                            b = Saturate5(lumaElementValue1 + ub);
                            javaPixelData[dataIndex1 + pixelDataQuadrantOffsets[quadrant] + deltaIndex] = (uint16_t)((r << 11) | (g << 5) | b);
                            r = Saturate5(lumaElementValue2 + vr);
                            g = Saturate6(lumaElementValue2 - ug - vg);
                            b = Saturate5(lumaElementValue2 + ub);
                            javaPixelData[dataIndex2 + pixelDataQuadrantOffsets[quadrant] + deltaIndex] = (uint16_t)((r << 11) | (g << 5) | b);
                        }
                    }
                }
            }
            imageDataOffset += 16;
        }
    }

    void DecodeVideo(uint8_t *stream, int stream_size, uint8_t *img, int *width, int *height)
    {
        int gob = 0;
        int pictureFormat;
        int resolution;
        int pictureType;
        int quantizerMode;
        int sliceCount;
        int blockCount;
        int frameIndex;
        int streamField = 0;
        int streamFieldBitIndex = 32;
        int streamIndex = 0;
        int sliceIndex = 0;
        bool pictureComplete = false;
        ImageSlice *imageSlice = NULL;
        uint16_t *javaPixelData = NULL;
        const int dataBlockBufferLength = 64;
        int16_t dataBlockBuffer[dataBlockBufferLength];
        bool blockY0HasAcComponents = false;
        bool blockY1HasAcComponents = false;
        bool blockY2HasAcComponents = false;
        bool blockY3HasAcComponents = false;
        bool blockCbHasAcComponents = false;
        bool blockCrHasAcComponents = false;

        while (!pictureComplete && streamIndex < (stream_size >> 2)) {
            // 
            AlignStreamData(&streamField, &streamFieldBitIndex);

            // Picture start code
            int code = ReadStreamData(stream, stream_size, &streamIndex, &streamField, &streamFieldBitIndex, 22);
            int startCode = code & (~0x1F);

            if (startCode == 32) {
                if ((code & 0x1F) == 0x1F) {
                    pictureComplete = true;
                }
                else {
                    if (sliceIndex++ == 0) {
                        pictureFormat = ReadStreamData(stream, stream_size, &streamIndex, &streamField, &streamFieldBitIndex, 2);
                        resolution    = ReadStreamData(stream, stream_size, &streamIndex, &streamField, &streamFieldBitIndex, 3);
                        pictureType   = ReadStreamData(stream, stream_size, &streamIndex, &streamField, &streamFieldBitIndex, 3);
                        quantizerMode = ReadStreamData(stream, stream_size, &streamIndex, &streamField, &streamFieldBitIndex, 5);
                        frameIndex    = ReadStreamData(stream, stream_size, &streamIndex, &streamField, &streamFieldBitIndex, 32);

                        switch (pictureFormat) {
                        case CIF:
                            *width = CIF_WIDTH << (resolution - 1);
                            *height = CIG_HEIGHT << (resolution - 1);
                            break;
                        case QVGA:
                            *width = VGA_WIDTH << (resolution - 1);
                            *height = VGA_HEIGHT << (resolution - 1);
                            break;
                        }

                        // We assume two bytes per pixel (RGB 565)
                        sliceCount = (*height) >> 4;
                        blockCount = (*width) >> 4;

                        if (imageSlice == NULL) imageSlice = new ImageSlice(blockCount);
                        if (javaPixelData == NULL) javaPixelData = new uint16_t[(*width) * (*height)];
                    }
                    else quantizerMode = ReadStreamData(stream, stream_size, &streamIndex, &streamField, &streamFieldBitIndex, 5);
                }
            }

            // 
            if (!pictureComplete) {
                for (int count = 0; count < blockCount; count++) {
                    int macroBlockEmpty = ReadStreamData(stream, stream_size, &streamIndex, &streamField, &streamFieldBitIndex, 1);
                    if (macroBlockEmpty == 0) {
                        int acCoefficientsTemp = ReadStreamData(stream, stream_size, &streamIndex, &streamField, &streamFieldBitIndex, 8);
                        blockY0HasAcComponents = (acCoefficientsTemp >> 0 & 1) == 1;
                        blockY1HasAcComponents = (acCoefficientsTemp >> 1 & 1) == 1;
                        blockY2HasAcComponents = (acCoefficientsTemp >> 2 & 1) == 1;
                        blockY3HasAcComponents = (acCoefficientsTemp >> 3 & 1) == 1;
                        blockCbHasAcComponents = (acCoefficientsTemp >> 4 & 1) == 1;
                        blockCrHasAcComponents = (acCoefficientsTemp >> 5 & 1) == 1;

                        if ((acCoefficientsTemp >> 6 & 1) == 1) {
                            int quantizer_modeTemp = ReadStreamData(stream, stream_size, &streamIndex, &streamField, &streamFieldBitIndex, 2);
                            quantizerMode = (int) ((quantizer_modeTemp < 2) ? ~quantizer_modeTemp : quantizer_modeTemp);
                        }

                        GetBlockBytes(stream, stream_size, dataBlockBuffer, dataBlockBufferLength, &streamIndex, &streamField, &streamFieldBitIndex, quantizerMode, blockY0HasAcComponents);
                        InverseTransform(dataBlockBuffer, imageSlice->MacroBlocks[count].DataBlocks[0]);
                        GetBlockBytes(stream, stream_size, dataBlockBuffer, dataBlockBufferLength, &streamIndex, &streamField, &streamFieldBitIndex, quantizerMode, blockY1HasAcComponents);
                        InverseTransform(dataBlockBuffer, imageSlice->MacroBlocks[count].DataBlocks[1]);
                        GetBlockBytes(stream, stream_size, dataBlockBuffer, dataBlockBufferLength, &streamIndex, &streamField, &streamFieldBitIndex, quantizerMode, blockY2HasAcComponents);
                        InverseTransform(dataBlockBuffer, imageSlice->MacroBlocks[count].DataBlocks[2]);
                        GetBlockBytes(stream, stream_size, dataBlockBuffer, dataBlockBufferLength, &streamIndex, &streamField, &streamFieldBitIndex, quantizerMode, blockY3HasAcComponents);
                        InverseTransform(dataBlockBuffer, imageSlice->MacroBlocks[count].DataBlocks[3]);
                        GetBlockBytes(stream, stream_size, dataBlockBuffer, dataBlockBufferLength, &streamIndex, &streamField, &streamFieldBitIndex, quantizerMode, blockCbHasAcComponents);
                        InverseTransform(dataBlockBuffer, imageSlice->MacroBlocks[count].DataBlocks[4]);
                        GetBlockBytes(stream, stream_size, dataBlockBuffer, dataBlockBufferLength, &streamIndex, &streamField, &streamFieldBitIndex, quantizerMode, blockCrHasAcComponents);
                        InverseTransform(dataBlockBuffer, imageSlice->MacroBlocks[count].DataBlocks[5]);
                    }
                }

                // Compose image slice
                ComposeImageSlice(imageSlice, sliceIndex, javaPixelData, *width, *height);
            }
        }

        // Convert 16bit pixel data to 8bit RGB
        for(int i = 0; i < (*width) * (*height); i++) {
            uint8_t r = (javaPixelData[i] & 0xF800) >> 11;
            uint8_t g = (javaPixelData[i] & 0x7E0) >> 5;
            uint8_t b = (javaPixelData[i] & 0x1F);
            *(img + i*3+0) = b << 3;
            *(img + i*3+1) = g << 2;
            *(img + i*3+2) = r << 3;
        }  

        // Release memory
        if (imageSlice) delete imageSlice;
        if (javaPixelData) delete [] javaPixelData;
    }
};

#endif
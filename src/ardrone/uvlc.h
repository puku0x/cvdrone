// The codes of decoding UVLC video is based on following site.
//   AR.Drone Development - 1.3.3 Receive and Decode Video Stream Part.3: Decode binary stream
//   http://ardrone-ailab-u-tokyo.blogspot.com/2012/07/133-receive-and-decode-video-stream.html

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

//#endregion

#include <inttypes.h>

// video header
typedef struct _videostream_header_t {
    uint32_t psc;         // Picture Start Code
    uint32_t pformat;     // Picture format 
    uint32_t resolution;  // Picture Resolution 
    uint32_t ptype;       // Picture Type
    uint8_t  pquant;      // Picture Quantizer
    uint32_t pframe;      // Picture Frame
} videostream_header_t;

// array to re-order zig-zag list to normal matrix
const int inverseZZ[64] = {  0,  1,  8, 16,  9,  2,  3, 10,
                            17, 24, 32, 25, 18, 11,  4,  5, 
                            12, 19, 26, 33, 40, 48, 41, 34, 
                            27, 20, 13,  6,  7, 14, 21, 28, 
                            35, 42, 49, 56, 57, 50, 43, 36, 
                            29, 22, 15, 23, 30, 37, 44, 51, 
                            58, 59, 52, 45, 38, 31, 39, 46, 
                            53, 60, 61, 54, 47, 55, 62, 63 };

// YUV -> RGB conversion
const int16_t   offset = 80;

// decode video
void decodeVideo(uint32_t *bufferedVideoStream, int *index, uint8_t *remainingbits, uint8_t *imageData, int *width, int *height);

// read header of UDP video stream 
videostream_header_t readHeader(uint32_t* bufferedVideoStream, int* index, uint8_t* remainingbits);

// retreive binary data with designated digit as 'readbits' from 'bufferedVideoStream'
uint32_t readBitsfromBuffer(uint32_t* bufferedVideoStream, int* index, uint8_t* remainingbits, int readbits);

// read macroblock components
void decodeMacroBlocks(uint32_t* bufferedVideoStream, int* index, uint8_t* remainingbits, uint8_t *imageData, int width, int height, uint8_t gobsc, uint8_t gobquant, uint8_t mb);

// decode blocks s.t. Y0, Y1, Y2, Y3 Cb, Cr
void decodeBlock(uint32_t* bufferedVideoStream, int* index, uint8_t* remainingbits, uint8_t ACflag, uint8_t* pixel, uint8_t gobquant);

// decode RLE
void decodeRLE(uint32_t* befferedVideoStream, int* index, uint8_t* remainingbits, uint8_t* zzlist_index);

// decode Huffman coding
uint8_t decodeHuff(uint32_t* bufferedVideoStream, int* index, uint8_t* remainingbits, int16_t* zzlist, uint8_t* zzlist_index);

// inver descrete cosine transform
void invertDCT(uint8_t* dst, int16_t* src);

// convert decoded YUV image to RGB
void convertYUV2RGB(uint8_t *imageData, int width, int height, uint8_t gobsc, uint8_t mb, uint8_t *pixel0, uint8_t *pixel1, uint8_t *pixel2, uint8_t *pixel3, uint8_t *pixel4, uint8_t *pixel5);

// set given value into the range of 8 bit unsigned integar
uint8_t window(int val);

videostream_header_t readHeader(uint32_t* bufferedVideoStream, int* index, uint8_t* remainingbits)
{
    // read video stream header :
    // Picture Start Code (22bits)
    // Picture format     (2)
    // Picture resolution (3)
    // Picture type       (3)
    // Picture Quantizer  (5)
    // Picture Frame      (32)

    *index  = 0;
    *remainingbits = 32;

    videostream_header_t videostream_header;
    videostream_header.psc        = readBitsfromBuffer(bufferedVideoStream, index, remainingbits, 22);
    videostream_header.pformat    = readBitsfromBuffer(bufferedVideoStream, index, remainingbits,  2);  
    videostream_header.resolution = readBitsfromBuffer(bufferedVideoStream, index, remainingbits,  3);  
    videostream_header.ptype      = readBitsfromBuffer(bufferedVideoStream, index, remainingbits,  3);  
    videostream_header.pquant     = (uint8_t)(32-readBitsfromBuffer(bufferedVideoStream, index, remainingbits,  5));
    videostream_header.pframe     = readBitsfromBuffer(bufferedVideoStream, index, remainingbits, 32);   

    // *note*
    // picture quantizer value (videostream_header.pquant) should be within [1:30], and the lower the value is, the better quality the picture is.
    // however,  the return value is always 31 (we are using AR.Drone 1) and the picture quality must be the best, so the pquant is simply substituted from 32.
    // If, in future, this pquant is to be changed, then you have to adjust the YUV->RGB convesion coeficients in function 'convertYUV2RGB' and 'convertYUV2GRY'.

    return videostream_header;
}

uint32_t readBitsfromBuffer(uint32_t* bufferedVideoStream, int* index, uint8_t* remainingbits, int readbits)
{
    // retreive binary data with digits designated by 'readbits' from 'bufferedVideoStream'
    // 'index' is the element of bufferedVideoStream array, currently being processed.
    // 'remainingbits' is the remaining binary digits that are not retreived yet.
    // after the desired data is taken out, corresponding data is flushed out from bufferedVideoStream

    // copy current buffer data and flush unrelated part
    uint32_t outputval =  bufferedVideoStream[*index];
    outputval >>= 32-readbits;

    if (*remainingbits<readbits) {
        // if there are less binary digits left in current array element, then read from the next element and merge.

        bufferedVideoStream[*index] <<= *remainingbits;
        int readmorebits = readbits-*remainingbits;

        *index += 1;
        *remainingbits = 32;

        outputval |= readBitsfromBuffer(bufferedVideoStream, index, remainingbits, readmorebits);

    }else{
        // remove the read data from buffered videodata 
        bufferedVideoStream[*index] <<= readbits;
        *remainingbits -= readbits;  
    }
    return outputval;
}

void decodeVideo(uint32_t *bufferedVideoStream, int *index, uint8_t* remainingbits, uint8_t *imageData, int *width, int *height)
{
    videostream_header_t videostream_header;
    uint8_t i, gobsc, gobquant, mbc, mb;

    // decode GOB#0~14
    for (i = 0; i < 15; i++) {
        if (i == 0) {
            // read header
            videostream_header = readHeader(bufferedVideoStream, index, remainingbits);
            gobsc    = videostream_header.psc & 24;
            gobquant = videostream_header.pquant;

            switch (videostream_header.pformat) {
                case 0x01:
                    *width = 88 << (videostream_header.resolution - 1);
                    *height = 72 << (videostream_header.resolution - 1);
                    break;
                case 0x02:
                    *width = 160 << (videostream_header.resolution - 1);
                    *height = 120 << (videostream_header.resolution - 1);
                break;
            }
        }
        else {
            // skip zero fill
            while (readBitsfromBuffer(bufferedVideoStream, index, remainingbits, 1) == 0) {}
            // read GOB header
            gobsc    = (uint8_t)readBitsfromBuffer(bufferedVideoStream, index, remainingbits, 5);
            gobquant = (uint8_t)(32-readBitsfromBuffer(bufferedVideoStream, index, remainingbits, 5));
        }

        // decode macroblocks
        for (mb = 0; mb < 20; mb++) {
            //read Coded Macroblock Bit
            mbc = readBitsfromBuffer(bufferedVideoStream, index, remainingbits, 1);
            // if MBC is zero then, decode remaining field
            if (mbc == 0) decodeMacroBlocks(bufferedVideoStream, index, remainingbits, imageData, *width, *height, gobsc, gobquant, mb);
        }
    }
}

void decodeMacroBlocks(uint32_t *bufferedVideoStream, int *index, uint8_t *remainingbits, uint8_t *imageData, int width, int height, uint8_t gobsc, uint8_t gobquant, uint8_t mb)
{
    uint8_t mbdes = readBitsfromBuffer(bufferedVideoStream, index, remainingbits, 8);

    uint8_t AcCoefficientY0 = (mbdes&(1<<0))!=0;
    uint8_t AcCoefficientY1 = (mbdes&(1<<1))!=0;
    uint8_t AcCoefficientY2 = (mbdes&(1<<2))!=0;
    uint8_t AcCoefficientY3 = (mbdes&(1<<3))!=0;
    uint8_t AcCoefficientCb = (mbdes&(1<<4))!=0;
    uint8_t AcCoefficientCr = (mbdes&(1<<5))!=0;

    if ((mbdes&(1<<6)) != 0) {
        printf("WARNING: Macroblock Differential Quantization is on.\n");
        uint8_t mbdiff = readBitsfromBuffer(bufferedVideoStream, index, remainingbits, 2);
    }

    uint8_t pixel0[64] = {0};
    uint8_t pixel1[64] = {0};
    uint8_t pixel2[64] = {0};
    uint8_t pixel3[64] = {0};
    uint8_t pixel4[64] = {0};
    uint8_t pixel5[64] = {0};

    decodeBlock(bufferedVideoStream, index, remainingbits, AcCoefficientY0, pixel0, gobquant);
    decodeBlock(bufferedVideoStream, index, remainingbits, AcCoefficientY1, pixel1, gobquant);
    decodeBlock(bufferedVideoStream, index, remainingbits, AcCoefficientY2, pixel2, gobquant);
    decodeBlock(bufferedVideoStream, index, remainingbits, AcCoefficientY3, pixel3, gobquant);
    decodeBlock(bufferedVideoStream, index, remainingbits, AcCoefficientCb, pixel4, gobquant);
    decodeBlock(bufferedVideoStream, index, remainingbits, AcCoefficientCr, pixel5, gobquant);

    convertYUV2RGB(imageData, width, height, gobsc, mb, pixel0, pixel1, pixel2, pixel3, pixel4, pixel5);
}

void decodeBlock(uint32_t* bufferedVideoStream, int* index, uint8_t* remainingbits, uint8_t ACflag, uint8_t* pixel, uint8_t gobquant)
{
    uint8_t j;

    // first we decode binary bits back to quantized and zig-zag-ordered list
    int16_t   zzlist[64]   = {0};
    uint8_t   zzlist_index = 1;

    // decode DC value (first 10 bits as 16 digit SIGNED int)
    zzlist[0] = (int16_t)readBitsfromBuffer(bufferedVideoStream, index, remainingbits, 10);

    // if the AC coefficient flag is on, then decode AC values
    if (ACflag == 1) {
        // decode Videostream until EOB is found
        uint8_t EOB = 0;
        while (EOB == 0 && zzlist_index <= 64) {                    
            // decode zero values (Run Length Encoding)
            decodeRLE(bufferedVideoStream, index, remainingbits, &zzlist_index);
            // decode non-zero values (Huffman Encoding)
            EOB = decodeHuff(bufferedVideoStream, index, remainingbits, zzlist, &zzlist_index);            
        }

        if (zzlist_index > 64) {
            printf("decode error:zig-zag list index %d\n", zzlist_index);
            return;
        }
    }

    // reorder zir-zag list & multiple quantization coefficient
    int16_t list[64] = {0};
    for (j = 0; j < zzlist_index; j++) list[inverseZZ[j]] = zzlist[j]*(1+gobquant*(1+ (inverseZZ[j]>>3) + (inverseZZ[j]&7)));

    // Inverse Discrete Cosine Transform
    invertDCT(pixel, list);
}

void decodeRLE(uint32_t *bufferedVideoStream, int *index, uint8_t *remainingbits, uint8_t *zzlist_index)
{
    uint8_t zerocount = 0;

    while (1) {
        if (zerocount > 16) {
            printf("error:decodHuff\n");
            return;
        }

        // if the bit is zero, then increase zero counter
        if (readBitsfromBuffer(bufferedVideoStream, index, remainingbits, 1) == 0) { 
            zerocount += 1;
        }
        // if the bit is one, then decode RLE
        else{
            // decode the number of sequential zeros in zig-zag list

            // there is no zero 
            if (zerocount == 0) {
                // printf("\tRLE:1 => None\n");
            }
            // there is only one zero
            else if(zerocount == 1) {
                *zzlist_index += 1;
                // printf("\tRLE:01 => 0\n");
            }
            // there is more than one zero
            else {
                uint32_t add = readBitsfromBuffer(bufferedVideoStream, index, remainingbits, zerocount-1);
                *zzlist_index += (1<<(zerocount-1)) + add;
            }

            break;
        }
    }
}

uint8_t decodeHuff(uint32_t* bufferedVideoStream, int* index, uint8_t* remainingbits, int16_t* zzlist, uint8_t* zzlist_index)
{
    uint8_t EOB = 0;
    uint8_t zerocount = 0;

    while (1) {
        if (zerocount>16) {
            printf("error:decodHuff\n");
            return 0;
        }

        if (readBitsfromBuffer(bufferedVideoStream, index, remainingbits, 1) == 0) { 
            // if the bit is zero, then increase zero counter
            zerocount  += 1;
        }else{ 
            // if the bit is one, then decode Huffman Encoding
            // decode the number of sequential zeros in zig-zag list
            if (zerocount==0) { // the value is +/-1
                uint8_t sign = readBitsfromBuffer(bufferedVideoStream, index, remainingbits, 1);
                *(zzlist+*zzlist_index) = (-2*sign+1);
                *zzlist_index +=1;
                // printf("\tHuff:1%1d=> %d\n",sign, (-2*sign+1));
            }else if(zerocount==1) { // EOB
                EOB = 1;
                // printf("\tHuff:01 => EOB\n");
            }else { // other values
                uint32_t add = readBitsfromBuffer(bufferedVideoStream, index, remainingbits, zerocount-1);
                uint8_t sign = readBitsfromBuffer(bufferedVideoStream, index, remainingbits, 1);

                *(zzlist+*zzlist_index) = (-2*sign+1)*((1<<(zerocount-1)) + add);
                *zzlist_index +=1;
            }
            return EOB;
        }
    }
}

void invertDCT(uint8_t* dst, int16_t* src)
{  
    int tmp0, tmp1, tmp2, tmp3, tmp10, tmp11, tmp12, tmp13;
    int z1, z2, z3, z4, z5; 
    int workspace[64] = {0};
    uint8_t j;

    // process column
    for (j=0;j<8;j++) {
        if (src[j+8]==0 && src[j+16]==0 && src[j+24]==0 && src[j+32]==0 && src[j+40]==0 && src[j+48]==0 && src[j+56]==0) {
            int DCvalue = src[j] << 1;
            workspace[j+ 0] = DCvalue;
            workspace[j+ 8] = DCvalue;
            workspace[j+16] = DCvalue;
            workspace[j+24] = DCvalue;
            workspace[j+32] = DCvalue;
            workspace[j+40] = DCvalue;
            workspace[j+48] = DCvalue;
            workspace[j+56] = DCvalue;            
        }else{
            // odd column
            tmp0 = (src[j] + src[j+32]) << 13;
            tmp1 = (src[j] - src[j+32]) << 13;
            tmp2 =  4433*src[j+16] - 10704*src[j+48];
            tmp3 = 10703*src[j+16] +  4433*src[j+48];

            tmp10 = tmp0 + tmp3;
            tmp13 = tmp0 - tmp3;
            tmp11 = tmp1 + tmp2;
            tmp12 = tmp1 - tmp2;

            // even column
            tmp0 = src[j+56];
            tmp1 = src[j+40];
            tmp2 = src[j+24];
            tmp3 = src[j+ 8];

            z1 = tmp0 + tmp3;
            z2 = tmp1 + tmp2;
            z3 = tmp0 + tmp2;
            z4 = tmp1 + tmp3;
            z5 = 9633*(z3 + z4);

            tmp0 *=   2446;
            tmp1 *=  16819;
            tmp2 *=  25172;
            tmp3 *=  12299;
            z1   *=  -7373;
            z2   *= -20995;
            z3   *= -16069;
            z4   *=  -3196;

            z3 += z5;
            z4 += z5;

            tmp0 += z1 + z3;
            tmp1 += z2 + z4;
            tmp2 += z2 + z3;
            tmp3 += z1 + z4;

            // substitute to temporary variable
            workspace[j+ 0] = ((tmp10 + tmp3 + (1 << 11)) >> 12); 
            workspace[j+ 8] = ((tmp11 + tmp2 + (1 << 11)) >> 12); 
            workspace[j+16] = ((tmp12 + tmp1 + (1 << 11)) >> 12); 
            workspace[j+24] = ((tmp13 + tmp0 + (1 << 11)) >> 12); 
            workspace[j+32] = ((tmp13 - tmp0 + (1 << 11)) >> 12); 
            workspace[j+40] = ((tmp12 - tmp1 + (1 << 11)) >> 12); 
            workspace[j+48] = ((tmp11 - tmp2 + (1 << 11)) >> 12); 
            workspace[j+56] = ((tmp10 - tmp3 + (1 << 11)) >> 12); 
        }
    }

    // process row
    for (j=0;j<64;j+=8) {
        // odd row
        z2 = workspace[j+2];
        z3 = workspace[j+6];
        z1 = (z2 + z3) *  4433;

        tmp2 = z1 - z3 * 15137;
        tmp3 = z1 + z2 *  6270;

        tmp0 = (workspace[j+0] + workspace[j+4]) << 13;
        tmp1 = (workspace[j+0] - workspace[j+4]) << 13;

        tmp10 = tmp0 + tmp3;
        tmp13 = tmp0 - tmp3;
        tmp11 = tmp1 + tmp2;
        tmp12 = tmp1 - tmp2;

        // even row
        tmp0 = workspace[j+7];
        tmp1 = workspace[j+5];
        tmp2 = workspace[j+3];
        tmp3 = workspace[j+1];

        z1 = tmp0 + tmp3;
        z2 = tmp1 + tmp2;
        z3 = tmp0 + tmp2;
        z4 = tmp1 + tmp3;
        z5 = (z3 + z4) * 9633;

        tmp0 *=  2446;
        tmp1 *= 16819;
        tmp2 *= 25172;
        tmp3 *= 12299;

        z1 *=  -7373;
        z2 *= -20995;
        z3 *= -16069;
        z4 *=  -3196;

        z3 += z5;
        z4 += z5;

        tmp0 += z1 + z3;
        tmp1 += z2 + z4;
        tmp2 += z2 + z3;
        tmp3 += z1 + z4;

        // output
        dst[j+ 0] = (uint16_t) ((tmp10 + tmp3) >> 17); 
        dst[j+ 7] = (uint16_t) ((tmp10 - tmp3) >> 17); 
        dst[j+ 1] = (uint16_t) ((tmp11 + tmp2) >> 17); 
        dst[j+ 6] = (uint16_t) ((tmp11 - tmp2) >> 17); 
        dst[j+ 2] = (uint16_t) ((tmp12 + tmp1) >> 17); 
        dst[j+ 5] = (uint16_t) ((tmp12 - tmp1) >> 17); 
        dst[j+ 3] = (uint16_t) ((tmp13 + tmp0) >> 17); 
        dst[j+ 4] = (uint16_t) ((tmp13 - tmp0) >> 17); 
    }
}

void convertYUV2RGB(uint8_t *imageData, int width, int height, uint8_t gobsc, uint8_t mb, uint8_t *pixel0, uint8_t *pixel1, uint8_t *pixel2, uint8_t *pixel3, uint8_t *pixel4, uint8_t *pixel5)
{
    int nChannels     = 3;
    int widthStep     = nChannels * sizeof(uint8_t) * width;
    uint8_t* img_data = imageData;

    int8_t u;
    int8_t v;
    int8_t chroma;

    uint8_t j, k;
    for (j=0;j<4;j++) {
        for (k=0;k<4;k++) {
            u = pixel4[j*8+k]-offset;
            v = pixel5[j*8+k]-offset;

            // B : Y+u+(u>>1)+(u>>2)+(u>>6)
            chroma = u+(u>>1)+(u>>2)+(u>>6);
            img_data[widthStep*(16*gobsc+2*j  ) +nChannels*(16*mb+2*k  )  ] = window(pixel0[(2*j  )*8+2*k  ]+chroma);
            img_data[widthStep*(16*gobsc+2*j  ) +nChannels*(16*mb+2*k+1)  ] = window(pixel0[(2*j  )*8+2*k+1]+chroma);
            img_data[widthStep*(16*gobsc+2*j+1) +nChannels*(16*mb+2*k  )  ] = window(pixel0[(2*j+1)*8+2*k  ]+chroma);
            img_data[widthStep*(16*gobsc+2*j+1) +nChannels*(16*mb+2*k+1)  ] = window(pixel0[(2*j+1)*8+2*k+1]+chroma);

            // G : Y-((u>>2)+(u>>4)+(u>>5))-((v>>1)+(v>>3)+(v>>4)+(v>>5))
            chroma = -(u>>2)-(u>>4)-(u>>5)-(v>>1)-(v>>3)-(v>>4)-(v>>5);
            img_data[widthStep*(16*gobsc+2*j  ) +nChannels*(16*mb+2*k  )+1] = window(pixel0[(2*j  )*8+2*k  ]+chroma);
            img_data[widthStep*(16*gobsc+2*j  ) +nChannels*(16*mb+2*k+1)+1] = window(pixel0[(2*j  )*8+2*k+1]+chroma);
            img_data[widthStep*(16*gobsc+2*j+1) +nChannels*(16*mb+2*k  )+1] = window(pixel0[(2*j+1)*8+2*k  ]+chroma);
            img_data[widthStep*(16*gobsc+2*j+1) +nChannels*(16*mb+2*k+1)+1] = window(pixel0[(2*j+1)*8+2*k+1]+chroma);

            // R : Y+v+(v>>2)+(v>>3)+(v>>5)
            chroma = v+(v>>2)+(v>>3)+(v>>5);
            img_data[widthStep*(16*gobsc+2*j  ) +nChannels*(16*mb+2*k  )+2] = window(pixel0[(2*j  )*8+2*k  ]+chroma);
            img_data[widthStep*(16*gobsc+2*j  ) +nChannels*(16*mb+2*k+1)+2] = window(pixel0[(2*j  )*8+2*k+1]+chroma);
            img_data[widthStep*(16*gobsc+2*j+1) +nChannels*(16*mb+2*k  )+2] = window(pixel0[(2*j+1)*8+2*k  ]+chroma);
            img_data[widthStep*(16*gobsc+2*j+1) +nChannels*(16*mb+2*k+1)+2] = window(pixel0[(2*j+1)*8+2*k+1]+chroma);

        }
        for (k=4;k<8;k++) {
            u = pixel4[j*8+k]-offset;
            v = pixel5[j*8+k]-offset;

            // B : Y+u+(u>>1)+(u>>2)+(u>>6)
            chroma = u+(u>>1)+(u>>2)+(u>>6);
            img_data[widthStep*(16*gobsc+2*j  ) +nChannels*(16*mb+2*k  )  ] = window(pixel1[(2*j  )*8+2*(k-4)  ]+chroma);
            img_data[widthStep*(16*gobsc+2*j  ) +nChannels*(16*mb+2*k+1)  ] = window(pixel1[(2*j  )*8+2*(k-4)+1]+chroma);
            img_data[widthStep*(16*gobsc+2*j+1) +nChannels*(16*mb+2*k  )  ] = window(pixel1[(2*j+1)*8+2*(k-4)  ]+chroma);
            img_data[widthStep*(16*gobsc+2*j+1) +nChannels*(16*mb+2*k+1)  ] = window(pixel1[(2*j+1)*8+2*(k-4)+1]+chroma);

            // G : Y-((u>>2)+(u>>4)+(u>>5))-((v>>1)+(v>>3)+(v>>4)+(v>>5))
            chroma = -(u>>2)-(u>>4)-(u>>5) -(v>>1)-(v>>3)-(v>>4)-(v>>5);
            img_data[widthStep*(16*gobsc+2*j  ) +nChannels*(16*mb+2*k  )+1] = window(pixel1[(2*j  )*8+2*(k-4)  ]+chroma);
            img_data[widthStep*(16*gobsc+2*j  ) +nChannels*(16*mb+2*k+1)+1] = window(pixel1[(2*j  )*8+2*(k-4)+1]+chroma);
            img_data[widthStep*(16*gobsc+2*j+1) +nChannels*(16*mb+2*k  )+1] = window(pixel1[(2*j+1)*8+2*(k-4)  ]+chroma);
            img_data[widthStep*(16*gobsc+2*j+1) +nChannels*(16*mb+2*k+1)+1] = window(pixel1[(2*j+1)*8+2*(k-4)+1]+chroma);

            // R : Y+v+(v>>2)+(v>>3)+(v>>5)
            chroma = v+(v>>2)+(v>>3)+(v>>5);
            img_data[widthStep*(16*gobsc+2*j  ) +nChannels*(16*mb+2*k  )+2] = window(pixel1[(2*j  )*8+2*(k-4)  ]+chroma);
            img_data[widthStep*(16*gobsc+2*j  ) +nChannels*(16*mb+2*k+1)+2] = window(pixel1[(2*j  )*8+2*(k-4)+1]+chroma);
            img_data[widthStep*(16*gobsc+2*j+1) +nChannels*(16*mb+2*k  )+2] = window(pixel1[(2*j+1)*8+2*(k-4)  ]+chroma);
            img_data[widthStep*(16*gobsc+2*j+1) +nChannels*(16*mb+2*k+1)+2] = window(pixel1[(2*j+1)*8+2*(k-4)+1]+chroma);
        }
    }
    for (j=4;j<8;j++) {
        for (k=0;k<4;k++) {
            u = pixel4[j*8+k]-offset;
            v = pixel5[j*8+k]-offset;

            // B : Y+u+(u>>1)+(u>>2)+(u>>6)
            chroma = u+(u>>1)+(u>>2)+(u>>6);
            img_data[widthStep*(16*gobsc+2*j  ) +nChannels*(16*mb+2*k  )  ] = window(pixel2[(2*(j-4)  )*8+2*k  ]+chroma);
            img_data[widthStep*(16*gobsc+2*j  ) +nChannels*(16*mb+2*k+1)  ] = window(pixel2[(2*(j-4)  )*8+2*k+1]+chroma);
            img_data[widthStep*(16*gobsc+2*j+1) +nChannels*(16*mb+2*k  )  ] = window(pixel2[(2*(j-4)+1)*8+2*k  ]+chroma);
            img_data[widthStep*(16*gobsc+2*j+1) +nChannels*(16*mb+2*k+1)  ] = window(pixel2[(2*(j-4)+1)*8+2*k+1]+chroma);

            // G : Y-((u>>2)+(u>>4)+(u>>5))-((v>>1)+(v>>3)+(v>>4)+(v>>5))
            chroma = -(u>>2)-(u>>4)-(u>>5) -(v>>1)-(v>>3)-(v>>4)-(v>>5);
            img_data[widthStep*(16*gobsc+2*j  ) +nChannels*(16*mb+2*k  )+1] = window(pixel2[(2*(j-4)  )*8+2*k  ]+chroma);
            img_data[widthStep*(16*gobsc+2*j  ) +nChannels*(16*mb+2*k+1)+1] = window(pixel2[(2*(j-4)  )*8+2*k+1]+chroma);
            img_data[widthStep*(16*gobsc+2*j+1) +nChannels*(16*mb+2*k  )+1] = window(pixel2[(2*(j-4)+1)*8+2*k  ]+chroma);
            img_data[widthStep*(16*gobsc+2*j+1) +nChannels*(16*mb+2*k+1)+1] = window(pixel2[(2*(j-4)+1)*8+2*k+1]+chroma);

            // R : Y+v+(v>>2)+(v>>3)+(v>>5)
            chroma = v+(v>>2)+(v>>3)+(v>>5);
            img_data[widthStep*(16*gobsc+2*j  ) +nChannels*(16*mb+2*k  )+2] = window(pixel2[(2*(j-4)  )*8+2*k  ]+chroma);
            img_data[widthStep*(16*gobsc+2*j  ) +nChannels*(16*mb+2*k+1)+2] = window(pixel2[(2*(j-4)  )*8+2*k+1]+chroma);
            img_data[widthStep*(16*gobsc+2*j+1) +nChannels*(16*mb+2*k  )+2] = window(pixel2[(2*(j-4)+1)*8+2*k  ]+chroma);
            img_data[widthStep*(16*gobsc+2*j+1) +nChannels*(16*mb+2*k+1)+2] = window(pixel2[(2*(j-4)+1)*8+2*k+1]+chroma);
        }
        for (k=4;k<8;k++) {
            u = pixel4[j*8+k]-offset;
            v = pixel5[j*8+k]-offset;

            // B : Y+u+(u>>1)+(u>>2)+(u>>6)
            chroma = u+(u>>1)+(u>>2)+(u>>6);
            img_data[widthStep*(16*gobsc+2*j  ) +nChannels*(16*mb+2*k  )  ] = window(pixel3[(2*(j-4)  )*8+2*(k-4)  ] +chroma);
            img_data[widthStep*(16*gobsc+2*j  ) +nChannels*(16*mb+2*k+1)  ] = window(pixel3[(2*(j-4)  )*8+2*(k-4)+1] +chroma);
            img_data[widthStep*(16*gobsc+2*j+1) +nChannels*(16*mb+2*k  )  ] = window(pixel3[(2*(j-4)+1)*8+2*(k-4)  ] +chroma);
            img_data[widthStep*(16*gobsc+2*j+1) +nChannels*(16*mb+2*k+1)  ] = window(pixel3[(2*(j-4)+1)*8+2*(k-4)+1] +chroma);

            // G : Y-((u>>2)+(u>>4)+(u>>5))-((v>>1)+(v>>3)+(v>>4)+(v>>5))
            chroma = -(u>>2)-(u>>4)-(u>>5) -(v>>1)-(v>>3)-(v>>4)-(v>>5);
            img_data[widthStep*(16*gobsc+2*j  ) +nChannels*(16*mb+2*k  )+1] = window(pixel3[(2*(j-4)  )*8+2*(k-4)  ] +chroma);
            img_data[widthStep*(16*gobsc+2*j  ) +nChannels*(16*mb+2*k+1)+1] = window(pixel3[(2*(j-4)  )*8+2*(k-4)+1] +chroma);
            img_data[widthStep*(16*gobsc+2*j+1) +nChannels*(16*mb+2*k  )+1] = window(pixel3[(2*(j-4)+1)*8+2*(k-4)  ] +chroma);
            img_data[widthStep*(16*gobsc+2*j+1) +nChannels*(16*mb+2*k+1)+1] = window(pixel3[(2*(j-4)+1)*8+2*(k-4)+1] +chroma);

            // R : Y+v+(v>>2)+(v>>3)+(v>>5)
            chroma = v+(v>>2)+(v>>3)+(v>>5);
            img_data[widthStep*(16*gobsc+2*j  ) +nChannels*(16*mb+2*k  )+2] = window(pixel3[(2*(j-4)  )*8+2*(k-4)  ] +chroma);
            img_data[widthStep*(16*gobsc+2*j  ) +nChannels*(16*mb+2*k+1)+2] = window(pixel3[(2*(j-4)  )*8+2*(k-4)+1] +chroma);
            img_data[widthStep*(16*gobsc+2*j+1) +nChannels*(16*mb+2*k  )+2] = window(pixel3[(2*(j-4)+1)*8+2*(k-4)  ] +chroma);
            img_data[widthStep*(16*gobsc+2*j+1) +nChannels*(16*mb+2*k+1)+2] = window(pixel3[(2*(j-4)+1)*8+2*(k-4)+1] +chroma);
        }
    }
}

uint8_t window(int val)
{
    if (val < 0)         return 0;
    else if (val > 0xFF) return 0xFF;
    else                 return (uint8_t)val;
}
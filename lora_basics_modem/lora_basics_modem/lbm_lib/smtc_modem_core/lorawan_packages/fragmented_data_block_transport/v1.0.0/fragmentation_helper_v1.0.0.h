/**
 * \file      fragmentation_helper_v1.0.0.h
 *
 * \brief     Helper for the LoRa-Alliance fragmentation decoder (v1.0.0)
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef FRAGMENTATION_HELPER_V1_0_0_H
#define FRAGMENTATION_HELPER_V1_0_0_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*!
 * If set to 1 the new API defining \ref FragDecoderWrite and
 * \ref FragDecoderReadfunction callbacks is used.
 */
#define FRAG_DECODER_FILE_HANDLING_NEW_API 1

/*!
 * Maximum number of fragment that can be handled.
 *
 * \remark This parameter has an impact on the memory footprint.
 */
#ifndef FRAG_MAX_NB
#define FRAG_MAX_NB 100
#endif

/*!
 * Maximum fragment size that can be handled.
 *
 * \remark This parameter has an impact on the memory footprint.
 */
#ifndef FRAG_MAX_SIZE
#define FRAG_MAX_SIZE 242
#endif
/*!
 * Maximum number of extra frames that can be handled.
 *
 * \remark This parameter has an impact on the memory footprint.
 */
#ifndef FRAG_MAX_REDUNDANCY
#define FRAG_MAX_REDUNDANCY FRAG_MAX_NB
#endif

#define FRAG_SESSION_FAILED ( int32_t ) 1
#define FRAG_SESSION_FINISHED_SUCCESSFULLY ( int32_t ) 0
#define FRAG_SESSION_NOT_STARTED ( int32_t ) - 2
#define FRAG_SESSION_ONGOING ( int32_t ) - 1

typedef struct sFragDecoderStatus
{
    uint16_t FragNbRx;
    uint16_t FragNbLost;
    uint16_t FragNbLastRx;
    uint8_t  MatrixError;
} FragDecoderStatus_t;

#if( FRAG_DECODER_FILE_HANDLING_NEW_API == 1 )
typedef struct sFragDecoderCallbacks
{
    /*!
     * Writes `data` buffer of `size` starting at address `addr`
     *
     * \param [IN] addr Address start index to write to.
     * \param [IN] data Data buffer to be written.
     * \param [IN] size Size of data buffer to be written.
     *
     * \retval status Write operation status [0: Success, -1 Fail]
     */
    int8_t ( *FragDecoderWrite )( uint32_t addr, uint8_t* data, uint32_t size );
    /*!
     * Reads `data` buffer of `size` starting at address `addr`
     *
     * \param [IN] addr Address start index to read from.
     * \param [IN] data Data buffer to be read.
     * \param [IN] size Size of data buffer to be read.
     *
     * \retval status Read operation status [0: Success, -1 Fail]
     */
    int8_t ( *FragDecoderRead )( uint32_t addr, uint8_t* data, uint32_t size );
} FragDecoderCallbacks_t;
#endif

#if( FRAG_DECODER_FILE_HANDLING_NEW_API == 1 )
/*!
 * \brief Initializes the fragmentation decoder
 *
 * \param [IN] fragNb     Number of expected fragments (without redundancy packets)
 * \param [IN] fragSize   Size of a fragment
 * \param [IN] callbacks  Pointer to the Write/Read functions.
 */
void FragDecoderInit( uint16_t fragNb, uint8_t fragSize, FragDecoderCallbacks_t* callbacks );
#else
/*!
 * \brief Initializes the fragmentation decoder
 *
 * \param [IN] fragNb     Number of expected fragments (without redundancy packets)
 * \param [IN] fragSize   Size of a fragment
 * \param [IN] file       Pointer to file buffer size
 * \param [IN] fileSize   File buffer size
 */
void FragDecoderInit( uint16_t fragNb, uint8_t fragSize, uint8_t* file, uint32_t fileSize );
#endif

#if( FRAG_DECODER_FILE_HANDLING_NEW_API == 1 )
/*!
 * \brief Gets the maximum file size that can be received
 *
 * \retval size FileSize
 */
uint32_t FragDecoderGetMaxFileSize( void );
#endif

/*!
 * \brief Function to decode and reconstruct the binary file
 *        Called for each receive frame
 *
 * \param [IN] fragCounter Fragment counter [1..(FragDecoder.FragNb + FragDecoder.Redundancy)]
 * \param [IN] rawData     Pointer to the fragment to be processed (length = FragDecoder.FragSize)
 *
 * \retval status          Process status. [FRAG_SESSION_ONGOING,
 *                                          FRAG_SESSION_FINISHED or
 *                                          FragDecoder.Status.FragNbLost]
 */
int32_t FragDecoderProcess( uint16_t fragCounter, uint8_t* rawData );

/*!
 * \brief Gets the current fragmentation status
 *
 * \retval status Fragmentation decoder status
 */
FragDecoderStatus_t FragDecoderGetStatus( void );

#ifdef __cplusplus
}
#endif

#endif  // FRAGMENTATION_HELPER_V1_0_0_H
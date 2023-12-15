/**
  ******************************************************************************
  * @file    ble_common.h
  * @author  MCD Application Team
  * @brief   Common file to BLE Middleware
  ******************************************************************************
  * @attention
 *
 * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BLE_COMMON_H
#define __BLE_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include "ble_conf.h"
#include "ble_dbg_conf.h"

/* -------------------------------- *
 *  Basic definitions               *
 * -------------------------------- */

#undef NULL
#define NULL                    0

#undef FALSE
#define FALSE                   0

#undef TRUE
#define TRUE                    (!0)


/* -------------------------------- *
 *  Macro delimiters                *
 * -------------------------------- */

#define M_BEGIN     do {

#define M_END       } while(0)


/* -------------------------------- *
 *  Some useful macro definitions   *
 * -------------------------------- */
#undef MAX
#define MAX( x, y ) ( ( ( x ) > ( y ) ) ? ( x ) : ( y ) )

#undef MIN
#define MIN( x, y ) ( ( ( x ) < ( y ) ) ? ( x ) : ( y ) )

#undef MODINC
#define MODINC( a, m )              \
    M_BEGIN( a )++;                 \
    if( ( a ) >= ( m ) ) ( a ) = 0; \
    M_END

#undef MODDEC
#define MODDEC( a, m )                     \
    M_BEGIN if( ( a ) == 0 )( a ) = ( m ); \
    ( a )--;                               \
    M_END

#undef MODADD
#define MODADD( a, b, m )                \
    M_BEGIN( a ) += ( b );               \
    if( ( a ) >= ( m ) ) ( a ) -= ( m ); \
    M_END

#undef MODSUB
#define MODSUB( a, b, m ) MODADD( a, ( m ) - ( b ), m )

#undef WIN32
#ifdef WIN32
#define ALIGN( n )
#else
#define ALIGN( n ) __attribute__( ( aligned( n ) ) )
#endif

#undef PAUSE
#define PAUSE( t )              \
    M_BEGIN                     \
    volatile int _i;            \
    for( _i = t; _i > 0; _i-- ) \
        ;                       \
    M_END

#undef DIVF
#define DIVF( x, y ) ( ( x ) / ( y ) )

#undef DIVC
#define DIVC( x, y ) ( ( ( x ) + ( y ) -1 ) / ( y ) )

#undef DIVR
#define DIVR( x, y ) ( ( ( x ) + ( ( y ) / 2 ) ) / ( y ) )

#undef SHRR
#define SHRR( x, n ) ( ( ( ( x ) >> ( ( n ) -1 ) ) + 1 ) >> 1 )

#undef BITN
#define BITN( w, n ) ( ( ( w )[( n ) / 32] >> ( ( n ) % 32 ) ) & 1 )

#undef BITNSET
#define BITNSET( w, n, b )                                          \
    M_BEGIN( w )[( n ) / 32] |= ( ( U32 )( b ) ) << ( ( n ) % 32 ); \
    M_END

/* -------------------------------- *
 *  Compiler                         *
 * -------------------------------- */
#undef PLACE_IN_SECTION
#define PLACE_IN_SECTION( __x__ )  __attribute__((section (__x__)))



#ifdef __cplusplus
}
#endif

#endif /*__BLE_COMMON_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/*
 * Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */
#include <string.h>
#include "cy_md4.h"
#include "cy_enterprise_security_error.h"


typedef struct cy_md4_context_
{
    uint32_t total[2];
    uint32_t a;
    uint32_t b;
    uint32_t c;
    uint32_t d;
    uint8_t  buffer[64];
} cy_md4_context_t;

/*
 * 32-bit integer get/put macros
 */
#ifndef MD4_GET_U32
#define MD4_GET_U32(n,b,i)                              \
{                                                       \
    (n) = ( (uint32_t) (b)[(i)    ]       )             \
        | ( (uint32_t) (b)[(i) + 1] <<  8 )             \
        | ( (uint32_t) (b)[(i) + 2] << 16 )             \
        | ( (uint32_t) (b)[(i) + 3] << 24 );            \
}
#endif

#ifndef MD4_PUT_U32
#define MD4_PUT_U32(n,b,i)                                \
{                                                         \
    (b)[(i)    ] = (uint8_t) ( ( (n)       ) & 0xFF );    \
    (b)[(i) + 1] = (uint8_t) ( ( (n) >>  8 ) & 0xFF );    \
    (b)[(i) + 2] = (uint8_t) ( ( (n) >> 16 ) & 0xFF );    \
    (b)[(i) + 3] = (uint8_t) ( ( (n) >> 24 ) & 0xFF );    \
}
#endif

static const uint8_t md4_padding[64] =
{
 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

/* Helper macros */
#define S(x,n) (((x) << (n)) | (((x) & 0xFFFFFFFF) >> (32 - (n))))
#define F1(x, y, z) (((x) & (y)) | ((~(x)) & (z)))
#define P1(a,b,c,d,x,s)                                 \
    do                                                  \
    {                                                   \
        (a) += F1((b),(c),(d)) + (x);                   \
        (a) = S((a),(s));                               \
    } while( 0 )

#define F2(x,y,z) (((x) & (y)) | ((x) & (z)) | ((y) & (z)))
#define P2(a,b,c,d,x,s)                                 \
    do                                                  \
    {                                                   \
        (a) += F2((b),(c),(d)) + (x) + 0x5A827999;      \
        (a) = S((a),(s));                               \
    } while( 0 )

#define F3(x,y,z) ((x) ^ (y) ^ (z))
#define P3(a,b,c,d,x,s)                                 \
    do                                                  \
    {                                                   \
        (a) += F3((b),(c),(d)) + (x) + 0x6ED9EBA1;      \
        (a) = S((a),(s));                               \
    } while( 0 )

static cy_rslt_t cy_md4_internal_process( cy_md4_context_t *ctx,
                                          const uint8_t data[64] )
{
    uint32_t x[16], a, b, c, d;

    MD4_GET_U32( x[ 0], data,  0 );
    MD4_GET_U32( x[ 1], data,  4 );
    MD4_GET_U32( x[ 2], data,  8 );
    MD4_GET_U32( x[ 3], data, 12 );
    MD4_GET_U32( x[ 4], data, 16 );
    MD4_GET_U32( x[ 5], data, 20 );
    MD4_GET_U32( x[ 6], data, 24 );
    MD4_GET_U32( x[ 7], data, 28 );
    MD4_GET_U32( x[ 8], data, 32 );
    MD4_GET_U32( x[ 9], data, 36 );
    MD4_GET_U32( x[10], data, 40 );
    MD4_GET_U32( x[11], data, 44 );
    MD4_GET_U32( x[12], data, 48 );
    MD4_GET_U32( x[13], data, 52 );
    MD4_GET_U32( x[14], data, 56 );
    MD4_GET_U32( x[15], data, 60 );

    a = ctx->a;
    b = ctx->b;
    c = ctx->c;
    d = ctx->d;

    /* Round 1 */
    P1( a, b, c, d, x[ 0],  3 );
    P1( d, a, b, c, x[ 1],  7 );
    P1( c, d, a, b, x[ 2], 11 );
    P1( b, c, d, a, x[ 3], 19 );
    P1( a, b, c, d, x[ 4],  3 );
    P1( d, a, b, c, x[ 5],  7 );
    P1( c, d, a, b, x[ 6], 11 );
    P1( b, c, d, a, x[ 7], 19 );
    P1( a, b, c, d, x[ 8],  3 );
    P1( d, a, b, c, x[ 9],  7 );
    P1( c, d, a, b, x[10], 11 );
    P1( b, c, d, a, x[11], 19 );
    P1( a, b, c, d, x[12],  3 );
    P1( d, a, b, c, x[13],  7 );
    P1( c, d, a, b, x[14], 11 );
    P1( b, c, d, a, x[15], 19 );


    /* Round 2 */
    P2( a, b, c, d, x[ 0],  3 );
    P2( d, a, b, c, x[ 4],  5 );
    P2( c, d, a, b, x[ 8],  9 );
    P2( b, c, d, a, x[12], 13 );
    P2( a, b, c, d, x[ 1],  3 );
    P2( d, a, b, c, x[ 5],  5 );
    P2( c, d, a, b, x[ 9],  9 );
    P2( b, c, d, a, x[13], 13 );
    P2( a, b, c, d, x[ 2],  3 );
    P2( d, a, b, c, x[ 6],  5 );
    P2( c, d, a, b, x[10],  9 );
    P2( b, c, d, a, x[14], 13 );
    P2( a, b, c, d, x[ 3],  3 );
    P2( d, a, b, c, x[ 7],  5 );
    P2( c, d, a, b, x[11],  9 );
    P2( b, c, d, a, x[15], 13 );


    /* Round 3 */
    P3( a, b, c, d, x[ 0],  3 );
    P3( d, a, b, c, x[ 8],  9 );
    P3( c, d, a, b, x[ 4], 11 );
    P3( b, c, d, a, x[12], 15 );
    P3( a, b, c, d, x[ 2],  3 );
    P3( d, a, b, c, x[10],  9 );
    P3( c, d, a, b, x[ 6], 11 );
    P3( b, c, d, a, x[14], 15 );
    P3( a, b, c, d, x[ 1],  3 );
    P3( d, a, b, c, x[ 9],  9 );
    P3( c, d, a, b, x[ 5], 11 );
    P3( b, c, d, a, x[13], 15 );
    P3( a, b, c, d, x[ 3],  3 );
    P3( d, a, b, c, x[11],  9 );
    P3( c, d, a, b, x[ 7], 11 );
    P3( b, c, d, a, x[15], 15 );


    ctx->a += a;
    ctx->b += b;
    ctx->c += c;
    ctx->d += d;

    return( CY_RSLT_SUCCESS );
}

void cy_md4_init( cy_md4_context_t *ctx )
{
    memset( ctx, 0, sizeof( cy_md4_context_t ) );
}

void cy_md4_free( cy_md4_context_t *ctx )
{
    if( ctx != NULL )
    {
        memset( ctx, 0, sizeof( cy_md4_context_t ) );
    }
}

cy_rslt_t cy_md4_start( cy_md4_context_t *ctx )
{
    ctx->total[0] = 0;
    ctx->total[1] = 0;

    ctx->a = 0x67452301;
    ctx->b = 0xEFCDAB89;
    ctx->c = 0x98BADCFE;
    ctx->d = 0x10325476;

    return( CY_RSLT_SUCCESS );
}

cy_rslt_t cy_md4_update( cy_md4_context_t *ctx,
                             const uint8_t *input,
                             size_t len )
{
    size_t fill;
    uint32_t left;
    cy_rslt_t ret = CY_RSLT_ENTERPRISE_SECURITY_UTIL_ERROR;

    if( len == 0 )
    {
        return( CY_RSLT_SUCCESS );
    }

    left = ctx->total[0] & 0x3F;
    fill = 64 - left;

    ctx->total[0] += (uint32_t) len;
    ctx->total[0] &= 0xFFFFFFFF;

    if( ctx->total[0] < (uint32_t) len )
    {
        ctx->total[1]++;
    }

    if( left && len >= fill )
    {
        memcpy( (void *) (ctx->buffer + left), (void *) input, fill );

        if( ( ret = cy_md4_internal_process( ctx, ctx->buffer ) ) != CY_RSLT_SUCCESS )
        {
            return( ret );
        }

        input += fill;
        len   -= fill;
        left = 0;
    }

    while( len >= 64 )
    {
        if( ( ret = cy_md4_internal_process( ctx, input ) ) != CY_RSLT_SUCCESS )
        {
            return( ret );
        }

        input += 64;
        len   -= 64;
    }

    if( len > 0 )
    {
        memcpy( (void *) (ctx->buffer + left), (void *) input, len );
    }

    return( CY_RSLT_SUCCESS );
}

cy_rslt_t cy_md4_finish( cy_md4_context_t *ctx,
                             uint8_t output[16] )
{
    uint32_t last;
    uint32_t padlen;
    uint32_t high, low;
    uint8_t msglen[8];
    cy_rslt_t ret = CY_RSLT_ENTERPRISE_SECURITY_UTIL_ERROR;

    high = ( ctx->total[0] >> 29 ) | ( ctx->total[1] <<  3 );
    low  = ( ctx->total[0] <<  3 );

    MD4_PUT_U32( low,  msglen, 0 );
    MD4_PUT_U32( high, msglen, 4 );

    last = ctx->total[0] & 0x3F;
    padlen = ( last < 56 ) ? ( 56 - last ) : ( 120 - last );

    ret = cy_md4_update( ctx, (uint8_t *)md4_padding, padlen );
    if( ret != CY_RSLT_SUCCESS )
    {
        return( ret );
    }

    if( ( ret = cy_md4_update( ctx, msglen, 8 ) ) != CY_RSLT_SUCCESS )
    {
        return( ret );
    }

    /* Copy to output */
    MD4_PUT_U32( ctx->a, output,  0 );
    MD4_PUT_U32( ctx->b, output,  4 );
    MD4_PUT_U32( ctx->c, output,  8 );
    MD4_PUT_U32( ctx->d, output, 12 );

    return( CY_RSLT_SUCCESS );
}

/*
 * @func  : cy_md4_func
 *
 * @brief : Perform MD4 Operation on given input
 *           output = MD4( input )
 */
cy_rslt_t cy_md4_func( const uint8_t *input,
                       size_t size,
                       uint8_t output[16] )
{
    cy_md4_context_t ctx;
    cy_rslt_t ret = CY_RSLT_ENTERPRISE_SECURITY_UTIL_ERROR;

    cy_md4_init( &ctx );

    if( ( ret = cy_md4_start( &ctx ) ) != CY_RSLT_SUCCESS )
    {
        goto exit;
    }

    if( ( ret = cy_md4_update( &ctx, input, size ) ) != CY_RSLT_SUCCESS )
    {
        goto exit;
    }

    if( ( ret = cy_md4_finish( &ctx, output ) ) != CY_RSLT_SUCCESS )
    {
        goto exit;
    }

exit:
    cy_md4_free( &ctx );

    return( ret );
}
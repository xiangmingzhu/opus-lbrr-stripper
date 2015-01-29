/* Copyright (c) 2011 Xiph.Org Foundation
   Written by Gregory Maxwell */
/*
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

   - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

   - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
   OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <time.h>
#if (!defined WIN32 && !defined _WIN32) || defined(__MINGW32__)
#include <unistd.h>
#else
#include <process.h>
#define getpid _getpid
#endif
#include "opus_multistream.h"
#include "opus.h"
#include "../src/opus_private.h"
#include "test_opus_common.h"

#define MAX_PACKET (1500)
#define SAMPLES (48000*30)
#define SSAMPLES (SAMPLES/3)
#define MAX_FRAME_SAMP (5760)

#include "tables.h"
#include "entdec.h"

/* Unpack predictor values and indices for entropy coding tables */
extern void silk_NLSF_unpack(
          opus_int16            ec_ix[],                        /* O    Indices to entropy tables [ LPC_ORDER ]     */
          opus_uint8            pred_Q8[],                      /* O    LSF predictor [ LPC_ORDER ]                 */
    const silk_NLSF_CB_struct   *psNLSF_CB,                     /* I    Codebook object                             */
    const opus_int              CB1_index                       /* I    Index of vector in first LSF codebook       */
);

/* Shell decoder, operates on one shell code frame of 16 pulses */
extern void silk_shell_decoder(
    opus_int                    *pulses0,                       /* O    data: nonnegative pulse amplitudes          */
    ec_dec                      *psRangeDec,                    /* I/O  Compressor data structure                   */
    const opus_int              pulses4                         /* I    number of pulses per pulse-subframe         */
);

/* Shell encoder, operates on one shell code frame of 16 pulses */
extern void silk_shell_encoder(
    ec_enc                      *psRangeEnc,                    /* I/O  compressor data structure                   */
    const opus_int              *pulses0                        /* I    data: nonnegative pulse amplitudes          */
);

#define test_silk_dec_map(a)                  ( silk_LSHIFT( (a),  1 ) - 1 )

static void test_silk_process_signs(ec_dec          *psRangeDec,
                                    ec_dec          *psRangeEnc,
                                    opus_int         needEnc,
                                    opus_int         pulses[],
                                    opus_int         length,
                                    const opus_int   signalType,
                                    const opus_int   quantOffsetType,
                                    const opus_int   sum_pulses[ MAX_NB_SHELL_BLOCKS ]
)
{
    opus_int         i, j, p;
    opus_uint8       icdf[ 2 ];
    opus_int         *q_ptr;
    const opus_uint8 *icdf_ptr;

    opus_int8 value;

    icdf[ 1 ] = 0;
    q_ptr = pulses;
    i = silk_SMULBB( 7, silk_ADD_LSHIFT( quantOffsetType, signalType, 1 ) );
    icdf_ptr = &silk_sign_iCDF[ i ];
    length = silk_RSHIFT( length + SHELL_CODEC_FRAME_LENGTH/2, LOG2_SHELL_CODEC_FRAME_LENGTH );
    for( i = 0; i < length; i++ ) {
        p = sum_pulses[ i ];
        if( p > 0 ) {
            icdf[ 0 ] = icdf_ptr[ silk_min( p & 0x1F, 6 ) ];
            for( j = 0; j < SHELL_CODEC_FRAME_LENGTH; j++ ) {
                if( q_ptr[ j ] > 0 ) {
                    value = ec_dec_icdf( psRangeDec, icdf, 8 );
                    //q_ptr[ j ] *= test_silk_dec_map(value);
                        if (needEnc) ec_enc_icdf( psRangeEnc, value, icdf, 8 );
                }
            }
        }
        q_ptr += SHELL_CODEC_FRAME_LENGTH;
    }
}

void test_silk_process_indices(ec_dec     *psRangeDec,
                               ec_dec     *psRangeEnc,
                               opus_int    needEnc,
                               opus_int    decode_LBRR,
                               opus_int    VAD_flag,
                               opus_int8  *pSignalType,
                               opus_int8  *pQuantOffsetType)
{
    opus_int   i, k, Ix;

    opus_int16 ec_ix[ MAX_LPC_ORDER ];
    opus_uint8 pred_Q8[ MAX_LPC_ORDER ];

    opus_int8 symbol;

    opus_int8 signalType;
    opus_int8 quantOffsetType;

    opus_int8 PERIndex;

    /*******************************************/
    /* Decode signal type and quantizer offset */
    /*******************************************/
    if( decode_LBRR || VAD_flag ) {
        Ix = ec_dec_icdf( psRangeDec, silk_type_offset_VAD_iCDF, 8 ) + 2;
            if (needEnc) ec_enc_icdf( psRangeEnc, Ix - 2, silk_type_offset_VAD_iCDF, 8 );
    } else {
        Ix = ec_dec_icdf( psRangeDec, silk_type_offset_no_VAD_iCDF, 8 );
            if (needEnc) ec_enc_icdf( psRangeEnc, Ix, silk_type_offset_no_VAD_iCDF, 8 );
    }

    *pSignalType      = signalType      = (opus_int8)silk_RSHIFT( Ix, 1 );
    *pQuantOffsetType = quantOffsetType = (opus_int8)( Ix & 1 );

    /****************/
    /* Decode gains */
    /****************/
    /* First subframe */
    /* Independent coding, in two stages: MSB bits followed by 3 LSBs */
    symbol = (opus_int8)ec_dec_icdf( psRangeDec, silk_gain_iCDF[ signalType ], 8 );
        if (needEnc) ec_enc_icdf( psRangeEnc, symbol, silk_gain_iCDF[ signalType ], 8 );
    symbol = (opus_int8)ec_dec_icdf( psRangeDec, silk_uniform8_iCDF, 8 );
        if (needEnc) ec_enc_icdf( psRangeEnc, symbol, silk_uniform8_iCDF, 8 );

    /* Remaining subframes */
    for( i = 1; i < 4; i++ ) {
        symbol = (opus_int8)ec_dec_icdf( psRangeDec, silk_delta_gain_iCDF, 8 );
            if (needEnc) ec_enc_icdf( psRangeEnc, symbol, silk_delta_gain_iCDF, 8 );
    }

    /**********************/
    /* Decode LSF Indices */
    /**********************/
    symbol = (opus_int8)ec_dec_icdf( psRangeDec, &silk_NLSF_CB_WB.CB1_iCDF[ ( signalType >> 1 ) * silk_NLSF_CB_WB.nVectors ], 8 );
        if (needEnc) ec_enc_icdf( psRangeEnc, symbol, &silk_NLSF_CB_WB.CB1_iCDF[ ( signalType >> 1 ) * silk_NLSF_CB_WB.nVectors ], 8 );

    silk_NLSF_unpack( ec_ix, pred_Q8, &silk_NLSF_CB_WB, symbol );

    for( i = 0; i < silk_NLSF_CB_WB.order; i++ ) {
        Ix = ec_dec_icdf( psRangeDec, &silk_NLSF_CB_WB.ec_iCDF[ ec_ix[ i ] ], 8 );
            if (needEnc) ec_enc_icdf( psRangeEnc, Ix, &silk_NLSF_CB_WB.ec_iCDF[ ec_ix[ i ] ], 8 );
        if( Ix == 0 ) {
            //Ix -= ec_dec_icdf( psRangeDec, silk_NLSF_EXT_iCDF, 8 );
            symbol = (opus_int8)ec_dec_icdf( psRangeDec, silk_NLSF_EXT_iCDF, 8 );
            Ix -= symbol;
                if (needEnc) ec_enc_icdf( psRangeEnc, symbol, silk_NLSF_EXT_iCDF, 8 );
        } else if( Ix == 2 * NLSF_QUANT_MAX_AMPLITUDE ) {
            //Ix += ec_dec_icdf( psRangeDec, silk_NLSF_EXT_iCDF, 8 );
            symbol = (opus_int8)ec_dec_icdf( psRangeDec, silk_NLSF_EXT_iCDF, 8 );
            Ix += symbol;
                if (needEnc) ec_enc_icdf( psRangeEnc, symbol, silk_NLSF_EXT_iCDF, 8 );
        }
    }

    /* Decode LSF interpolation factor */
    symbol = (opus_int8)ec_dec_icdf( psRangeDec, silk_NLSF_interpolation_factor_iCDF, 8 );
        if (needEnc) ec_enc_icdf( psRangeEnc, symbol, silk_NLSF_interpolation_factor_iCDF, 8 );


    if( signalType == TYPE_VOICED )
    {
        /*********************/
        /* Decode pitch lags */
        /*********************/
        /* Get lag index */
        symbol = (opus_int8)ec_dec_icdf( psRangeDec, silk_pitch_lag_iCDF, 8 );
            if (needEnc) ec_enc_icdf( psRangeEnc, symbol, silk_pitch_lag_iCDF, 8 );
        symbol = (opus_int8)ec_dec_icdf( psRangeDec, silk_uniform8_iCDF, 8 );
            if (needEnc) ec_enc_icdf( psRangeEnc, symbol, silk_uniform8_iCDF, 8 );

        /* Get countour index */
        symbol = (opus_int8)ec_dec_icdf( psRangeDec, silk_pitch_contour_iCDF, 8 );
            if (needEnc) ec_enc_icdf( psRangeEnc, symbol, silk_pitch_contour_iCDF, 8 );

        /********************/
        /* Decode LTP gains */
        /********************/
        /* Decode PERIndex value */
        PERIndex = (opus_int8)ec_dec_icdf( psRangeDec, silk_LTP_per_index_iCDF, 8 );
            if (needEnc) ec_enc_icdf( psRangeEnc, PERIndex, silk_LTP_per_index_iCDF, 8 );

        for( k = 0; k < 4; k++ ) {
            symbol = (opus_int8)ec_dec_icdf( psRangeDec, silk_LTP_gain_iCDF_ptrs[ PERIndex ], 8 );
                if (needEnc) ec_enc_icdf( psRangeEnc, symbol, silk_LTP_gain_iCDF_ptrs[ PERIndex ], 8 );
        }

        /**********************/
        /* Decode LTP scaling */
        /**********************/
        symbol = (opus_int8)ec_dec_icdf( psRangeDec, silk_LTPscale_iCDF, 8 );
            if (needEnc) ec_enc_icdf( psRangeEnc, symbol, silk_LTPscale_iCDF, 8 );

    }

    /***************/
    /* Decode seed */
    /***************/
    symbol = (opus_int8)ec_dec_icdf( psRangeDec, silk_uniform4_iCDF, 8 );
        if (needEnc) ec_enc_icdf( psRangeEnc, symbol, silk_uniform4_iCDF, 8 );
}


void test_silk_process_pulses(ec_dec         *psRangeDec,
                              ec_dec         *psRangeEnc,
                              opus_int        needEnc,
                              const opus_int  signalType,
                              const opus_int  quantOffsetType)
{
    opus_int   i, j, k, iter, abs_q, nLS, RateLevelIndex;
    opus_int   sum_pulses[ MAX_NB_SHELL_BLOCKS ], nLshifts[ MAX_NB_SHELL_BLOCKS ];
    opus_int   *pulses_ptr;
    const opus_uint8 *cdf_ptr;

    opus_int   pulses[320];
    opus_int   frame_length = 320;
    opus_int8  value;

    /*********************/
    /* Decode rate level */
    /*********************/
    RateLevelIndex = ec_dec_icdf( psRangeDec, silk_rate_levels_iCDF[ signalType >> 1 ], 8 );
        if (needEnc) ec_enc_icdf( psRangeEnc, RateLevelIndex, silk_rate_levels_iCDF[ signalType >> 1 ], 8 );

    /* Calculate number of shell blocks */
    silk_assert( 1 << LOG2_SHELL_CODEC_FRAME_LENGTH == SHELL_CODEC_FRAME_LENGTH );
    iter = silk_RSHIFT( frame_length, LOG2_SHELL_CODEC_FRAME_LENGTH );
    if( iter * SHELL_CODEC_FRAME_LENGTH < frame_length ) {
        silk_assert( frame_length == 12 * 10 ); /* Make sure only happens for 10 ms @ 12 kHz */
        iter++;
    }

    /***************************************************/
    /* Sum-Weighted-Pulses Decoding                    */
    /***************************************************/
    cdf_ptr = silk_pulses_per_block_iCDF[ RateLevelIndex ];
    for( i = 0; i < iter; i++ ) {
        nLshifts[ i ] = 0;
        sum_pulses[ i ] = ec_dec_icdf( psRangeDec, cdf_ptr, 8 );
            if (needEnc) ec_enc_icdf( psRangeEnc, sum_pulses[ i ], cdf_ptr, 8 );

        /* LSB indication */
        while( sum_pulses[ i ] == MAX_PULSES + 1 ) {
            nLshifts[ i ]++;
            /* When we've already got 10 LSBs, we shift the table to not allow (MAX_PULSES + 1) */
            sum_pulses[ i ] = ec_dec_icdf( psRangeDec,
                    silk_pulses_per_block_iCDF[ N_RATE_LEVELS - 1] + ( nLshifts[ i ] == 10 ), 8 );
                if (needEnc) ec_enc_icdf( psRangeEnc, sum_pulses[ i ], silk_pulses_per_block_iCDF[ N_RATE_LEVELS - 1] + ( nLshifts[ i ] == 10 ), 8 );
        }
    }

    /***************************************************/
    /* Shell decoding                                  */
    /***************************************************/
    for( i = 0; i < iter; i++ ) {
        if( sum_pulses[ i ] > 0 ) {
            silk_shell_decoder( &pulses[ silk_SMULBB( i, SHELL_CODEC_FRAME_LENGTH ) ], psRangeDec, sum_pulses[ i ] );
                if (needEnc) silk_shell_encoder(psRangeEnc, &pulses[ silk_SMULBB( i, SHELL_CODEC_FRAME_LENGTH ) ]);
        } else {
            silk_memset( &pulses[ silk_SMULBB( i, SHELL_CODEC_FRAME_LENGTH ) ], 0, SHELL_CODEC_FRAME_LENGTH * sizeof( opus_int ) );
        }
    }

    /***************************************************/
    /* LSB Decoding                                    */
    /***************************************************/
    for( i = 0; i < iter; i++ ) {
        if( nLshifts[ i ] > 0 ) {
            nLS = nLshifts[ i ];
            pulses_ptr = &pulses[ silk_SMULBB( i, SHELL_CODEC_FRAME_LENGTH ) ];
            for( k = 0; k < SHELL_CODEC_FRAME_LENGTH; k++ ) {
                abs_q = pulses_ptr[ k ];
                for( j = 0; j < nLS; j++ ) {
                    abs_q = silk_LSHIFT( abs_q, 1 );
                    //abs_q += ec_dec_icdf( psRangeDec, silk_lsb_iCDF, 8 );
                    value = ec_dec_icdf( psRangeDec, silk_lsb_iCDF, 8 );
                    abs_q += value;
                        if (needEnc) ec_enc_icdf( psRangeEnc, value, silk_lsb_iCDF, 8 );
                }
                pulses_ptr[ k ] = abs_q;
            }
            /* Mark the number of pulses non-zero for sign decoding. */
            sum_pulses[ i ] |= nLS << 5;
        }
    }

    /****************************************/
    /* Decode and add signs to pulse signal */
    /****************************************/
    test_silk_process_signs( psRangeDec, psRangeEnc, needEnc, pulses, frame_length, signalType, quantOffsetType, sum_pulses );
}


int test_range_dec_enc(unsigned char * packet, int len, unsigned char * newPacket, int *pNewLen)
{
    ec_dec rangeDec;
    ec_enc rangeEnc;

    int VAD_flag;
    int LBRR_flag;

    opus_int8 signalType;
    opus_int8 quantOffsetType;

    int newLen;

	if ((len > MAX_PACKET) || (packet == newPacket))
    {
        printf("parameter is invalid\n");
        return 0;
    }

    if (*packet != 0x48)
    {
        printf("only accept Silk-only, WIDEBAND, 20ms, mono\n");
        return 0;
    }

	// dec
    ec_dec_init(&rangeDec, &packet[1], len - 1);
    VAD_flag  = ec_dec_bit_logp(&rangeDec, 1);
    LBRR_flag = ec_dec_bit_logp(&rangeDec, 1);

	if (!LBRR_flag)
	{
		return 0;
	}

	newPacket[0] = 0x48;

    // enc
    ec_enc_init(&rangeEnc, &newPacket[1], MAX_PACKET - 1);
    ec_enc_bit_logp(&rangeEnc, VAD_flag, 1);
    ec_enc_bit_logp(&rangeEnc, 0 /*LBRR_flag*/, 1);

    // skip LBRR frame
    test_silk_process_indices(&rangeDec, &rangeEnc, 0, 1, VAD_flag, &signalType, &quantOffsetType);
    test_silk_process_pulses(&rangeDec, &rangeEnc, 0, signalType, quantOffsetType);

    // process normal SILK frame
    test_silk_process_indices(&rangeDec, &rangeEnc, 1, 0, VAD_flag, &signalType, &quantOffsetType);
    test_silk_process_pulses(&rangeDec, &rangeEnc, 1, signalType, quantOffsetType);

    newLen = (ec_tell(&rangeEnc) + 7) >> 3;
    ec_enc_done(&rangeEnc);

    // remove trailing zeros
    while ((newLen > 2) && (0 == newPacket[newLen])) newLen--;

    *pNewLen = newLen + 1; // add 1 for TOC byte

    return 1;
}

void test(const char * pcmFileInput, int fec)
{
	int err;
	int i;

	OpusEncoder *enc;
	OpusDecoder *dec;
	OpusDecoder *dec2;

	short src_buffer[320];
	short out_buffer[320];
	unsigned char packet[MAX_PACKET];
	unsigned char newpacket[MAX_PACKET];

    int read_size = 0;
    int frames = 0;
	int samples_per_frame = 320; // 20ms for 16k sampling rate

	FILE* fp;
	FILE* fpOut;
	FILE* fpOutPCM;
	FILE* fpOutTranscoded;
	FILE* fpOutPCMTranscoded;

	char strFileOut[128];

    unsigned __int64 tscStart;
    unsigned __int64 tscEncode = 0, tscDecode = 0, tscTranscode = 0;

	////////////////////////////////////
	enc = opus_encoder_create(16000, 1, OPUS_APPLICATION_VOIP, &err);
    if(err != OPUS_OK || enc==NULL)test_failed();

	if (fec)
	{
		if(opus_encoder_ctl(enc, OPUS_SET_INBAND_FEC(1))!=OPUS_OK)test_failed();
		if(opus_encoder_ctl(enc, OPUS_SET_PACKET_LOSS_PERC(10))!=OPUS_OK)test_failed();
	}
	else
	{
		if(opus_encoder_ctl(enc, OPUS_SET_INBAND_FEC(0))!=OPUS_OK)test_failed();
	}

    if(opus_encoder_ctl(enc, OPUS_SET_FORCE_MODE(MODE_SILK_ONLY))!=OPUS_OK)test_failed();

    if(opus_encoder_ctl(enc, OPUS_SET_BITRATE(35000))!=OPUS_OK)test_failed();
	if(opus_encoder_ctl(enc, OPUS_SET_VBR(1))!=OPUS_OK)test_failed();

    if(opus_encoder_ctl(enc, OPUS_SET_FORCE_CHANNELS(1))!=OPUS_OK)test_failed();
    if(opus_encoder_ctl(enc, OPUS_SET_COMPLEXITY(2))!=OPUS_OK)test_failed();
	if(opus_encoder_ctl(enc, OPUS_SET_BANDWIDTH(OPUS_BANDWIDTH_WIDEBAND))!=OPUS_OK)test_failed();
	////////////////////////////////////
	dec = opus_decoder_create(16000, 1, &err);
    if(err != OPUS_OK || dec==NULL)test_failed();

	dec2 = opus_decoder_create(16000, 1, &err);
    if(err != OPUS_OK || dec2==NULL)test_failed();
	////////////////////////////////////

	fp = fopen(pcmFileInput, "rb");
    if (fp == NULL)
    {
        printf("file %s open fail.\n", pcmFileInput);
        return;
    }

    strcpy(strFileOut, pcmFileInput);
    if (fec) strcat(strFileOut, ".fec.opus"); else strcat(strFileOut, ".nofec.opus");

	fpOut = fopen(strFileOut, "wb");
    if (fpOut == NULL)
    {
        printf("file %s open fail.\n", strFileOut);
        return;
    }

	strcpy(strFileOut, pcmFileInput);
    if (fec) strcat(strFileOut, ".dec.fec.pcm"); else strcat(strFileOut, ".dec.nofec.pcm");

	fpOutPCM = fopen(strFileOut, "wb");
    if (fpOutPCM == NULL)
    {
        printf("file %s open fail.\n", strFileOut);
        return;
    }

	strcpy(strFileOut, pcmFileInput);
    strcat(strFileOut, ".transcoded.opus");

    fpOutTranscoded = fopen(strFileOut, "wb");
    if (fpOutTranscoded == NULL)
    {
        printf("file %s open fail.\n", strFileOut);
        return;
    }

	strcpy(strFileOut, pcmFileInput);
    strcat(strFileOut, ".transcoded.pcm");

    fpOutPCMTranscoded = fopen(strFileOut, "wb");
    if (fpOutPCMTranscoded == NULL)
    {
        printf("file %s open fail.\n", strFileOut);
        return;
    }

	fprintf(stderr, "FEC = %d\n", fec);

	while(1)
    {
		int len, out_samples, newlen;

        // read input file
        read_size = fread(src_buffer, sizeof(short), samples_per_frame, fp);
        if (read_size != samples_per_frame)
        {
            printf("\nTotal %d frames.\n", frames);
            break;
        }

		frames++;

        tscStart = __rdtsc();
		len = opus_encode(enc, src_buffer, samples_per_frame, packet, MAX_PACKET);
        tscEncode += __rdtsc() - tscStart;

		fwrite(packet, sizeof(unsigned char), len, fpOut);

		printf("\nFrame: %d, TOC: %02X\n", frames, packet[0]);
		for (i = 1; i < len; i++)
		{
			printf("%02X ", packet[i]);
		}
		printf("\n");

        tscStart = __rdtsc();
		out_samples = opus_decode(dec, packet, len, out_buffer, MAX_FRAME_SAMP, 0);
        tscDecode += __rdtsc() - tscStart;

		fwrite(out_buffer, sizeof(short), out_samples, fpOutPCM);

        // remove LBRR frames
        tscStart = __rdtsc();
        err = test_range_dec_enc(packet, len, newpacket, &newlen);
        tscTranscode += __rdtsc() - tscStart;

        if (err)
        {
            printf("LBRR frame exists\n");

			fwrite(newpacket, sizeof(unsigned char), newlen, fpOutTranscoded);

			printf("newlen = %d\n", newlen);

    		for (i = 1; i < newlen; i++)
    		{
    			printf("%02X ", newpacket[i]);
    		}
    		printf("\n");
        }
		else
		{
			printf("No LBRR frame\n");

			fwrite(packet, sizeof(unsigned char), len, fpOutTranscoded);
		}

        // decode and write to file
        if (err)
        {
		    out_samples = opus_decode(dec2, newpacket, newlen, out_buffer, MAX_FRAME_SAMP, 0);
        }
        else
        {
            out_samples = opus_decode(dec2, packet, len, out_buffer, MAX_FRAME_SAMP, 0);
        }

		fwrite(out_buffer, sizeof(short), out_samples, fpOutPCMTranscoded);
	}

	fclose(fp);
	fclose(fpOut);
	fclose(fpOutPCM);
	fclose(fpOutTranscoded);
    fclose(fpOutPCMTranscoded);

    printf("   Encode = %10lld cycles\n", tscEncode);
	printf("   Decode = %10lld cycles\n", tscDecode);
	printf("Transcode = %10lld cycles\n", tscTranscode);
    printf("Transcode/(Encode+Decode)=%1.4f\n", (tscTranscode*1.0)/(tscEncode+tscDecode));
}

int main(int _argc, char **_argv)
{
	int fec;

    if(_argc != 3)
    {
        fprintf(stderr,"Usage: %s <filename> 1:fec;0:nofec\n",_argv[0]);
        return 1;
    }

    fec = atoi(_argv[2]);
	test(_argv[1], fec);

    return 0;
}

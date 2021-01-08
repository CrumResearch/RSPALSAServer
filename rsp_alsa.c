/*
* rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
* Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
* Copyright (C) 2012-2013 by Hoernchen <la@tfc-server.de>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>

#include "rsp_alsa_api.h"

#include "getopt/getopt.h"

#define HAVE_STRUCT_TIMESPEC
#include <pthread.h>

#include <sdrplay_api.h>

static int fsc = 0;
static int rfc = 0;
static int grc = 0;
static int timeout = 500;
static pthread_t tcp_worker_thread;
static pthread_t command_thread;

static pthread_mutex_t ll_mutex;
static pthread_cond_t cond;

struct llist {
	char *data;
	size_t len;
	struct llist *next;
};

typedef struct { /* structure size must be multiple of 2 bytes */
	char magic[4];
	uint32_t tuner_type;
	uint32_t tuner_gain_count;
} dongle_info_t;

double atofs(char *s)
/* standard suffixes */
{
	char last;
	int len;
	double suff = 1.0;
	len = strlen(s);
	last = s[len - 1];
	s[len - 1] = '\0';
	switch (last) {
	case 'g':
	case 'G':
		suff *= 1e3;
		/* fall-through */
	case 'm':
	case 'M':
		suff *= 1e3;
		/* fall-through */
	case 'k':
	case 'K':
		suff *= 1e3;
		suff *= atof(s);
		s[len - 1] = last;
		return suff;
	}
	s[len - 1] = last;
	return atof(s);
}

static int global_numq = 0;
static struct llist *ll_buffers = 0;
static int llbuf_num = 500;

static int overload = 0;

static volatile int do_exit = 0;
static volatile int ctrlC_exit = 0;

#define RSP_ALSA_VERSION_MAJOR (1)
#define RSP_ALSA_VERSION_MINOR (1)

#define MAX_DECIMATION_FACTOR (64)
#define MAX_DEVS 4
#define WORKER_TIMEOUT_SEC 3
#define DEFAULT_BW_T sdrplay_api_BW_1_536
#define DEFAULT_FREQUENCY (100000000)
#define DEFAULT_SAMPLERATE (2000000)
#define DEFAULT_AGC_SETPOINT -30
#define DEFAULT_GAIN_REDUCTION 40
#define DEFAULT_LNA_STATE 4
#define DEFAULT_AGC_STATE 1
#define RTLSDR_TUNER_R820T 5

static int bwType = sdrplay_api_BW_Undefined;
static int last_gain_idx = 0;
static int verbose = 0;
static uint8_t max_lnastate;

sdrplay_api_DeviceT devices[MAX_DEVS];
sdrplay_api_DeviceT *chosenDev;
sdrplay_api_CallbackFnsT cbFns;

sdrplay_api_DeviceParamsT *deviceParams;
sdrplay_api_RxChannelParamsT *chParams;

// *************************************
#define GAIN_STEPS (29)

const uint8_t rsp1_am_gains_lnastates[]       = {  3, 3, 3, 3, 3, 3, 3, 1, 1, 1, 1, 1, 1, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp1_am_gains_ifgains[]         = { 59,56,53,50,47,44,41,58,55,52,49,46,43,45,42,58,55,52,49,46,43,41,38,35,32,29,26,23,20 };
const uint8_t rsp1_vhf_gains_lnastates[]      = {  3, 3, 3, 3, 3, 3, 3, 1, 1, 1, 1, 1, 1, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp1_vhf_gains_ifgains[]        = { 59,56,53,50,47,44,41,58,55,52,49,46,43,45,42,58,55,52,49,46,43,41,38,35,32,29,26,23,20 };
const uint8_t rsp1_band3_gains_lnastates[]    = {  3, 3, 3, 3, 3, 3, 3, 1, 1, 1, 1, 1, 1, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp1_band3_gains_ifgains[]      = { 59,56,53,50,47,44,41,58,55,52,49,46,43,45,42,58,55,52,49,46,43,41,38,35,32,29,26,23,20 };
const uint8_t rsp1_bandx_gains_lnastates[]    = {  3, 3, 3, 3, 3, 3, 3, 1, 1, 1, 1, 1, 1, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp1_bandx_gains_ifgains[]      = { 59,56,53,50,47,44,41,58,55,52,49,46,43,45,42,58,55,52,49,46,43,41,38,35,32,29,26,23,20 };
const uint8_t rsp1_band45_gains_lnastates[]   = {  3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0 };
const uint8_t rsp1_band45_gains_ifgains[]     = { 59,57,54,52,50,47,45,43,40,38,36,33,31,29,27,24,22,27,24,22,32,29,27,25,22,27,25,22,20 };
const uint8_t rsp1_lband_gains_lnastates[]    = {  3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp1_lband_gains_ifgains[]      = { 59,57,55,52,50,48,46,43,41,44,42,53,51,49,47,44,42,45,43,40,38,36,34,31,29,27,25,22,20 };

const uint8_t rsp1a_am_gains_lnastates[]      = {  6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 4, 4, 3, 3, 3, 3, 3, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp1a_am_gains_ifgains[]        = { 59,55,52,48,45,41,57,53,49,46,42,44,40,56,52,48,45,41,44,40,43,45,41,38,34,31,27,24,20 };
const uint8_t rsp1a_vhf_gains_lnastates[]     = {  9, 9, 9, 9, 9, 9, 8, 7, 7, 7, 7, 7, 6, 6, 5, 5, 4, 3, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp1a_vhf_gains_ifgains[]       = { 59,55,52,48,45,41,42,58,54,51,47,43,46,42,44,41,43,42,44,40,43,45,42,38,34,31,27,24,20 };
const uint8_t rsp1a_band3_gains_lnastates[]   = {  9, 9, 9, 9, 9, 9, 8, 7, 7, 7, 7, 7, 6, 6, 5, 5, 4, 3, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp1a_band3_gains_ifgains[]     = { 59,55,52,48,45,41,42,58,54,51,47,43,46,42,44,41,43,42,44,40,43,45,42,38,34,31,27,24,20 };
const uint8_t rsp1a_bandx_gains_lnastates[]   = {  9, 9, 9, 9, 9, 9, 8, 7, 7, 7, 7, 7, 6, 6, 5, 5, 4, 3, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp1a_bandx_gains_ifgains[]     = { 59,55,52,48,45,41,42,58,54,51,47,43,46,42,44,41,43,42,44,40,43,45,42,38,34,31,27,24,20 };
const uint8_t rsp1a_band45_gains_lnastates[]  = {  9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 7, 6, 6, 5, 5, 4, 4, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp1a_band45_gains_ifgains[]    = { 59,55,52,48,44,41,56,52,49,45,41,44,46,42,45,41,44,40,44,40,42,46,42,38,35,31,27,24,20 };
const uint8_t rsp1a_lband_gains_lnastates[]   = {  8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 6, 5, 5, 4, 4, 3, 2, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp1a_lband_gains_ifgains[]     = { 59,55,52,48,45,41,56,53,49,46,42,43,46,42,44,41,43,48,44,40,43,45,42,38,34,31,27,24,20 };

const uint8_t rsp2_am_gains_lnastates[]       = {  8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 6, 5, 5, 4, 4, 4, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp2_am_gains_ifgains[]         = { 59,55,52,48,44,41,56,52,49,45,41,44,45,41,48,44,40,45,42,43,49,46,42,38,35,31,27,24,20 };
const uint8_t rsp2_vhf_gains_lnastates[]      = {  8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 6, 5, 5, 4, 4, 4, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp2_vhf_gains_ifgains[]        = { 59,55,52,48,44,41,56,52,49,45,41,44,45,41,48,44,40,45,42,43,49,46,42,38,35,31,27,24,20 };
const uint8_t rsp2_band3_gains_lnastates[]    = {  8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 6, 5, 5, 4, 4, 4, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp2_band3_gains_ifgains[]      = { 59,55,52,48,44,41,56,52,49,45,41,44,45,41,48,44,40,45,42,43,49,46,42,38,35,31,27,24,20 };
const uint8_t rsp2_bandx_gains_lnastates[]    = {  8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 6, 5, 5, 4, 4, 4, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp2_bandx_gains_ifgains[]      = { 59,55,52,48,44,41,56,52,49,45,41,44,45,41,48,44,40,45,42,43,49,46,42,38,35,31,27,24,20 };
const uint8_t rsp2_band45_gains_lnastates[]   = {  5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 3, 3, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp2_band45_gains_ifgains[]     = { 59,56,53,50,48,45,42,58,55,52,49,47,44,41,43,40,44,41,42,46,43,40,37,34,31,29,26,23,20 };
const uint8_t rsp2_lband_gains_lnastates[]    = {  4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp2_lband_gains_ifgains[]      = { 59,56,54,51,48,45,43,40,56,54,51,48,45,43,40,43,41,44,41,44,42,39,36,34,31,28,25,23,20 };
const uint8_t rsp2_hiz_gains_lnastates[]      = {  4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rsp2_hiz_gains_ifgains[]        = { 59,56,54,51,48,45,43,40,56,54,51,48,45,43,40,43,41,44,41,44,42,39,36,34,31,28,25,23,20 };

const uint8_t rspduo_am_gains_lnastates[]     = {  6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 4, 4, 3, 3, 3, 3, 3, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rspduo_am_gains_ifgains[]       = { 59,55,52,48,45,41,57,53,49,46,42,44,40,56,52,48,45,41,44,40,43,45,41,38,34,31,27,24,20 };
const uint8_t rspduo_vhf_gains_lnastates[]    = {  9, 9, 9, 9, 9, 9, 8, 7, 7, 7, 7, 7, 6, 6, 5, 5, 4, 3, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rspduo_vhf_gains_ifgains[]      = { 59,55,52,48,45,41,42,58,54,51,47,43,46,42,44,41,43,42,44,40,43,45,42,38,34,31,27,24,20 };
const uint8_t rspduo_band3_gains_lnastates[]  = {  9, 9, 9, 9, 9, 9, 8, 7, 7, 7, 7, 7, 6, 6, 5, 5, 4, 3, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rspduo_band3_gains_ifgains[]    = { 59,55,52,48,45,41,42,58,54,51,47,43,46,42,44,41,43,42,44,40,43,45,42,38,34,31,27,24,20 };
const uint8_t rspduo_bandx_gains_lnastates[]  = {  9, 9, 9, 9, 9, 9, 8, 7, 7, 7, 7, 7, 6, 6, 5, 5, 4, 3, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rspduo_bandx_gains_ifgains[]    = { 59,55,52,48,45,41,42,58,54,51,47,43,46,42,44,41,43,42,44,40,43,45,42,38,34,31,27,24,20 };
const uint8_t rspduo_band45_gains_lnastates[] = {  9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 7, 6, 6, 5, 5, 4, 4, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rspduo_band45_gains_ifgains[]   = { 59,55,52,48,44,41,56,52,49,45,41,44,46,42,45,41,44,40,44,40,42,46,42,38,35,31,27,24,20 };
const uint8_t rspduo_lband_gains_lnastates[]  = {  8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 6, 5, 5, 4, 4, 3, 2, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rspduo_lband_gains_ifgains[]    = { 59,55,52,48,45,41,56,53,49,46,42,43,46,42,44,41,43,48,44,40,43,45,42,38,34,31,27,24,20 };
const uint8_t rspduo_hiz_gains_lnastates[]    = {  4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rspduo_hiz_gains_ifgains[]      = { 59,56,54,51,48,45,43,40,56,54,51,48,45,43,40,43,41,44,41,44,42,39,36,34,31,28,25,23,20 };

const uint8_t rspdx_am_gains_lnastates[]      = { 18,18,18,18,18,18,17,16,14,13,12,11,10, 9, 7, 6, 5, 5, 5, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rspdx_am_gains_ifgains[]        = { 59,55,52,48,45,41,41,40,43,42,42,41,41,40,42,42,47,44,40,43,42,42,41,38,34,31,27,24,20 };
const uint8_t rspdx_vhf_gains_lnastates[]     = { 26,26,26,26,26,25,23,22,20,19,17,16,14,13,11,10, 8, 7, 5, 5, 5, 3, 2, 0, 0, 0, 0, 0, 0 };
const uint8_t rspdx_vhf_gains_ifgains[]       = { 59,55,50,46,41,40,42,40,42,40,42,41,42,41,43,41,43,41,49,45,40,42,40,42,38,33,29,24,20 };
const uint8_t rspdx_band3_gains_lnastates[]   = { 26,26,26,26,26,25,23,22,20,19,17,16,14,13,11,10, 8, 7, 5, 5, 5, 3, 2, 0, 0, 0, 0, 0, 0 };
const uint8_t rspdx_band3_gains_ifgains[]     = { 59,55,50,46,41,40,42,40,42,40,42,41,42,41,43,41,43,41,49,45,40,42,40,42,38,33,29,24,20 };
const uint8_t rspdx_bandx_gains_lnastates[]   = { 27,27,27,27,27,26,24,23,21,20,18,17,15,14,12,11, 9, 8, 6, 6, 5, 3, 2, 0, 0, 0, 0, 0, 0 };
const uint8_t rspdx_bandx_gains_ifgains[]     = { 59,55,50,46,41,40,42,40,42,40,42,41,42,41,43,41,43,41,46,42,40,42,40,42,38,33,29,24,20 };
const uint8_t rspdx_band45_gains_lnastates[]  = { 20,20,20,20,20,20,18,17,16,14,13,12,11, 9, 8, 7, 7, 5, 4, 3, 2, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rspdx_band45_gains_ifgains[]    = { 59,55,51,48,44,40,42,42,41,43,42,41,41,43,42,44,40,43,42,41,40,46,43,39,35,31,28,24,20 };
const uint8_t rspdx_lband_gains_lnastates[]   = { 18,18,18,18,18,18,16,15,14,13,11,10, 9, 8, 7, 6, 6, 6, 5, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rspdx_lband_gains_ifgains[]     = { 59,55,52,48,44,40,43,42,41,41,43,42,41,41,40,48,45,41,40,42,42,41,42,39,35,31,27,24,20 };
const uint8_t rspdx_hiz_gains_lnastates[]     = { 18,18,18,18,18,18,17,16,14,13,12,11,10, 9, 7, 6, 5, 5, 5, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t rspdx_hiz_gains_ifgains[]       = { 59,55,52,48,45,41,41,40,43,42,42,41,41,40,42,42,47,44,40,43,42,42,41,38,34,31,27,24,20 };

typedef enum {
	RSP_MODEL_UNKNOWN = 0,
	RSP_MODEL_RSP1 = 1,
	RSP_MODEL_RSP1A = 2,
	RSP_MODEL_RSP2 = 3,
	RSP_MODEL_RSPDUO = 4,
	RSP_MODEL_RSPDX = 5
} rsp_model_t;

typedef enum {
	BAND_UNKNOWN = 0,
	BAND_AM = 1,
	BAND_VHF = 2,
	BAND_3 = 3,
	BAND_X = 4,
	BAND_45 = 5,
	BAND_L = 6,
	BAND_AM_HIZ = 7
} rsp_band_t;

typedef struct {
	rsp_model_t model;
	char *name;
	uint8_t antenna_input_count;
	char third_antenna_name[13];
	int third_antenna_freq_limit;
	uint8_t tuner_count;
	uint32_t capabilities;

	uint8_t min_ifgr;
	uint8_t max_ifgr;

	const uint8_t* am_lna_states;
	const uint8_t* am_if_gains;
	const uint8_t* vhf_lna_states;
	const uint8_t* vhf_if_gains;
	const uint8_t* band3_lna_states;
	const uint8_t* band3_if_gains;
	const uint8_t* bandx_lna_states;
	const uint8_t* bandx_if_gains;
	const uint8_t* band45_lna_states;
	const uint8_t* band45_if_gains;
	const uint8_t* lband_lna_states;
	const uint8_t* lband_if_gains;
	const uint8_t* hiz_lna_states;
	const uint8_t* hiz_if_gains;

} rsp_capabilities_t;

static rsp_capabilities_t device_caps[] = {
	{
		.model = RSP_MODEL_RSP1,
		.name = "RSP1",
		.antenna_input_count = 1,
		.third_antenna_name = "",
		.third_antenna_freq_limit = 0,
		.tuner_count = 1,
		.capabilities = RSP_CAPABILITY_AGC,
		.min_ifgr = 20,
		.max_ifgr = 59,

		.am_lna_states = rsp1_am_gains_lnastates,
		.am_if_gains = rsp1_am_gains_ifgains,
		.vhf_lna_states = rsp1_vhf_gains_lnastates,
		.vhf_if_gains = rsp1_vhf_gains_ifgains,
		.band3_lna_states = rsp1_band3_gains_lnastates,
		.band3_if_gains = rsp1_band3_gains_ifgains,
		.bandx_lna_states = rsp1_bandx_gains_lnastates,
		.bandx_if_gains = rsp1_bandx_gains_ifgains,
		.band45_lna_states = rsp1_band45_gains_lnastates,
		.band45_if_gains = rsp1_band45_gains_ifgains,
		.lband_lna_states = rsp1_lband_gains_lnastates,
		.lband_if_gains = rsp1_lband_gains_ifgains,
		.hiz_lna_states = NULL,
		.hiz_if_gains = NULL,
	},
	{
		.model = RSP_MODEL_RSP1A,
		.name = "RSP1A",
		.antenna_input_count = 1,
		.third_antenna_name = "",
		.third_antenna_freq_limit = 0,
		.tuner_count = 1,
		.capabilities = RSP_CAPABILITY_AGC |
						RSP_CAPABILITY_BIAS_T |
						RSP_CAPABILITY_DAB_NOTCH |
						RSP_CAPABILITY_BROADCAST_NOTCH,
		.min_ifgr = 20,
		.max_ifgr = 59,

		.am_lna_states = rsp1a_am_gains_lnastates,
		.am_if_gains = rsp1a_am_gains_ifgains,
		.vhf_lna_states = rsp1a_vhf_gains_lnastates,
		.vhf_if_gains = rsp1a_vhf_gains_ifgains,
		.band3_lna_states = rsp1a_band3_gains_lnastates,
		.band3_if_gains = rsp1a_band3_gains_ifgains,
		.bandx_lna_states = rsp1a_bandx_gains_lnastates,
		.bandx_if_gains = rsp1a_bandx_gains_ifgains,
		.band45_lna_states = rsp1a_band45_gains_lnastates,
		.band45_if_gains = rsp1a_band45_gains_ifgains,
		.lband_lna_states = rsp1a_lband_gains_lnastates,
		.lband_if_gains = rsp1a_lband_gains_ifgains,
		.hiz_lna_states = NULL,
		.hiz_if_gains = NULL,
	},
	{
		.model = RSP_MODEL_RSP2,
		.name = "RSP2",
		.antenna_input_count = 3,
		.third_antenna_name = "Antenna Hi-Z",
		.third_antenna_freq_limit = 30000000,
		.tuner_count = 1,
		.capabilities = RSP_CAPABILITY_AGC |
						RSP_CAPABILITY_BIAS_T |
						RSP_CAPABILITY_RF_NOTCH |
						RSP_CAPABILITY_REF_IN |
						RSP_CAPABILITY_REF_OUT,
		.min_ifgr = 20,
		.max_ifgr = 59,

		.am_lna_states = rsp2_am_gains_lnastates,
		.am_if_gains = rsp2_am_gains_ifgains,
		.vhf_lna_states = rsp2_vhf_gains_lnastates,
		.vhf_if_gains = rsp2_vhf_gains_ifgains,
		.band3_lna_states = rsp2_band3_gains_lnastates,
		.band3_if_gains = rsp2_band3_gains_ifgains,
		.bandx_lna_states = rsp2_bandx_gains_lnastates,
		.bandx_if_gains = rsp2_bandx_gains_ifgains,
		.band45_lna_states = rsp2_band45_gains_lnastates,
		.band45_if_gains = rsp2_band45_gains_ifgains,
		.lband_lna_states = rsp2_lband_gains_lnastates,
		.lband_if_gains = rsp2_lband_gains_ifgains,
		.hiz_lna_states = rsp2_hiz_gains_lnastates,
		.hiz_if_gains = rsp2_hiz_gains_ifgains,
	},
	{
		.model = RSP_MODEL_RSPDUO,
		.name = "RSPduo",
		.antenna_input_count = 3,
		.third_antenna_name = "Antenna Hi-Z",
		.third_antenna_freq_limit = 30000000,
		.tuner_count = 2,
		.capabilities = RSP_CAPABILITY_AGC |
						RSP_CAPABILITY_BIAS_T |
						RSP_CAPABILITY_AM_NOTCH |
						RSP_CAPABILITY_DAB_NOTCH |
						RSP_CAPABILITY_BROADCAST_NOTCH |
						RSP_CAPABILITY_REF_IN |
						RSP_CAPABILITY_REF_OUT,
		.min_ifgr = 20,
		.max_ifgr = 59,

		.am_lna_states = rspduo_am_gains_lnastates,
		.am_if_gains = rspduo_am_gains_ifgains,
		.vhf_lna_states = rspduo_vhf_gains_lnastates,
		.vhf_if_gains = rspduo_vhf_gains_ifgains,
		.band3_lna_states = rspduo_band3_gains_lnastates,
		.band3_if_gains = rspduo_band3_gains_ifgains,
		.bandx_lna_states = rspduo_bandx_gains_lnastates,
		.bandx_if_gains = rspduo_bandx_gains_ifgains,
		.band45_lna_states = rspduo_band45_gains_lnastates,
		.band45_if_gains = rspduo_band45_gains_ifgains,
		.lband_lna_states = rspduo_lband_gains_lnastates,
		.lband_if_gains = rspduo_lband_gains_ifgains,
		.hiz_lna_states = rspduo_hiz_gains_lnastates,
		.hiz_if_gains = rspduo_hiz_gains_ifgains,
	},
	{
		.model = RSP_MODEL_RSPDX,
		.name = "RSPdx",
		.antenna_input_count = 3,
		.third_antenna_name = "Antenna C",
		.third_antenna_freq_limit = 200000000,
		.tuner_count = 1,
		.capabilities = RSP_CAPABILITY_AGC |
						RSP_CAPABILITY_BIAS_T |
						RSP_CAPABILITY_DAB_NOTCH |
						RSP_CAPABILITY_BROADCAST_NOTCH |
						RSP_CAPABILITY_REF_IN,
		.min_ifgr = 20,
		.max_ifgr = 59,

		.am_lna_states = rspdx_am_gains_lnastates,
		.am_if_gains = rspdx_am_gains_ifgains,
		.vhf_lna_states = rspdx_vhf_gains_lnastates,
		.vhf_if_gains = rspdx_vhf_gains_ifgains,
		.band3_lna_states = rspdx_band3_gains_lnastates,
		.band3_if_gains = rspdx_band3_gains_ifgains,
		.bandx_lna_states = rspdx_bandx_gains_lnastates,
		.bandx_if_gains = rspdx_bandx_gains_ifgains,
		.band45_lna_states = rspdx_band45_gains_lnastates,
		.band45_if_gains = rspdx_band45_gains_ifgains,
		.lband_lna_states = rspdx_lband_gains_lnastates,
		.lband_if_gains = rspdx_lband_gains_ifgains,
		.hiz_lna_states = rspdx_hiz_gains_lnastates,
		.hiz_if_gains = rspdx_hiz_gains_ifgains,
	},
};

static int extended_mode = 0;
static int hardware_version = 0;
static rsp_capabilities_t *hardware_caps = NULL;
static rsp_model_t hardware_model = RSP_MODEL_UNKNOWN;
static rsp_tcp_sample_format_t sample_format = RSP_TCP_SAMPLE_FORMAT_UINT8;
static rsp_band_t current_band = BAND_UNKNOWN;
static int current_antenna_input = 0;
static unsigned int current_frequency;
static int lna_state = DEFAULT_LNA_STATE;
static int agc_state = DEFAULT_AGC_STATE;
static int agc_set_point = DEFAULT_AGC_SETPOINT;
static int gain_reduction = DEFAULT_GAIN_REDUCTION;
static int sample_shift = 2;

// *************************************

static void sighandler(int signum)
{
	fprintf(stderr, "Signal (%d) caught, ask for exit!\n", signum);
	do_exit = 1;
}

void event_callback(sdrplay_api_EventT eventId, sdrplay_api_TunerSelectT tunerS, sdrplay_api_EventParamsT *params, void* cbContext)
{
	switch (eventId)
	{
	case sdrplay_api_GainChange:
		if (params->gainParams.gRdB < 200)
		{
			printf("gRdB: %u, lnaGRdB: %u, LNAstate: %u, Gain: %0.1f\n", params->gainParams.gRdB, params->gainParams.lnaGRdB, chParams->tunerParams.gain.LNAstate, params->gainParams.currGain);
		}
		break;
	case sdrplay_api_PowerOverloadChange:
		sdrplay_api_Update(chosenDev->dev, chosenDev->tuner, sdrplay_api_Update_Ctrl_OverloadMsgAck, sdrplay_api_Update_Ext1_None);

		if (params->powerOverloadParams.powerOverloadChangeType == sdrplay_api_Overload_Detected)
		{
			printf("adc overload detected\n");
			overload = 1;
		}
		else if (params->powerOverloadParams.powerOverloadChangeType == sdrplay_api_Overload_Corrected)
		{
			printf("adc overload corrected\n");
			overload = 0;
		}
		break;
	case sdrplay_api_DeviceRemoved:
		printf("RSP removed\n");
		sdrplay_api_Uninit(chosenDev->dev);
		sdrplay_api_ReleaseDevice(chosenDev);
		sdrplay_api_Close();
		exit(1);
	case sdrplay_api_RspDuoModeChange:
		printf("RSPduo mode changed\n");
		break;
	}
}

// WORK IN PROGRESS



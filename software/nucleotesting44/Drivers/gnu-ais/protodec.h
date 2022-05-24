
/*
 *	protodec.h
 *
 *	(c) Ruben Undheim 2008
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <stdint.h>

#ifndef INC_PROTODEC_H
#define INC_PROTODEC_H

#define ST_SKURR 1
#define ST_PREAMBLE 2
#define ST_STARTSIGN 3
#define ST_DATA 4
#define ST_STOPSIGN 5

#define DEMOD_BUFFER_LEN 450
#define MAX_AIS_PACKET_TYPE 27
#define NMEABUFFER_LEN 128

struct demod_state_t {
	char chanid;
	int32_t state;
	uint32_t offset;// dongle dependent parameter??????
	int32_t nskurr, npreamble, nstartsign, ndata, nstopsign;
	
	int32_t antallenner;
	uint8_t *buffer;
	uint8_t *rbuffer;
	char *tbuffer;
	int32_t bufferpos;
	char last;
	int32_t antallpreamble;
	int32_t bitstuff;
	int32_t receivedframes;
	int32_t lostframes;
	int32_t lostframes2;
	int32_t debug;// added

	uint8_t seqnr;

    uint32_t startsample;
    int32_t add_sample_num;

	struct serial_state_t *serial;
	
	char *nmea;

//	int message_ctr = 0;
//	char saved_ais_messages[10][50];// length 47 char

//	int (*array)[50];

};

void protodec_initialize(struct demod_state_t *d, struct serial_state_t *serial, char chanid, int32_t add_sample_num);
void protodec_reset(struct demod_state_t *d);
void protodec_getdata(int32_t bufferlengde, struct demod_state_t *d);
void protodec_decode(char *in, int32_t count, struct demod_state_t *d, uint32_t samplenum);

#endif

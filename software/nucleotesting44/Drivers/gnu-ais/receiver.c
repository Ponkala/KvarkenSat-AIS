
/*
 *	receiver.c
 *
 *	(c) Ruben Undheim 2008
 *	(c) Heikki Hannikainen 2008
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


//#include "config.h"

#include "../gnu-ais/receiver.h"

#include <string.h>
#include <time.h>

#include "arm_math.h"

#include "../gnu-ais/filter.h"
#include "../gnu-ais/hmalloc.h"

//static int sound_levellog=1;

receiver_on_level_changed on_sound_level_changed=NULL;

static float32_t coeffs[]={
   2.5959e-55, 2.9479e-49, 1.4741e-43, 3.2462e-38, 3.1480e-33,
   1.3443e-28, 2.5280e-24, 2.0934e-20, 7.6339e-17, 1.2259e-13,
   8.6690e-11, 2.6996e-08, 3.7020e-06, 2.2355e-04, 5.9448e-03,
   6.9616e-02, 3.5899e-01, 8.1522e-01, 8.1522e-01, 3.5899e-01,
   6.9616e-02, 5.9448e-03, 2.2355e-04, 3.7020e-06, 2.6996e-08,
   8.6690e-11, 1.2259e-13, 7.6339e-17, 2.0934e-20, 2.5280e-24,
   1.3443e-28, 3.1480e-33, 3.2462e-38, 1.4741e-43, 2.9479e-49,
   2.5959e-55
};
#define COEFFS_L 36 


struct receiver *init_receiver(char name, int32_t num_ch, int32_t ch_ofs, int32_t add_sample_num)
{
	printf("\nthis is receiver.c,  %c speaking \n", name);
	struct receiver *rx;

	rx = (struct receiver *) hmalloc(sizeof(struct receiver));
	memset(rx, 0, sizeof(struct receiver));

	rx->filter = filter_init(COEFFS_L, coeffs);

    rx->decoder = hmalloc(sizeof(struct demod_state_t));
	protodec_initialize(rx->decoder, NULL, name, add_sample_num);

    rx->name = name;
	rx->lastbit = 0;
	rx->num_ch = num_ch;
	rx->ch_ofs = ch_ofs;
	rx->pll = 0;
	rx->pllinc = 0x10000 / 5;
	rx->prev = 0;
	rx->last_levellog = 0;

	return rx;
}

void free_receiver(struct receiver *rx)
{
	if (rx) {
		filter_free(rx->filter);
		hfree(rx);
	}
}

#define	INC	16
#define FILTERED_LEN 32000

void receiver_run(struct receiver *rx, int16_t *buf, int32_t len)
{
	float32_t out; //change by Oskari to override filtering and decimation.
	//int16_t out;
	int32_t curr, bit;
	char b;
	int16_t maxval = 0;
	int32_t level_distance;
	float32_t level;
	int32_t rx_num_ch = rx->num_ch;
    float32_t filtered[FILTERED_LEN];
	int32_t i;
	
	/* len is number of samples available in buffer for each
	 * channels - something like 1024, regardless of number of channels */
	
	buf += rx->ch_ofs;
	
	/*if (len > FILTERED_LEN) commented out by Oskari
		abort();
*/
	maxval = filter_run_buf(rx->filter, buf, filtered, rx_num_ch, len);
	
	for (i = 0; i < len; i++) {
        rx->samplenum++;
        //out = buf[i]; // addition by Oskari
		out = filtered[i]; //commented out by Oskari
		curr = (out > 0);
		
		if ((curr ^ rx->prev) == 1) {
			if (rx->pll < (0x10000 / 2)) {
				rx->pll += rx->pllinc / INC;
			} else {
				rx->pll -= rx->pllinc / INC;
			}
		}
		rx->prev = curr;

		rx->pll += rx->pllinc;

		if (rx->pll > 0xffff) { // 65535
			/* slice */
			bit = (out > 0);
			/* nrzi decode */
			b = !(bit ^ rx->lastbit);
			/* feed to the decoder */
			protodec_decode(&b, 1, rx->decoder, rx->samplenum);

			rx->lastbit = bit;
			rx->pll &= 0xffff;
		}
	}

	/* calculate level, and log it */
/*
	level = (float)maxval / (float)32768 * (float)100;
	level_distance = time(NULL) - rx->last_levellog;
	
    if (level > 95.0 && (level_distance >= 30 || level_distance >= sound_levellog)) {
        if (on_sound_level_changed != NULL) on_sound_level_changed(level, rx->ch_ofs, 1);
        time(&rx->last_levellog);
    } else if (sound_levellog != 0 && level_distance >= sound_levellog) {
        if (on_sound_level_changed != NULL) on_sound_level_changed(level, rx->ch_ofs, 0);
        time(&rx->last_levellog);
    }
*/
}


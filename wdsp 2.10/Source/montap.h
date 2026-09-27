/*  montap.h

This file is part of a program that implements a Software-Defined Radio.

Copyright (C) 2026 Jeremy McInerney, N0NBH

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

/*  Acknowledgment

    This file was developed with the assistance of Claude (Anthropic), an AI
    assistant, in collaboration with Jeremy McInerney, N0NBH.  Claude does not
    hold copyright; all copyright in this file remains with Jeremy McInerney as
    stated above.  No code was copied from any other implementation.
*/

/*  A monitor tap for the transmit chain: a COPY of the audio at one chosen
    point in xtxa, never an insertion.  Every stage in TXA reads and writes
    midbuff in place, so the same object, asked at each site, can copy whichever
    one is selected without any stage knowing it is there.  It cannot alter a
    sample on its way to the radio: it only ever reads midbuff.

    Sites are numbered in xtxa order.  Up to MONTAP_AFTER_REVERB the buffer
    holds real audio in the I slots with Q at zero; from MONTAP_AFTER_COMP on,
    bp0 has formed the analytic signal and the I slot is the audio.  Either way
    the caller takes the I slots.  */

#ifndef _montap_h
#define _montap_h

#define MONTAP_AFTER_PANEL    1		/* after mic gain and the phase rotator */
#define MONTAP_AFTER_EQ       2		/* after the AMSQ gate and the equaliser */
#define MONTAP_AFTER_LEVELER  3
#define MONTAP_AFTER_CFC      4		/* CFC with its post-EQ */
#define MONTAP_AFTER_REVERB   5
#define MONTAP_AFTER_COMP     6		/* after COMP, CESSB and their band-passes */
#define MONTAP_AFTER_ALC      7
#define MONTAP_OUT            8		/* after the generators and up-slew: what leaves the chain */
#define MONTAP_SITES          8

typedef struct _montap
{
	int     run;
	int     site;			/* which site copies; 0 copies nowhere */
	int     size;			/* complex samples per block */
	double *in;				/* the chain's midbuff */
	double *buf;			/* the last copied block, size complex */
	int     fresh;			/* set by a copy, cleared by a read */
	CRITICAL_SECTION cs_update;
} montap, *MONTAP;

extern MONTAP create_montap (int run, int site, int size, double* in);

extern void destroy_montap (MONTAP a);

extern void flush_montap (MONTAP a);

extern void xmontap (MONTAP a, int site);

extern void setBuffers_montap (MONTAP a, double* in);

extern void setSize_montap (MONTAP a, int size);

/*  TXA properties  */

extern __declspec (dllexport) void SetTXAMonitorTap (int channel, int run, int site);

extern __declspec (dllexport) int GetTXAMonitorBlock (int channel, double* out);

#endif

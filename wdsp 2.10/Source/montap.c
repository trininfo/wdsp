/*  montap.c

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

/*  See montap.h for what this is.  Two rules it lives by:

      - it never writes midbuff.  xmontap reads it and nothing else, so no
        stage's output can be changed by the monitor being on, at any site;
      - the copy and the read are the only work inside the critical section,
        and neither allocates, so the chain's thread is never held for longer
        than a memcpy of one block.  */

#include "comm.h"

MONTAP create_montap (int run, int site, int size, double* in)
{
	MONTAP a = (MONTAP) malloc0 (sizeof (montap));
	a->run   = run;
	a->site  = site;
	a->size  = size;
	a->in    = in;
	a->buf   = (double *) malloc0 (a->size * sizeof (complex));
	a->fresh = 0;
	InitializeCriticalSectionAndSpinCount (&a->cs_update, 2500);
	return a;
}

void destroy_montap (MONTAP a)
{
	DeleteCriticalSection (&a->cs_update);
	_aligned_free (a->buf);
	_aligned_free (a);
}

void flush_montap (MONTAP a)
{
	EnterCriticalSection (&a->cs_update);
	memset (a->buf, 0, a->size * sizeof (complex));
	a->fresh = 0;
	LeaveCriticalSection (&a->cs_update);
}

void xmontap (MONTAP a, int site)
{
	if (a->run && site == a->site)
	{
		EnterCriticalSection (&a->cs_update);
		memcpy (a->buf, a->in, a->size * sizeof (complex));
		a->fresh = 1;
		LeaveCriticalSection (&a->cs_update);
	}
}

void setBuffers_montap (MONTAP a, double* in)
{
	a->in = in;
}

void setSize_montap (MONTAP a, int size)
{
	EnterCriticalSection (&a->cs_update);
	_aligned_free (a->buf);
	a->size  = size;
	a->buf   = (double *) malloc0 (a->size * sizeof (complex));
	a->fresh = 0;
	LeaveCriticalSection (&a->cs_update);
}

/********************************************************************************************************
*																										*
*										TXA Properties													*
*																										*
********************************************************************************************************/

/*  Select the site, or none.  A site outside 1..MONTAP_SITES copies nowhere,
    which is the safe reading of a number the caller got wrong.  */
PORT
void SetTXAMonitorTap (int channel, int run, int site)
{
	MONTAP a = txa[channel].montap.p;
	EnterCriticalSection (&a->cs_update);
	a->run  = run;
	a->site = (site >= 1 && site <= MONTAP_SITES) ? site : 0;
	a->fresh = 0;
	LeaveCriticalSection (&a->cs_update);
}

/*  Copy the last block out, `size` complex samples, and say whether it was new.
    A block is handed over once: a second read before the next copy returns 0
    and leaves `out` alone, so a caller polling faster than the chain runs does
    not play the same audio twice.  */
PORT
int GetTXAMonitorBlock (int channel, double* out)
{
	MONTAP a = txa[channel].montap.p;
	int fresh;
	EnterCriticalSection (&a->cs_update);
	fresh = a->fresh;
	if (fresh)
	{
		memcpy (out, a->buf, a->size * sizeof (complex));
		a->fresh = 0;
	}
	LeaveCriticalSection (&a->cs_update);
	return fresh;
}

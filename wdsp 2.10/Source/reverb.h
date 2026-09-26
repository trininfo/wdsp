/*  reverb.h

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
    assistant, in collaboration with Jeremy McInerney, N0NBH.  Claude contributed
    to algorithm design, code drafting and documentation during an iterative
    development process.  Claude does not hold copyright; all copyright in this
    file remains with Jeremy McInerney as stated above.

    The reverberator topology is the figure-eight plate of Jon Dattorro, "Effect
    Design, Part 1: Reverberator and Other Filters", Journal of the Audio
    Engineering Society 45(9), 1997.  The delay-line proportions of that design
    are used; the integer lengths here are chosen independently as primes near
    the scaled targets (see calc_reverb), and no code was copied from any
    implementation of it.
*/

#ifndef _reverb_h
#define _reverb_h

/*  One delay line.  `mask` is length-1 for a power-of-two buffer, so the read
    and write wrap with an AND rather than a branch; `len` is the delay actually
    in use, which is always at most `size`.  Keeping the two separate is what
    lets a parameter change alter the delay without reallocating anything.  */
typedef struct _rvline
{
	double *buf;
	int     size;		/* allocated, a power of two */
	int     mask;		/* size - 1 */
	int     len;		/* delay in samples, <= size - 1 */
	int     w;			/* write cursor */
} rvline;

typedef struct _reverb
{
	int     run;
	int     size;
	double *in;
	double *out;
	int     rate;

	/*  Operator parameters, in the units a person uses.  Everything derived
	    from them is recomputed by calc_reverb_params, which allocates nothing. */
	double  mix;				/* 0..1, wet against dry */
	double  dry_db;				/* trim on the dry path, -60..0 */
	double  wet_db;				/* trim on the wet path, -60..0 */
	double  out_db;				/* trim on the sum, -12..+6 */
	double  decay_seconds;		/* RT60 of the tank, 0.1..7 */
	double  predelay_ms;		/* 0..100 */
	double  damping;			/* 0..0.9, high-frequency loss per pass */
	double  low_cut_hz;			/* 20..1000 */
	double  high_cut_hz;		/* 1000..20000 */
	double  diffusion;			/* 0..1 */
	double  mod_rate_hz;		/* 0.05..5 */
	double  mod_depth;			/* 0..2 */

	/*  Derived gains and coefficients.  */
	double  dry_gain;
	double  wet_gain;
	double  out_gain;
	double  decay;				/* tank loop gain */
	double  damp_coeff;			/* one-pole low-pass in each tank half */
	double  bw_coeff;			/* input low-pass, from high_cut_hz */
	double  hp_coeff;			/* input high-pass, from low_cut_hz */
	double  in_diff_1;			/* input diffuser coefficients */
	double  in_diff_2;
	double  dec_diff_1;			/* tank allpass coefficients */
	double  dec_diff_2;
	double  mod_inc;			/* phase increment per sample */
	double  mod_excursion;		/* samples, peak */

	/*  Lines.  The two tank halves are suffixed a and b.  */
	rvline  predelay;
	rvline  diff1, diff2, diff3, diff4;		/* input diffusers */
	rvline  ap_a1, ap_a2, dl_a1, dl_a2;		/* tank half a */
	rvline  ap_b1, ap_b2, dl_b1, dl_b2;		/* tank half b */

	/*  State.  */
	double  bw_z;				/* input low-pass memory */
	double  hp_z;				/* input high-pass memory */
	double  damp_a;				/* damping memories */
	double  damp_b;
	double  fb_a;				/* the figure-eight's cross-coupling */
	double  fb_b;
	double  mod_phase;

	CRITICAL_SECTION cs_update;
} reverb, *REVERB;

extern void calc_reverb        (REVERB a);
extern void decalc_reverb      (REVERB a);
extern void calc_reverb_params (REVERB a);
extern void flush_reverb       (REVERB a);
extern REVERB create_reverb    (int run, int size, double* in, double* out, int rate,
                                double mix, double dry_db, double wet_db, double out_db,
                                double decay_seconds, double predelay_ms, double damping,
                                double low_cut_hz, double high_cut_hz, double diffusion,
                                double mod_rate_hz, double mod_depth);
extern void destroy_reverb     (REVERB a);
extern void xreverb            (REVERB a);
extern void setBuffers_reverb    (REVERB a, double* in, double* out);
extern void setSamplerate_reverb (REVERB a, int rate);
extern void setSize_reverb       (REVERB a, int size);

__declspec(dllexport) void SetTXAReverbRun (int channel, int run);
__declspec(dllexport) void SetTXAReverbConfig (int channel,
                                               double mix, double dry_db, double wet_db, double out_db,
                                               double decay_seconds, double predelay_ms, double damping,
                                               double low_cut_hz, double high_cut_hz, double diffusion,
                                               double mod_rate_hz, double mod_depth);

#endif  // _reverb_h

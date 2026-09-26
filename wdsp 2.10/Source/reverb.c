/*  reverb.c

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
    Engineering Society 45(9), 1997: a pre-delay, an input bandwidth filter, four
    cascaded allpass diffusers, and a tank of two cross-coupled halves each
    holding a modulated allpass, a delay, a damping low-pass, a second allpass
    and a second delay.  No code was copied from any implementation of it.
*/

/*  Where this runs, and why it matters.

    xtxa places this AFTER the continuous frequency compressor and BEFORE bp0.
    That is deliberate:

      - everything downstream - COMP, CESSB overshoot control and the ALC - acts
        on the reverb tail as it does on the dry signal, so the tail cannot push
        the transmitter past full scale by construction rather than by a limit
        written here;
      - xgen(gen1), which produces TUN and the two-tone, is downstream, so a
        reverb left on does not colour a tune-up or an intermodulation
        measurement;
      - xiqc, PureSignal's correction, is far below, so the predistorter never
        sees a signal this stage has changed after the fact.

    The signal here is real audio in the I slot with Q at zero: TXA's panel is
    created with inselect=2 and copy=0, and nothing between it and bp0 writes Q -
    bp0 is where the analytic signal is formed.  So this is one mono tank, and Q
    is passed through untouched rather than zeroed, so that a future stage which
    did put something there would not have it silently discarded.
*/

#include "comm.h"

/*  Dattorro's design is specified at 29761 Hz.  His delay lengths are held here
    as SECONDS, because a transmit chain runs at 48 or 96 kHz and copying his
    sample counts would give a reverb several times too short - the single
    easiest way to get this wrong, and it would sound like a small bright room
    rather than a plate.  */
#define RV_REF_RATE   29761.0

/*  The proportions, in seconds.  */
#define RV_T_DIFF1    (142.0  / RV_REF_RATE)
#define RV_T_DIFF2    (107.0  / RV_REF_RATE)
#define RV_T_DIFF3    (379.0  / RV_REF_RATE)
#define RV_T_DIFF4    (277.0  / RV_REF_RATE)
#define RV_T_AP_A1    (672.0  / RV_REF_RATE)
#define RV_T_AP_B1    (908.0  / RV_REF_RATE)
#define RV_T_DL_A1    (4453.0 / RV_REF_RATE)
#define RV_T_DL_B1    (4217.0 / RV_REF_RATE)
#define RV_T_AP_A2    (1800.0 / RV_REF_RATE)
#define RV_T_AP_B2    (2656.0 / RV_REF_RATE)
#define RV_T_DL_A2    (3720.0 / RV_REF_RATE)
#define RV_T_DL_B2    (3163.0 / RV_REF_RATE)

/*  Peak modulation of the first tank allpass at mod_depth = 1, in seconds
    (8 samples at the reference rate).  */
#define RV_T_MOD      (8.0 / RV_REF_RATE)

/*  The longest pre-delay offered, seconds.  Buffers are sized for it once, so
    moving the pre-delay slider costs no allocation.  */
#define RV_MAX_PREDELAY 0.100

/*  Below this, a sample is treated as silence.

    A reverb decays toward zero forever, and on x86 the last stretch of that
    decay is denormal, where each multiply can cost tens of times what a normal
    one does.  WDSP does not set flush-to-zero anywhere, so a tail left to fade
    on its own would quietly make the transmit thread miss its deadline minutes
    after the operator stopped talking.  1e-20 is some 400 dB below full scale:
    inaudible, and nowhere near the -60 dB the decay time is measured to.  */
#define RV_DENORM     1.0e-20

static double rv_squelch (double x)
{
	if (x > -RV_DENORM && x < RV_DENORM) return 0.0;
	return x;
}

/*  The smallest prime at least n, for n >= 2.  Delay lengths are made mutually
    prime so the lines do not share factors: common factors let their echoes
    reinforce at the same instants, which is what makes a cheap reverb ring with
    a metallic pitch instead of decaying smoothly.  */
static int rv_prime_at_least (int n)
{
	int i, r;
	if (n <= 2) return 2;
	if ((n & 1) == 0) n++;
	for (;; n += 2)
	{
		r = 1;
		for (i = 3; i * i <= n; i += 2)
			if (n % i == 0) { r = 0; break; }
		if (r) return n;
	}
}

static int rv_pow2_at_least (int n)
{
	int p = 1;
	while (p < n) p <<= 1;
	return p;
}

/*  A line long enough for `len` samples of delay plus `headroom` more, which the
    modulated allpasses need so that a read can swing earlier than its nominal
    tap without running past the write cursor.  */
static void rv_line_alloc (rvline* L, int len, int headroom)
{
	L->size = rv_pow2_at_least (len + headroom + 4);
	L->mask = L->size - 1;
	L->len  = len;
	L->w    = 0;
	L->buf  = (double *) malloc0 (L->size * sizeof (double));
}

static void rv_line_free (rvline* L)
{
	if (L->buf) _aligned_free (L->buf);
	L->buf = 0;
}

static void rv_line_clear (rvline* L)
{
	if (L->buf) memset (L->buf, 0, L->size * sizeof (double));
	L->w = 0;
}

static void rv_line_write (rvline* L, double v)
{
	L->w = (L->w + 1) & L->mask;
	L->buf[L->w] = v;
}

/*  The sample `len` back from the cursor.  */
static double rv_line_read (rvline* L)
{
	return L->buf[(L->w - L->len) & L->mask];
}

/*  `d` samples back, linearly interpolated.  Used by the modulated allpasses and
    by the output taps.  */
static double rv_line_read_frac (rvline* L, double d)
{
	int    i = (int) d;
	double f = d - (double) i;
	double a = L->buf[(L->w - i)     & L->mask];
	double b = L->buf[(L->w - i - 1) & L->mask];
	return a + f * (b - a);
}

/*  A tap at a fraction of the line's length, for the output.  */
static double rv_line_tap (rvline* L, double frac)
{
	return rv_line_read_frac (L, frac * (double) L->len);
}

/*  The Schroeder allpass, H(z) = (-g + z^-m) / (1 - g z^-m):

        w = the delayed value of v
        v = x + g w
        y = w - g v

    Stable for |g| < 1, which calc_reverb_params guarantees.  */
static double rv_allpass (rvline* L, double g, double x)
{
	double w = rv_line_read (L);
	double v = x + g * w;
	rv_line_write (L, v);
	return w - g * v;
}

/*  As above, with the tap modulated.  The nominal delay sits `excursion` inside
    the line so the read can swing either way without passing the cursor.  */
static double rv_allpass_mod (rvline* L, double g, double x, double delay)
{
	double w = rv_line_read_frac (L, delay);
	double v = x + g * w;
	rv_line_write (L, v);
	return w - g * v;
}

/*  A one-pole low-pass, y += c (x - y), with c the fraction of the difference
    taken each sample.  */
static double rv_lowpass (double* z, double c, double x)
{
	*z += c * (x - *z);
	return *z;
}

void calc_reverb (REVERB a)
{
	int mod_head;
	double r = (double) a->rate;
	/*  Every length scales with the rate, and each is taken to the next prime so
	    that no two lines share a factor.  */
	int l_pre   = (int) (RV_MAX_PREDELAY * r) + 2;
	int l_d1    = rv_prime_at_least ((int) (RV_T_DIFF1 * r));
	int l_d2    = rv_prime_at_least ((int) (RV_T_DIFF2 * r));
	int l_d3    = rv_prime_at_least ((int) (RV_T_DIFF3 * r));
	int l_d4    = rv_prime_at_least ((int) (RV_T_DIFF4 * r));
	int l_ap_a1 = rv_prime_at_least ((int) (RV_T_AP_A1 * r));
	int l_ap_b1 = rv_prime_at_least ((int) (RV_T_AP_B1 * r));
	int l_dl_a1 = rv_prime_at_least ((int) (RV_T_DL_A1 * r));
	int l_dl_b1 = rv_prime_at_least ((int) (RV_T_DL_B1 * r));
	int l_ap_a2 = rv_prime_at_least ((int) (RV_T_AP_A2 * r));
	int l_ap_b2 = rv_prime_at_least ((int) (RV_T_AP_B2 * r));
	int l_dl_a2 = rv_prime_at_least ((int) (RV_T_DL_A2 * r));
	int l_dl_b2 = rv_prime_at_least ((int) (RV_T_DL_B2 * r));

	/*  Headroom for the deepest modulation the parameter range allows, so the
	    depth can be changed later without reallocating.  */
	mod_head = (int) (RV_T_MOD * r * 2.0) + 4;

	rv_line_alloc (&a->predelay, l_pre,   0);
	rv_line_alloc (&a->diff1,    l_d1,    0);
	rv_line_alloc (&a->diff2,    l_d2,    0);
	rv_line_alloc (&a->diff3,    l_d3,    0);
	rv_line_alloc (&a->diff4,    l_d4,    0);
	rv_line_alloc (&a->ap_a1,    l_ap_a1, mod_head);
	rv_line_alloc (&a->ap_b1,    l_ap_b1, mod_head);
	rv_line_alloc (&a->dl_a1,    l_dl_a1, 0);
	rv_line_alloc (&a->dl_b1,    l_dl_b1, 0);
	rv_line_alloc (&a->ap_a2,    l_ap_a2, 0);
	rv_line_alloc (&a->ap_b2,    l_ap_b2, 0);
	rv_line_alloc (&a->dl_a2,    l_dl_a2, 0);
	rv_line_alloc (&a->dl_b2,    l_dl_b2, 0);

	calc_reverb_params (a);
	flush_reverb (a);
}

void decalc_reverb (REVERB a)
{
	rv_line_free (&a->predelay);
	rv_line_free (&a->diff1);
	rv_line_free (&a->diff2);
	rv_line_free (&a->diff3);
	rv_line_free (&a->diff4);
	rv_line_free (&a->ap_a1);
	rv_line_free (&a->ap_b1);
	rv_line_free (&a->dl_a1);
	rv_line_free (&a->dl_b1);
	rv_line_free (&a->ap_a2);
	rv_line_free (&a->ap_b2);
	rv_line_free (&a->dl_a2);
	rv_line_free (&a->dl_b2);
}

/*  Coefficients only.  Nothing here allocates or frees, so a slider can move at
    input rate without the transmit thread waiting on the heap.  */
void calc_reverb_params (REVERB a)
{
	double r = (double) a->rate;
	double loop, pre;

	a->dry_gain = pow (10.0, a->dry_db / 20.0);
	a->wet_gain = pow (10.0, a->wet_db / 20.0);
	a->out_gain = pow (10.0, a->out_db  / 20.0);

	/*  The pre-delay, in samples, bounded by what was allocated.  */
	pre = a->predelay_ms * 0.001 * r;
	if (pre < 1.0) pre = 1.0;
	if (pre > (double) (a->predelay.size - 2)) pre = (double) (a->predelay.size - 2);
	a->predelay.len = (int) pre;

	/*  Decay.  One trip round the figure eight passes both halves' delays and
	    both halves' allpasses, so the loop is their total length; a gain of g per
	    trip decays 20 log10(g) dB in that time, and RT60 is where that reaches
	    -60 dB.  Hence g = 10^(-3 T / RT60).

	    Without this the same slider would mean a different decay at 48 and
	    96 kHz, which is the kind of fault an operator reports as "it sounds
	    different on this band".  */
	loop = (double) (a->dl_a1.len + a->dl_a2.len + a->dl_b1.len + a->dl_b2.len
	               + a->ap_a1.len + a->ap_a2.len + a->ap_b1.len + a->ap_b2.len) / r;
	if (a->decay_seconds < 0.05) a->decay_seconds = 0.05;
	a->decay = pow (10.0, -3.0 * loop / a->decay_seconds);
	/*  A loop gain at or above one does not decay at all.  The ceiling is what
	    stops a long decay time plus a rate change from turning the tank into an
	    oscillator feeding the transmitter.  */
	if (a->decay > 0.98) a->decay = 0.98;
	if (a->decay < 0.0)  a->decay = 0.0;

	/*  One-pole coefficients.  c = 1 - exp(-2 pi f / rate) is the exact
	    single-pole response at f, and it stays sane as f approaches Nyquist,
	    which the naive 2 pi f / rate does not.  */
	a->bw_coeff = 1.0 - exp (-2.0 * PI * a->high_cut_hz / r);
	if (a->bw_coeff > 1.0) a->bw_coeff = 1.0;
	a->hp_coeff = 1.0 - exp (-2.0 * PI * a->low_cut_hz / r);
	if (a->hp_coeff > 1.0) a->hp_coeff = 1.0;

	/*  Damping is given as the loss, so a value of 0 is a bright plate and 0.9 a
	    dark one.  It becomes the low-pass coefficient inverted: more loss is a
	    lower corner, hence a smaller coefficient.  */
	a->damp_coeff = 1.0 - a->damping;
	if (a->damp_coeff < 0.05) a->damp_coeff = 0.05;
	if (a->damp_coeff > 1.0)  a->damp_coeff = 1.0;

	/*  Allpass coefficients from one diffusion control.  All four are held below
	    0.75: an allpass is only allpass while |g| < 1, and the tank's own
	    allpasses sit inside the decay loop, where a coefficient close to one
	    turns diffusion into ringing.  */
	a->in_diff_1 = 0.50 + 0.25 * a->diffusion;
	a->in_diff_2 = 0.40 + 0.22 * a->diffusion;
	a->dec_diff_1 = 0.30 + 0.40 * a->diffusion;
	a->dec_diff_2 = 0.25 + 0.35 * a->diffusion;

	a->mod_inc = 2.0 * PI * a->mod_rate_hz / r;
	a->mod_excursion = RV_T_MOD * r * a->mod_depth;
}

void flush_reverb (REVERB a)
{
	rv_line_clear (&a->predelay);
	rv_line_clear (&a->diff1);
	rv_line_clear (&a->diff2);
	rv_line_clear (&a->diff3);
	rv_line_clear (&a->diff4);
	rv_line_clear (&a->ap_a1);
	rv_line_clear (&a->ap_b1);
	rv_line_clear (&a->dl_a1);
	rv_line_clear (&a->dl_b1);
	rv_line_clear (&a->ap_a2);
	rv_line_clear (&a->ap_b2);
	rv_line_clear (&a->dl_a2);
	rv_line_clear (&a->dl_b2);
	a->bw_z = 0.0;
	a->hp_z = 0.0;
	a->damp_a = 0.0;
	a->damp_b = 0.0;
	a->fb_a = 0.0;
	a->fb_b = 0.0;
	a->mod_phase = 0.0;
}

REVERB create_reverb (int run, int size, double* in, double* out, int rate,
                      double mix, double dry_db, double wet_db, double out_db,
                      double decay_seconds, double predelay_ms, double damping,
                      double low_cut_hz, double high_cut_hz, double diffusion,
                      double mod_rate_hz, double mod_depth)
{
	REVERB a = (REVERB) malloc0 (sizeof (reverb));
	a->run  = run;
	a->size = size;
	a->in   = in;
	a->out  = out;
	a->rate = rate;
	a->mix = mix;
	a->dry_db = dry_db;
	a->wet_db = wet_db;
	a->out_db = out_db;
	a->decay_seconds = decay_seconds;
	a->predelay_ms = predelay_ms;
	a->damping = damping;
	a->low_cut_hz = low_cut_hz;
	a->high_cut_hz = high_cut_hz;
	a->diffusion = diffusion;
	a->mod_rate_hz = mod_rate_hz;
	a->mod_depth = mod_depth;
	calc_reverb (a);
	InitializeCriticalSectionAndSpinCount (&a->cs_update, 2500);
	return a;
}

void destroy_reverb (REVERB a)
{
	DeleteCriticalSection (&a->cs_update);
	decalc_reverb (a);
	_aligned_free (a);
}

void xreverb (REVERB a)
{
	int i;
	double x, wet, dry, mod, da, db, ta, tb;

	EnterCriticalSection (&a->cs_update);
	if (a->run)
	{
		for (i = 0; i < a->size; i++)
		{
			/*  Real audio only; Q is left as it was found.  */
			dry = a->in[2 * i + 0];
			a->out[2 * i + 1] = a->in[2 * i + 1];

			/*  Pre-delay, then the input band limit: a low-pass at high_cut_hz,
			    and a high-pass made by subtracting a low-pass at low_cut_hz.
			    Limiting what enters the tank rather than what leaves it is what
			    keeps a plate from sounding like a filtered copy of the voice.  */
			rv_line_write (&a->predelay, dry);
			x = rv_line_read (&a->predelay);
			x = rv_lowpass (&a->bw_z, a->bw_coeff, x);
			x = x - rv_lowpass (&a->hp_z, a->hp_coeff, x);

			/*  Four cascaded allpasses: the early diffusion, which turns one
			    impulse into a dense burst before the tank ever sees it.  */
			x = rv_allpass (&a->diff1, a->in_diff_1, x);
			x = rv_allpass (&a->diff2, a->in_diff_1, x);
			x = rv_allpass (&a->diff3, a->in_diff_2, x);
			x = rv_allpass (&a->diff4, a->in_diff_2, x);

			/*  The modulated taps.  Moving them slowly stops the tank's own
			    resonances standing still, which is what a fixed plate sounds
			    like: a held note over the tail.  The two halves are driven in
			    quadrature so they never swing together.  */
			mod = sin (a->mod_phase);
			da = (double) a->ap_a1.len + a->mod_excursion * mod;
			db = (double) a->ap_b1.len + a->mod_excursion * cos (a->mod_phase);
			a->mod_phase += a->mod_inc;
			if (a->mod_phase > 2.0 * PI) a->mod_phase -= 2.0 * PI;

			/*  Half a.  Its input is the diffused signal plus what half b sent
			    last sample - the cross-coupling that makes the figure eight one
			    loop rather than two tanks.  */
			ta = x + a->fb_b;
			ta = rv_allpass_mod (&a->ap_a1, -a->dec_diff_1, ta, da);
			rv_line_write (&a->dl_a1, ta);
			ta = rv_line_read (&a->dl_a1);
			ta = rv_lowpass (&a->damp_a, a->damp_coeff, ta) * a->decay;
			ta = rv_allpass (&a->ap_a2, a->dec_diff_2, ta);
			rv_line_write (&a->dl_a2, ta);
			ta = rv_line_read (&a->dl_a2) * a->decay;

			/*  Half b, the same with its own lengths.  */
			tb = x + a->fb_a;
			tb = rv_allpass_mod (&a->ap_b1, -a->dec_diff_1, tb, db);
			rv_line_write (&a->dl_b1, tb);
			tb = rv_line_read (&a->dl_b1);
			tb = rv_lowpass (&a->damp_b, a->damp_coeff, tb) * a->decay;
			tb = rv_allpass (&a->ap_b2, a->dec_diff_2, tb);
			rv_line_write (&a->dl_b2, tb);
			tb = rv_line_read (&a->dl_b2) * a->decay;

			a->fb_a = rv_squelch (ta);
			a->fb_b = rv_squelch (tb);

			/*  Six taps at unrelated points in the four long lines, with mixed
			    signs.  One tap per half would be a single echo with a pitch;
			    several cancel each other's periodicity, which is what makes the
			    tail read as a space instead of a repeat.  */
			wet = rv_line_tap (&a->dl_a1, 0.312) + rv_line_tap (&a->dl_b2, 0.717)
			    - rv_line_tap (&a->dl_b1, 0.483) + rv_line_tap (&a->dl_a2, 0.628)
			    + rv_line_tap (&a->dl_a1, 0.851) - rv_line_tap (&a->dl_b1, 0.174);
			wet *= 0.1666666666666667;

			/*  mix blends, the two trims scale each path into the blend, and
			    out_gain trims the sum.  */
			a->out[2 * i + 0] = a->out_gain * (dry * a->dry_gain * (1.0 - a->mix)
			                                 + wet * a->wet_gain * a->mix);
		}
	}
	else if (a->in != a->out)
		memcpy (a->out, a->in, a->size * sizeof (complex));
	LeaveCriticalSection (&a->cs_update);
}

void setBuffers_reverb (REVERB a, double* in, double* out)
{
	a->in  = in;
	a->out = out;
}

void setSamplerate_reverb (REVERB a, int rate)
{
	decalc_reverb (a);
	a->rate = rate;
	calc_reverb (a);
}

void setSize_reverb (REVERB a, int size)
{
	a->size = size;
	flush_reverb (a);
}

/********************************************************************************************************
*																										*
*										TXA Properties													*
*																										*
********************************************************************************************************/

PORT
void SetTXAReverbRun (int channel, int run)
{
	REVERB a = txa[channel].reverb.p;
	EnterCriticalSection (&a->cs_update);
	if (a->run != run)
	{
		a->run = run;
		/*  Empty it on the way in as well as the way out.  A tank still holding
		    the last transmission would put that audio on the air at the start of
		    the next one.  */
		flush_reverb (a);
	}
	LeaveCriticalSection (&a->cs_update);
}

/*  Every parameter but `run` in one call.

    One setter rather than twelve because nothing here needs to reallocate: the
    lines were sized at create time for the widest pre-delay and the deepest
    modulation the ranges allow, so this is arithmetic.  Twelve setters would each
    have taken the critical section while the transmit thread waited, and a panel
    sending a whole configuration would have done it twelve times.  */
PORT
void SetTXAReverbConfig (int channel,
                         double mix, double dry_db, double wet_db, double out_db,
                         double decay_seconds, double predelay_ms, double damping,
                         double low_cut_hz, double high_cut_hz, double diffusion,
                         double mod_rate_hz, double mod_depth)
{
	REVERB a = txa[channel].reverb.p;
	EnterCriticalSection (&a->cs_update);
	a->mix = mix;
	a->dry_db = dry_db;
	a->wet_db = wet_db;
	a->out_db = out_db;
	a->decay_seconds = decay_seconds;
	a->predelay_ms = predelay_ms;
	a->damping = damping;
	a->low_cut_hz = low_cut_hz;
	a->high_cut_hz = high_cut_hz;
	a->diffusion = diffusion;
	a->mod_rate_hz = mod_rate_hz;
	a->mod_depth = mod_depth;
	calc_reverb_params (a);
	LeaveCriticalSection (&a->cs_update);
}

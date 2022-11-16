/*---------------------------------------------------------------
 * Copyright (c) 2017
 * Broadcom Corporation
 * All Rights Reserved.
 *---------------------------------------------------------------
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software
 * without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit
 * persons to whom the Software is furnished to do
 * so, subject to the following conditions:
 *
 *
 * Redistributions of source code must retain the above
 * copyright notice, this list of conditions and
 * the following disclaimers.
 *
 *
 * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimers in the documentation and/or other materials
 * provided with the distribution.
 *
 *
 * Neither the name of Broadcom Coporation,
 * nor the names of its contributors may be used to endorse
 * or promote products derived from this Software without
 * specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE CONTIBUTORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * ________________________________________________________________
 *
 * histogram.h
 * Suppport for isochonronous traffic testing
 *
 * by Robert J. McMahon (rjmcmahon@rjmcmahon.com, bob.mcmahon@broadcom.com)
 * -------------------------------------------------------------------
 */
#ifndef HISTOGRAMC_H
#define HISTOGRAMC_H

typedef struct histogram_t {
    unsigned int id;
    unsigned int *mybins;
    unsigned int bincount;
    unsigned int binwidth;
    unsigned int populationcnt;
    float offset;
    unsigned int cntloweroutofbounds;
    unsigned int cntupperoutofbounds;
    char *myname;
    char *outbuf;
    float units;
    double ci_lower;
    double ci_upper;
    struct histogram_t *prev;
} histogram_t;

extern histogram_t *histogram_init(unsigned int bincount, unsigned int binwidth, float offset,\
				   float units, double ci_lower, double ci_upper, unsigned int id, char *name);
extern void histogram_delete(histogram_t *h);
extern int histogram_insert(histogram_t *h, float value);
extern void histogram_clear(histogram_t *h);
extern void histogram_add(histogram_t *to, histogram_t *from);
extern void histogram_print(histogram_t *h, double, double, int);
#endif // HISTOGRAMC_H

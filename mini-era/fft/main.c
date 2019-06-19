/*
 * Copyright 2019 IBM
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
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

/*
 * This programs provides a simple C implementation of 1D FFT computation
 * using the FFTW3 library. It mimics the FFT computation done in GNU Radio,
 * also using FFTW3:
 * https://github.com/gnuradio/gnuradio/blob/master/gr-fft/lib/fft.cc
 */

#include <stdlib.h>
#include <time.h>
#include <fftw3.h>

#define BILLION 1000000000.0;

void generate_input(fftwf_complex *d_inbuf, int fft_size)
{
	int i;
	srand(0);

	for (i=0; i<fft_size; i++)
	{
		d_inbuf[i][0] = rand();
		d_inbuf[i][1] = rand();
	}

	/*
	printf("  Input Data:\n");
	for (i=0; i<fft_size; i++)
	{
		printf("  %3d  %12f  %12f\n", i, d_inbuf[i][0], d_inbuf[i][1]);
	}
	*/
}


int main(int argc, char** argv)
{
	int i, fft_count, fft_size;
	fftwf_complex *d_inbuf, *d_outbuf;
	fftwf_plan my_plan;

	/* For profiling purposes */
	struct timespec start, end;
	double time_spent;

	if (argc != 3)
	{
		printf("Usage: %s <fft_size> <fft_count>\n", argv[0]);
		printf("  fft_size:  FFT size\n");
		printf("  fft_count: number of FFTs to compute\n");
		return -1;
	}

	fft_size  = atoi(argv[1]);
	fft_count = atoi(argv[2]);
	printf("Computing %d FFTs of size %d...\n", fft_count, fft_size);

	d_inbuf  = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * fft_size);
	d_outbuf = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * fft_size);

	/* Initialize the input buffer with random complex numbers */
	generate_input(d_inbuf, fft_size);

	/* Enable multi-threading (optional) */
	/*
	int nthreads = 1;
	fftwf_init_threads();
	fftwf_plan_with_nthreads(nthreads);
	*/

	/* Create plan (configuration) to compute a 1D FFT (single-precision floating-point) */
	my_plan = fftwf_plan_dft_1d(fft_size, d_inbuf, d_outbuf, FFTW_FORWARD, FFTW_MEASURE);

	clock_gettime(CLOCK_REALTIME, &start);

	for (i=0; i<fft_count; i++)
	{
		/* Here is where the actual FFT computation takes place */
		fftwf_execute(my_plan);
	}

	clock_gettime(CLOCK_REALTIME, &end);

	/* Destroy plan and free buffers */
	fftwf_destroy_plan(my_plan);
	fftwf_free(d_inbuf);
	fftwf_free(d_outbuf);

	/* Print out execution statistics */
	time_spent = (end.tv_sec - start.tv_sec) +
				 (end.tv_nsec - start.tv_nsec) / BILLION;
	printf("Time elpased is %.9f seconds\n", time_spent);

	return 0;
}

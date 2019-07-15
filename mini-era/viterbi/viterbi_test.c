/*
 * viterbi_test.c
 * Author Name: Varun Mannam
 * Created on: Jul 11, 2019
 * Function: takes input from a file in the following format and checks the
 * Viterbi_decoder functions properly or not in C language
 * Input File format:
 * 	DEBUG - OFDM params   : 6 288 192 6 (n_bpsc, n_cbps, n_dbps, encoding )
 DEBUG - Frame params  : 34 2 90 576 384 (psdu_size,n_sym,n_pad, n_encoded_bits,n_data_bits  )
 DEBUG - Before Viterbi: 000001011001001110001010110001011101101001010101000001001101001011100101001001111111101100100111111101000100001001100101110001011010101101101010000001100010110000011010010100001101011001011110110110110100110011000011001100110011101011110011111001010001011000110001101101000101010100110111111000000101100101011010000111010010000110110010110000000111011100110100000110101010111010101111011111000001000001101100111010010100100101010110000000000001100010101000010001100011110100100001111000111111011100110000011011001011111101001000100100010111001101010011101111101010000101110111
 DEBUG - After Depunct : 000200120112001200121102001201021102001201121012101200120102101200020012001210120012011210021012001200121112111210121002100211121112101200021002001200121002101211020012011201021012101210120102000200121002010211020002011201020102100200121012011200120112110211021102110210021102011200020112001210021102011210120112110201121112001201020012011200021102001210121012000210120102100211021112111200020002101210021012011201020002111201020102000211021102010211020002000211120112100211021002000211021012010211120102101211120112111200020012000200121012100211120102010210021002101201021102000200020002001210020102101200020102001210020112110210021002001211120002111211120112100211020002011201120012011211121012001200021002100201021112001210120102011210121112101201020002101211021112000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
 DEBUG - After Viterbi : 000100110001011101001011000001100110101001110011100110000101011100111111011010101001111000111101110010100011011000001101110001101110001011101010010010011111001100101011000110000100101111010101111110100101000110100110111000000000001111111011100011000001101000001010100101110000011110000000000000011100111101101000010101011111010010100011011100011111110000111011110010110010010000001000
 DEBUG - After descramb: 9 0 8 0 0 0 66 66 66 66 66 66 35 35 35 35 35 35 255 255 255 255 255 255 0 0 120 120 120 120 120 120 98 83 214 153
 DEBUG - CRC error     : 0
 *
 *
 Steps:  De-interleaved -> depuncture -> Viterbi Decoder -> desrambler -> check the output at each stage
 and report CRC result comparison.
 *  Source: https://github.com/bastibl/gr-ieee802-11/blob/maint-3.8/lib/viterbi_decoder/base.cc
 *
 * C implementation_source:https://github.com/IBM/era/tree/master/mini-era/viterbi
 *
 *
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "utils.h"
#include "viterbi_decoder_generic.h"
#include "base.h"

extern void descrambler(uint8_t* in, int psdusize, uint8_t* ref);
extern uint8_t* decode(ofdm_param *ofdm, frame_param *frame, uint8_t *in);
int main(int argc, char* argv[])
{
  int encoding, n_bpsc, n_cbps, n_dbps, pdsu_size, n_sym, n_pad, n_encoded_bits, n_data_bits;
  // creating a FILE variable
  FILE *fptr;
  int count, error_count;
  // integer variables
  if (argc ==1)
    {
      printf(">>>>>> Missing File name here. >>>>>> \n");
    }
  if (argc >2)
    {
      printf(">>>>>> More number of arguments are provided than required one argument:(i.e., Filename in .txt format). >>>>>> \n");
    }
  printf("file path is %s", argv[1]);

  fptr = fopen(argv[1], "r");
  if(fptr==NULL)
    {
      printf(">>>>>> Input file does not exist or can not be opened. >>>>>>\n");
    }
  fclose(fptr);

  //read params from file
  fptr = fopen(argv[1], "r");
  char str1[] = "DEBUG - OFDM params   : "; //len = 24
  char str2[] = "DEBUG - Frame params  : "; //len = 24
  char str3[] = "DEBUG - Before Viterbi: "; //len = 24
  char str4[] = "DEBUG - After Viterbi : "; //len = 24
  char str5[] = "DEBUG - After descramb: "; //len = 24
  printf(">>>>>> Get OFDM parameters from file (0:n_bpsc , 1:n_cbps, 2:n_dbps, 3:encoding ). >>>>>> \n");
  // ofdm params extraction
  int line_num =1;
  int len1 = 1000; //temp length for params
  char temp[len1];

  int ofdm_values[4];
  while(fgets(temp, len1, fptr) != NULL)
    {
      if((strstr(temp, str1)) != NULL)
	{
	  char *ptr = temp;
	  int k=0;
	  while (*ptr) {
	    if (isdigit(*ptr)) {
	      long val = strtol(ptr, &ptr, 10);
	      ofdm_values[k] = val;
	      printf(">>>>>> OFDM param [%d] is %d >>>>>> \n",k,ofdm_values[k]);
	      k++;
	    } else {
	      ptr++;
	    }
	  }
	  n_bpsc = ofdm_values[0];
	  n_cbps = ofdm_values[1];
	  n_dbps = ofdm_values[2];
	  encoding = ofdm_values[3];
	}
      line_num++;
    }
  fclose(fptr);

  printf(">>>>>> Get frame parameters from file(0:pdsu_size, 1:n_sym, 2:n_pad, 3:n_encoded_bits, 4:n_data_bits ). >>>>>> \n");
  fptr = fopen(argv[1], "r");
  // frame param extraction
  line_num = 1;
  char temp2[len1];
  int frame_values[5];
  count = 0;
  while(fgets(temp2, len1, fptr) != NULL)
    {		//printf("%s", temp2);
      if((strstr(temp2, str2)) != NULL)
	{
	  char *ptr = temp2;
	  int k=0;
	  while (*ptr)
	    {
	      if (isdigit(*ptr))
		{
		  long val = strtol(ptr, &ptr, 10);
		  frame_values[k] = val;
		  printf(">>>>>> Frame param [%d] is %d >>>>>> \n",k,frame_values[k]);
		  k++;
		}
	      else
		{
		  ptr++;
		}
	      count++;
	    }
	  pdsu_size =  frame_values[0];
	  n_sym =  frame_values[1];
	  n_pad = frame_values[2];
	  n_encoded_bits = frame_values[3];
	  n_data_bits = frame_values[4];
	}
      line_num++;
    }
  fclose(fptr);

  ofdm_param ofdm = {   encoding,   //  encoding   : 0 = BPSK_1_2
			13,   //  rate_field : rate field ofSIGNAL header //Taken constant
			n_bpsc,   //  n_bpsc     : coded bits per subcarrier
			n_cbps,   //  n_cbps     : coded bits per OFDM symbol
			n_dbps }; //  n_dbps     : data bits per OFDM symbol

  frame_param frame = {  pdsu_size,    // psdu_size      : PSDU size in bytes
			 n_sym,    // n_sym          : number of OFDM symbols
			 n_pad,    // n_pad          : number of padding bits in DATA field
			 n_encoded_bits,    // n_encoded_bits : number of encoded bits
			 n_data_bits };  // n_data_bits: number of data bits, including service and padding

  //read input data (de-interleaved data before Viterbi-decoder) from file
  // frame param extraction
  int MAX_Encoded_bits = frame.n_encoded_bits;
  int MAX_Decoded_bits = frame.n_data_bits;
  int MAX_Descram_bytes = frame.psdu_size +2;

  uint8_t input[MAX_Encoded_bits]; //={0};
  uint8_t reference[MAX_Decoded_bits]; // ={0}; 1000 is maximum used here
  uint8_t descramble[MAX_Descram_bytes]; // ={0}; 1000 is maximum used here
  printf(">>>>>> Get de-interleaved bits from file. >>>>>> \n");
  fptr = fopen(argv[1], "r");
  line_num = 1;
  int len2 = frame.n_encoded_bits + strlen(str3)+10; //additional 10 values
  char temp3[len2];
  while(fgets(temp3, len2, fptr) != NULL)
    {
      if((strstr(temp3, str3)) != NULL)
	{
	  char *ptr = temp3;
	  int k=0;
	  for (int i =0;i<frame.n_encoded_bits;i++)
	    {
	      input[i] = *(ptr+24+i) - '0'; //in file: 24: strlen(str3)
	    }
	}
      line_num++;
    }
  fclose(fptr);

  fptr = fopen(argv[1], "r");
  printf(">>>>>> Get Viterbi-decoder output bits from file. >>>>>> \n");
  line_num = 1;
  int len3 = frame.n_data_bits + strlen(str4)+10; //additional 10 values
  char temp4[len3];

  while(fgets(temp4, len3, fptr) != NULL)
    {
      if((strstr(temp4, str4)) != NULL)
	{
	  char *ptr = temp4;
	  for (int i =0;i<frame.n_data_bits;i++)
	    {
	      reference[i] = *(ptr+24+i) - '0'; // in file: 24: strlen(str4)

	    }
	}
      line_num++;
    }
  fclose(fptr);

  fptr = fopen(argv[1], "r");
  printf(">>>>>> Get Descrambler output bytes from file. >>>>>> \n");
  line_num = 1;
  int len4 = (frame.psdu_size +2)*4 + strlen(str5) + 10;
  //(frame.psdu_size +2) bytes of data and each-one max value: 255 (3 chars) + 1 space (total = 4 charc) + additional 10 values
  char temp5[len4];
  while(fgets(temp5, len4, fptr) != NULL)
    {
      if((strstr(temp5, str5)) != NULL)
	{
	  char *ptr = temp5;
	  int k=0;
	  while (*ptr)
	    {
	      if (isdigit(*ptr))
		{
		  long val = strtol(ptr, &ptr, 10);
		  descramble[k] = val;
		  k++;
		}
	      else
		{
		  ptr++;
		}
	      count++;
	    }
	}
      line_num++;
    }
  fclose(fptr);

  int input_s = (int)(sizeof(input)/sizeof(input[0])); //input bits_length
  int ref_s = (int)(sizeof(reference)/sizeof(reference[0])); //decoder output reference bits length
  int descram_s = (int)(sizeof(descramble)/sizeof(descramble[0])); //decoder descramble reference bytes length

  uint8_t *in = input;
  uint8_t *ref = reference;
  uint8_t *descram_ref = descramble;

  // our main module starts from here
  uint8_t *result;
  result = decode(&ofdm, &frame, in);
  int count_res =0;
  int result_s = (int)(sizeof(result)/sizeof(result[0]));
  printf(">>>>>> Decoded bits are here >>>>>> \n");
  error_count = 0;
  int max_bits = frame.n_data_bits;
  for (int i = 0; i < max_bits ; i++)
    {
      if (result[i] != reference[i])
	{
	  printf(">>>>>> Miscompare: result[%d] = %u vs %u = EXPECTED_VALUE[%d] >>>>>>\n", i, result[i], reference[i], i);
    	  error_count++;
	}
    }
  if (error_count != 0)
    {
      printf(">>>>>> Mismatch in the decoder algorithm, please check the inputs and algorithm one last time. >>>>>> \n");
    }
  else
    {
      printf("!!!!!! Great Job, Viterbi decoder algorithm works fine for the given configuration. !!!!!! \n");
    }
  //descrambler
  int psdusize = frame.psdu_size;
  uint8_t *descram;

  descrambler(result,psdusize, descram_ref);

  return 0;
}

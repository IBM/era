The enc?.txt files are input data files for testing the viterbi_butterfly2_generic routine, and represent
a set of possible runs using the (three) supported encoding schemes.  

The data is generated in GnuRadio.
This captures the inputs and outputs of the butterfly function before and after each call. 

These are for three different settings for encoding. 
All are for a payload that is 6 bytes long (with the wifi overhead, the viterbi decoder decodes 36 bytes)

The format for the OFDM parameters line is: d_ofdm.n_bpsc, d_ofdm.n_cbps, d_ofdm.n_dbps, d_ofdm.encoding

The format for the Frame parameters line is: d_frame.psdu_size, d_frame.n_sym, d_frame.n_pad, d_frame.n_encoded_bits, d_frame.n_data_bits

Do let me know if you would prefer a different format for the data. There are few hundred input output pairs for the butterfly function.

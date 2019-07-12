CC     = gcc -std=c99
CFLAGS = 

AR      = ar rcs

L_SRC = viterbi_standalone.c viterbi_decoder_generic.c descrambler_function.c
L_OBJ = $(L_SRC:%.c=%.o)

T_SRC = viterbi_test.c
T_OBJ = $(T_SRC:%.c=%.o)

libviterbi.a: $(L_OBJ)
	$(AR) $@ $(L_OBJ)

viterbi_test: $(T_OBJ) libviterbi.a
	$(CC) $(T_OBJ) -L . -lviterbi -o $@

clean:
	rm viterbi_test libviterbi.a $(L_OBJ) 



.c.o:
	$(CC) $*.c $(CFLAGS) -c 


depend:;	makedepend -fMakefile -- $(CFLAGS) -- $(L_SRC) $(T_SRC)
# DO NOT DELETE THIS LINE -- make depend depends on it.

viterbi_standalone.o: viterbi_decoder_generic.h base.h utils.h
viterbi_decoder_generic.o: viterbi_decoder_generic.h base.h utils.h
viterbi_decoder_generic.o: viterbi_standalone.h
descrambler_function.o: viterbi_standalone.h base.h utils.h
viterbi_test.o: viterbi_standalone.h viterbi_decoder_generic.h utils.h base.h 
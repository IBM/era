CC     = gcc -std=c99
CFLAGS = 

AR      = ar rcs

L_SRC = viterbi_standalone.c viterbi_decoder_generic.c 
L_OBJ = $(L_SRC:%.c=%.o)

T_SRC = test.c
T_OBJ = $(T_SRC:%.c=%.o)

libviterbi.a: $(L_OBJ)
	$(AR) $@ $(L_OBJ)
	#$(AR) libviterbi.a $(L_OBJ)
	#$(CC) viterbi_decoder_generic.c -c -o viter.o
	#$(CC) viterbi_standalone.c -c -o stand.o
	#ar rcs $@ stand.o viter.o
	#rm *.o

test: $(T_OBJ) libviterbi.a
	$(CC) $(T_OBJ) -L . -lviterbi -o $@

clean:
	rm test libviterbi.a $(L_OBJ) 



.c.o:
	$(CC) $*.c $(CFLAGS) -c 


depend:;	makedepend -fMakefile -- $(CFLAGS) -- $(L_SRC) $(T_SRC)
# DO NOT DELETE THIS LINE -- make depend depends on it.

viterbi_standalone.o: viterbi_decoder_generic.h base.h utils.h
viterbi_standalone.o: /usr/include/stdint.h /usr/include/standards.h
viterbi_standalone.o: /usr/include/stdbool.h /usr/include/stdlib.h
viterbi_standalone.o: /usr/include/sys/wait.h /usr/include/strict_stdtypes.h
viterbi_standalone.o: /usr/include/sys/types.h /usr/include/sys/inttypes.h
viterbi_standalone.o: /usr/include/end_strict_stdtypes.h
viterbi_standalone.o: /usr/include/sys/m_types.h /usr/include/sys/vm_types.h
viterbi_standalone.o: /usr/include/sys/resource.h /usr/include/sys/time.h
viterbi_standalone.o: /usr/include/sys/signal.h /usr/include/sys/context.h
viterbi_standalone.o: /usr/include/sys/m_param.h /usr/include/sys/mstsave.h
viterbi_standalone.o: /usr/include/sys/m_signal.h
viterbi_standalone.o: /usr/include/sys/localedef.h /usr/include/sys/limits.h
viterbi_standalone.o: /usr/include/float.h /usr/include/sys/lc_core.h
viterbi_standalone.o: /usr/include/locale.h /usr/include/sys/localedef31.h
viterbi_standalone.o: /usr/include/stdio.h /usr/include/va_list.h
viterbi_decoder_generic.o: /usr/include/stdio.h /usr/include/va_list.h
viterbi_decoder_generic.o: viterbi_decoder_generic.h base.h utils.h
viterbi_decoder_generic.o: /usr/include/stdint.h /usr/include/standards.h
viterbi_decoder_generic.o: /usr/include/stdbool.h /usr/include/stdlib.h
viterbi_decoder_generic.o: /usr/include/sys/wait.h
viterbi_decoder_generic.o: /usr/include/strict_stdtypes.h
viterbi_decoder_generic.o: /usr/include/sys/types.h
viterbi_decoder_generic.o: /usr/include/sys/inttypes.h
viterbi_decoder_generic.o: /usr/include/end_strict_stdtypes.h
viterbi_decoder_generic.o: /usr/include/sys/m_types.h
viterbi_decoder_generic.o: /usr/include/sys/vm_types.h
viterbi_decoder_generic.o: /usr/include/sys/resource.h
viterbi_decoder_generic.o: /usr/include/sys/time.h /usr/include/sys/signal.h
viterbi_decoder_generic.o: /usr/include/sys/context.h
viterbi_decoder_generic.o: /usr/include/sys/m_param.h
viterbi_decoder_generic.o: /usr/include/sys/mstsave.h
viterbi_decoder_generic.o: /usr/include/sys/m_signal.h
viterbi_decoder_generic.o: /usr/include/sys/localedef.h
viterbi_decoder_generic.o: /usr/include/sys/limits.h /usr/include/float.h
viterbi_decoder_generic.o: /usr/include/sys/lc_core.h /usr/include/locale.h
viterbi_decoder_generic.o: /usr/include/sys/localedef31.h
viterbi_decoder_generic.o: viterbi_standalone.h
test.o: viterbi_standalone.h utils.h /usr/include/stdint.h
test.o: /usr/include/standards.h /usr/include/stdbool.h /usr/include/stdlib.h
test.o: /usr/include/sys/wait.h /usr/include/strict_stdtypes.h
test.o: /usr/include/sys/types.h /usr/include/sys/inttypes.h
test.o: /usr/include/end_strict_stdtypes.h /usr/include/sys/m_types.h
test.o: /usr/include/sys/vm_types.h /usr/include/sys/resource.h
test.o: /usr/include/sys/time.h /usr/include/sys/signal.h
test.o: /usr/include/sys/context.h /usr/include/sys/m_param.h
test.o: /usr/include/sys/mstsave.h /usr/include/sys/m_signal.h
test.o: /usr/include/sys/localedef.h /usr/include/sys/limits.h
test.o: /usr/include/float.h /usr/include/sys/lc_core.h /usr/include/locale.h
test.o: /usr/include/sys/localedef31.h

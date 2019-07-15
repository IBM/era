CC     = gcc -std=c99
CFLAGS = 

AR      = ar rcs

L_SRC = viterbi_standalone.c viterbi_decoder_generic.c 
L_OBJ = $(L_SRC:%.c=obj/%.o)
L_OBJ_ESP = $(L_SRC:%.c=obj_esp/%.o)

T_SRC = test.c
T_OBJ = $(T_SRC:%.c=obj/%.o)
T_OBJ_ESP = $(T_SRC:%.c=obj_esp/%.o)

all:	vtest vtest_esp
	echo "Done..."

lib:	libviterbi.a
	

libviterbi.a: $(L_OBJ)
	$(AR) $@ $(L_OBJ)

libviterbi_esp.a: $(L_OBJ_ESP)
	$(AR) $@ $(L_OBJ_ESP)


vtest: $(T_OBJ) libviterbi.a
	$(CC) $(T_OBJ) -L . -lviterbi -o $@

vtest_esp: $(T_OBJ_ESP) libviterbi_esp.a
	$(CC) $(T_OBJ_ESP) -L . -lviterbi_esp -o $@


clean:
	rm vtest libviterbi.a $(L_OBJ)
	rm vtest_esp libviterbi_esp.a $(L_OBJ_ESP)


obj/%.o: %.c
	$(CC) $*.c $(CFLAGS) -c -o $@

obj_esp/%.o: %.c
	$(CC) $*.c $(CFLAGS) -DUSE_ESP_INTERFACE -c -o $@

obj/viterbi_standalone.o: viterbi_decoder_generic.h base.h utils.h
obj/viterbi_decoder_generic.o: viterbi_decoder_generic.h base.h utils.h
obk/viterbi_decoder_generic.o: viterbi_standalone.h
obj/vtest.o: viterbi_standalone.h utils.h /usr/include/stdint.h

obj_esp/viterbi_standalone.o: viterbi_decoder_generic.h base.h utils.h
obj_esp/viterbi_decoder_generic.o: viterbi_decoder_generic.h base.h utils.h
obj_esp/viterbi_decoder_generic.o: viterbi_standalone.h
obj_esp/vtest.o: viterbi_standalone.h utils.h /usr/include/stdint.h


depend:;	makedepend -fMakefile -- $(CFLAGS) -- $(L_SRC) $(T_SRC)
# DO NOT DELETE THIS LINE -- make depend depends on it.

viterbi_standalone.o: viterbi_decoder_generic.h base.h utils.h
viterbi_decoder_generic.o: viterbi_decoder_generic.h base.h utils.h
viterbi_decoder_generic.o: viterbi_standalone.h
vtest.o: viterbi_standalone.h utils.h /usr/include/stdint.h

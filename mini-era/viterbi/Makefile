CC     = gcc -std=c99
CFLAGS = 

AR      = ar rcs

L_SRC = viterbi_standalone.c viterbi_decoder_generic.c 
L_OBJ = $(L_SRC:%.c=obj/%.o)
L_OBJ_ESP = $(L_SRC:%.c=obj_esp/%.o)

LF_SRC = viterbi_standalone.c viterbi_flat.c 
LF_OBJ = $(LF_SRC:%.c=obj_f/%.o)
LF_OBJ_ESP = $(LF_SRC:%.c=obj_f_esp/%.o)

LCK_OBJ = $(L_SRC:%.c=obj_ck/%.o)
LCK_OBJ_ESP = $(L_SRC:%.c=obj_ck_esp/%.o)

T_SRC = vtest.c
T_OBJ = $(T_SRC:%.c=obj/%.o)
T_OBJ_ESP = $(T_SRC:%.c=obj_esp/%.o)

VT_SRC = viterbi_test.c descrambler_function.c
VT_OBJ = $(VT_SRC:%.c=obj/%.o)
VT_OBJ_ESP = $(VT_SRC:%.c=obj_esp/%.o)

FT_SRC = viterbi_test.c descrambler_function.c
FT_OBJ = $(FT_SRC:%.c=obj/%.o)
FT_OBJ_ESP = $(FT_SRC:%.c=obj_esp/%.o)

all:	vtest vtest_esp viterbi_test viterbi_test_esp viterbi_ftest viterbi_ftest_esp 
	echo Done building all

lib:	libviterbi.a libviterbi_esp.a libviterbiCk.a libviterbiCk_esp.a libviterbiF.a libviterbiF_esp.a
	echo Built $@


libviterbi.a: $(L_OBJ)
	$(AR) $@ $(L_OBJ)

libviterbi_esp.a: $(L_OBJ_ESP)
	$(AR) $@ $(L_OBJ_ESP)

libviterbiCk.a: $(LCK_OBJ)
	$(AR) $@ $(LCK_OBJ)

libviterbiCk_esp.a: $(LCK_OBJ_ESP)
	$(AR) $@ $(LCK_OBJ_ESP)

libviterbiF.a: $(LF_OBJ)
	$(AR) $@ $(LF_OBJ)

libviterbiF_esp.a: $(LF_OBJ_ESP)
	$(AR) $@ $(LF_OBJ_ESP)



vtest: $(T_OBJ) libviterbiCk.a
	$(CC) $(T_OBJ) -L . -lviterbiCk -o $@

vtest_esp: $(T_OBJ_ESP) libviterbiCk_esp.a
	$(CC) $(T_OBJ_ESP) -L . -lviterbiCk_esp -o $@

viterbi_test: $(VT_OBJ) libviterbi.a
	$(CC) $(VT_OBJ) -L . -lviterbi -o $@

viterbi_test_esp: $(VT_OBJ_ESP) libviterbi_esp.a
	$(CC) $(VT_OBJ_ESP) -L . -lviterbi_esp -o $@

viterbi_ftest: $(FT_OBJ) libviterbiF.a
	$(CC) $(FT_OBJ) -L . -lviterbiF -o $@

viterbi_ftest_esp: $(FT_OBJ_ESP) libviterbiF_esp.a
	$(CC) $(FT_OBJ_ESP) -L . -lviterbiF_esp -o $@

clean:
	-rm libviterbi.a $(L_OBJ)
	-rm libviterbi_esp.a $(L_OBJ_ESP)
	-rm libviterbiCk.a $(LCK_OBJ)
	-rm libviterbiCk_esp.a $(LCK_OBJ_ESP)
	-rm libviterbiF.a $(LF_OBJ)
	-rm libviterbiF_esp.a $(LF_OBJ_ESP)
	-rm vtest $(T_OBJ) 
	-rm vtest_esp $(T_OBJ_ESP)
	-rm viterbi_test $(VT_OBJ) 
	-rm viterbi_test_esp $(VT_OBJ_ESP)


obj/%.o: %.c
	$(CC) $*.c $(CFLAGS) -c -o $@

obj_esp/%.o: %.c
	$(CC) $*.c $(CFLAGS) -DUSE_ESP_INTERFACE -c -o $@

obj_ck/%.o: %.c
	$(CC) $*.c $(CFLAGS) -DDO_RUN_TIME_CHECKING -c -o $@

obj_ck_esp/%.o: %.c
	$(CC) $*.c $(CFLAGS) -DUSE_ESP_INTERFACE -c -o $@

obj_f/%.o: %.c
	$(CC) $*.c $(CFLAGS) -c -o $@

obj_f_esp/%.o: %.c
	$(CC) $*.c $(CFLAGS) -DUSE_ESP_INTERFACE -c -o $@


obj/viterbi_standalone.o: viterbi_decoder_generic.h base.h utils.h
obj/viterbi_decoder_generic.o: viterbi_decoder_generic.h base.h utils.h
obj/viterbi_decoder_generic.o: viterbi_standalone.h
obj/vtest.o: viterbi_standalone.h utils.h /usr/include/stdint.h
obj/viterbi_test.o: viterbi_standalone.h utils.h /usr/include/stdint.h

obj_ck/viterbi_standalone.o: viterbi_decoder_generic.h base.h utils.h
obj_ck/viterbi_decoder_generic.o: viterbi_decoder_generic.h base.h utils.h
obj_ck/viterbi_decoder_generic.o: viterbi_standalone.h
obj_ck/vtest.o: viterbi_standalone.h utils.h /usr/include/stdint.h
obj_ck/viterbi_test.o: viterbi_standalone.h utils.h /usr/include/stdint.h

obj_f/viterbi_standalone.o: viterbi_decoder_generic.h base.h utils.h 
obj_f/viterbi_flat.o: viterbi_decoder_generic.h base.h utils.h
obj_f/viterbi_flat.o: viterbi_standalone.h
obj_f/viterbi_ftest.o: viterbi_standalone.h utils.h /usr/include/stdint.h

obj_esp/viterbi_standalone.o: viterbi_decoder_generic.h base.h utils.h
obj_esp/viterbi_decoder_generic.o: viterbi_decoder_generic.h base.h utils.h
obj_esp/viterbi_decoder_generic.o: viterbi_standalone.h
obj_esp/vtest.o: viterbi_standalone.h utils.h /usr/include/stdint.h
obj_esp/viterbi_test.o: viterbi_standalone.h utils.h /usr/include/stdint.h

obj_ck_esp/viterbi_standalone.o: viterbi_decoder_generic.h base.h utils.h
obj_ck_esp/viterbi_decoder_generic.o: viterbi_decoder_generic.h base.h utils.h
obj_ck_esp/viterbi_decoder_generic.o: viterbi_standalone.h
obj_ck_esp/vtest.o: viterbi_standalone.h utils.h /usr/include/stdint.h
obj_ck_esp/viterbi_test.o: viterbi_standalone.h utils.h /usr/include/stdint.h

obj_f_esp/viterbi_standalone.o: viterbi_decoder_generic.h base.h utils.h
obj_f_esp/viterbi_flat.o: viterbi_decoder_generic.h base.h utils.h
obj_f_esp/viterbi_flat.o: viterbi_standalone.h
obj_f_esp/viterbi_ftest.o: viterbi_standalone.h utils.h /usr/include/stdint.h


depend:;	makedepend -fMakefile -- $(CFLAGS) -- $(L_SRC) $(T_SRC)
# DO NOT DELETE THIS LINE -- make depend depends on it.


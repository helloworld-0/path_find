NF_HOME=..
include ../inc.mk

#---------custom define begin---------------------
MOD_NAME := jps 
SUB_MODULES := . $(subst /,, $(wildcard */))
APP_CFILE=$(foreach dir,$(SUB_MODULES),$(wildcard $(dir)/*.c))

APP_EXE := $(NF_HOME)/bin/$(MOD_NAME)

CINC := $(addprefix -I ,$(SUB_MODULES)) $(COMMON_CINC)
APP_OBJ := $(APP_CFILE:.c=.o)
.PHONY: all clean

all: $(APP_EXE)

$(APP_EXE): $(APP_OBJ) $(APP_LIB) 
	$(CC) $(CFLAGS) $^ -o $@

clean:
	$(RM) -fv $(APP_EXE) $(APP_OBJ) 

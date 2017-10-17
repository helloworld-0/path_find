NF_HOME=..
include ../inc.mk

COMMON_LIB := $(NF_SRC)/lib/libcommon.a

COMMON_CFILE=$(wildcard *.c) $(wildcard */*.c)
COMMON_CXXFILE=$(wildcard *.cpp) $(wildcard */*.cpp)

ALL_OBJS := $(COMMON_CFILE:.c=.o) $(COMMON_CXXFILE:.cpp=.o)

ALL_TARGETS := $(ALL_OBJS)

.PHONY: all clean

all: $(ALL_OBJS) $(COMMON_LIB)

clean:
	@$(RM) -v *.o  *.d *.out *~ *.gch *.bak
	@$(RM) -v $(ALL_OBJS)
	@$(RM) -v $(ALL_TARGETS)
	@$(RM) -v $(COMMON_LIB)

$(COMMON_LIB): $(ALL_TARGETS)
	$(AR) rv $@ $^ 


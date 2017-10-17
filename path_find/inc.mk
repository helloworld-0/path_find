CC := /usr/bin/g++
CXX := /usr/bin/g++
RM := /bin/rm -rf
AR := /usr/bin/ar 

VPATH += /lib64:/usr/lib64

NF_SRC=$(NF_HOME)
NF_COMMON_SRC=$(NF_SRC)/common
#--------------CFLAGS--------------------------------
#CFLAGS+= -fPIC -Wall -pipe -Werror -g -Winvalid-pch 
CFLAGS+= -g  -O0

ifdef _RELEASE
CFLAGS += -D NDEBUG
endif

#---------------define macros-----------------------
SUB_DIRS=$(dir $(wildcard $(1)/*/.)) 
INC_SUB_DIRS=$(addprefix -I ,$(dir $(wildcard $(1)/*/.)))

#---------------xy include---------------------------
#CINC = $(addprefix -I ,$(dir $(wildcard $(NF_SRC)/dbschema/*/.)))
COMMON_CINC := -I $(NF_COMMON_SRC)

#----------------libs----------------------------
#APP_LIB = $(NF_SRC)/lib/libcommon.a 

%.d: %.c
	set -e; rm -f $@;
	$(CC) -MM $(CFLAGS) $(CINC) $< > $@;
	sed -i 's,$(notdir $(basename $@))\.o[ :]*,$(basename $@).o $(basename $@).d : ,g' $@;
%.d: %.cpp
	set -e; rm -f $@;
	$(CXX) -MM $(CFLAGS) $(CINC) $< > $@;
	sed -i 's,$(notdir $(basename $@))\.o[ :]*,$(basename $@).o $(basename $@).d : ,g' $@;
%.i: %.c
	$(CC) -E $(CFLAGS) $(CINC) $< -o $@

%.i: %.cpp
	$(CXX) -E $(CFLAGS) $(CINC) $< -o $@

%.i: %.cc
	$(CXX) -E $(CFLAGS) $(CINC) $< -o $@

%.o: %.c
	$(CC) -c $(CFLAGS) $(CINC) $< -o $@

%.o: %.cpp 
	$(CXX) -c $(CFLAGS) $(CINC) $< -o $@

%.o: %.cc
	$(CXX) -c $(CFLAGS) $(CINC) $< -o $@

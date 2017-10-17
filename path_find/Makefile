.PHONY: all clean PRE_MAKE

NF_HOME= .
include ./inc.mk

#预编译模块
PRE_DIRS = common

#所有需要编译的模块
ALL_DIRS := $(subst /,,$(dir $(wildcard */[Mm]akefile)))
CLEAN_DIRS := $(addprefix _clean_,$(ALL_DIRS))

#所有的应用程序
ALL_APPS := $(filter-out $(PRE_DIRS), $(ALL_DIRS))

.PHONY: $(ALL_APPS) $(CLEAN_DIRS) $(PRE_DIRS)

all:
	$(MAKE) $(PRE_DIRS)
	$(MAKE) $(ALL_APPS)
release:
	$(MAKE) all _RELEASE=1

$(ALL_APPS):
	$(MAKE) -C $@ all
$(PRE_DIRS):
	$(MAKE) -C $@ all

$(CLEAN_DIRS):
	$(MAKE) -C $(patsubst _clean_%,%,$@) clean
clean:$(CLEAN_DIRS)
	-@echo Clean finished !
	-@date


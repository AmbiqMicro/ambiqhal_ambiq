#******************************************************************************
#
# Makefile - Rules for building the libraries, examples and docs.
#
# ${copyright}
#
# This is part of revision ${version} of the AmbiqSuite Development Package.
#
#******************************************************************************

##### INTERNAL BEGIN #####
# DON'T USE all ON BUILDPROJ Makefiles.
##### INTERNAL END #####

SUBDIRS=$(dir $(wildcard */Makefile))

$(SUBDIRS)::
	+$(MAKE) -C $@ $(MAKECMDGOALS)

.PHONY: subdirs all clean
all subdirs clean: $(SUBDIRS)

##### INTERNAL BEGIN #####
.PHONY: realclean
realclean: $(SUBDIRS)
SUBDIRSBUILD := 1
##### INTERNAL END #####
ifneq ($(SUBDIRSBUILD),)
.DEFAULT_GOAL := subdirs
else
.DEFAULT_GOAL := all
endif


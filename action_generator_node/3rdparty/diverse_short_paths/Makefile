### diverse Makefile ###

SHELL		= /bin/bash
CXX		= g++
CXXFLAGS	= -Wall -Wextra -Werror -std=c++0x -O3 -march=native
CURDIR		= /home/cav2/repos/diverse_short_paths
PROG		= diverse
EXEC		= build/bin/$(PROG)
STATICLIB	= build/lib/diverse_short_paths.a
SRCDIR		= src
INCDIR		= include
ANPLPREFIX	= /usr/ANPLprefix
PCHINC		= $(INCDIR)/pch.h
DIRS		= build/bin build/$(SRCDIR)
INCS		= $(wildcard $(INCDIR)/*.h) $(wildcard $(INCDIR)/**/*.h)
CXXFLAGS	+= -I$(INCDIR) -I$(ANPLPREFIX)/include
LDFLAGS		= -L$(ANPLPREFIX)/lib -lompl -lboost_graph
PCH		= $(PCHINC).pch
SRCS		= $(shell echo $(wildcard $(SRCDIR)/*.cpp) | tr " " "\n" | sort | tr "\n" " ")
OBJS    	= $(subst $(SRCDIR)/,build/$(SRCDIR)/,$(SRCS:.cpp=.o))


define SSH
	ssh -t dione "source .bashrc; cd ${CURDIR}; $(1)"
endef

all: $(PCH) $(INCS) $(SRCS) $(EXEC) $(STATICLIB)

sync:
	rsync -c -i -r * hera:/home/cav2/repos/diverse_short_paths

remote:	sync
	$(call SSH,make -j 15)

again:
	+@$(MAKE) clean
	+@$(MAKE)

r.again: sync
	$(call SSH,make again -j 15)

test: sync remote
	$(call SSH,PYTHONUNBUFFERED=1 ./test.py ${PLOTS})
	rsync -c -i -r hera:/home/cav2/repos/diverse_short_paths/*.png .

install: all
	mkdir -p /usr/ANPLprefix/include/diverse_short_paths
	cp -r include/* ${ANPLPREFIX}/include/diverse_short_paths
	cp ${STATICLIB} ${ANPLPREFIX}/lib

${EXEC}: $(OBJS) | dirs
	@echo -e "\033[1;34m[Linking ${EXEC}]\033[0m"
	${CXX} ${OBJS} ${LOCALLIB} ${LDFLAGS} -o $@

${STATICLIB}: $(OBJS) | dirs
	@echo -e "\033[1;34m[Creating library ${STATICLIB}]\033[0m"
	mkdir -p build/lib
	ar rvs $@ $(shell echo ${OBJS} | sed 's#build/${SRCDIR}/Main.o##')

build/%.o: %.cpp $(PCH) $(INCS) | dirs
	@echo -e "\033[1;35m[Compiling $(subst $(SRCDIR)/,,$<)]\033[0m"
	${CXX} -c ${CXXFLAGS} ${LOCALINC} -include ${PCHINC} $< -o $@

${PCH}: $(PCHINC)
	@echo -e "\033[1;36m[Precompiling headers]\033[0m"
	${CXX} -x c++-header ${CXXFLAGS} ${LOCALINC} $< -o $@

.PHONY: dirs clean link sync remote r.again r.clean r.link
.SILENT: dirs clean link

dirs:
	for dir in ${DIRS}; do \
		mkdir -p $$dir; \
	done

clean:
	rm -f ${PCH} ${OBJS} ${EXEC}
	for dir in $(TOOLS); do \
		cd $$dir; \
		$(MAKE) clean; \
	done
r.clean: sync
	$(call SSH,make clean)

link:
	rm -R ${EXEC}
	+@$(MAKE) ${EXEC}

r.link: sync
	$(call SSH,make link)


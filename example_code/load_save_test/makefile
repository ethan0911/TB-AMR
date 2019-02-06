#Based on the Colby CS makefile tutorial:
#http://www.cs.colby.edu/maxwell/courses/tutorials/maketutor/

export OMPI_CC=clang

#INCLUDE = -I/usr/include/
INCLUDE = -I/opt/p4est-nathan/DEBUG/include/
#LIBDIR  = -L/usr/X11R6/lib 
LIBDIR  = -L/opt/p4est-nathan/DEBUG/lib
LINKER_INPUT = -lp4est -lsc

BINDIR=.
SRCDIR=.
ODIR=obj

#We let the following be set by the environment/
#the makefile at the top of the recursion

#COMPILERFLAGS = -Weverything -O3
CC = mpicc
#CFLAGS = $(COMPILERFLAGS) $(INCLUDE) $(LIBDIR)
CFLAGS = $(INCLUDE) 

OBJ = $(patsubst $(SRCDIR)/%.c,$(ODIR)/%.o,$(wildcard $(SRCDIR)/*.c)) 

#make c and cpp files into .o files
#$^ means all arguments to the right of the :
#$@ means the argument to the left of the : 
#so, take all the .c and .cpp files, output as (ODIR)/%.o
$(ODIR)/%.o: $(SRCDIR)/%.c
	$(CC) -c -o $@ $< $(CFLAGS) -MJ $(addsuffix .json, $@)


#take all object files, glue them together
$(BINDIR)/loadsave_test: $(OBJ) 
	$(CC) -o $@ $^ $(CFLAGS) $(LIBDIR) $(LINKER_INPUT)
	./gen_compile_database.sh

.PHONY: clean 

clean:
	rm -f $(ODIR)/*.o $(ODIR)/*.json $(BINDIR)/loadsave_test

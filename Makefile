##### Directories

SRCDIR = src
INCDIR = include
OBJDIR = build
DEPDIR = .deps

##### Compiler/linker options

INCL = -I $(INCDIR) -I $(SRCDIR)
CXX = clang++-3.7
CXXFLAGS = -O3 -c -Wall -Wno-sign-compare -fpic $(INCL) $(GPROF)
#LDFLAGS = -O3 -Wno-sign-compare -shared
LDFLAGS = -O3 -Wno-sign-compare -dynamiclib $(GPROF)

##### Files

VPATH = $(shell find -L $(INCDIR) $(SRCDIR) -type d \( ! -name '.*' \))
SOURCES = $(shell find -L $(SRCDIR) -name '*.cpp')
OBJS = $(addprefix $(OBJDIR)/, $(notdir $(SOURCES:.cpp=.o)))
DEPS = $(addprefix $(DEPDIR)/, $(notdir $(SOURCES:.cpp=.d)))
CPPEXAMPLE = $(addprefix examples/cpp_models/, $(shell ls examples/cpp_models))

##### Targets

.PHONY: core directory library cpp_models clean

core: directory $(DEPS) $(OBJS) pomdpx

# Rule for creating directories needed for build
directory:
	@mkdir -p $(OBJDIR) $(DEPDIR)

# Rules for generating dependencies
$(DEPDIR)/%.d: %.cpp
	@mkdir -p $(DEPDIR); \
	$(CXX) -MM $(CXXFLAGS) $< > $@; \
	sed -ie 's;\(.*\)\.o:;$(OBJDIR)/\1.o $(DEPDIR)/\1.d:;g' $@

# Include generated dependencies
-include $(DEPS)

# Rules for creating object files
$(OBJDIR)/%.o: %.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ 

# Rules for creating library from the object files
lib: $(OBJS)
	$(CXX) $(OBJS) $(LDFLAGS) -I $(INCDIR) -o $(OBJDIR)/libdespot.so

# Rules for compiling the executables for the cpp models in examples/cpp_models
cpp_models:
	$(foreach var, $(CPPEXAMPLE), make -C $(var);)

# Rule for installing the library
install:
	sudo cp -r build/libdespot.so usr/lib/
	sudo cp -r include /usr/include/despot

# Rules for repository cleaning
clean:
	rm -rf $(OBJDIR) $(DEPDIR)

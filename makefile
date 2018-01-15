CC :=g++ -g
SRCDIR:=src
BUILDDIR:=build
TARGET :=bin/runner
SRCEXT:=cpp
SOURCES:=$(shell find $(SRCDIR) -type f -name "*.$(SRCEXT)")
OBJECTS:=$(patsubst $(SRCDIR)/%, $(BUILDDIR)/%,$(SOURCES:.$(SRCEXT)=.o))
DEP:=$(OBJECTS:%.o=%.d)
CFLAGS:= -std=c++11
CFLAGS+=`pkg-config --cflags opencv`
LDFLAGS= `pkg-config --libs  opencv`

LDFLAGS+= -L/home/hzh/software/opencv3_1_0/opencv/3rdparty/ippicv/unpack/ippicv_lnx/lib/intel64 -lippicv
#LDFLAGS+= -L/home/hzh/Downloads/leptonica-1.73/build/src -llept -ltiff
INC:= -I include

$(TARGET):$(OBJECTS)
	@echo "Linking..."
	@echo $(SOURCES)
	@echo "$(CC) $^ -o $(TARGET) $(LIB)"
	$(CC) -o $(TARGET)  $^ $(LDFLAGS) 
-include $(DEP)
$(BUILDDIR)/%.o: $(SRCDIR)/%.$(SRCEXT)
	@mkdir -p $(BUILDDIR)
	@echo "$(CC) $(CFLAGS) $(INC) -c -o $@ $<"
	$(CC) $(CFLAGS) $(INC) -c -o $@ $<
$(BUILDDIR)/%.d: $(SRCDIR)/%.cpp
	echo "$(CC) -M $(CFLAGS) $(INC) $< > $@.tmp"
	$(CC) -M $(CFLAGS) $(INC) $< > $@.tmp;
	sed 's,\($*\)\.o[ :]*,$(BUILDDIR)/\1.o $@ : ,g' < $@.tmp > $@;
	#rm -f $@.tmp
clean:
	@echo "cleaning...";
	@echo "$(RM) -r $(BUILDDIR) $(TARGET)";
	$(RM) -r $(BUILDDIR)/* $(TARGET)
.PHONY:clean

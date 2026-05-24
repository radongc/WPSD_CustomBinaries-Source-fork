# AIBridge IMBE codec wrapper.
#
# Builds libaibridge_imbe.so linking against mbelib (which must already
# be installed; install.sh handles that).
#
# Manual build:
#   make
# Cleanup:
#   make clean

CXX      ?= g++
CXXFLAGS ?= -O2 -fPIC -Wall -Wextra
LDFLAGS  ?= -shared
LDLIBS   ?= -lmbe

TARGET := libaibridge_imbe.so
SRC    := imbe_wrapper.cc

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $< $(LDLIBS)

clean:
	rm -f $(TARGET) *.o

.PHONY: clean

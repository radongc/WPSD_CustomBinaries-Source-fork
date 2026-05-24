# AIBridge IMBE codec wrapper.
#
# Builds libaibridge_imbe.so linking against:
#   - mbelib (decoder; system-installed via install.sh)
#   - OP25's imbe_vocoder (encoder; static lib in IMBE_VOC_DIR)
#
# Manual build:
#   make IMBE_VOC_DIR=/opt/imbe_vocoder
# Cleanup:
#   make clean

CXX         ?= g++
CXXFLAGS    ?= -O2 -fPIC -Wall -Wextra
LDFLAGS     ?= -shared

# Path to the OP25 imbe_vocoder source tree (with its build/ subdir).
# install.sh stages this at /opt/imbe_vocoder.
IMBE_VOC_DIR ?= /opt/imbe_vocoder

CPPFLAGS    += -I$(IMBE_VOC_DIR)
LDLIBS      ?= -L$(IMBE_VOC_DIR)/build -limbe_vocoder -lmbe -lm

TARGET := libaibridge_imbe.so
SRC    := imbe_wrapper.cc

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) -o $@ $< $(LDLIBS)

clean:
	rm -f $(TARGET) *.o

.PHONY: clean

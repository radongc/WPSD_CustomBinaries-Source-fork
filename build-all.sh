#!/bin/bash

echo "cleaning..."
find . -type d -print0 | xargs -0 -I {} sh -c 'cd "{}" && make clean > /dev/null 2>&1'

cwd=$(pwd)

SRC_DIR=$HOME/dev/WPSD_CustomBinaries-Source
DEST_DIR=$HOME/dev/WPSD-Binaries
MAKEFILE="Makefile"

# update version date str.
find . -name Version.h -exec sed -i -e "/const char\* VERSION =/ s/\"[^\"]*\"/\"$(date +'%Y%m%d')_WPSD\"/" -e "/const wxString VERSION =/ s/wxT(\"[^\"]*\");/wxT(\"$(date +'%Y%m%d')_WPSD\");/" {} \;

# build!
cd $SRC_DIR/APRSGateway && make clean && make -j$(nproc) && make install && make clean \
  && cd $SRC_DIR/MMDVMHost && make clean && make -f Makefile.WPSD -j$(nproc) && make -f Makefile.WPSD install && make clean \
  && cd $SRC_DIR/DAPNETGateway && make clean && make -j$(nproc) && make install && make clean \
  && cd $SRC_DIR/DMRGateway && make clean && make -f Makefile.WPSD -j$(nproc) && make -f Makefile.WPSD install && make clean \
  && cd $SRC_DIR/AMBEServer && make clean && make -j$(nproc) && make install && make clean \
  && cd $SRC_DIR/ircDDBGateway && make -f $MAKEFILE clean && make -f $MAKEFILE -j$(nproc) && make -f $MAKEFILE install && make -f $MAKEFILE clean \
  && cd $SRC_DIR/MMDVMCal && make clean && make -j$(nproc) && make install && make clean \
  && cd $SRC_DIR/MMDVM_CM/DMR2YSF && make clean && make -j$(nproc) && make install && make clean \
  && cd $SRC_DIR/MMDVM_CM/DMR2NXDN && make clean && make -j$(nproc) && make install && make clean \
  && cd $SRC_DIR/MMDVM_CM/YSF2DMR && make clean && make -j$(nproc) && make install && make clean \
  && cd $SRC_DIR/MMDVM_CM/YSF2P25 && make clean && make -j$(nproc) && make install && make clean \
  && cd $SRC_DIR/MMDVM_CM/YSF2NXDN && make clean && make -j$(nproc) && make install && make clean \
  && cd $SRC_DIR/NXDNClients/NXDNGateway && make -f $MAKEFILE clean && make -f $MAKEFILE -j$(nproc) all && make -f $MAKEFILE install && make -f $MAKEFILE clean \
  && cd $SRC_DIR/NXDNClients/NXDNParrot && make clean && make -j$(nproc) all && make clean \
  && cd $SRC_DIR/P25Clients/P25Gateway && make -f $MAKEFILE clean && make -f $MAKEFILE -j$(nproc) all && make -f $MAKEFILE install && make -f $MAKEFILE clean \
  && cd $SRC_DIR/P25Clients/P25Parrot && make clean && make -j$(nproc) all && make install && make clean \
  && cd $SRC_DIR/YSFClients/YSFGateway && make -f $MAKEFILE clean && make -f $MAKEFILE -j$(nproc) && make -f $MAKEFILE install && make -f $MAKEFILE clean \
  && cd $SRC_DIR/YSFClients/DGIdGateway && make -f $MAKEFILE clean && make -f $MAKEFILE -j$(nproc) && make -f $MAKEFILE install && make -f $MAKEFILE clean \
  && cd $SRC_DIR/YSFClients/YSFParrot && make clean && make -j$(nproc) && make install && make clean \
  && cd $SRC_DIR/NextionDriver && make clean && make -j$(nproc) && make install && make clean \
  && cd $SRC_DIR/teensy_loader_cli && make clean && make -j$(nproc) && make install && make clean \
  && strip $(find $DEST_DIR -type f -executable -exec file -i '{}' \; | grep 'x-executable; charset=binary' | sed 's/:.*//g')

cd $DEST_DIR
git commit -a -m '* Minor: update from upstream & rebuilds'
cd $cwd

echo "cleaning..."
find . -type d -print0 | xargs -0 -I {} sh -c 'cd "{}" && make clean > /dev/null 2>&1'

exit 0


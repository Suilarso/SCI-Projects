#!/bin/sh

# JJ2160816 - Created for iART project

#JJ1220816 - Below echo added by JJ
echo "Updating iART software in progress. Please wait....."

DIR="/home/pi/PythonProject/iART_BD"
DATADIR="/home/pi/PythonProject/iART_BD/Data"
UTILITYDIR="/home/pi/PythonProject/iART_BD/iART_Utility"
SRCDIR="./iART_BD"
SCRIPT_DIR="$( cd "$( dirname "$0" )" && pwd )"
#JJ3170816 - Above statement will return /bin to SCRIPT_DIR variable

# Set the working directory to the script directory
cd $SCRIPT_DIR

#JJ3170816 - We need this for iART project
#JJ3170816 - Need to determine what need to be removed.
#JJ3170816 - At the moment just remove InHouseAPI, Thermostat, and AR1100_TouchScreen
if [ ! -d $DIR ]; then
    # Delete existing iART application
#JJ_     rm -f $DIR/InHouseAPI.py
#JJ_     rm -f $DIR/Thermostat75C.py
#JJ_     rm -f $DIR/AR1100_TouchScreen.py
#JJ_ else
#JJ1220816 - Below echo added by JJ
    echo "Create iART folder"
    # Create directory path before copying files
    mkdir -p $DIR
    mkdir -p $DIR/__pycache__
fi

#JJ4150916 - Create Data directory
if [ ! -d $DATADIR ]; then
    echo "Create Data folder"
    # Create directory path before copying files
    mkdir -p $DATADIR
fi

#JJ4150916 - Create iART utility directory
if [ ! -d $UTILITYDIR ]; then
    echo "Create utility folder"
    # Create directory path before copying files
    mkdir -p $UTILITYDIR
    mkdir -p $UTILITYDIR/__pycache__
    mkdir -p $UTILITYDIR/CalibrationData
    mkdir -p $UTILITYDIR/Data
fi

#JJ1220816 - Below echo added by JJ
echo "Transfering files....."

# ------------------------------------------------------------
#JJ4180816 - Here we copy BD materials.
#
if [ -f $SRCDIR/barda.py ]; then
   cp -f -p $SRCDIR/barda.py $DIR/barda.py
fi

if [ -f $SRCDIR/barda_fpga.rbt ]; then
   cp -f -p $SRCDIR/barda_fpga.rbt $DIR/barda_fpga.rbt
fi
# ------------------------------------------------------------

# ------------------------------------------------------------
#JJ4180816 - Here we copy all ui files.
#
if [ -f $SRCDIR/KeyBoardWidget.ui ]; then
   cp -f -p $SRCDIR/KeyBoardWidget.ui $DIR/KeyBoardWidget.ui
fi

if [ -f $SRCDIR/MainWindow.ui ]; then
   cp -f -p $SRCDIR/MainWindow.ui $DIR/MainWindow.ui
fi

if [ -f $SRCDIR/NumberKeyBoard.ui ]; then
   cp -f -p $SRCDIR/NumberKeyBoard.ui $DIR/NumberKeyBoard.ui
fi

if [ -f $SRCDIR/ProcessTab.ui ]; then
   cp -f -p $SRCDIR/ProcessTab.ui $DIR/ProcessTab.ui
fi

if [ -f $SRCDIR/ResultsTab.ui ]; then
   cp -f -p $SRCDIR/ResultsTab.ui $DIR/ResultsTab.ui
fi

if [ -f $SRCDIR/SettingTab.ui ]; then
   cp -f -p $SRCDIR/SettingTab.ui $DIR/SettingTab.ui
fi

if [ -f $SRCDIR/SplashScreen.ui ]; then
   cp -f -p $SRCDIR/SplashScreen.ui $DIR/SplashScreen.ui
fi
# ------------------------------------------------------------

# ------------------------------------------------------------
# Copy the database file if needed (for case that DB structure changes)
#
#JJ4150916 if [ -f Results.db ]; then
#JJ4150916    cp -f -p $SRCDIR/Results.db $DIR/Results.db
#JJ4150916 fi

#JJ4150916 if [ -f Results_bk.db ]; then
#JJ4150916    cp -f -p $SRCDIR/Results_bk.db $DIR/Results_bk.db
#JJ4150916 fi
# ------------------------------------------------------------

# ------------------------------------------------------------
#JJ4180816 - Here we copy all low level programs.
#
if [ -f $SRCDIR/InHouseAPI.py ]; then
   cp -f -p $SRCDIR/InHouseAPI.py $DIR/InHouseAPI.py
fi

if [ -f $SRCDIR/Thermostat75C.py ]; then
   cp -f -p $SRCDIR/Thermostat75C.py $DIR/Thermostat75C.py
fi

if [ -f $SRCDIR/AR1100_TouchScreen.py ]; then
   cp -f -p $SRCDIR/AR1100_TouchScreen.py $DIR/AR1100_TouchScreen.py
fi
# ------------------------------------------------------------

# ------------------------------------------------------------
#JJ4180816 - Here we copy all GUI codes.
#
if [ -f $SRCDIR/__init__.py ]; then
   cp -f -p $SRCDIR/__init__.py $DIR/__init__.py
fi

if [ -f $SRCDIR/CommandList.py ]; then
   cp -f -p $SRCDIR/CommandList.py $DIR/CommandList.py
fi

if [ -f $SRCDIR/http_server_with_upload.py ]; then
   cp -f -p $SRCDIR/http_server_with_upload.py $DIR/http_server_with_upload.py
fi

if [ -f $SRCDIR/Icons_rc.py ]; then
   cp -f -p $SRCDIR/Icons_rc.py $DIR/Icons_rc.py
fi

if [ -f $SRCDIR/KeyboardWidget.py ]; then
   cp -f -p $SRCDIR/KeyboardWidget.py $DIR/KeyboardWidget.py
fi

if [ -f $SRCDIR/KeyPushButton.py ]; then
   cp -f -p $SRCDIR/KeyPushButton.py $DIR/KeyPushButton.py
fi

if [ -f $SRCDIR/log.py ]; then
   cp -f -p $SRCDIR/log.py $DIR/log.py
fi

if [ -f $SRCDIR/MainTest.py ]; then
   cp -f -p $SRCDIR/MainTest.py $DIR/MainTest.py
fi

if [ -f $SRCDIR/NumberKeyBoard.py ]; then
   cp -f -p $SRCDIR/NumberKeyBoard.py $DIR/NumberKeyBoard.py
fi

if [ -f $SRCDIR/ProcessTab.py ]; then
   cp -f -p $SRCDIR/ProcessTab.py $DIR/ProcessTab.py
fi

if [ -f $SRCDIR/ResultsTab.py ]; then
   cp -f -p $SRCDIR/ResultsTab.py $DIR/ResultsTab.py
fi

if [ -f $SRCDIR/ResultTable.py ]; then
   cp -f -p $SRCDIR/ResultTable.py $DIR/ResultTable.py
fi

if [ -f $SRCDIR/setSysDate.py ]; then
   cp -f -p $SRCDIR/setSysDate.py $DIR/setSysDate.py
fi

if [ -f $SRCDIR/SettingTab.py ]; then
   cp -f -p $SRCDIR/SettingTab.py $DIR/SettingTab.py
fi

if [ -f $SRCDIR/SplashScreen.py ]; then
   cp -f -p $SRCDIR/SplashScreen.py $DIR/SplashScreen.py
fi

if [ -f $SRCDIR/start.py ]; then
   cp -f -p $SRCDIR/start.py $DIR/start.py
fi

if [ -f $SRCDIR/Ui_KeyboardWidget.py ]; then
   cp -f -p $SRCDIR/Ui_KeyboardWidget.py $DIR/Ui_KeyboardWidget.py
fi

if [ -f $SRCDIR/Ui_MainWindow.py ]; then
   cp -f -p $SRCDIR/Ui_MainWindow.py $DIR/Ui_MainWindow.py
fi

if [ -f $SRCDIR/Ui_NumberKeyBoard.py ]; then
   cp -f -p $SRCDIR/Ui_NumberKeyBoard.py $DIR/Ui_NumberKeyBoard.py
fi

if [ -f $SRCDIR/Ui_ProcessTab.py ]; then
   cp -f -p $SRCDIR/Ui_ProcessTab.py $DIR/Ui_ProcessTab.py
fi

if [ -f $SRCDIR/Ui_ResultsTab.py ]; then
   cp -f -p $SRCDIR/Ui_ResultsTab.py $DIR/Ui_ResultsTab.py
fi

if [ -f $SRCDIR/Ui_SettingTab.py ]; then
   cp -f -p $SRCDIR/Ui_SettingTab.py $DIR/Ui_SettingTab.py
fi

if [ -f $SRCDIR/Ui_SplashScreen.py ]; then
   cp -f -p $SRCDIR/Ui_SplashScreen.py $DIR/Ui_SplashScreen.py
fi

if [ -f $SRCDIR/UsbDriveWatcher.py ]; then
   cp -f -p $SRCDIR/UsbDriveWatcher.py $DIR/UsbDriveWatcher.py
fi
# ------------------------------------------------------------

# ------------------------------------------------------------
#JJ4180816 - Copy the rev.txt file if needed
#
#JJ4150916 if [ -f SerialID.txt ]; then
#JJ4150916    cp -f -p $SRCDIR/SerialID.txt $DIR/SerialID.txt
#JJ4150916 fi
# ------------------------------------------------------------

# Delete the old AF configuration file if needed
#JJ if [ -f $DIR/JOMAP_configuration.cfg ]; then
#JJ    rm -f $DIR/JOMAP_configuration.cfg
#JJ fi

# ------------------------------------------------------------
#JJ4180816 - 
#
if [ -f $SRCDIR/Icons.qrc ]; then
   cp -f -p $SRCDIR/Icons.qrc $DIR/Icons.qrc
fi
# ------------------------------------------------------------

# ------------------------------------------------------------
#JJ4180816 - Copy setting file if needed
#
#JJ4150916 if [ -f SettingsFile.csv ]; then
#JJ4150916    cp -f -p $SRCDIR/SettingsFile.csv $DIR/SettingsFile.csv
#JJ4150916 fi
# ------------------------------------------------------------

# ------------------------------------------------------------
#JJ4150916 - Copy contents of Data and iART_Utility folders
#
cp $SRCDIR/__pycache__/*.* $DIR/__pycache__/.
cp $SRCDIR/Data/*.* $DATADIR/.
cp $SRCDIR/iART_Utility/*.* $UTILITYDIR/.
cp $SRCDIR/iART_Utility/__pycache__/*.* $UTILITYDIR/__pycache__/.
#JJ4150916 - Nothing to copy
#cp $SRCDIR/iART_Utility/CalibrationData/*.* $UTILITYDIR/CalibrationData/.
cp $SRCDIR/iART_Utility/Data/*.* $UTILITYDIR/Data/.

# DATADIR="/home/pi/PythonProject/iART_BD/Data"
# UTILITYDIR="/home/pi/PythonProject/iART_BD/iART_Utility"
# SRCDIR="./iART_BD"
# ------------------------------------------------------------


# Flush the buffer to disk
sync

#JJ1220816 - Below echo added by JJ
echo "Transfering files complete"

# ------------------------------------------------------------
#JJ4180816 - copy scripts in /home/root/microimager
#
#JJ if [ -f load_3g.sh ]; then
#JJ    cp -f -p load_3g.sh /home/root/microimager/
#JJ fi

# Copy Chinese font file and others
#JJ if [ -f GMIN.ttf ]; then
#JJ    cp -f -p GMIN.ttf /usr/local/Trolltech/QtEmbedded-4.7.3-arm/lib/lib/fonts
#JJ fi

# Copy the u-boot files if needed
#JJ if [ -f MLO ]; then
#JJ    cp -f MLO /media/mmcblk0p1/MLO
#JJ fi
#JJ if [ -f u-boot.bin ]; then
#JJ    cp -f u-boot.bin /media/mmcblk0p1/u-boot.bin
#JJ fi
# ------------------------------------------------------------

# ------------------------------------------------------------
# Change anything in filesystem here if needed
#
#JJ if [ -f /etc/init.d/psplash ]; then
#JJ    mv /etc/init.d/psplash /etc/init.d/psplash_original
#JJ fi

# Copy new opencv mark detection lib
#JJ if [ -f libopencv_markdetect2.so.2.4.7 ]; then
#JJ    cp -f -p libopencv_markdetect2.so.2.4.7 /usr/lib/opencv/libopencv_markdetect2.so.2.4.7
   # create symbolic link if not existed
#JJ    if [ ! -L /usr/lib/opencv/libopencv_markdetect2.so.2.4 ]; then
#JJ    	ln -s /usr/lib/opencv/libopencv_markdetect2.so.2.4.7 /usr/lib/opencv/libopencv_markdetect2.so.2.4
#JJ    fi
#JJ    if [ ! -L /usr/lib/opencv/libopencv_markdetect2.so ]; then
#JJ    	ln -s /usr/lib/opencv/libopencv_markdetect2.so.2.4.7 /usr/lib/opencv/libopencv_markdetect2.so
#JJ    fi
#JJ fi

# Delete old font
#JJ if [ -f /usr/local/Trolltech/QtEmbedded-4.7.3-arm/lib/lib/fonts/HanaMinA.ttf ]; then
#JJ    rm -f /usr/local/Trolltech/QtEmbedded-4.7.3-arm/lib/lib/fonts/HanaMinA.ttf
#JJ fi
#if [ -d mici ]; then
#   cp -rf mici /home/root/microimager
#fi
#JJ if [ ! -d /home/root/microimager/mici ]; then
#JJ    mkdir /home/root/microimager/mici
#JJ fi

# Copy new Jenoptik SDK library - CimSDK
#JJ if [ -f libcimsdk.so ]; then
#JJ    cp -f -p libcimsdk.so /usr/lib/libcimsdk.so
#JJ    chown root:root libcimsdk.so
#JJ    chmod a+x libcimsdk.so
#JJ fi

#JJ if [ -f libusb.so.1.0.19 ]; then
#JJ    cp -f -p libusb.so.1.0.19 /usr/lib/libusb.so.1.0.19
   # create symbolic link if not existed
#JJ    if [ ! -L /usr/lib/libusb.so ]; then
#JJ    	ln -s /usr/lib/libusb.so.1.0.19 /usr/lib/libusb.so.1.0
#JJ    	ln -s /usr/lib/libusb.so.1.0 /usr/lib/libusb.so
#JJ     chown root:root libusb.so.1.0.19
#JJ     chmod a+x libusb.so.1.0.19
#JJ    fi
#JJ fi
# ------------------------------------------------------------

# ------------------------------------------------------------
# Update Jenoptik optical module firmware here if needed
#
#JJ if [ -f uusb.ldr ]; then
#JJ    echo "Updating Jenoptik optical module firmware"
#JJ    ./uusb-update-linux-arm uusb.ldr
#JJ fi
# ------------------------------------------------------------

# ------------------------------------------------------------
# Update 3G wireless software components here if needed
#
# Create directory path before copying files
#JJ if [ ! -d /etc/ppp/ ]; then
#JJ    mkdir /etc/ppp/
#JJ fi
#JJ if [ ! -d /etc/ppp/peers ]; then
#JJ    mkdir /etc/ppp/peers
#JJ fi
#JJ if [ ! -d /home/root/microimager/3g ]; then
#JJ    mkdir /home/root/microimager/3g
#JJ fi

# copy libcurl and create the symbolic link in /usr/lib/
#JJ if [ -f libcurl.so.4.4.0 ]; then
#JJ    cp -f -p libcurl.so.4.4.0 /usr/lib/
   #delete the old synbolic link before creating a new one
#JJ    rm /usr/lib/libcurl.so.4
#JJ    ln -s /usr/lib/libcurl.so.4.4.0 /usr/lib/libcurl.so.4
#JJ fi

# ------------------------------------------------------------

# Get the PMP software file name
#JJ PMP_FILE="$( ls pmp_app.s28 )"

# Update the PMP software
#JJ echo "Updating PMP software"
#JJ ./pmp_updater $PMP_FILE /dev/ttyO1

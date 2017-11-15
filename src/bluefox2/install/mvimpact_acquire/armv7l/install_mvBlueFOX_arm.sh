#!/bin/bash

#This Script is made for custom ARM versions only!!!

# Get the targets platform and if it is called "i686" we know it is a x86 system, else it s x86_64
TARGET="armv7l"

DEF_DIRECTORY=/opt/mvIMPACT_acquire
PRODUCT=mvBlueFOX
API=mvIMPACT_acquire
TARNAME=mvBlueFOX
LINK=mvimpact-acquire
LDFILE=acquire.conf
USER=$(whoami)

if [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
   echo
   echo 'Installation script for the '$PRODUCT' driver.'
   echo
   echo "The default installation path is: "$DEF_DIRECTORY
   echo "Usage:		./install_mvBlueFOR.sh [OPTION] ... "
   echo "Example:	./install_mvBlueFOX.sh -p /myPath"
   echo
   echo "Arguments:"
   echo "-h	--help		Display this help."
   echo "-p	--path		Set the directory where the files shall be installed."
   echo
   exit
fi   

# Print out ASCII-Art Logo.
clear;
echo ""
echo ""
echo ""
echo ""
echo "                                 ===     ===      MMM~,    M                     "
echo "                                  ==+    ==       M   .M   M                     "
echo "                                  .==   .=+       M    M.  M   M    MM   ~MMM    "
echo "                                   ==+  ==.       MMMMM.   M   M    MM  M:   M   "
echo "              ..                   .== ,==        M   =M   M   M    MM +MMMMMMM  "
echo "    MMMM   DMMMMMM      MMMMMM      =====         M    MM  M   M    MM 7M        "
echo "    MMMM MMMMMMMMMMM :MMMMMMMMMM     ====         M    M+  M   MM  DMM  MM   ,   "
echo "    MMMMMMMMMMMMMMMMMMMMMMMMMMMMM                 IMM+°    M    .M:       =MI    "
echo "    MMMMMMM   .MMMMMMMM    MMMMMM                                                "
echo "    MMMMM.      MMMMMM      MMMMM                                                "
echo "    MMMMM       MMMMM       MMMMM                 MMMMMM    MMMMM    MM.   M     "
echo "    MMMMM       MMMMM       MMMMM                 M       MM    .MM  .M: .M      "
echo "    MMMMM       MMMMM       MMMMM                 M      .M       M~  .M~M       "
echo "    MMMMM       MMMMM       MMMMM                 MMMMMM MM       MD   +MM       "
echo "    MMMMM       MMMMM       MMMMM                 M       M       M    M MM      "
echo "    MMMMM       MMMMM       MMMMM                 M       MM     MM  :M. .M8     "
echo "    MMMMM       MMMMM       MMMMM                 M        .MMMMM    M     MD    "
echo "    MMMMM       MMMMM       MMMMM                                                "
echo ""
echo "=================================================================================="
sleep 1

# Check if the user did specify that we shall use a specific directory instead of DEF_DIRECTORY
if [ "$1" == "-p" ] || [ "$1" == "--path" ] ; then
    if [ $(echo "$2") ] ; then
      DEF_DIRECTORY=$2
    else
      echo
      echo "WARNING: Path option used with no defined path, will use: $DEF_DIRECTORY directory"
    fi
else
   echo
   echo "No target directory specified, default directory: $DEF_DIRECTORY will be used..."
fi

# Get the source directory (the directory where the files for the installation are) and cd to it
# (The script file must be in the same directory as the source TGZ) !!!
if which dirname >/dev/null; then
    SCRIPTSOURCEDIR=($PWD"/"$(dirname $0))
fi
if [ "$SCRIPTSOURCEDIR" != "$PWD" ]; then
   if [ -z $SCRIPTSOURCEDIR ] || [ "$SCRIPTSOURCEDIR" == "." ]; then
      SCRIPTSOURCEDIR=$PWD
   fi
   cd $SCRIPTSOURCEDIR
fi

# Set variables for GenICam and mvIMPACT_acquire for later use
if grep -q '/etc/ld.so.conf.d/' /etc/ld.so.conf; then
   ACQUIRE_LDSOCONF_FILE=/etc/ld.so.conf.d/acquire.conf
else
   ACQUIRE_LDSOCONF_FILE=/etc/ld.so.conf
fi

# Make sure the environment variables are set at the next boot as well
if grep -q '/etc/profile.d/' /etc/profile; then
   ACQUIRE_EXPORT_FILE=/etc/profile.d/acquire.sh
else
   ACQUIRE_EXPORT_FILE=/etc/profile
fi

# Get driver name, version, file. In case of multiple *.tgz files in the folder select the newest version.
if [ "$( ls | grep -c mvBlueFOX*.tgz )" != "0" ] ; then
  TARNAME=`ls mvBlueFOX*.tgz|tail -1 | sed -e s/\\.tgz//`
  TARFILE=`ls mvBlueFOX*.tgz|tail -1`
  VERSION=`ls mvBlueFOX*.tgz|tail -1 | sed -e s/\\mvBlueFOX// | sed -e s/\\-$TARGET-// | sed -e s/\\.tgz//`
  ACT=$API-$TARGET-$VERSION
  ACT2=$ACT
fi

  
YES_NO=
# Here we will ask the user if we shall start the installation process
echo
echo "-----------------------------------------------------------------------------------"
echo "Configuration:"
echo "-----------------------------------------------------------------------------------"
echo
echo "Installation for user:		"$USER " (you can change the owner of "$DEF_DIRECTORY""
echo "				with 'chown/chmod' )"
echo "Installation directory:		"$DEF_DIRECTORY
echo "Source directory:		"$SCRIPTSOURCEDIR
echo "Version:			"$VERSION
echo "Platform:			"$TARGET
echo "TAR-File:			"$TARFILE
echo
echo "ldconfig:"
echo "mvIMPACT_acquire:		"$ACQUIRE_LDSOCONF_FILE
echo
echo "Exports:"
echo "mvIMPACT_acquire:		"$ACQUIRE_EXPORT_FILE
echo
echo "-----------------------------------------------------------------------------------"
echo
echo "Do you want to continue (default is 'yes')?"
echo "Hit 'n' + <Enter> for 'no', or just <Enter> for 'yes'."
read YES_NO

# If the user is choosing no, we will abort the installation, else we will start the process.
if [ "$YES_NO" == "n" ] || [ "$YES_NO" == "N" ]; then
  echo "Quit!"
  exit
fi

# Create the *.conf files if the system is supporting ld.so.conf.d
if grep -q '/etc/ld.so.conf.d/' /etc/ld.so.conf; then
  sudo rm -f $ACQUIRE_LDSOCONF_FILE; sudo touch $ACQUIRE_LDSOCONF_FILE
fi

# Create the export files if the system is supporting profile.d
if grep -q '/etc/profile.d/' /etc/profile; then
  sudo rm -f $ACQUIRE_EXPORT_FILE; sudo touch $ACQUIRE_EXPORT_FILE
fi

# Check if the destination directory exist, else create it
if ! [ -d $DEF_DIRECTORY ]; then
  # the destination directory does not yet exist
  # first try to create it as a normal user
  mkdir -p $DEF_DIRECTORY >/dev/null 2>&1
  if ! [ -d $DEF_DIRECTORY ]; then
    # that didn't work
    # now try it as superuser
    sudo mkdir -p $DEF_DIRECTORY
  fi
  if ! [ -d $DEF_DIRECTORY  ]; then
    echo 'ERROR: Could not create target directory' $DEF_DIRECTORY '.'
    echo 'Problem:'$?
    echo 'Maybe you specified a partition that was mounted read only?'
    echo
    exit
  fi
else
  echo 'Installation directory already exists.'
fi

# in case the directory already existed BUT it belongs to other user
sudo chown -R $USER:$USER $DEF_DIRECTORY

# Check the actual tarfile
if ! [ -r $TARFILE ]; then
  echo 'ERROR: could not read' $TARFILE.
  echo
  exit
fi

# needed at compile time (used during development, but not shipped with the final program)
ACT=$API-$VERSION.tar

# Now unpack the tarfile into /tmp
cd /tmp
tar xzf $SCRIPTSOURCEDIR/$TARFILE

# Now check if we can unpack the tar file with the device independent stuff
# this is entirely optional
if [ -r /tmp/$ACT2 ]; then
   cd /tmp
   #tar xvf /tmp/$ACT
   cp -r $ACT2/* $DEF_DIRECTORY
   cd $DEF_DIRECTORY
   if grep -q 'MVIMPACT_ACQUIRE_DIR=' $ACQUIRE_EXPORT_FILE; then
      echo 'MVIMPACT_ACQUIRE_DIR already defined in' $ACQUIRE_EXPORT_FILE.
   else
      sudo sh -c "echo 'export MVIMPACT_ACQUIRE_DIR=$DEF_DIRECTORY' >> $ACQUIRE_EXPORT_FILE"
   fi

   if grep -q "$DEF_DIRECTORY/lib/$TARGET" $ACQUIRE_LDSOCONF_FILE; then
      echo "$DEF_DIRECTORY/lib/$TARGET already defined in" $ACQUIRE_LDSOCONF_FILE.
   else
      sudo sh -c "echo '$DEF_DIRECTORY/lib/$TARGET' >> $ACQUIRE_LDSOCONF_FILE"
   fi
else
  echo
  echo "ERROR: Could not read: /tmp/"$ACT2
  exit
fi

# This variable must be exported, or else wxPropView-related make problems can arise (wxPropGrid cannot be found)
export MVIMPACT_ACQUIRE_DIR=$DEF_DIRECTORY

# Set the libs to ldconfig
sudo /sbin/ldconfig

# Clen up /tmp
rm -r -f /tmp/$ACT2 /tmp/$API-$VERSION

# install needed libraries and compiler
COULD_NOT_INSTALL="Could not find apt-get or yast; please install >%s< manually."

# check if we have g++
if ! which g++ >/dev/null 2>&1; then
   if which apt-get >/dev/null 2>&1; then
      sudo apt-get -q install g++
   elif sudo which yast >/dev/null 2>&1; then
      YASTBIN=`sudo which yast`
      sudo $YASTBIN --install gcc-c++
   else
      printf "$COULD_NOT_INSTALL" "g++"
   fi
fi

if ! which make >/dev/null 2>&1; then
   if sudo which yast >/dev/null 2>&1; then
      YASTBIN=`sudo which yast`
      sudo $YASTBIN --install make
   else
      printf "$COULD_NOT_INSTALL" "make"
   fi
fi

# check if we have libexpat
if which apt-get >/dev/null 2>&1; then
  sudo apt-get -q install libexpat1-dev
elif sudo which yast >/dev/null 2>&1; then
  YASTBIN=`sudo which yast`
  sudo $YASTBIN --install expat libexpat-devel
else
  printf "$COULD_NOT_INSTALL" "libexpat"
fi

INPUT_REQUEST="Do you want to install >%s< (default is 'yes')?\nHit 'n' + <Enter> for 'no', or just <Enter> for 'yes'.\n"
YES_NO=

# do we want to install wxWidgets?
if ! which wx-config >/dev/null 2>&1; then
   echo
   printf "$INPUT_REQUEST" "the necessary wx libraries"
   echo "This is highly recommended, as without them, you cannot run wxPropView and mvDeviceConfigure."
   echo
   read YES_NO
   if [ "$YES_NO" == "n" ] || [ "$YES_NO" == "N" ]; then
      echo 'Not installing wx libraries'
   else
      if which apt-get >/dev/null 2>&1; then
         echo 'Installing wx libraries'
         sudo apt-get -q install libwxgtk2.8-0 libwxbase2.8-0
      elif sudo which yast >/dev/null 2>&1; then
         echo 'Installing wx libraries'
         YASTBIN=`sudo which yast`
         sudo $YASTBIN --install wxGTK-devel
      else
         printf "$COULD_NOT_INSTALL" "wx libraries"
      fi
   fi
fi

echo
echo "Do you want the tools and samples to be built (default is 'yes')?"
echo "Hit 'n' + <Enter> for 'no', or just <Enter> for 'yes'."
read YES_NO
if [ "$YES_NO" == "n" ] || [ "$YES_NO" == "N" ]; then
   echo 'The tools and samples were not built.'
   echo 'To build them yourself, type:'
   echo '  cd ~/mvimpact-acquire/apps'
   echo "  make native"
   echo '  sudo /sbin/ldconfig'
else
   make $TARGET -j4
   sudo /sbin/ldconfig

    # Shall the MV tools be linked in /usr/bin?
   echo "Do you want to set a link to /usr/bin for wxPropView and mvDeviceConfigure (default is 'yes')?"
   echo "Hit 'n' + <Enter> for 'no', or just <Enter> for 'yes'."
   read YES_NO
   if [ "$YES_NO" == "n" ] || [ "$YES_NO" == "N" ]; then
      echo "Will not set any new link to /usr/bin."
   else
      if [ -r /usr/bin ]; then
         # Set wxPropView
         if [ -r $DEF_DIRECTORY/Precompiled/wxPropView ]; then
            sudo rm -f /usr/bin/wxPropView
            sudo ln -s $DEF_DIRECTORY/Precompiled/wxPropView /usr/bin/wxPropView
         fi
         # Set mvDeviceConfigure
         if [ -r $DEF_DIRECTORY/Precompiled/mvDeviceConfigure ]; then
            sudo rm -f /usr/bin/mvDeviceConfigure
            sudo ln -s $DEF_DIRECTORY/Precompiled/mvDeviceConfigure /usr/bin/mvDeviceConfigure
         fi
      fi
   fi
fi

echo
echo "Do you want to copy 51-mvbf.rules to /etc/udev/rules.d for non-root user support (default is 'yes')?"
echo "Hit 'n' + <Enter> for 'no', or just <Enter> for 'yes'."
read YES_NO
if [ "$YES_NO" == "n" ] || [ "$YES_NO" == "N" ]; then
   echo
   echo 'To grant non-root user support,'
   echo 'copy 51-mvbf.rules the file to /etc/udev/rules.d'
   echo
else
   sudo cp -f $DEF_DIRECTORY/Scripts/51-mvbf.rules /etc/udev/rules.d
fi

# check if plugdev group exists and the user is member of it
echo
if [ "$(grep -c plugdev /etc/group)" == "0" ]; then
   echo "Group 'plugdev' don't exists, this is necessary to run as non-root user, do you want to create it"
   echo "and add users to 'plugdev' (default is 'yes')?"
   echo "Hit 'n' + <Enter> for 'no', or just <Enter> for 'yes'."
   read YES_NO
   if [ "$YES_NO" == "n" ] || [ "$YES_NO" == "N" ]; then
      echo
      echo "'plugdev' will be not created and you can't run the device as non-root user!"
      echo "If you want non-root users support, you will need to create 'plugdev'"
      echo "and add the users to this group."
   else
      sudo /usr/sbin/groupadd -g 46 plugdev
      sudo /usr/sbin/usermod -a $USER -G plugdev
      echo "Group 'plugdev' created and user '"$USER"' added to it."
   fi
else
   if [ "$(groups | grep plugdev -c)" == "0" ]; then
   sudo /usr/sbin/usermod -a $USER -G plugdev
   fi
fi

echo
echo "-----------------------------------------------------------------------------------"
echo "                            Installation successful!                               "
echo "-----------------------------------------------------------------------------------"
echo
echo "Do you want to reboot now /default is 'yes')?"
echo "Hit 'n' + <Enter> for 'no', or just <Enter> for 'yes'."
read YES_NO
if [ "$YES_NO" == "n" ] || [ "$YES_NO" == "N" ]; then
   echo "You need to reboot manually to complete the installation."
else
   sudo shutdown -r now
fi

#!/bin/bash
if [ $# -lt 2 ]; then
    echo "
Usage is:
   $0 \"debians to install\" \"debians to remove\" [-y]
   If -y is the last argument, installation is performed without confirmation
Example:
   $0 \"pal-dubnium-pal-metapkg-tiago\" \"pal-dubnium-pal-metapkg-pmb2\"
   $0 \"pal-dubnium-pal-metapkg-tiago\" \"\"
"
    exit
fi
if [ "$EUID" -ne 0 ]
  then echo "Please run as root"
  exit
fi
TO_BE_INSTALLED=$1
TO_BE_REMOVED=$2
APT_FLAGS=""
if [[ "$3" == "-y" ]]; then
    APT_FLAGS="-y"
fi
echo "Installing $TO_BE_INSTALLED"
echo "Removing $TO_BE_REMOVED"

DEVICES=`ls /dev/sd* | grep -v sda`
MOUNT_DIR="/media/cdrom"
MOUNTED="no"
PAL_SOFTWARE_ISO_PREFIX="PAL-Robotics-Software-"
for m in $DEVICES
do
    mount $m $MOUNT_DIR 2>/dev/null
    if [ -f $MOUNT_DIR/.disk/info ]; then
        if [ "`grep Lubuntu $MOUNT_DIR/.disk/info`" != "" ]; then
            MOUNTED="yes"
            break
        else
            umount $MOUNT_DIR 2>/dev/null
        fi
    else
        umount $MOUNT_DIR 2>/dev/null
    fi
done
if [[ "$MOUNTED" == "no" ]]; then
    echo "PAL-Robotics-Software ISO not found, have you plugged it in?"
    exit
fi

MEDIA_DIR="/media/cdrom"
SOURCE_FILE="/etc/apt/sources.list"
SOURCE_FILE_DIR="/etc/apt/sources.list.d"
REPOS_LIST=`ls $MOUNT_DIR/pool | xargs`

echo "PAL-Robotics-Software ISO found in $MEDIA_DIR"
mv $SOURCE_FILE $SOURCE_FILE.bak
mv $SOURCE_FILE_DIR $SOURCE_FILE_DIR.bak
echo "deb file://"$MEDIA_DIR" trusty $REPOS_LIST" >> $SOURCE_FILE
set -e
apt-get update
apt-get remove $APT_FLAGS $TO_BE_REMOVED && apt-get install $APT_FLAGS $TO_BE_INSTALLED
umount $MEDIA_DIR
mv $SOURCE_FILE.bak $SOURCE_FILE
mv $SOURCE_FILE_DIR.bak $SOURCE_FILE_DIR


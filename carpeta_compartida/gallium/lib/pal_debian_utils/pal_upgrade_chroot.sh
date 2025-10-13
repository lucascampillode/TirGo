#!/bin/bash

set -u
set -e

# pick the most recent pal distro
UBUNTU_DISTRO=`lsb_release -sc`
PAL_DISTRO=`ls /opt/pal/ | sort | tail -n 1`
PAL_ROBOT=`cat /etc/hostname | cut -d"-" -f1`


PAL_SOURCES_LIST="/etc/apt/sources.list.d/$PAL_DISTRO.list"
PAL_PUBLIC_SOURCES_LIST="/etc/apt/sources.list.d.public/$PAL_DISTRO.list"
SOURCE_PREFIX=""

REPO_POSTFIX=""
PAL_REPO="pal-stable"
mount_point=""
include_system_pkgs=""
system_pkgs_only=""
keep_sources=""


LOG_FILE="/var/log/upgrades/pal_upgrade_chroot_errors_$NOW.log"

regenerate_pal_sources()
{
    if [ -n "$mount_point" ]; then # use pendrive if mount_point is set
        echo "deb file:$mount_point $UBUNTU_DISTRO main restricted thirdparty
" > $PAL_SOURCES_LIST
    else
        if [ "$keep_sources" == "True" ] && [[ ! -z $SOURCE_PREFIX ]]; then # Use the sources that were already configured
            echo "Using existing sources: $SOURCE_PREFIX"

        elif [ -f $PAL_PUBLIC_SOURCES_LIST ]; then # use public sources if present 
            
            SOURCE_PREFIX="deb https://packages.pal-robotics.com/$PAL_ROBOT/$PAL_DISTRO"
            echo "Using SOURCE_PREFIX = $SOURCE_PREFIX"

        else
            SOURCE_PREFIX="deb http://archives.basestation/$PAL_DISTRO"
            echo "Using SOURCE_PREFIX = $SOURCE_PREFIX"

        fi

        # Create new sources.list
        echo "$SOURCE_PREFIX/ubuntu$REPO_POSTFIX/ ${UBUNTU_DISTRO} main restricted universe multiverse
$SOURCE_PREFIX/ubuntu$REPO_POSTFIX/ ${UBUNTU_DISTRO}-security main restricted universe multiverse
$SOURCE_PREFIX/ubuntu$REPO_POSTFIX/ ${UBUNTU_DISTRO}-updates main restricted universe multiverse

$SOURCE_PREFIX/ros$REPO_POSTFIX/ ${UBUNTU_DISTRO} main
            
$SOURCE_PREFIX/$PAL_REPO/ ${UBUNTU_DISTRO} main confs thirdparty
" > $PAL_SOURCES_LIST
    fi
}

check_pkgs_availability()
{   
    local available="True"
    for pkg in $1; do
        local result=""
        if [ "$PAL_REPO" == "pal-staging" ]; then
            result=`apt-cache show $pkg | grep Filename | grep staging`
        else
            result=`apt-cache show $pkg | grep Filename | grep -v staging`
        fi

        if [ "$result" == "" ]; then
            echo "${pkg} is not available!"  
            available="False"
        fi
    done

    echo $available
}

start_system_pkgs_upgrade()
{   
    echo "Upgrading system packages"
    SYSTEM_PKGS=$(dpkg -l | grep pal-$PAL_DISTRO | awk '{ print $2 }' | grep -e repositories -e pal-$PAL_DISTRO-system -e pal-$PAL_DISTRO-pal-metapkg-system)
    system_pkgs_available=$(check_pkgs_availability $SYSTEM_PKGS)
    SYSTEM_PKGS_VERSIONS=""

    if [ "$system_pkgs_available" == "True" ]; then
        for pkg in ${SYSTEM_PKGS}; do

            version=""
            if [ "$PAL_REPO" == "pal-staging" ]; then
                version=`apt-cache show ${pkg} | grep Version | grep staging | awk '{ print $2 }' | head -1`
            else
                version=`apt-cache show ${pkg} | grep Version | grep -v staging | awk '{ print $2 }' | head -1`
            fi
            SYSTEM_PKGS_VERSIONS+=" ${pkg}=${version}"

            # The repository packages replace the sources during installation. 
            # To avoid issues with stable/staging regenerate desired sources
            if [[ ! -z `echo $pkg | grep repositories` ]]; then
                regenerate_pal_sources
            fi

        done

        apt-get install --reinstall --download-only --allow-change-held-packages --allow-downgrades -y $SYSTEM_PKGS_VERSIONS
        apt-get install --reinstall --allow-change-held-packages --allow-downgrades -y $SYSTEM_PKGS_VERSIONS
        regenerate_pal_sources

        echo "Upgraded all system packages"

    else
        echo "ERROR: System packages are not available, cancelling upgrade!"
        exit 1
    fi
}

# clean apt
echo "cleaning apt ..."
apt-get clean
rm -rf /var/lib/apt/lists
apt-get clean
echo "clean!"

# parse arguments
for i in "$@"
do
    case $i in
        use-staging)
            PAL_REPO="pal-staging"
            REPO_POSTFIX="-staging"
            ;;
        include-system-pkgs)
            include_system_pkgs="True"
            ;;
        system-pkgs-only)
            system_pkgs_only="True"
            ;;     
        keep-sources)
            keep_sources="True"
            ;;                       
        -d|--debug)
            set -x
            ;;
        -m=*|--mount=*)
            mount_point="${i#*=}"
            shift
            ;;
        *)
            # unknown option
            ;;
    esac
done

# Get the current source configuration
if [ "$keep_sources" == "True" ]; then
    SOURCE_PREFIX=`sed -n 1p $PAL_SOURCES_LIST | sed -n "s/\(^deb.*$PAL_DISTRO\).*/\1/p"`
fi

regenerate_pal_sources
echo "updating ..."
# Don't show warnings and errors that basically duplicate what is already displayed in std1
# If thats the case redirect to an error file
apt-get update 2> $LOG_FILE | { grep --color=never '^Err' || true; }

if [ "${PIPESTATUS[0]}" != "0" ]; then
  echo "Update failed, upgrade aborted"
  exit 1
fi

echo "updated!"
pushd /tmp

# get list of current metapkgs
META_PKGS=$(dpkg -l | grep pal-metapkg | grep -v pal-$PAL_DISTRO-pal-metapkg-system | awk '{print $2}')
REGULAR_PKGS=$(dpkg -l | grep pal-$PAL_DISTRO | awk '{ print $2 }' | grep -v repositories | grep -v pal-$PAL_DISTRO-system | grep -v pal-metapkg)

# delete any leftover packages from previous upgrades
for pkg in $META_PKGS; do
    rm -rf $pkg*
done

#Update system packages
if [ "$include_system_pkgs" == "True" ] || [ "$system_pkgs_only" == "True" ] ; then

    echo "Starting system pkg update"
    start_system_pkgs_upgrade
fi

#Update metapkgs
if [ "$system_pkgs_only" != "True" ] ; then

    echo "updating ..."
    # Don't show warnings and errors that basically duplicate what is already displayed in std1
    # If thats the case redirect to an error file
    apt-get update 2> $LOG_FILE | { grep --color=never '^Err' || true; }

    if [ "${PIPESTATUS[0]}" != "0" ]; then
      echo "Update failed, upgrade aborted"
      exit 1
    fi

    echo "updated!"

    # check if all packages are available
    meta_pkgs_available=$(check_pkgs_availability $META_PKGS)
    regular_pkgs_available=$(check_pkgs_availability $REGULAR_PKGS)

    # proceed only if all pkgs have at least one version available
    if [ "$meta_pkgs_available" == "True" ] &&  [ "$regular_pkgs_available" == "True" ] ; then


        # download packages to be installed
        echo "downloading new version of current packages ..."
        apt-get install --reinstall --download-only -y $META_PKGS
        echo "downloaded!"
        apt-get install --reinstall --download-only -y $REGULAR_PKGS
        echo "downloading upgradable packages ..."
        apt-get upgrade --download-only -y
        echo "downloaded!"

        echo "removing old packages..."
        # remove all pal-$PAL_DISTRO-* except for repositories and system packages
        apt-get remove --yes $META_PKGS
        apt-get remove --yes $REGULAR_PKGS

        echo "old packages removed!"

        if [ "${PAL_REPO}" == "pal-staging" ]; then
            # hold system packages when switching to staging
            apt-mark hold pal-$PAL_DISTRO-system-* pal-$PAL_DISTRO-repositories-* > /dev/null
        else
            # make sure system packages are not hold (undo behaviour of previous versions)
            apt-mark unhold pal-$PAL_DISTRO-system-* pal-$PAL_DISTRO-repositories-* > /dev/null
        fi

        # make sure kernel packages are not upgraded, this is spammy
        apt-mark hold linux-image* linux-headers* linux-firmware* > /dev/null

        # upgrade remaining packages
        echo "upgrading/installing new packages ..."
        apt-get upgrade --with-new-pkgs --yes >> /var/log/upgrades/pal_upgrade_$NOW.log

        # regenerate sources in case they were modified in the upgrade
        regenerate_pal_sources
        apt-get update
        echo "Installing new packages ..."
        # install the latest pal packages
        apt-get install --yes $META_PKGS

        # clean
        apt-get clean
        echo "DONE!"

    else
        echo "ERROR: previous packages are not available, cancelling upgrade!"
        exit 1

    fi

fi

# clean
apt-get clean

popd

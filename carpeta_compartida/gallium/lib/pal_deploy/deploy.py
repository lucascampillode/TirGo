#!/usr/bin/env python

from __future__ import print_function
from builtins import input

import os
import re
import sys
import argparse
import subprocess

# pretty - A miniature library that provides a Python print and stdout
# wrapper that makes colored terminal text easier to use (eg. without
# having to mess around with ANSI escape sequences). This code is public
# domain - there is no license except that you must leave this header.
#
# Copyright (C) 2008 Brian Nez <thedude at bri1 dot com>
#
# With modifications
# (C) 2013 Paul M <pmathieu@willowgarage.com>
# (C) 2014 Adolfo Rodriguez Tsouroukdissian <adolfo.rodriguez@pal-robotics.com>

codeCodes = {
    'black':     '0;30', 'bright gray':   '0;37',
    'blue':      '0;34', 'white':         '1;37',
    'green':     '0;32', 'bright blue':   '1;34',
    'cyan':      '0;36', 'bright green':  '1;32',
    'red':       '0;31', 'bright cyan':   '1;36',
    'purple':    '0;35', 'bright red':    '1;31',
    'yellow':    '0;33', 'bright purple': '1;35',
    'dark gray': '1;30', 'bright yellow': '1;33',
    'normal':   '0'
}


def printc(text, color, file=sys.stdout):
    """Print in color."""
    stdout_tty = file == sys.stdout and sys.stdout.isatty()
    stderr_tty = file == sys.stderr and sys.stderr.isatty()
    if stdout_tty or stderr_tty:
        print("\033[{}m{}\033[0m".format(codeCodes[color], text), file=file)
    else:
        print(text)


def print_err(msg):
    printc("Error: {}".format(msg), 'red', file=sys.stderr)


def print_ok(msg):
    printc(msg, 'green')


def parse_args():
    parser = argparse.ArgumentParser(description='Deploy built packages to '
                                     'a robot. The default behavior is to '
                                     'deploy *all* packages from any found '
                                     'workspace. Use --package to only '
                                     'deploy a single package.',
                                     epilog="""e.g.: {} reemc-4c -u root -p
                                     pal_tts
                                     -c=\"-DCMAKE_CXX_FLAGS_DEPLOY='-DNDEBUG -O3'\" """
                                     .format(
                                         os.path.basename(sys.argv[0])))
    parser.add_argument('robot', help='hostname to deploy to (e.g. reemh3-2c)')
    parser.add_argument('--user', '-u', help='username (default: pal)',
                        default='pal')
    parser.add_argument('--yes', '-y', action='store_true',
                        help="don't ask for confirmation, do it")
    parser.add_argument('--package', '-p', default="", metavar='PKG',
                        help="deploy specific packages")
    parser.add_argument('--install_prefix', '-i', default="",
                        help="Directory to deploy files")
    parser.add_argument('--cmake_args', '-c', default="",
                        help="Extra cmake args like --cmake_args=\"-DCMAKE_CXX_FLAGS='-DNDEBUG'\"")
    parser.add_argument('--make_args', '-b', default="",
                        help="Extra make args like --make_args=\"-j1'\"")
    parser.add_argument('--deploy_dependencies', '-d',  action='store_true',
                        help="When used with --package, deploys its dependencies as well")

    return parser.parse_args()


def list_packages(space):
    share = os.path.join(space, 'share')
    try:
        dirs = os.listdir(share)
    except StopIteration:
        print_err("No package found.")
        return
    for pkg in sorted(dirs):
        printc('=> Deploying package {}'.format(pkg), 'bright purple')


def ask_user(cmds):
    if isinstance(cmds, str):
        cmds = [cmds]
    printc("I'm about to run the following command{} in {}:"
           .format('s' if len(cmds) > 1 else '', os.getcwd()), 'cyan')
    for c in cmds:
        printc("  {}".format(c), 'yellow')
    r = input("Do it? (Y/n): ")
    if r == '' or r.lower() == 'y':
        return True
    return False

def prepare_install(install_prefix, yes=False, cmake_args="", package_list="",
                    deploy_dependencies=False, make_args=""):
    #Configure profile and also obtain install path
    cxx_flags=subprocess.check_output("/usr/bin/dpkg-buildflags --get CXXFLAGS",
                                      shell=True).decode('UTF-8')
    cxx_flags = cxx_flags.strip() #Remove trailing endline
    profile_config = subprocess.check_output(
        "catkin config --install --profile pal_deploy  -x _pal_deploy \
        --cmake-args -DCATKIN_BUILD_BINARY_PACKAGE=0 \
        -DCMAKE_CXX_FLAGS_DEBUG='-g -O0 -UNDEBUG' \
        -DCMAKE_C_FLAGS_DEBUG='-g -O0' -UNDEBUG \
        -DCMAKE_CXX_FLAGS_DEPLOY:String='" + cxx_flags + "' \
        -DCMAKE_BUILD_TYPE=Deploy \
        -DCATKIN_ENABLE_TESTING=OFF", shell=True).decode('UTF-8')

    inst_dir = None
    for line in profile_config.splitlines():
        match = re.match("Workspace:[^/]+(/.*)$", line)
        if match:
            inst_dir = match.group(1) + "/install_pal_deploy"

    deprecated_version = False
    if not inst_dir:
        print_err("Cannot find install workspace in catkin config output")
        raise RuntimeError
    try:
        subprocess.check_call("catkin clean --profile pal_deploy -i -y > /dev/null", shell=True)
    except subprocess.CalledProcessError as e:
        #Old version
        subprocess.check_call("catkin clean --profile pal_deploy -i -c > /dev/null", shell=True)
        deprecated_version = True


    extra_catkin_args = ""

    #If package list is not empty we'll compile everything, hence --no-deps is not needed
    if package_list != "" and not deploy_dependencies:
        extra_catkin_args="--no-deps"

    inst_space = os.path.join(inst_dir, install_prefix.strip('/'))
    if not deprecated_version:
        profile_config = subprocess.check_output(
            "catkin config --profile pal_deploy --install-space {}".format(inst_space), shell=True)
    else:
        #Old version ignores --install-space
        cmake_args = " -DCMAKE_INSTALL_PREFIX={} ".format(inst_space) + cmake_args

    cmd = ("catkin build --force-cmake {} --profile pal_deploy {} \
--cmake-args {} --make-args {}".format(package_list, extra_catkin_args, cmake_args, make_args))
    printc("Preparing install space", 'cyan')
    subprocess.check_call(cmd, shell=True)

    print_ok("Using catkin install space: {}".format(inst_space))
    list_packages(inst_space)
    return inst_space

def ros_package_path_contains_pal():
    """
    Check if the ROS_PACKAGE_PATH contains /opt/pal/<distribution>
    as if we deploy without that variable in a robot, on next reboot it will not boot because
    of not having /opt/pal/<distribution> packages in the path.
    :return: True if it contains it, False otherwise.
    """
    rpt = os.environ.get('ROS_PACKAGE_PATH')
    if rpt:
        if not '/opt/pal' in rpt:
            printc("\n\nNo '/opt/pal/<distribution>' found in ROS_PACKAGE_PATH, robot will not boot after reboot if you deploy!!\n\n", "red")
            r = input("Continue anyways? (y/N): ")
            if r == '' or r.lower() == 'n':
                return False
    else:
        printc("\n\nNO ROS_PACKAGE_PATH ENVIRONMENT VARIABLE, SOMETHING IS VERY WRONG\n\n", "red")
        return False
    return True


def main():
    args = parse_args()

    if not ros_package_path_contains_pal():
        return

    if args.install_prefix == "":
        args.install_prefix = "/home/{}/deployed_ws".format(args.user)
    path = prepare_install(args.install_prefix, yes=args.yes,
                            cmake_args=args.cmake_args,
                            package_list=args.package,
                           deploy_dependencies=args.deploy_dependencies,
                           make_args=args.make_args)

    if path is None:
        return

    os.chdir(path)
    cmd = ('rsync -avz {}/ {}@{}:{}'
           .format(path, args.user, args.robot, args.install_prefix))
    #cmd = ('rsync -avz {} {}@{}:{}'
           #.format(' '.join(dirs), args.user, args.robot, args.install_prefix))
    if args.yes or ask_user(cmd):
        printc("Syncing binaries with robot...", 'cyan')
        subprocess.check_call(cmd.split())
        print("\n******************")
        print_ok("Done. Time to try!")
        print("******************")


if __name__ == "__main__":
    try:
        main()
    except SystemExit:
        pass
    except:
        print_err("Woops, something went wrong. Trace below:")
        raise

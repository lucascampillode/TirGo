# generated from pal_catkin_environment/env-hooks/1.pal_distro.sh.in

if [ -n "$PAL_DISTRO" -a "$PAL_DISTRO" != "gallium" ]; then
  echo "PAL_DISTRO was set to '$PAL_DISTRO' before. Please make sure that the environment does not mix paths from different distributions."
fi
export PAL_DISTRO=gallium

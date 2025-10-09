#!/bin/bash


if [[ $# < 2 ]]; then
  echo "Usage: $0 input_ply_file output_ply_file [depth]"
  echo "depth influences the detail of the generated model. Default is 5, with higher providing more detail at the expense of computation power"
  exit 1
fi


input_ply=$1
output_ply=$2
if [[ $# == 3 ]]; then
    depth=$3
else
    depth=5
fi
WORK_DIR=`mktemp -d`
cd $WORK_DIR
echo "Working on $WORK_DIR"
input_pcd="input.pcd"
normal_pcd="normal.pcd"
surface_vtk="surface.vtk"
surface_pcd="surface.pcd"
surface_ply="surface.ply"
surface_hull_vtk="surface_hull.vtk"
surface_hull_ply="surface_hull.ply"
set -e
echo -e "\033[32mConverting input .ply to .pcd"
pcl_ply2pcd $input_ply $input_pcd
echo -e "\033[32mEstimating normal from provided pointcloud"
pcl_normal_estimation $input_pcd $normal_pcd -radius 0.02
echo -e "\033[32mReconstructing surface from normals"
pcl_poisson_reconstruction $normal_pcd $surface_vtk -depth $depth
echo -e "\033[32mConverting .vtk surface to .pcd"
pcl_vtk2ply $surface_vtk $surface_ply
meshlabserver -i $surface_ply  -o $output_ply -s `rospack find pal_pcl`/scripts/close_and_invert.mlx -om vn
echo -e "\033[32mDone. Try increasing/decreasing depth for different results. Check the result executing \"meshlab $output_ply\""
#meshlab $output_ply



# The lines below were used to apply a convex hull to the resulting surface, but it doesn't respect face orientation and difficults the usae with learnisfts
#pcl_vtk2pcd $surface_vtk $surface_pcd
#echo -e "\033[32mComputing convex hull to simplify the surface"
#pcl_compute_hull $surface_pcd $surface_hull_vtk
#echo -e "\033[32mConverting simplified .vtk surface to .ply"
#pcl_vtk2ply $surface_hull_vtk $surface_hull_ply
#echo -e "\033[32mDone. If the resulting surface is too simple and doesn't have enough details use $WORK_DIR/$surface_ply instead, at the cost of at least 10x computing power"
#cp $surface_hull_ply $output_ply

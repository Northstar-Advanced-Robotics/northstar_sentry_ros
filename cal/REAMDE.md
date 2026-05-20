## Use mrcal with 10x10 chessboard

To get images use ros to take images:

run 
ros2 run image_view image_saver --ros-args -p save_all_image:=false -p filename_format:="cam_image_%04d.png" -r image:=/left/camera/color/image_raw
and for each image run 
ros2 service call /save std_srvs/srv/Empty

once you have images run 
mrcal-calibrate-cameras   --focal 600   --object-spacing 0.0161   --object-width-n 10   --object-height-n 10   --lensmodel LENSMODEL_OPENCV8   --explore   "cal/cal_images/cam_image_*.png"

you get a .cameramodel file wich you need to convert to ROS .yaml format
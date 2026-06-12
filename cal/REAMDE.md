## Use mrcal with 10x10 chessboard

To get images use ros to take images:

run 
ros2 run image_view image_saver --ros-args -p save_all_image:=false -p filename_format:="cam_image_%04d.png" -r image:=/left/camera/color/image_raw

ros2 run image_view image_saver --ros-args -p save_all_image:=true -p filename_format:="cal/cal_images_front_1920/cam_image_%04d.png" -r image:=/image_raw
and for each image run 
ros2 service call /save std_srvs/srv/Empty


cameras to serial number
left: 827312073427
back: 851112061763
right: 827312073868

run:
ros2 run realsense2_camera realsense2_camera_node --ros-args --params-file cal/realsense_cal_config.yaml

once you have images run 
mrcal-calibrate-cameras   --focal 600   --object-spacing 0.016002   --object-width-n 10   --object-height-n 10   --lensmodel LENSMODEL_OPENCV8   --explore   "cal/cal_images/cam_image_*.png"

you get a .cameramodel file wich you need to convert to ROS .yaml format

converting to tagslam config

intrinsics	[ mrcal[0], mrcal[1], mrcal[2], mrcal[3] ]
distortion_coeffs	[ mrcal[4], mrcal[5], mrcal[6], mrcal[7], mrcal[8], mrcal[9], mrcal[10], mrcal[11] ]

arducam
        Type: Video Capture

        [0]: 'YUYV' (YUYV 4:2:2)
                Size: Discrete 1920x1200
                        Interval: Discrete 0.020s (50.000 fps)
                        Interval: Discrete 0.033s (30.000 fps)
                        Interval: Discrete 0.067s (15.000 fps)
                Size: Discrete 960x600
                        Interval: Discrete 0.013s (80.000 fps)
                        Interval: Discrete 0.017s (60.000 fps)
                        Interval: Discrete 0.033s (30.000 fps)
                        Interval: Discrete 0.067s (15.000 fps)

ros2 run v4l2_camera v4l2_camera_node --ros-args --params-file cal/v4l2_cal_config.yaml

to change framerate exit the container and run:
v4l2-ctl -d /dev/v4l/by-id/usb-Arducam_Arducam_B0495__USB3_2.3MP__Arducam_202500915_0001-video-index0 \
  --set-parm=15

v4l2-ctl -d /dev/v4l/by-id/usb-Arducam_Arducam_B0495__USB3_2.3MP__Arducam_202500915_0001-video-index0  --set-fmt-video=width=960,height=600

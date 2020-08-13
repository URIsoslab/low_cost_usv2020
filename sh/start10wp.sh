echo "$PMTK251,115200*1F" > /dev/ttyS5
echo "$PMTK300,200,0,0,0,0*2F" > /dev/ttyS5
roslaunch usv_gnc usv_wp_track10.launch

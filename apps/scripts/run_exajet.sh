export OSPRAY_TAMR_METHOD=trilinear

# parse the *.vtu file folder at the end of the command 
# /usr/sci/data/ospray/exajet-d12/surfaces_vtp/*.vtp

EXAJET_DATA=~/data/tamr/exajet/exajet

vglrun ./tamrViewer \
    -t landing \
    -d ${EXAJET_DATA} \
    -f vorticity_magnitude \
    -vr 0 1000 \
    -iso 750 \
	--vol-cmap exajet_teaser_vol_cmap.png \
    -iso-oct ${EXAJET_DATA} \
    -iso-field velocity_magnitude \
    -iso-vr 30 190 \
    --iso-cmap ice_fire_opaque.png \
	--eye 7.52209 -18.2986 6.59513 --dir 0.0302035 0.942275 -0.333477 --up -0.132508 0.334459 0.933048 \
    --fov 15 \
    --size 2400 600 \
    --exa-instance \
    --ao 1 \
    $@



#    --hide-volume \
#   --vol-cmap transparent.png \

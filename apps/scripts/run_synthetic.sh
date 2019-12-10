export OSPRAY_TAMR_METHOD=$1

if [[ -z "$1" ]]; then
    echo "Pass the desired reconstruction method to the script"
    exit 1
fi

SYNTHETIC_DATA=~/data/tamr/synthetic/synthetic2

vglrun ./tamrViewer \
    -t synthetic\
    -d ${SYNTHETIC_DATA} \
    -vr 0 42 \
    -iso 6.5 \
    --eye -0.713888 0.525825 -3.73112 \
    --dir 0.460855 0.290873 0.838455 \
    --up  -0.13911 0.956761 -0.255454 \
    -iso-vr 0 42 \




#    --eye -0.910556 -1.93517 -1.73874 \
#    --dir 0.472559 0.638915 0.607023 \
#    --up  0.0613079 0.663281 -0.745856 \

# light information (hard coded)
# dir -0.654 -0.038 1.0
# color 1.f-0.713888 0.525825 -3.73112
# density 4.0


#   --vol-cmap transparent.png \

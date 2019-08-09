# amr_project
Visualization project w / Timo Heister, Feng Wang, Will Usher et al.


Please build p4est with this configuration:
```bash
../configure CFLAGS="-g -O0" --with-gnu-ld --enable-debug F77=gfortran FC=gfortran --enable-mpi --enable-static=no --disable-memalign CPPFLAGS="-DSC_NOCOUNT_MALLOC -DSC_NOCOUNT_REFCOUNT -DSC_NOCOUNT_LOGINDENT"
```

## build voxel octree
```bash
bash ../modules/amr_project/apps/scripts/octGenSynthetic.sh <your path>/sythetic
```

## visualize the voxel octree
```bash
OSPRAY_TAMR_METHOD=trilinear bash ../modules/amr_project/apps/scripts/runTAMR.sh synthetic <your path>/synthetic -vr 0 64 -iso 6.5 --use-tf-widget
```

Customized arguments:

OSPRAY_TAMR_METHOD: specify the interpolation method. options: nearest(default), current, finest, octant, trilinear.

-f(--field): specify the field of the data. must be set for NASA data

-vr(--valueRange): specify the value range of the field.

-iso: specify the isovalue to generate the isosurface. Isosurface will not be generated if this value is not set.

--use-tf-widget: set to use the tranfer function widget. 


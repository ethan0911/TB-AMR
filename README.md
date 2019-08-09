# amr_project
Visualization project w / Timo Heister, Feng Wang, Will Usher et al.

## Installation
Please build [p4est](http://p4est.org/) with this configuration:
```bash
../configure CFLAGS="-g -O0" --with-gnu-ld --enable-debug F77=gfortran FC=gfortran --enable-mpi --enable-static=no --disable-memalign CPPFLAGS="-DSC_NOCOUNT_MALLOC -DSC_NOCOUNT_REFCOUNT -DSC_NOCOUNT_LOGINDENT"
```

## Usage
### p4estViewer 
#### Example Usage
#### Notable command line flags 
* `--use-tf-widget` turns on the transfer function widget. **Known issue:** If we enable the transfer function widget, the "default", hard-coded transfer function will be used until the camera position is changed / the user interacts with the transfer function widget. (See [#11](https://github.com/n8xm/amr_project/issues/11).)
* `-b "<benchmark params>"` is used to pass a string that contains benchmark parameters to OSPRay for use with my benchmarking script. (Currently this doesn't do anything except make OSPRay parse and print out the parameters). Example usage: `./p4estViewer -b "32 cam_param_path 3 5 subdir_name octree" -t p4est -i mandel1_03`
* `-i <octree_name>`: Specify path to serialized octree  
* `-t <type>`: Specify type of data. Supported types include, but are not necessarily limited to, `p4est`, `synthetic`, and `exajet`.

### build octree (synthetic data)
```bash
bash ../modules/amr_project/apps/scripts/octGenSynthetic.sh <your path>/sythetic
```

### visualize octree (synthetic data)
```bash
OSPRAY_TAMR_METHOD=trilinear bash ../modules/amr_project/apps/scripts/runTAMR.sh synthetic <your path>/synthetic -vr 0 64 -iso 6.5 --use-tf-widget
```

Customized arguments:

OSPRAY_TAMR_METHOD: specify the interpolation method. options: nearest(default), current, finest, octant, trilinear.

-f(--field): specify the field of the data. must be set for NASA data

-vr(--valueRange): specify the value range of the field.

-iso: specify the isovalue to generate the isosurface. Isosurface will not be generated if this value is not set.

--use-tf-widget: set to use the tranfer function widget. 


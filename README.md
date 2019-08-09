# amr_project
Visualization project w / Timo Heister, Feng Wang, Will Usher et al.

## Installation
### P4est
Please build [p4est](http://p4est.org/) with this configuration:
```bash
../configure CFLAGS="-g -O0" --with-gnu-ld --enable-debug F77=gfortran FC=gfortran --enable-mpi --enable-static=no --disable-memalign CPPFLAGS="-DSC_NOCOUNT_MALLOC -DSC_NOCOUNT_REFCOUNT -DSC_NOCOUNT_LOGINDENT"
```
set `p4est_DIR` to the p4est install folder. 

### TransferFunctionWidget
Plese pull this [transfer function widget](https://github.com/wilsonCernWq/TransferFunctionModule) and set `TFN_MODULE_ROOT` to the widget folder.

### VTK library
In order to load the NASA exajet airplane mesh. you need to specity the `VTK_DIR` to the vtk installation folder. 


## Usage
### tamrViewer 
#### Example Usage
#### Notable command line flags 
* `OSPRAY_TAMR_METHOD` is used to specify the interpolation method. options:`nearest`,`current`, `finest`, `octant`,`trilinear`.
* `-t <type>`: Specify type of data. Supported types include, but are not necessarily limited to, `p4est`, `synthetic`, and `exajet`.
* `-i <octree_name>`: Specify path to serialized octree  
* `-f(--field)` is used to specify the field of the data. must be set for NASA data
* `-vr(--valueRange)` is used to specify the value range of the field. `synthetic`:[0,64],`p4est`:[0,1],`exajet`: density[1.2,1.205], y_vorticity[-10,20].
* `-iso` is used to set the isovalue for generating the isosurface. Isosurface will not be generated if this value is not set.
* `--use-tf-widget` turns on the transfer function widget. **Known issue:** If we enable the transfer function widget, the "default", hard-coded transfer function will be used until the camera position is changed / the user interacts with the transfer function widget. (See [#11](https://github.com/n8xm/amr_project/issues/11).)
* `-b "<benchmark params>"` is used to pass a string that contains benchmark parameters to OSPRay for use with my benchmarking script. (Currently this doesn't do anything except make OSPRay parse and print out the parameters). Example usage: `./tamrViewer -b "32 cam_param_path 3 5 subdir_name octree" -t p4est -i mandel1_03`

```
OSPRAY_TAMR_METHOD=trilinear ./tamrViewer -t synthetic -i ~/data/tamr/synthetic/sythetic1 --use-tf-widget -vr 0 64 -iso 6.5
```

### build octree (synthetic data)
```bash
bash ../modules/amr_project/apps/scripts/octGenSynthetic.sh <your path>/sythetic
```

### visualize octree (synthetic data)
```bash
OSPRAY_TAMR_METHOD=trilinear bash ../modules/amr_project/apps/scripts/runTAMR.sh synthetic <your path>/synthetic -vr 0 64 -iso 6.5 --use-tf-widget
```


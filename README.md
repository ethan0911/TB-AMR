# amr_project
Visualization project w/Feng Wang, Nathan Marshak, Will Usher, Timo Heisteret al.

## Installation
### OSPRay
Please clone [OSPRay](https://gitlab.com/Twinklebear/ospray-tamr-fork-temp.git) 

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
* `--vol-cmap` user-defined tranfer function for volume rendering 
* `--iso-cmap` user-defined tranfer function for isosurface 
* `--iso-vr` value range for isosurface raytracing
* `--iso-field` color isosurface according to other scalar field
* `--exa-instance` instance the geometry


```
OSPRAY_TAMR_METHOD=trilinear ./tamrViewer -t synthetic -i ~/data/tamr/synthetic/sythetic -vr 0 64 -iso 6.5
```

### build octree (synthetic data)
```bash
bash ../modules/amr_project/apps/scripts/gen_octree_synthetic.sh <your path>/sythetic
```

### visualize octree (synthetic data)
```bash
bash ../modules/amr_project/apps/scripts/run_synthetic.sh synthetic <your path>/synthetic
```


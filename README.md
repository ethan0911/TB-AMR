# amr_project
Visualization project w / Timo Heister, Feng Wang, Will Usher et al.


Please build p4est with this configuration:
```bash
../configure CFLAGS="-g -O0" --with-gnu-ld --enable-debug F77=gfortran FC=gfortran --enable-mpi --enable-static=no --disable-memalign CPPFLAGS=-DSC_NOCOUNT_MALLOC
```

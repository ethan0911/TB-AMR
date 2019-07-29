#include "TAMRVolume.ih"
// #include "FindCell.ih"
// #include "FindDualCell.ih"


/************************************************************
 *  Nearest interpolation
 ***********************************************************/
varying float TAMR_nearest(const void *uniform _self, 
                                  const varying vec3f &P)
{
  uniform TAMRVolume *uniform self = (uniform TAMRVolume *uniform)_self;

  vec3f lP;
  self->transformWorldToLocal(self,P,lP);

  CellRef cell = findLeafCell(self->_voxelAccel,lP);

  return cell.value;
}


export void TAMR_install_nearest(void *uniform _self)
{
  uniform TAMRVolume *uniform self = (uniform TAMRVolume *uniform)_self;
  self->super.sample = TAMR_nearest;
}

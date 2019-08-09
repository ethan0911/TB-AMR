#include "TAMRVolume.ih"
#include "FindCell.ih"
#include "FindDualCell.ih"


/************************************************************
 *  Current level interpolation
 ***********************************************************/
varying float TAMR_current(const void *uniform _self, 
                                  const varying vec3f &P)
{
  uniform TAMRVolume *uniform self = (uniform uniform TAMRVolume *uniform)_self;

  vec3f lP;
  self->transformWorldToLocal(self,P,lP);

  CellRef cell = findLeafCell(self->_voxelAccel,lP);
  if(cell.value == 0.f)
    return cell.value;

  DualCell dcell;
  initDualCell(dcell, lP, cell.width);

  findDualCell(self->_voxelAccel,dcell);

  return lerp(dcell);
}

export void TAMR_install_current(void *uniform _self)
{
  uniform TAMRVolume *uniform self = (uniform TAMRVolume *uniform)_self;
  self->super.sample = TAMR_current;
}

#include "TAMRVolume.ih"
// #include "FindCell.ih"
// #include "FindDualCell.ih"


varying float TAMR_trilinear(const void *uniform _self, const varying vec3f &P)
{
  uniform TAMRVolume *uniform self =
      (uniform uniform TAMRVolume * uniform) _self;

  vec3f lP;  // local amr space
  self->transformWorldToLocal(self, P, lP);

  const CellRef cell = findLeafCell(self->_voxelAccel, lP);

  if(cell.value == 0.f)
    return cell.value;

  Octant O;
  DualCell D;

  return doTrilinear(_self, cell, lP, O, D);
}


export void TAMR_install_trilinear(void *uniform _self)
{
  uniform TAMRVolume *uniform self = (uniform TAMRVolume *uniform)_self;
  self->super.sample = TAMR_trilinear;
}



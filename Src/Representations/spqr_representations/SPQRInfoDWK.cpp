#include "SPQRInfoDWK.h"

SPQRInfoDWKCompressed::SPQRInfoDWKCompressed(const SPQRInfoDWK& mySPQRInfoDWK)
: x(mySPQRInfoDWK.x){}

SPQRInfoDWKCompressed::operator SPQRInfoDWK() const
{
  SPQRInfoDWK spqr_dwk;
  spqr_dwk.x = 0.0;
  return spqr_dwk;
}

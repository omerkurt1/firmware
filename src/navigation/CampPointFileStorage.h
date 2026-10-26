#pragma once

#include "CampPointStore.h"

// Plaintext prototype backend. Disabled in encrypted-storage builds until camp files participate
// in that subsystem's provisioning/migration/lock lifecycle. CRC is not authentication.
class CampPointFileStorage : public CampPointStorage
{
  public:
    CampPointStatus read(uint8_t *record) override;
    CampPointStatus write(const uint8_t *record) override;
};

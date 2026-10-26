#pragma once

#include "CampPointStore.h"
#include "concurrency/Lock.h"
#include "navigation/NavigationHeadingValidator.h"
#include "navigation/NavigationTypes.h"
#include "navigation/PositionFreshnessTracker.h"

class NodeDB;

class NavigationService
{
  public:
    // The database and optional camp store must outlive this service. Selection is volatile,
    // not a group-membership check. All camp-store access is serialized by this service's mutex.
    explicit NavigationService(NodeDB &database, CampPointStore *campStore = nullptr) : database(database), campStore(campStore)
    {
    }
    bool selectTarget(uint32_t nodeNum);
    void clearTarget();
    uint32_t getTarget() const;
    bool isCampTarget() const;
    CampActionStatus getCampStatus() const;
    // Restore once after filesystem initialization. Rendering never accesses flash.
    CampActionStatus restoreCamp();
    CampActionStatus selectCamp();
    // Requires a LOCAL coordinate report received less than 30 seconds ago, not a fixed position.
    // Receipt age does not establish the GNSS measurement age or physical fix accuracy.
    CampActionStatus saveCamp();
    CampActionStatus deleteCamp();
    bool setFreshnessPolicies(const PositionFreshnessPolicy &local, const PositionFreshnessPolicy &target);
    bool setHeadingPolicy(const NavigationHeadingPolicy &policy);
    // Supply measurement metadata on every read; no cached heading or caller-only valid flag bypasses validation.
    NavigationSnapshot getSnapshot(const NavigationHeadingSample &heading = {}) const;

  private:
    NodeDB &database;
    mutable concurrency::Lock mutex;
    uint32_t targetNode = 0;
    bool campTarget = false;
    CampPointStore *campStore;
    NavigationCoordinate campCoordinate;
    CampActionStatus campStatus = CampActionStatus::UNAVAILABLE;
    CampActionStatus restoreCampLocked();
    PositionFreshnessPolicy localPolicy;
    PositionFreshnessPolicy targetPolicy;
    NavigationHeadingPolicy headingPolicy;
};

extern NavigationService *navigationService;

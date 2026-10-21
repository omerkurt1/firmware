#pragma once

#include "concurrency/Lock.h"
#include "navigation/NavigationTypes.h"
#include "navigation/PositionFreshnessTracker.h"

class NodeDB;

class NavigationService
{
  public:
    // The database must outlive this service. Selection is volatile, not a group-membership check.
    explicit NavigationService(NodeDB &database) : database(database) {}
    bool selectTarget(uint32_t nodeNum);
    void clearTarget();
    uint32_t getTarget() const;
    bool setFreshnessPolicies(const PositionFreshnessPolicy &local, const PositionFreshnessPolicy &target);
    // Supply a current, validated heading on each read; the service never caches a compass reading.
    NavigationSnapshot getSnapshot(const NavigationHeading &heading = {}) const;

  private:
    NodeDB &database;
    mutable concurrency::Lock mutex;
    uint32_t targetNode = 0;
    PositionFreshnessPolicy localPolicy;
    PositionFreshnessPolicy targetPolicy;
};

extern NavigationService *navigationService;

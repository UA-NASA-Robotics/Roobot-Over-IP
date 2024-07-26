#include "statusManager.h"

statusManager::statusManager() {
    this->initialized = false;
    this->configured = false;
    this->chainFunctional = false;
    this->neighborAcquired = false;
    this->hasError = false;
    this->ErrorInoperable = false;
}

statusManager::~statusManager() {}

uint8_t statusManager::getSystemStatus() {
    if (!this->initialized) {
        return statusReportConstants::INITIALIZING;
    } else if (!this->configured) {
        return statusReportConstants::BLANKSTATE;
    } else if (this->hasError) {
        if (this->errorInoperable) {
            return statusReportConstants::NOTOPERABLE;
        } else {
            return statusReportConstants::OPERATINGWITHERRORS;
        }
    } else if (!this->chainFunctional) {
        return statusReportConstants::OPERATINGWITHOUTCHAIN;
    } else {
        return statusReportConstants::OPERATING;
    }
}

bool statusManager::getOperable() { return (this->initialized && !this->errorInoperable) }

void statusManager::initializedCallback() { this->initialized = true; }

void statusManager::configuredCallback() { this->configured = true; }

void statusManager::errorCallback(bool inoperable) {
    this->hasError = true;
    this->errorInoperable = inoperable;
}

void statusManager::clearError() {
    this->hasError = false;
    this->errorInoperable = false;
}

void statusManager::chainNeighborCallback(bool neighborAcquired, bool chainFunctional) {
    this->neighborAcquired = neighborAcquired;
    this->chainFunctional = chainFunctional;
}

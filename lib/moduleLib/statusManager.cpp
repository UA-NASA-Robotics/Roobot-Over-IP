#include "statusManager.h"

statusManager::statusManager::statusManager() {
    this->initialized = false;
    this->configured = false;
    this->chainFunctional = false;
    this->neighborAcquired = false;
    this->hasError = false;
    this->errorInoperable = false;
    this->disconnectCallback = nullptr;
    this->reconnectCallback = nullptr;
    this->isConnected = false;
    this->lastPacketTime = 0;
}

statusManager::statusManager::~statusManager() {}

uint8_t statusManager::statusManager::getSystemStatus() {
#if DEBUG && defined(__AVR__)
    Serial.print(F("Reporting status"));
#endif

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

bool statusManager::statusManager::getOperable() {
    return (this->initialized && !this->errorInoperable);
}

void statusManager::statusManager::notifyInitializedStatus() { this->initialized = true; }

void statusManager::statusManager::notifySystemConfigured() { this->configured = true; }

void statusManager::statusManager::notifySystemError(bool inoperable) {
    this->hasError = true;
    this->errorInoperable = inoperable;
}

void statusManager::statusManager::notifyClearError() {
    this->hasError = false;
    this->errorInoperable = false;
}

void statusManager::statusManager::notifyChainNeighborStatus(bool neighborAcquired,
                                                             bool chainFunctional) {
    this->neighborAcquired = neighborAcquired;
    this->chainFunctional = chainFunctional;
}

void statusManager::statusManager::notifyPacketReceived() {
#if defined(__AVR__)
    lastPacketTime = millis();
    // this->isConnected = true; //updated later
#else
#error "Architecture not yet supported"
#endif
}

bool statusManager::statusManager::isConnectionTimeout() {
#if defined(__AVR__)
    return (millis() - lastPacketTime) >= WatchdogConstants::WATCHDOG_TIMEOUT;
#else
#error "Architecture not yet supported"
#endif
}

void statusManager::statusManager::setDisconnectCallback(void (*callback)()) {
    this->disconnectCallback = callback;
}

void statusManager::statusManager::setReconnectCallback(void (*callback)()) {
    this->reconnectCallback = callback;
}

void statusManager::statusManager::tickDisconnectWatchdog() {
    if (this->isConnectionTimeout()) {  // if yes timeout
        if (isConnected) {
            if (this->disconnectCallback != nullptr) {
                this->disconnectCallback();
            }
            this->isConnected = false;
        }
    } else {                 // if no timeout
        if (!isConnected) {  // reset and call reconnect callback
            if (this->reconnectCallback != nullptr) {
                this->reconnectCallback();
            }
            this->isConnected = true;
        }
    }
}
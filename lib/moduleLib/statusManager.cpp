#include "statusManager.h"

statusManager::statusManager::statusManager() {
    this->_initialized = false;
    this->_configured = false;
    this->_chainFunctional = false;
    this->_neighborAcquired = false;
    this->_hasError = false;
    this->_errorInoperable = false;
    this->_disconnectCallback = nullptr;
    this->_reconnectCallback = nullptr;
    this->_isConnected = false;
    this->_lastPacketTime = 0;
}

statusManager::statusManager::~statusManager() {}

uint8_t statusManager::statusManager::getSystemStatus() {
    if (!this->_initialized) {
        return statusReportConstants::INITIALIZING;
    } else if (!this->_configured) {
        return statusReportConstants::BLANK_STATE;
    } else if (this->_hasError) {
        if (this->_errorInoperable) {
            return statusReportConstants::NOT_OPERABLE;
        } else {
            return statusReportConstants::OPERATING_WITH_ERRORS;
        }
    } else if (!this->_chainFunctional) {
        return statusReportConstants::OPERATING_WITHOUT_CHAIN;
    } else {
        return statusReportConstants::OPERATING;
    }
}

bool statusManager::statusManager::getOperable() {
    return (this->_initialized && !this->_errorInoperable);
}

void statusManager::statusManager::notifyInitializedStatus() { this->_initialized = true; }

void statusManager::statusManager::notifySystemConfigured() { this->_configured = true; }

void statusManager::statusManager::notifySystemError(bool inoperable) {
    this->_hasError = true;
    this->_errorInoperable = inoperable;
}

void statusManager::statusManager::notifyClearError() {
    this->_hasError = false;
    this->_errorInoperable = false;
}

void statusManager::statusManager::notifyChainNeighborStatus(bool neighborAcquired,
                                                             bool chainFunctional) {
    this->_neighborAcquired = neighborAcquired;
    this->_chainFunctional = chainFunctional;
}

void statusManager::statusManager::notifyPacketReceived() {
#if defined(__AVR__)
    _lastPacketTime = millis();
    // this->isConnected = true; //updated later
#else
#error "Architecture not yet supported"
#endif
}

bool statusManager::statusManager::isConnectionTimeout() {
#if defined(__AVR__)
    return (millis() - _lastPacketTime) >= WatchdogConstants::WATCHDOG_TIMEOUT;
#else
#error "Architecture not yet supported"
#endif
}

void statusManager::statusManager::setDisconnectCallback(void (*callback)()) {
    this->_disconnectCallback = callback;
}

void statusManager::statusManager::setReconnectCallback(void (*callback)()) {
    this->_reconnectCallback = callback;
}

void statusManager::statusManager::tickDisconnectWatchdog() {
    if (this->isConnectionTimeout()) {  // if yes timeout
        if (_isConnected) {
            if (this->_disconnectCallback != nullptr) {
                this->_disconnectCallback();
            }
            this->_isConnected = false;
        }
    } else {                  // if no timeout
        if (!_isConnected) {  // reset and call reconnect callback
            if (this->_reconnectCallback != nullptr) {
                this->_reconnectCallback();
            }
            this->_isConnected = true;
        }
    }
}
#ifndef statusManager_H
#define statusManager_H

#include <stdint.h>

#include "../UDP-API/sysAdmin.h"

#ifdef __AVR__
#include <Arduino.h>
#endif

// Set the default debug mode for the statusManager
#ifndef DEBUG
#define DEBUG false
#endif

namespace statusManager {
class statusManager {
   private:
    bool _initialized;       // Whether the system is initialized
    bool _configured;        // Whether the system is configured
    bool _chainFunctional;   // Whether the chain is functional
    bool _neighborAcquired;  // Whether a neighbor has been acquired
    bool _hasError;          // Whether the system has an error
    bool _errorInoperable;   // Whether the error is inoperable

    void (*_disconnectCallback)();  // Callback function when system is disconnected to ros node
    void (*_reconnectCallback)();   // Callback function when system is reconnected to ros node
    bool _isConnected;              // Whether the system is connected to a ROS node
    unsigned long _lastPacketTime;  // The time of the last packet received

   public:
    /**
     * @brief Construct a new status Manager object
     *
     */
    statusManager();

    /**
     * @brief Destroy the status Manager object
     *
     */
    ~statusManager();

    /**
     * @brief Get the detailed system status
     *
     * @return uint8_t, see statusReportConstants
     */
    uint8_t getSystemStatus();

    /**
     * @brief Get the system status as operable or not
     *
     * @return true, if the system is operable
     * @return false, if the system is not operable
     */
    bool getOperable();

    /**
     * @brief Notify the status manager that the system is initialized
     *
     */
    void notifyInitializedStatus();

    /**
     * @brief Notify the status manager that the system is configured, may be called multiple times
     *
     */
    void notifySystemConfigured();

    /**
     * @brief Notify the status manager that the system has encountered an error
     *
     * @param inoperable , true if the module is now inoperable
     */
    void notifySystemError(bool inoperable);

    /**
     * @brief Notify the status manager that the system has cleared the error
     *
     */
    void notifyClearError();

    /**
     * @brief Notify the status manager of the chain status
     *
     * @param neighborAcquired , true if a neighbor has been acquired
     * @param chainFunctional , true if the whole chain is functional
     */
    void notifyChainNeighborStatus(bool neighborAcquired, bool chainFunctional);

    /**
     * @brief Notify the status manager that the module has received a packet, ie connection working
     *
     */
    void notifyPacketReceived();

    /**
     * @brief Check if the system is connected to a ROS node, watchdog timeout included. Use this to
     * check for a failure stop condition. No need to filter, output will not jitter.
     *
     * @return true, if the system is connected to a ROS node
     * @return false, if the system is not connected to a ROS node, and should stop
     */
    bool isConnectionTimeout();

    /**
     * @brief Set a callback function to be called when the system is disconnected from the ROS node
     *
     * @param callback , void () function pointer
     */
    void setDisconnectCallback(void (*callback)());

    /**
     * @brief Set a callback function to be called when the system is reconnected to the ROS node
     *
     * @param callback , void () function pointer
     */
    void setReconnectCallback(void (*callback)());

    /**
     * @brief Tick the disconnect watchdog to check for connection timeouts. This will call the
     * disconnect when appropriate
     *
     */
    void tickDisconnectWatchdog();
};
};  // namespace statusManager

#endif

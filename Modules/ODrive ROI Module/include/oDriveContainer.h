#ifndef ODRIVECONTAINER_H
#define ODRIVECONTAINER_H
#include "../../../lib/ModuleCodec.h"
#include "../../../lib/Packet.h"
#include "../../../lib/moduleLib/statusManager.h"
#include "oDriveController.h"

template <int N>
class ODriveContainer {
   private:
    ODriveController *oDriveControllers[N];
    uint8_t size;

   public:
    /**
     * @brief Construct a new ODriveContainer object
     *
     */
    ODriveContainer();

    /**
     * @brief Destroy the ODriveContainer object
     *
     */
    ~ODriveContainer();

    /**
     * @brief Append an ODriveController to the container
     *
     * @param oDriveController , manually created ODriveController
     */
    void append(ODriveController &oDriveController);

    /**
     * @brief Set the ODriveController object at the index
     *
     * @param oDriveController
     * @param index
     */
    void set(ODriveController &oDriveController, int index);

    /**
     * @brief Inits all the ODriveControllers in the container
     *
     */
    void init();

    /**
     * @brief Used as a pause callback from the moduleStatusManager for all the ODriveControllers
     *
     */
    void pause();

    /**
     * @brief Resume callback from the moduleStatusManager for all the ODriveControllers
     *
     */
    void resume();

    /**
     * @brief Mainloop functions for all odrive controllers to perform their tasks
     *
     */
    void tick();

    /**
     * @brief Takes a generalPacket and forwards it to the correct ODriveController based on
     * subdevice ID
     *
     * @param packet
     * @return ROIPackets::Packet
     */
    ROIPackets::Packet handleGeneralPacket(ROIPackets::Packet packet);
};

// Note all the function definitions are in the header file because of the template class
// requirement of Cpp

template <int N>
ODriveContainer<N>::ODriveContainer() {
    size = N;
    for (int i = 0; i < size; i++) {
        oDriveControllers[i] = nullptr;
    }
}

template <int N>
ODriveContainer<N>::~ODriveContainer() {  // Destructor
    for (int i = 0; i < size; i++) {
        delete oDriveControllers[i];
    }
    // delete[] oDriveControllers;
}

template <int N>
void ODriveContainer<N>::append(ODriveController &oDriveController) {
    for (int i = 0; i < size; i++) {
        if (oDriveControllers[i] == nullptr) {
            oDriveControllers[i] = &oDriveController;
            return;
        }
    }
}

template <int N>
void ODriveContainer<N>::set(ODriveController &oDriveController, int index) {
    static_assert(index < N, "Index out of bounds");
    if (index < size) {
        oDriveControllers[index] = &oDriveController;
    }
}

template <int N>
void ODriveContainer<N>::init() {
    for (int i = 0; i < size; i++) {
        if (oDriveControllers[i] != nullptr) {
            oDriveControllers[i]->init();
        }
    }
}

template <int N>
void ODriveContainer<N>::pause() {
    for (int i = 0; i < size; i++) {
        if (oDriveControllers[i] != nullptr) {
            oDriveControllers[i]->pause();
        }
    }
}

template <int N>
void ODriveContainer<N>::resume() {
    for (int i = 0; i < size; i++) {
        if (oDriveControllers[i] != nullptr) {
            oDriveControllers[i]->resume();
        }
    }
}

template <int N>
void ODriveContainer<N>::tick() {
    for (int i = 0; i < size; i++) {
        if (oDriveControllers[i] != nullptr) {
            oDriveControllers[i]->tick();
        }
    }
}

template <int N>
ROIPackets::Packet ODriveContainer<N>::handleGeneralPacket(ROIPackets::Packet packet) {
    uint8_t subDevice = packet.getSubDeviceID();
    if (subDevice < size && oDriveControllers[subDevice] != nullptr) {
        return oDriveControllers[subDevice]->handleGeneralPacket(packet);
    }
    ROIPackets::Packet replyPacket = packet.swapReply();
    replyPacket.setData(0);
    return replyPacket;
}

#endif
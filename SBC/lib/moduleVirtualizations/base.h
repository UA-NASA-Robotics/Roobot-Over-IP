#ifndef BASEMODULE_H
#define BASEMODULE_H

#include <iostream>

#include "../../../lib/ModuleCodec.h"
#include "../../../lib/Packet.h"
#include "../../../lib/UnityTypes.h"

/*

This is the base module template* for all virtual modules (See virtualization layer). It defines
base interface functions that all virtual modules must implement.

*not a cpp template, but a template for developers to use when creating new virtual modules.

*/

class BaseModule {
   protected:
    virtual void ResponseCallback(
        ROIPackets::Packet packet) = 0;  // Callback function for when a response is received
                                         // (Called by TransportAgent, given response packet)

    virtual void
    MaintainState() = 0;  // Function called by TransportAgent to maintain the state of
                          // the module, ie refresh any data coming from the physical
                          // module; keep read values up to date. It may need to issue a bunch of
                          // packets back to the transport agent to get all the data it needs.
                          // Responses to all packets will get returned in the callback

   public:
    virtual bool PushState() = 0;  // Pushes the current state of the module to the physical module
    virtual bool PullState() = 0;  // Pulls the current state of the module from the physical module

    friend class TransportAgent;
};

#endif  // BASEMODULE_H
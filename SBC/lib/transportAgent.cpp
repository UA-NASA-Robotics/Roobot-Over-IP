#include "transportAgent.h"

TransportAgent::TransportAgent() {
    // Initialize the transport agent thread
    transportAgentThread = std::thread(&TransportAgent::TransportAgent::transportAgentWorker, this);
}

TransportAgent::~TransportAgent() {
    // Join the transport agent thread
    transportAgentThread.join();

    // Clear the module vector
    modules.clear();
}

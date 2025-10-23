#ifndef SYS_ADMIN_H
#define SYS_ADMIN_H

#include "packetTypes.h"

namespace sysAdminConstants {
// Code and information to be used when building a sysAdminPacket
// Note there is no general packet constants. These should be with individual module sub-classes
// which can assign different meaning to code and data. The sysAdmin network is standardized for all
// modules.

// sysAdmin code bit n is a chain message flag, it should be '|' with any other code that must be
// passed along in the sysAdminPacket. Be careful when requesting all devices on a network to send a
// payload heavy response.

/*----------------- Metadata Codes -----------------*/
constexpr metaConstant NO_CHAIN_META =
    0;  // Metadata code for a sysAdminPacket that should not be circulated.
constexpr metaConstant CHAIN_MESSAGE_META =
    0b1000000000000000;  // Metadata code for a sysAdminPacket that MUST be circulated around the
// module chain. Note that the reply host address must be |= into the metadata with the chain
// metacode so that replies are sent to the originator of the chain message, not just the previous
// neighbor.

/*----------------- Action Codes -----------------*/
constexpr actionConstant BLANK =
    0b0000000000000000;  // action code for a blank packet (should not be sent)

constexpr actionConstant PING =
    0b0100000000000000;  // action code for a admin Packet that should respond
// if awake and ready, and a module identifier.
constexpr actionConstant PONG =
    0b1100000000000000;  // action code for a admin Packet that should respond
constexpr actionConstant PING_LOOP_BACK =
    0b0010000000000000;  // action sent only when a chain message is a PING and the next chain
                         // member is the origin. This is a loopback message so the origin knows the
                         // chain is complete.

constexpr actionConstant STATUS_REPORT =
    0b1010000000000000;  // action code for a admin Packet that should.
// elicit status information as a response.

constexpr actionConstant BLACK_LIST = 0b0110000000000000;  // action code for a admin Packet that

constexpr actionConstant FIRMWARE_REPORT =
    0b1110000000000000;  // action code for a admin Packet that
// should elicit firmware information as a response.

}  // namespace sysAdminConstants

namespace statusReportConstants {

typedef uint8_t statusConstant;

constexpr statusConstant NULL_CODE = 0;              // No status code/invalid status code
constexpr statusConstant OPERATING = 1;              // Operating normally, no errors
constexpr statusConstant OPERATING_WITH_ERRORS = 2;  // Operating with soft errors
constexpr statusConstant OPERATING_WITHOUT_CHAIN =
    3;                                      // Operating normally, but unable to form network chain
constexpr statusConstant NOT_OPERABLE = 4;  // Not operable, hard error
constexpr statusConstant INITIALIZING = 5;  // Initializing, not ready for operation
constexpr statusConstant BLANK_STATE = 6;   // Blank state, Device is ready to operate, but requires
// configuration before use. Use to signal a device that
// has been freshly powered on or reset.

}  // namespace statusReportConstants

namespace blacklistConstants {
typedef uint8_t blacklistConstant;

constexpr blacklistConstant NULL_CODE = 0;    // No blacklist code/invalid blacklist code
constexpr blacklistConstant BLACKLISTED = 1;  // Blacklisted, do not send messages to this device

constexpr blacklistConstant ADD_BLACKLIST = 2;     // Add this device to the blacklist
constexpr blacklistConstant REMOVE_BLACKLIST = 3;  // Remove this device from the blacklist
constexpr blacklistConstant LIST_BLACKLIST = 4;    // Clear the blacklist

}  // namespace blacklistConstants

namespace WatchdogConstants {
constexpr uint16_t MAINTAIN_SLEEP_TIME = 100;  // The time to sleep between maintainState
                                               // loops, in milliseconds for the ROS nodes.

constexpr uint16_t WATCHDOG_TIMEOUT =
    MAINTAIN_SLEEP_TIME * 5;  // The time in ms before the watchdog times out
}  // namespace WatchdogConstants

#endif  // SYS_ADMIN_H
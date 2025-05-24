## sysAdmin

General actions that can be performed across all modules. Some modules may override responses to these actions, but they should all be able to respond to them.

SysAdmin Packets can have both metadata and action codes, but not subDeviceIDs. The metadata currently carries both additional packet information, such as wether it should be chained around the network and the reply octet. The action code determines the action to be performed. Note while a sysAdmin request can be sent to all modules in the chain, responses do not propagate through the chain, they are only sent back to the original sender, whose host address octet has been embedded in the second byte of metadata.

Options:

-   [Ping](#ping)
-   [Status Report](#status-report)
-   [Blacklist](#blacklist)

### Ping

Call a ping packet on the sysAdmin port with action code: `sysAdminConstants::PING`

A ping packet is a simple packet that is sent to the sysAdmin port to check if the client is there, and what it is.
No payload is required, and the response will be a pong packet.

#### Return

action code: `sysAdminConstants::PONG`

Payload [2 bytes]

-   0: Ready, 0 if not operable, 1 if operable.
-   1: Client type, see available options in `moduleTypesConstants` namespace.

### Status Report

Call a status report packet on the sysAdmin port with action code: `sysAdminConstants::STATUS_REPORT`

A status report packet is a packet that is sent to the sysAdmin port to check the status of the client. This works on every module.
No payload is required, and the response will be a status report packet.

#### Return

action code: `sysAdminConstants::STATUS_REPORT`

Payload [14 bytes]:

-   0: Status code, see available options in `statusReportConstants` namespace.
-   1: Time alive, hours
-   2: Time alive, minutes
-   3: Time alive, seconds
-   4: Supply voltage \* 100, high byte
-   5: Supply voltage \* 100, low byte
-   6: Type of client, see available options in `moduleTypesConstants` namespace.
-   7: Chain Neighbor Host Address Octet
-   8: Mac Address Octet 1
-   9: Mac Address Octet 2
-   10: Mac Address Octet 3
-   11: Mac Address Octet 4
-   12: Mac Address Octet 5
-   13: Mac Address Octet 6

### Blacklist

Call a blacklist packet on the sysAdmin port with action code: `sysAdminConstants::BLACK_LIST`

This command can add remove or list blacklisted devices. The payload determines the action:

#### Payload

[2 Bytes]

-   0: Payload Action code, see available options in `blacklistConstants` namespace. `blacklistConstants::ADD_BLACKLIST`, `blacklistConstants::REMOVE_BLACKLIST`,`
-   1: Device octet to blacklist or remove from blacklist.

[1 Byte]

-   0: Action code, `blacklistConstants::LIST_BLACKLIST`

#### Return

[1 Byte] `ADD_BLACKLIST` or `REMOVE_BLACKLIST`:

-   0: Success, 0 if not successful, 1 if successful.

[N Bytes] `LIST_BLACKLIST`:

Each byte is a device octet that is blacklisted.

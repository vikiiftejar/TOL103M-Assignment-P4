# TOL103M-Assignment-P4
Crane Control Protocol (CCP) client implementation

**Course:** TÖL103M – Fall 2024  
**Instructors:** Hyytiä and Ágústsson  

## Description
This assignment implements a real-time Crane Control Protocol (CCP) client on top of LowNet.

The protocol enables exclusive, reliable, and time-bounded control of a crane device using a
stateful communication protocol with acknowledgements and retransmissions.

## Implemented Features

### Milestone I – Connection Establishment
- Three-way handshake with proof-of-work challenge
- Exclusive control acquisition
- Graceful connection termination

### Milestone II – Reliable Control Actions
- Sequenced crane action commands
- Retransmissions on missing acknowledgements
- Cumulative ACK and NAK handling
- Status-based flow control

### Milestone III – Automated Test Procedure
- Execution of the required crane control test sequence
- TEST-mode handshake support
- Acknowledgement-based synchronization

## Notes
- Relies on P2 and P3 networking and security features
- Uses LowNet protocol 0x05 (CCP)

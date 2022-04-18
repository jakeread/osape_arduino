/*
arduino-ports/vp_arduinoWire.h

turns Wire instances into competent bus link layers for OSAP 

Jake Read at the Center for Bits and Atoms
(c) Massachusetts Institute of Technology 2022

This work may be reproduced, modified, distributed, performed, and
displayed for any purpose, but must acknowledge the squidworks and ponyo
projects. Copyright is retained and must be preserved. The work is provided as
is; no warranty is provided, and users accept all liability.
*/

#include "../osap_config.h"

#ifdef INCLUDE_WIRE_VPORT

#ifndef ARDU_WIRELINK_H_
#define ARDU_WIRELINK_H_

#include <Arduino.h>
#include <Wire.h>
#include "../osape/core/vertex.h"

#define WIRELINK_BUFSIZE 255 

class VBus_ArduinoWire : public VBus {
  public:
    void begin(void);
    // -------------------------------- our own loop, cts, and send... 
    void loop(void) override; 
    void send(uint8_t* data, uint16_t len, uint8_t rxAddr) override; 
    boolean cts(uint8_t rxAddr) override; 
    // -------------------------------- data 
    TwoWire* wire;
    static void onRecieve(int count);
    // -------------------------------- constructors
    VBus_ArduinoWire(Vertex* _parent, String _name, TwoWire* _wire, uint8_t _ownRxAddr);
};

#endif 
#endif 
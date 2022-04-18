/*
arduino-ports/vp_arduinoWire.cpp

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

#include "vb_arduinoWire.h"
#include "../osap_debug.h"

// static stash: same per instance, 
uint8_t stash[32];
uint8_t stashLen = 0;

VBus_ArduinoWire::VBus_ArduinoWire(Vertex* _parent, String _name, TwoWire* _wire, uint8_t _ownRxAddr
) : VBus ( _parent, _name ) {
  wire = _wire;
  ownRxAddr = _ownRxAddr;
}

void VBus_ArduinoWire::begin(void){
  wire->begin(ownRxAddr);
  wire->onReceive(this->onRecieve);
}

void VBus_ArduinoWire::onRecieve(int count){
  Wire.readBytes(stash, count);
  stashLen = count;
}

void VBus_ArduinoWire::loop(void){
  // check incoming, 
  if(stashLen > 0){
    if(stackEmptySlot(this, VT_STACK_ORIGIN)){
      stackLoadSlot(this, VT_STACK_ORIGIN, stash, stashLen);
    }
    stashLen = 0;
  }
}

void VBus_ArduinoWire::send(uint8_t* data, uint16_t len, uint8_t rxAddr){
  digitalWrite(A1, HIGH);
  // this'll be the big hangup, 
  if(len > 32) return;
  // this might guard, if we are already rx'ing... 
  if(wire->available()) return;
  // become host, 
  wire->end();
  wire->begin();
  // transmit, 
  wire->beginTransmission(rxAddr);
  wire->write(data, len);
  uint8_t res = wire->endTransmission();
  // become guest again, 
  wire->end();
  wire->begin(ownRxAddr);
  // check, 
  //if(res != 0) 
  // DEBUG("res " + String(res) + " txd " + String(len));
  digitalWrite(A1, LOW);
}

boolean VBus_ArduinoWire::cts(uint8_t rxAddr){
  return true;
}

#endif 
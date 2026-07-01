//ARAM Toggle Actuator     Example Code                rev: 6/5/2026

/*This example shows a simple 1 actuator bluetooth MIDI orchestra, add up to 6 actuators*/ 
//IMPORTANT VIEW MESSAGE UNDER BLEMidi Import

//import ARAM Toggle Actuator Library
#include "ARAM_Toggle_Actuator.h";


//import ARAM Metal Orchestra Library
#include "ARAM_Metal_Orchestra.h";

//import bluetooth midi library
#include <BLEMidi.h>
//IMPORTANT:
//          search nimBLE-Arduino
//          downgrade library to 1.4.3
//          this is a dependency for BLEMIDI


//define what pins we're gonna use
#define CNTRL_PIN_A 1     //change me to your desired cntrl pin


//declare 1 toggle actuator
ARAM_TOGGLE_ACTUATOR a1;

//declare an instument
ARAM_METAL_ORCHESTRA_INSTRUMENT metalOrchestra;



/*This is a simple MIDI queue, you could do without this, but too many MIDI commands would cause issues*/
//If you dont want a queue delete the following: ->
struct MidiEvent {
  bool isNoteOn;
  uint8_t channel;
  uint8_t pitch;
  uint8_t velocity;
};

#define MIDI_QUEUE_SIZE 16
volatile MidiEvent midiQueue[MIDI_QUEUE_SIZE];
volatile uint8_t queueHead = 0;
volatile uint8_t queueTail = 0;

void pushMidiEvent(bool isNoteOn, uint8_t channel, uint8_t pitch, uint8_t velocity) {
  uint8_t nextHead = (queueHead + 1) % MIDI_QUEUE_SIZE;
  if (nextHead != queueTail) {
    midiQueue[queueHead].isNoteOn = isNoteOn;
    midiQueue[queueHead].channel = channel;
    midiQueue[queueHead].pitch = pitch;
    midiQueue[queueHead].velocity = velocity;
    queueHead = nextHead;
  }
}
//<------------


void handleNoteOn(uint8_t channel, uint8_t pitch, uint8_t velocity, uint16_t timestamp) {
  pushMidiEvent(true, channel, pitch, velocity);
  //if you want to get rid of the queue simply call noteOn here
  //metalOrchestra.handleNoteOn(channel, pitch, velocity, 0);
}

void handleNoteOff(uint8_t channel, uint8_t pitch, uint8_t velocity, uint16_t timestamp) {
  pushMidiEvent(false, channel, pitch, velocity);
  //if you want to get rid of the queue simply call noteOff here
  //metalOrchestra.handleNoteOff(channel, pitch, velocity, 0);
}





void setup() {

    //declare an actuator at 800 hertz, it will play other frequencies, but ...
    //this is the prefered frequency if the instrument has multiple actuators to choose from
  a1  = ARAM_TOGGLE_ACTUATOR(CNTRL_PIN_A,800);
  metalOrchestra.addToggleActuator(a1);
  //optionally set a force curve to tune this actuator to the resonator
  //a1.setFrequencyDamping(0, 800, 0.99);

  //set instument mode to resonator, meaning we'll play frequencies
  metalOrchestra.setInstrumentMode(ARAM_METAL_ORCHESTRA_Resonator);

  // Initialize the built-in BLEMidiServer object
  BLEMidiServer.begin("A.R.A.M. Metal Orchestra");

  // Register callbacks to the global object
  BLEMidiServer.setNoteOnCallback(handleNoteOn);
  BLEMidiServer.setNoteOffCallback(handleNoteOff);

}


void loop() {
  metalOrchestra.update();

  
  // Process the queued MIDI events <- if you dont want a queue you can remove this entire block, leaving only update()
  if (queueTail != queueHead) {
    MidiEvent event;
    event.isNoteOn = midiQueue[queueTail].isNoteOn;
    event.channel = midiQueue[queueTail].channel;
    event.pitch = midiQueue[queueTail].pitch;
    event.velocity = midiQueue[queueTail].velocity;
    if(event.velocity==0)event.isNoteOn = false;
    
    queueTail = (queueTail + 1) % MIDI_QUEUE_SIZE;
    double frequency = midiNoteToFrequency(event.pitch);
      if (event.isNoteOn) {
        metalOrchestra.handleNoteOn(event.channel, event.pitch, event.velocity, 0);
      } else {
        metalOrchestra.handleNoteOff(event.channel, event.pitch, event.velocity, 0);
      }
    
  }

}

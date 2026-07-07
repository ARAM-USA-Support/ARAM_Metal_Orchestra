//ARAM_Metal_Orchestra Library      -  Jack Serlin                rev: 6/30/2026               A.R.A.M. - American Robotics Assisted Manufacturing

#include <ARAM_Toggle_Actuator.h>;
#ifndef ARAM_ORCHESTRA_0
#define ARAM_ORCHESTRA_0

#define MAX_ARAM_TOGGLE_ACTUATORS_IN_INSTRUMENT 20
// Helper to convert MIDI note to frequency
double midiNoteToFrequency(byte note) {
  return 440.0 * pow(2.0, (note - 69.0) / 12.0);
}

enum ARAM_METAL_ORCHESTRA_INSTRUMENT_MODE {ARAM_METAL_ORCHESTRA_Resonator,ARAM_METAL_ORCHESTRA_Striker};
enum ARAM_METAL_ORCHESTRA_STRIKE_MODE {ARAM_METAL_ORCHESTRA_Continous,ARAM_METAL_ORCHESTRA_Impulse};
enum ARAM_METAL_ORCHESTRA_MAPPING_MODE {ARAM_METAL_ORCHESTRA_Closest,ARAM_METAL_ORCHESTRA_LRU,ARAM_METAL_ORCHESTRA_Channel};

//ARAM_METAL_ORCHESTRA_INSTRUMENT class
class ARAM_METAL_ORCHESTRA_INSTRUMENT{

  public:
  ARAM_METAL_ORCHESTRA_INSTRUMENT(){
    for(int i = 0; i < MAX_ARAM_TOGGLE_ACTUATORS_IN_INSTRUMENT; i++){
      this->actuators[i] = NULL;
      playableFrequencies[i] = 0;
      playableFrequenciesIndices[i] = -1;
      LSUPlayedFrequenciesArray[i] = -1;
      LSUPlayedFrequenciesCount[i] = 0;
      playingFrequencies[i] = -1;

    }
    numActuators = 0;
    maxFrequency = 0;
    mode = ARAM_METAL_ORCHESTRA_Resonator;
    strikeMode = ARAM_METAL_ORCHESTRA_Continous;
    mappingMode = ARAM_METAL_ORCHESTRA_LRU;
  }

  ARAM_METAL_ORCHESTRA_INSTRUMENT(ARAM_METAL_ORCHESTRA_INSTRUMENT &other){
  }

  int addToggleActuator(ARAM_TOGGLE_ACTUATOR & actuatator);
  void update();
  void playFrequency(double frequency, int milliseconds);
  void handleNoteOn(uint8_t channel, uint8_t pitch, uint8_t velocity, uint16_t timestamp);
  void handleNoteOff(uint8_t channel, uint8_t pitch, uint8_t velocity, uint16_t timestamp);
  void setInstrumentMode(ARAM_METAL_ORCHESTRA_INSTRUMENT_MODE newMode);
  void setStrikeMode(ARAM_METAL_ORCHESTRA_STRIKE_MODE newMode);
  void setMappingMode(ARAM_METAL_ORCHESTRA_MAPPING_MODE newMode);
  
  private:
  ARAM_TOGGLE_ACTUATOR * actuators [MAX_ARAM_TOGGLE_ACTUATORS_IN_INSTRUMENT];
  double playableFrequencies [MAX_ARAM_TOGGLE_ACTUATORS_IN_INSTRUMENT];
  int playableFrequenciesIndices [MAX_ARAM_TOGGLE_ACTUATORS_IN_INSTRUMENT];
  double LSUPlayedFrequenciesArray [MAX_ARAM_TOGGLE_ACTUATORS_IN_INSTRUMENT];
  int LSUPlayedFrequenciesCount [MAX_ARAM_TOGGLE_ACTUATORS_IN_INSTRUMENT];
  int numActuators;
  void _addPlayableFrequency(double freq, int index);
  int _getIndexOfActuatorWithClosestFrequency(double targetFrequency);
  int _getLSUActuatorIndex(double frequency);
  void _clearLSUtable();
  ARAM_METAL_ORCHESTRA_INSTRUMENT_MODE mode;
  ARAM_METAL_ORCHESTRA_STRIKE_MODE strikeMode;
  ARAM_METAL_ORCHESTRA_MAPPING_MODE mappingMode;

    
  #if ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  #endif

  double maxFrequency;
  int maxFrequencyIndex;


  double playingFrequencies [MAX_ARAM_TOGGLE_ACTUATORS_IN_INSTRUMENT];



  
};


int ARAM_METAL_ORCHESTRA_INSTRUMENT::_getLSUActuatorIndex(double frequency){
  int foundIndex = -1;
  int placementIndex = numActuators-1;
  int minRemaining = numActuators;
  int minRemainingIndex = 0;
  for(int i = 0; i < numActuators; i++){
    if((LSUPlayedFrequenciesArray[i] == -1)&&(placementIndex==numActuators-1)){
      LSUPlayedFrequenciesArray[i] = frequency;
      LSUPlayedFrequenciesCount[i] = numActuators;
      return playableFrequenciesIndices[i];
    }
    if((LSUPlayedFrequenciesArray[i] > frequency)&&(placementIndex==numActuators-1)){
      placementIndex = i;
    }
    if(LSUPlayedFrequenciesArray[i] == frequency){
      LSUPlayedFrequenciesCount[i] = numActuators;
      foundIndex = i;
    }
    if(minRemaining > LSUPlayedFrequenciesCount[i]){
      minRemaining = LSUPlayedFrequenciesCount[i];
      minRemainingIndex = i;
    }
  }

  if(foundIndex == -1){
    for(int i = minRemainingIndex; i  < numActuators - 1; i++){
      LSUPlayedFrequenciesArray[i] = LSUPlayedFrequenciesArray[i+1];
      LSUPlayedFrequenciesCount[i] = LSUPlayedFrequenciesCount[i+1];
    }
    for(int i = numActuators - 2; i  >= placementIndex ; i--){
      LSUPlayedFrequenciesArray[i+1] = LSUPlayedFrequenciesArray[i];
      LSUPlayedFrequenciesCount[i+1] = LSUPlayedFrequenciesCount[i];
    }
    LSUPlayedFrequenciesArray[placementIndex] = frequency;
    LSUPlayedFrequenciesCount[placementIndex] = numActuators;
    foundIndex = placementIndex;
  }


  
  for(int i = 0; i < numActuators; i++){
    LSUPlayedFrequenciesCount[i] --;
  }
  return playableFrequenciesIndices[foundIndex];
}

void ARAM_METAL_ORCHESTRA_INSTRUMENT::_clearLSUtable(){
  for(int i = 0; i < MAX_ARAM_TOGGLE_ACTUATORS_IN_INSTRUMENT; i++){
    this->actuators[i] = NULL;
    LSUPlayedFrequenciesArray[i] = -1;
    LSUPlayedFrequenciesCount[i] = 0;
  }
}


void ARAM_METAL_ORCHESTRA_INSTRUMENT::setInstrumentMode(ARAM_METAL_ORCHESTRA_INSTRUMENT_MODE newMode){
  mode = newMode;
  if(mode == ARAM_METAL_ORCHESTRA_Resonator){
    for(int i = 0; i < numActuators; i++){
      actuators[i]->setActuatorMode(ARAM_TOGGLE_ACTUATOR_Frequency_Generator);
    }
  }
}

void ARAM_METAL_ORCHESTRA_INSTRUMENT::setStrikeMode(ARAM_METAL_ORCHESTRA_STRIKE_MODE newMode){
  strikeMode = newMode;
}
void ARAM_METAL_ORCHESTRA_INSTRUMENT::setMappingMode(ARAM_METAL_ORCHESTRA_MAPPING_MODE newMode){
  mappingMode = newMode;
}


int ARAM_METAL_ORCHESTRA_INSTRUMENT::addToggleActuator(ARAM_TOGGLE_ACTUATOR & actuatator){
  for(int i = 0; i < MAX_ARAM_TOGGLE_ACTUATORS_IN_INSTRUMENT; i++){
    if((this->actuators[i] == NULL)){
      actuators[i] = &(actuatator);
      _addPlayableFrequency(actuatator.getFrequency(),i);
      numActuators ++;
      return i;
    }
  }
  return 0;
}

void ARAM_METAL_ORCHESTRA_INSTRUMENT::update(){
  for(int i = 0; i < MAX_ARAM_TOGGLE_ACTUATORS_IN_INSTRUMENT; i++){
    if((this->actuators[i] == NULL))return;
    actuators[i]->update();
    if(i%3==1)yield();
  }
}

void ARAM_METAL_ORCHESTRA_INSTRUMENT::_addPlayableFrequency(double frequency, int index){
  int addedIndex = -1;
  if( maxFrequency < frequency ){
    maxFrequency = frequency;
    maxFrequencyIndex = index;
  }
  for(int i = 0; i < MAX_ARAM_TOGGLE_ACTUATORS_IN_INSTRUMENT; i++){
    if( (playableFrequenciesIndices[i]==-1)||( (playableFrequencies[i] > frequency) ) ){
      int addedIndex = i;
      for(int z = MAX_ARAM_TOGGLE_ACTUATORS_IN_INSTRUMENT-1; z > addedIndex; z--){
        playableFrequenciesIndices[z] = playableFrequenciesIndices[z-1];
        playableFrequencies[z] = playableFrequencies[z-1];
      }
      playableFrequenciesIndices[i] = index;
      playableFrequencies[i] = frequency;
      break;
    }
  }
}


int  ARAM_METAL_ORCHESTRA_INSTRUMENT::_getIndexOfActuatorWithClosestFrequency(double targetFrequency){
  if(targetFrequency >= maxFrequency){
    return maxFrequencyIndex;
  }
  for(int i = 0; i < numActuators; i++){
    if(playableFrequencies[i] > targetFrequency){
      if(i > 0){
        if( (targetFrequency - playableFrequencies[i-1]) < (playableFrequencies[i] - targetFrequency) ){
          return playableFrequenciesIndices[i-1];
        }
      }
      return playableFrequenciesIndices[i];
    }
  }
  return -1;
}


void ARAM_METAL_ORCHESTRA_INSTRUMENT::playFrequency(double frequency, int milliseconds){
  int index = _getIndexOfActuatorWithClosestFrequency(frequency);
  if(index == -1) return;
  ARAM_TOGGLE_ACTUATOR * actuator = actuators[index];
  if(mode == ARAM_METAL_ORCHESTRA_Resonator){
    actuator->vibrateAtFrequency(frequency);
    actuator->schedule(milliseconds);
  }else if(mode == ARAM_METAL_ORCHESTRA_Striker){
    actuator->schedule(milliseconds);
    actuator->strike();
  }
}

// Callback when a Note On message is received
void ARAM_METAL_ORCHESTRA_INSTRUMENT::handleNoteOn(uint8_t channel, uint8_t pitch, uint8_t velocity, uint16_t timestamp) {
  /*actuators[0]->vibrateAtFrequency((double)midiNoteToFrequency(pitch));
  return;*/
  if (velocity <= 0) {
    handleNoteOff(channel, pitch, 0,timestamp);
    return;
  }

  int timing = 0;
  double frequency = midiNoteToFrequency(pitch);
  if(strikeMode == ARAM_METAL_ORCHESTRA_Continous || mode == ARAM_METAL_ORCHESTRA_Resonator){
    timing = 1000;
  }else if(strikeMode == ARAM_METAL_ORCHESTRA_Impulse){
    timing = 50 + ((int)velocity/4);
  }

  int actInd = -1;
  if(mappingMode == ARAM_METAL_ORCHESTRA_LRU){
    actInd = _getLSUActuatorIndex((double)frequency);
    playingFrequencies[actInd] = frequency;
  }else if(mappingMode == ARAM_METAL_ORCHESTRA_Closest){
    actInd = _getIndexOfActuatorWithClosestFrequency(frequency);
  }
  if(actInd == -1 ) return;
  ARAM_TOGGLE_ACTUATOR * actuator = actuators[actInd];
  
  if(mode == ARAM_METAL_ORCHESTRA_Resonator){
    actuator->setVolume(ARAM_TOGGLE_ACTUATOR_VOLUME_DAMPING_CYCLES*(velocity/127.0));
    actuator->vibrateAtFrequency(frequency);
  }else if(mode == ARAM_METAL_ORCHESTRA_Striker){
    actuator->schedule(timing);
    actuator->strike();
  }

}

// Callback when a Note Off message is received
void ARAM_METAL_ORCHESTRA_INSTRUMENT::handleNoteOff(uint8_t channel, uint8_t pitch, uint8_t velocity, uint16_t timestamp) {

  if(!((strikeMode == ARAM_METAL_ORCHESTRA_Continous )||( mode == ARAM_METAL_ORCHESTRA_Resonator))) return;
  double frequency = midiNoteToFrequency(pitch);

  int actInd = -1;
  if(mappingMode == ARAM_METAL_ORCHESTRA_LRU){
    for(int i = 0; i < numActuators; i++){
      if(playingFrequencies[i] == frequency){
        actInd = i;
        playingFrequencies[i] = -1;
        break;
      }
    }
  }else if(mappingMode == ARAM_METAL_ORCHESTRA_Closest){
    actInd = _getIndexOfActuatorWithClosestFrequency(frequency);
  }
  if(actInd == -1 ) return;
  ARAM_TOGGLE_ACTUATOR * actuator = actuators[actInd];

  actuator->endMovement();
  return;
}

#endif


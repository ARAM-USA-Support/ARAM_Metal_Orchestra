//ARAM_Toggle_Actuator Library      -  Jack Serlin                rev: 6/30/2026               A.R.A.M. - American Robotics Assisted Manufacturing

#ifndef ARAM_MOTCA_0
#define ARAM_MOTCA_0

#if defined(ESP32)
  #include <soc/gpio_reg.h>
  #include <soc/io_mux_reg.h>
#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  #define IS_ARDUINO_ARCH
#else
  #define IS_GENERIC_ARCH
#endif


#define SET_RAW_PIN_OUT_HIGH(reg,a) reg |= (1<<a)
#define SET_RAW_PIN_OUT_LOW(reg,a) reg &= (~(1<<a))

#define MAX_SCHEDULED_NOTES 100
#define MICROS_TO_MILIS (1000)

#if defined(ESP32)
void fastWrite(uint8_t pin, uint8_t state){
  if((pin < 0)||pin>(48))return;
  if(pin <= 31){
    if(state==HIGH){
      *(volatile uint32_t*)(GPIO_OUT_W1TS_REG) = 1<<pin;
    }else{
      *(volatile uint32_t*)(GPIO_OUT_W1TC_REG) = 1<<pin;
    }
    return;
  }

  #if defined(GPIO_OUT1_W1TS_REG)
  if(state==HIGH){
    *(volatile uint32_t*)(GPIO_OUT1_W1TS_REG) = 1<<(pin - 32);
  }else{
    *(volatile uint32_t*)(GPIO_OUT1_W1TC_REG) = 1<<(pin - 32);
  }
  return;
  #endif
}
#else
void fastWrite(uint8_t pin, uint8_t state){
  digitalWrite(pin,state);
}
#endif
#define ARAM_TOGGLE_ACTUATOR_DEFAULT_VOLTAGE (4.5)
#define ARAM_TOGGLE_ACTUATOR_ASSUMED_FRICTION (0.05)
#define ARAM_TOGGLE_ACTUATOR_ASSUMED_FG (0.008*9.8)
#define ARAM_TOGGLE_ACTUATOR_ASSUMED_PLUNGER_MASS (0.007)
#define ARAM_TOGGLE_ACTUATOR_ASSUMED_FORCE_PER_VOLT (0.22/5)
#define ARAM_TOGGLE_ACTUATOR_ASSUMED_PLUNGER_TRAVEL (0.0045)
#define ARAM_TOGGLE_ACTUATOR_MIN_PULSE_INTERVAL (5)
#define ARAM_TOGGLE_ACTUATOR_DEFAULT_STRIKE_WAIT (0)
#define ARAM_TOGGLE_ACTUATOR_MAX_TOTAL_VIBRATION_TIME (2000)
#define ARAM_TOGGLE_ACTUATOR_HIT_TIME_OFF_FLAG (1)
#define ARAM_TOGGLE_ACTUATOR_VOLUME_DAMPING_CYCLES (10)
#define ARAM_TOGGLE_ACTUATOR_MAX_HARMONICS (8)
#define ARAM_TOGGLE_ACTUATOR_DUTY_RESOLUTION (9)
#define ARAM_TOGGLE_ACTUATOR_MAX_CHANNELS (6)
#define ARAM_TOGGLE_ACTUATOR_VOID_CHANNEL_CONST (255)
#define ARAM_TOGGLE_DAMPING_FREQ_RESOLUTION (24)
#define ARAM_TOGGLE_DAMPING_FREQ_MAX (1200.0)

enum ARAM_TOGGLE_ACTUATOR_MODE {ARAM_TOGGLE_ACTUATOR_Actuator,ARAM_TOGGLE_ACTUATOR_Frequency_Generator};
enum ARAM_TOGGLE_ACTUATOR_VIBRATION_MODE {ARAM_TOGGLE_ACTUATOR_Vibration_None,ARAM_TOGGLE_ACTUATOR_Vibration_On,ARAM_TOGGLE_ACTUATOR_Vibration_Full_Stroke,ARAM_TOGGLE_ACTUATOR_Vibration_Pitch};

static int usedChannels [6] = {0,0,0,0,0,0};
//ARAM toggle actuator object, represents a toggle actuator
class ARAM_TOGGLE_ACTUATOR{


    public:
    ARAM_TOGGLE_ACTUATOR(){

    }
    //define with frequency for use in instrument
    ARAM_TOGGLE_ACTUATOR(int pin, double frequency = 0){
       
      pinMode(pin,OUTPUT);
      fastWrite(pin,LOW);

      //store pin data into struct
      this->pin = pin;
      setFrequency(frequency);
      this->curState = 0;
      this->timeClock = micros();
      this->nextSwitch = 0;
      this->isSilent = 0;
      this->doBeat = 0;
      
      travelTimeUp = 1;
      travelTimeDown = 1;

      vibrationMode = ARAM_TOGGLE_ACTUATOR_Vibration_On;
      setActuatorMode(ARAM_TOGGLE_ACTUATOR_Frequency_Generator);
      resetFrequencyDampingTable();
      
      isPerpToGravity = 0;
      setVoltage(ARAM_TOGGLE_ACTUATOR_DEFAULT_VOLTAGE);
      this->hitTarget = 0;
      nextScheduledSwitch = 0;
      nextHitTime = 0;
      currentlyStruck = 0;
      strikeVibrationDelay = ARAM_TOGGLE_ACTUATOR_DEFAULT_STRIKE_WAIT;
      doVolumeDamping = 0;
      volume = ARAM_TOGGLE_ACTUATOR_VOLUME_DAMPING_CYCLES;
      curVolumeCycle = 0;
      harmonicTarget = 1;
      frequencyAutoHarmonicCutoff = 100;
      _updateVibrationTime();

      isScheduledToStop = 0;
      hasCompletedStop = 0;
      scheduledFrequency = -1;
      targetFreq = frequency;
      glideRate = 1; 
      isLEDCAttached = 0;
      _channel = ARAM_TOGGLE_ACTUATOR_VOID_CHANNEL_CONST;
      turnOnLEDCMode();
      _resolution = ARAM_TOGGLE_ACTUATOR_DUTY_RESOLUTION;

    }

    void strike( int );
    void retract(int );
    void update();
    void schedule(int time);
    void generateBeat(int strikeLength, int timing);
    void stopBeat();
    double getFrequency();
    void clearSchedule();


    void vibrateAtFrequency(double frequency);
    void setVoltage(double);
    void setVibrationMode(ARAM_TOGGLE_ACTUATOR_VIBRATION_MODE vibrationMode);
    void setFrequency(double frequency);
    void setStrikeDelay(int delay);
    void setActuatorMode(ARAM_TOGGLE_ACTUATOR_MODE actuatorMode);
    void endMovement();
    void setVolumeDamping(int newValue);
    void setVolume(int volume);
    void setFrequencyHarmonic(int harmonic);
    void setHarmonicFrequencyCutoff(double frequencyCutoff);

    void turnOffLEDCMode();
    void turnOnLEDCMode();

    void resetFrequencyDampingTable();
    void setFrequencyDamping(double minFrequency, double maxFrequency, double dampingAmount);

    //private:
    int pin;
    int curState;
    uint32_t timeClock;
    uint32_t nextSwitch;
    int nextScheduledSwitch;
    int isSilent;
    int doBeat;
    int beatStikeLength;
    int beatTiming;
    double freq;

    int hitTarget;
    int travelTimeUp;
    int travelTimeDown;
    double voltage;
    uint32_t nextHitTime;
    int currentlyStruck;

    uint32_t vibrationTimeUp;
    uint32_t vibrationTimeDown;
    int strikeVibrationDelay;
    ARAM_TOGGLE_ACTUATOR_VIBRATION_MODE vibrationMode;
    ARAM_TOGGLE_ACTUATOR_MODE actuatorMode;

    int doVolumeDamping;
    int volume;
    int curVolumeCycle;

    int harmonicTarget;
    double frequencyAutoHarmonicCutoff;

    int isPerpToGravity;

    int isScheduledToStop;
    int hasCompletedStop;
    double scheduledFrequency;

    double targetFreq;
    double glideRate; // Hz per update cycle

    int isLEDCMode;
    int isLEDCAttached;
    uint32_t _LEDCRampTimer;
    double _LEDCDynamicRampTimer;
    uint8_t _resolution;
    uint8_t _channel; 

    double frequencyDamping [ARAM_TOGGLE_DAMPING_FREQ_RESOLUTION];


    void _updateStepperPinOutputs();
    void _updateVibrationTime();
    void _updateFrequencyVibrationTime(double frequency);

    void dettachLEDC();
    void attachLEDC();

  double getVibrateRatio(double frequency){
    double ratio = ((double)(travelTimeUp))/(travelTimeUp+travelTimeDown);

    double adj = 0;
    /*double maxAdjFreq = 890;
    double minAdjFreq2 = 450;
    double minAdjFreq3 = 300;
    double minAdjFreq4 = 150;
    if(frequency < maxAdjFreq){
      adj =  0.85*(1-ratio) + ratio;
      if(frequency > minAdjFreq2){
        adj = 0.20*(1-ratio) + ratio;
      }
      if(frequency > minAdjFreq3){
        adj = 0.40*(1-ratio) + ratio;
      }
      if(frequency > minAdjFreq4){
        adj = 0.67*(1-ratio) + ratio;
      }
    }
    return ratio * (1  - adj) + adj;
    */
    double frequencyStep = ARAM_TOGGLE_DAMPING_FREQ_MAX/ARAM_TOGGLE_DAMPING_FREQ_RESOLUTION;
    int frequencyDampingTableIndex = frequency / frequencyStep;
    if(frequencyDampingTableIndex < 0) frequencyDampingTableIndex = 0;
    else if(frequencyDampingTableIndex >= ARAM_TOGGLE_DAMPING_FREQ_RESOLUTION) frequencyDampingTableIndex = ARAM_TOGGLE_DAMPING_FREQ_RESOLUTION-1;
    double adjAmt = frequencyDamping[frequencyDampingTableIndex];
    adj = adjAmt*(1-ratio) + ratio;
    
    return ratio + (1-ratio)*adjAmt;
  }

  

  void setTone(double frequency) {
    uint32_t frequint = (uint32_t)(frequency + 0.5);
    double ratio = getVibrateRatio(frequency);
    // 1. Constrain ratio to safe bounds (0.0 to 1.0)
    if (ratio < 0.0) ratio = 0.0;
    if (ratio > 1.0) ratio = 1.0;
    ratio = 0.75;

    // 2. Calculate the raw integer duty cycle value based on the resolution
    uint32_t maxDuty = (1 << _resolution) - 1; // e.g., 255 for 8-bit
    uint32_t dutyValue = (uint32_t)(ratio * maxDuty);

    // 3. Update the hardware registers
    #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
      // Core v3.x API
      ledcChangeFrequency(pin, frequint, _resolution);
      ledcWrite(pin, dutyValue);
    #else
      // Core v2.x API
      ledcSetup(_channel, frequint, _resolution);
      ledcWrite(_channel, dutyValue);
    #endif
  }

  void setQuiet() {
    uint32_t frequint = (uint32_t)(freq);
    targetFreq = freq;

    uint32_t maxDuty = (1 << _resolution); 
    uint32_t dutyValue = (uint32_t)( maxDuty);

    // 3. Update the hardware registers
    #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
      // Core v3.x API
      ledcChangeFrequency(pin, frequint, _resolution);
      ledcWrite(pin, dutyValue);
    #else
      // Core v2.x API
      ledcSetup(_channel, frequint, _resolution);
      ledcWrite(_channel, dutyValue);
    #endif
  }

  void assignChannel(){
    if(_channel != ARAM_TOGGLE_ACTUATOR_VOID_CHANNEL_CONST) return;
    for(int i = 0; i < ARAM_TOGGLE_ACTUATOR_MAX_CHANNELS; i++){
      if(usedChannels[i]==0){
        usedChannels[i] = 1;
        _channel = i;
        return;
      }
    }
  }
  void yeildChannel(){
    if(_channel < 0 || _channel >= ARAM_TOGGLE_ACTUATOR_MAX_CHANNELS) return;
    usedChannels[_channel] = 0;
    _channel = ARAM_TOGGLE_ACTUATOR_VOID_CHANNEL_CONST;
  }

};

void ARAM_TOGGLE_ACTUATOR::resetFrequencyDampingTable(){

    double frequencyStep = ARAM_TOGGLE_DAMPING_FREQ_MAX/ARAM_TOGGLE_DAMPING_FREQ_RESOLUTION;
    double frequency = 0;
    for(int i = 0; i < ARAM_TOGGLE_DAMPING_FREQ_RESOLUTION; i++){
      frequencyDamping[i] = 0;
      frequency = (1+i)*frequencyStep;
      if(frequency < 890){
        frequencyDamping[i] =  0.85;
        if(frequency > 450){
          frequencyDamping[i] = 0.20;
        }
        if(frequency > 300){
          frequencyDamping[i] = 0.40;
        }
        if(frequency > 150){
          frequencyDamping[i] = 0.67;
        }
      }
    }

  }

  void ARAM_TOGGLE_ACTUATOR::setFrequencyDamping(double minFrequency, double maxFrequency, double dampingAmount){

    double frequencyStep = ARAM_TOGGLE_DAMPING_FREQ_MAX/ARAM_TOGGLE_DAMPING_FREQ_RESOLUTION;
    double frequency = 0;
    for(int i = 0; i < ARAM_TOGGLE_DAMPING_FREQ_RESOLUTION; i++){
      frequency = (1+i)*frequencyStep;
      if( (minFrequency > frequency)||(maxFrequency <= frequency) ){
        continue;
      }
      frequencyDamping[i] = dampingAmount;
    }

  }

void ARAM_TOGGLE_ACTUATOR::dettachLEDC(){
  if(!isLEDCAttached)return;
  isLEDCAttached = 0;
  // 1. Detach from LEDC (Version-compatible check)
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcDetach(pin); 
  #else
    ledcDetachPin(pin);
  #endif

  // 2. Set as standard output
  pinMode(pin, OUTPUT);
  yeildChannel();
}

void ARAM_TOGGLE_ACTUATOR::attachLEDC(){
  assignChannel();
  if(isLEDCAttached)return;
  isLEDCAttached = 1;
  // Re-attach to the hardware PWM (Version-compatible check)
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcAttach(pin, (uint32_t)freq, _resolution);
  #else
    ledcSetup(_channel, (uint32_t)freq, _resolution);
    ledcAttachPin(pin, _channel);
  #endif
}


void ARAM_TOGGLE_ACTUATOR::turnOffLEDCMode(){
  isLEDCMode = 0;
  dettachLEDC();
}

void ARAM_TOGGLE_ACTUATOR::turnOnLEDCMode(){
  isLEDCMode = 1;
  //attachLEDC();
}

//Do volume damping option
void ARAM_TOGGLE_ACTUATOR::setVolumeDamping(int newValue){
  doVolumeDamping = newValue;
}

//used to set volume, only has effect if doVolumeDamping is on
void  ARAM_TOGGLE_ACTUATOR::setVolume(int volumeIn){
  if(volumeIn > ARAM_TOGGLE_ACTUATOR_VOLUME_DAMPING_CYCLES)volumeIn = ARAM_TOGGLE_ACTUATOR_VOLUME_DAMPING_CYCLES;
  if(volumeIn < 0) volumeIn = 0;
  volume = volumeIn;
}

//set current standard harmonic setting - 1 is default
void ARAM_TOGGLE_ACTUATOR::setFrequencyHarmonic(int harmonic){
  if(harmonic < 1) harmonic = 1;
  harmonicTarget = harmonic;
  _updateFrequencyVibrationTime(freq);
}

//used to establish a lowest frequency before utilizing harmonics instead
void ARAM_TOGGLE_ACTUATOR::setHarmonicFrequencyCutoff(double frequencyCutoff){
  if(frequencyCutoff < 0) frequencyCutoff = 0;
  frequencyAutoHarmonicCutoff = frequencyCutoff;
  _updateFrequencyVibrationTime(freq);
}

//used to update vibration params
void  ARAM_TOGGLE_ACTUATOR::_updateFrequencyVibrationTime(double frequency){
  if(frequency <=0) return;
  int oldHarmonicTarget = harmonicTarget;
  if(frequencyAutoHarmonicCutoff/2 > frequency){
    int temp = 2;
    double tempCutOff = frequencyAutoHarmonicCutoff/2;
    while(tempCutOff >  frequency){
      temp += 1;
      tempCutOff = frequencyAutoHarmonicCutoff/temp;
    }
    if(temp > ARAM_TOGGLE_ACTUATOR_MAX_HARMONICS) temp = ARAM_TOGGLE_ACTUATOR_MAX_HARMONICS;
    harmonicTarget = temp;
  }

  double ratio = (((double)travelTimeDown)/travelTimeUp)/1.5;
  uint32_t period = (uint32_t)((1000000.0/(frequency*harmonicTarget)));
  uint32_t pitchResonantPeriod = period/(2*ratio);//(int)(freq/100);//
  uint32_t maximumTime = ((uint32_t)(travelTimeUp/2))*1000;
  uint32_t minimumTime = ARAM_TOGGLE_ACTUATOR_MIN_PULSE_INTERVAL;
  if(pitchResonantPeriod > maximumTime){
    vibrationTimeUp = maximumTime;
  }
  else if(pitchResonantPeriod < minimumTime){
    vibrationTimeUp = minimumTime;
  }
  else{
    vibrationTimeUp = pitchResonantPeriod;
  }

  
  maximumTime = (travelTimeDown/2)*1000;
  pitchResonantPeriod = (uint32_t)((1000000.0/frequency)*ratio)/(2);

  if(pitchResonantPeriod > maximumTime){
    vibrationTimeDown = maximumTime;
  }
  else if(pitchResonantPeriod < minimumTime){
    vibrationTimeDown = minimumTime;
  }
  else{
    vibrationTimeDown = pitchResonantPeriod;
  }

  vibrationTimeUp += (period)*(harmonicTarget - 1);

  harmonicTarget = oldHarmonicTarget;

}

//end current movement
void ARAM_TOGGLE_ACTUATOR::endMovement(){

  if(isLEDCAttached){
    //dettachLEDC();
    //strike(1);
    setQuiet();
    nextSwitch = 0;
    nextHitTime = ARAM_TOGGLE_ACTUATOR_HIT_TIME_OFF_FLAG;
    this->currentlyStruck = 0; 
    this->hitTarget = false; 
    nextScheduledSwitch = 0;
    return;
  }


  if((!isScheduledToStop)&&!(hasCompletedStop)){
    isScheduledToStop = 1;
    hasCompletedStop = 0;
    return;
  }
  if(!hasCompletedStop) return;


  //if we there is a vibration scheduled, start it now
  if(scheduledFrequency!= -1){
    //nextHitTime = 0;
    currentlyStruck = 1;
    this->hitTarget = true;
    vibrationMode = ARAM_TOGGLE_ACTUATOR_Vibration_Pitch;
    targetFreq = scheduledFrequency;
    nextSwitch = micros() + ARAM_TOGGLE_ACTUATOR_MAX_TOTAL_VIBRATION_TIME*MICROS_TO_MILIS;
    scheduledFrequency = -1;
    nextHitTime = micros() +  ((uint32_t)this->vibrationTimeDown);
    curState = 1;
    _updateStepperPinOutputs();
    nextScheduledSwitch = 0;
    isScheduledToStop = 0;
    hasCompletedStop = 0;
    return;
  }
  else if( ( actuatorMode == ARAM_TOGGLE_ACTUATOR_Frequency_Generator) ){
    //this->strike(0);
    nextSwitch = 0;
    nextHitTime = ARAM_TOGGLE_ACTUATOR_HIT_TIME_OFF_FLAG;
    this->currentlyStruck = 0; 
    this->hitTarget = false; 
    //nextScheduledSwitch = 0;
  }
  else if( actuatorMode == ARAM_TOGGLE_ACTUATOR_Actuator){
    //this->retract(0);
    nextSwitch = 0;
    nextScheduledSwitch = 0;
  }

  hasCompletedStop = 0;
 

}

//used to update vibration params depending on actuator state
void ARAM_TOGGLE_ACTUATOR::_updateVibrationTime(){
  if(vibrationMode == ARAM_TOGGLE_ACTUATOR_Vibration_Pitch){
    _updateFrequencyVibrationTime(freq);
  }
  else if(vibrationMode == ARAM_TOGGLE_ACTUATOR_Vibration_On){
    vibrationTimeDown = travelTimeDown/2;
    vibrationTimeUp = travelTimeUp;
  }else if(vibrationMode == ARAM_TOGGLE_ACTUATOR_Vibration_Full_Stroke){
    vibrationTimeDown = travelTimeDown + strikeVibrationDelay;
    vibrationTimeUp = travelTimeUp + strikeVibrationDelay;
  }
}

//sets actuators frequency
void ARAM_TOGGLE_ACTUATOR::setFrequency(double frequency){
  freq = frequency;
  _updateVibrationTime();
}

//causes actuator to vibrate at a specific frequency
void ARAM_TOGGLE_ACTUATOR::vibrateAtFrequency(double frequency){
  if(isLEDCMode){
    attachLEDC();
    targetFreq = frequency;
    freq = frequency;
    setTone(targetFreq);
    nextSwitch = micros() + ARAM_TOGGLE_ACTUATOR_MAX_TOTAL_VIBRATION_TIME*MICROS_TO_MILIS;
    return;
  }
  if(nextSwitch==0){
    //strike(0);
    nextSwitch = 0;
    nextHitTime = 0;
    currentlyStruck = 1;
    this->hitTarget = true;
    vibrationMode = ARAM_TOGGLE_ACTUATOR_Vibration_Pitch;
    _updateFrequencyVibrationTime(frequency);
    //schedule(ARAM_TOGGLE_ACTUATOR_MAX_TOTAL_VIBRATION_TIME);
    nextSwitch = micros() + ARAM_TOGGLE_ACTUATOR_MAX_TOTAL_VIBRATION_TIME*MICROS_TO_MILIS;

  }else{
    /*endMovement();
    scheduledFrequency = frequency;*/

    targetFreq = frequency;
    nextSwitch = micros() + ARAM_TOGGLE_ACTUATOR_MAX_TOTAL_VIBRATION_TIME*MICROS_TO_MILIS;
  }
  //vibrationMode = ARAM_TOGGLE_ACTUATOR_Vibration_Pitch;
  //_updateFrequencyVibrationTime(frequency);
  //schedule(ARAM_TOGGLE_ACTUATOR_MAX_TOTAL_VIBRATION_TIME);
  //nextSwitch = micros() + ARAM_TOGGLE_ACTUATOR_MAX_TOTAL_VIBRATION_TIME*MICROS_TO_MILIS;


}


//calualtes time of travel when gravity is opposed to plunger
int __calculateTravelTimeAgainstGravity(double volts){
  return 1000*sqrt(2*(ARAM_TOGGLE_ACTUATOR_ASSUMED_PLUNGER_TRAVEL)*ARAM_TOGGLE_ACTUATOR_ASSUMED_PLUNGER_MASS/(ARAM_TOGGLE_ACTUATOR_ASSUMED_FORCE_PER_VOLT*volts - ARAM_TOGGLE_ACTUATOR_ASSUMED_FG - ARAM_TOGGLE_ACTUATOR_ASSUMED_FRICTION));
}

//calulates time of travel when gravity is with plunger
int __calculateTravelTimeWithGravity(double volts){
  return 1000*sqrt(2*(ARAM_TOGGLE_ACTUATOR_ASSUMED_PLUNGER_TRAVEL)*ARAM_TOGGLE_ACTUATOR_ASSUMED_PLUNGER_MASS/(ARAM_TOGGLE_ACTUATOR_ASSUMED_FORCE_PER_VOLT*volts + ARAM_TOGGLE_ACTUATOR_ASSUMED_FG - ARAM_TOGGLE_ACTUATOR_ASSUMED_FRICTION));
}

//calculates time of travel when plunger is perpendicular to gravity
int __calculateTravelTimeNoGravity(double volts){
  return 1000*sqrt(2*(ARAM_TOGGLE_ACTUATOR_ASSUMED_PLUNGER_TRAVEL)*ARAM_TOGGLE_ACTUATOR_ASSUMED_PLUNGER_MASS/(ARAM_TOGGLE_ACTUATOR_ASSUMED_FORCE_PER_VOLT*volts - ARAM_TOGGLE_ACTUATOR_ASSUMED_FRICTION));
}

//sets voltage for actuator - mostly effects travel time calulations
void ARAM_TOGGLE_ACTUATOR::setVoltage(double voltageIn){
  this->voltage = voltageIn;
  if(!isPerpToGravity){
    this->travelTimeUp = __calculateTravelTimeAgainstGravity(voltageIn);
    this->travelTimeDown = __calculateTravelTimeWithGravity(voltageIn);
  }else{

    this->travelTimeUp = __calculateTravelTimeNoGravity(voltageIn);
    this->travelTimeDown = this->travelTimeUp;
  }
  _updateVibrationTime();
}

//sets actuator vibration mode
void ARAM_TOGGLE_ACTUATOR::setVibrationMode(ARAM_TOGGLE_ACTUATOR_VIBRATION_MODE vibrationMode){
  this->vibrationMode = vibrationMode;
}

//set actuator mode
void ARAM_TOGGLE_ACTUATOR::setActuatorMode(ARAM_TOGGLE_ACTUATOR_MODE actuatorMode){
  this->actuatorMode = actuatorMode;
  if(actuatorMode == ARAM_TOGGLE_ACTUATOR_Frequency_Generator){
    turnOnLEDCMode();
    strike(1);
  }else{
    turnOffLEDCMode();
    retract(1);
  }
}

//sets strike delay (hit duration)
void ARAM_TOGGLE_ACTUATOR::setStrikeDelay(int delay){
  this->strikeVibrationDelay = delay;
}

//clear the actuators scheduled actuations
void ARAM_TOGGLE_ACTUATOR::clearSchedule(){
  nextScheduledSwitch = 0;
}

//updates pin outputs 
void ARAM_TOGGLE_ACTUATOR::_updateStepperPinOutputs(){

  //update pin states
  if(this->curState)fastWrite(this->pin,HIGH);
  else fastWrite(this->pin,LOW);
}

//Causes the actuator to strike
void ARAM_TOGGLE_ACTUATOR::strike(int skipTargetUpdate = 0){
  //update pin outputs
  this->curState = 1;
  _updateStepperPinOutputs();
  if(!skipTargetUpdate){
    this->hitTarget = 0;
    currentlyStruck = 0;
    nextHitTime = micros() +  ((uint32_t)this->travelTimeUp)*MICROS_TO_MILIS;
  }
}

//returns defined frequency for actuator
double ARAM_TOGGLE_ACTUATOR::getFrequency(){
  return this->freq;
}

//Causes Actuator to retract
void ARAM_TOGGLE_ACTUATOR::retract(int skipTargetUpdate = 0){
  //update pin outputs
  this->curState = 0;
  _updateStepperPinOutputs();
  if(!skipTargetUpdate){
    this->hitTarget = 0;
    currentlyStruck = 0;
    nextHitTime = micros() +  ((uint32_t)this->travelTimeDown)*MICROS_TO_MILIS;
  }
}

//stops automated beat generation
void ARAM_TOGGLE_ACTUATOR::stopBeat(){
  this->doBeat = 0;
}

//starts automated beat generation, strike length defines hit duration, timing defines interval between strikes
void ARAM_TOGGLE_ACTUATOR::generateBeat( int strikeLength, int timing ){
  this->doBeat = 1;
  this->beatStikeLength = strikeLength;
  this->beatTiming = timing;
}

//updates the actuators state, call this during loop to utilize scheduling
void ARAM_TOGGLE_ACTUATOR::update(){

  // Glide towards the target frequency if they do not match
  if (freq != targetFreq) {
    if (freq < targetFreq) {
      freq += glideRate;
      if (freq > targetFreq) freq = targetFreq;
    } else {
      freq -= glideRate;
      if (freq < targetFreq) freq = targetFreq;
    }
    _updateVibrationTime(); // Recalculate timing parameters on the fly
    if(isLEDCMode){
      attachLEDC();
      setTone(freq);
    }
  }

  timeClock = micros();
  if(nextSwitch==0){
    if(isScheduledToStop){
      isScheduledToStop = 0;
      hasCompletedStop = 1;
      endMovement();
      return;
    }
    if(this->nextScheduledSwitch == 0){
      if(!this->doBeat) return;
      if(!(this->curState)){
        strike(0);
        this->nextScheduledSwitch = ((uint32_t)this->beatStikeLength)*MICROS_TO_MILIS;
      }
      else{
        retract(0);
        this->nextScheduledSwitch = ((uint32_t)this->beatTiming)*MICROS_TO_MILIS - (((uint32_t)this->beatStikeLength)*MICROS_TO_MILIS);
      }
    } 
    nextSwitch = micros()+ this->nextScheduledSwitch;
    nextScheduledSwitch = 0;
  }
  if(timeClock > nextSwitch){
    if(doBeat) { nextSwitch = 0; return; }
    if((isScheduledToStop||hasCompletedStop)&&(actuatorMode==ARAM_TOGGLE_ACTUATOR_Frequency_Generator)){
      isScheduledToStop = 0;
      hasCompletedStop = 1;
      endMovement();
      return;
    }
    isScheduledToStop = 1;
    if((actuatorMode==ARAM_TOGGLE_ACTUATOR_Frequency_Generator)){
      nextSwitch = micros() +  ((uint32_t)this->vibrationTimeUp);
      strike(1);
    }else{
      nextSwitch = micros() +  ((uint32_t)this->vibrationTimeDown);
      retract(1);
    }
    nextHitTime = ARAM_TOGGLE_ACTUATOR_HIT_TIME_OFF_FLAG;
  }else if((nextHitTime!=ARAM_TOGGLE_ACTUATOR_HIT_TIME_OFF_FLAG)&&(timeClock > nextHitTime)){
    if(!this->hitTarget){
      this->hitTarget = true;
      if(curState){
        currentlyStruck = 1;
      }
      if((curState&&(vibrationMode != ARAM_TOGGLE_ACTUATOR_Vibration_None))&&(actuatorMode==ARAM_TOGGLE_ACTUATOR_Actuator)){
        nextHitTime = micros() +  ((uint32_t)strikeVibrationDelay)*MICROS_TO_MILIS;
      }
    }else if(nextHitTime!=ARAM_TOGGLE_ACTUATOR_HIT_TIME_OFF_FLAG){
      if(currentlyStruck && (vibrationMode != ARAM_TOGGLE_ACTUATOR_Vibration_None)){

        if(isLEDCMode){

          if(isScheduledToStop){
            endMovement();
          }

          return;
        }


        if(this->curState){
          if((isScheduledToStop||hasCompletedStop)&&(actuatorMode==ARAM_TOGGLE_ACTUATOR_Frequency_Generator)){
            isScheduledToStop = 0;
            hasCompletedStop = 1;
            endMovement();
            return;
          }
          curVolumeCycle ++;
          if((curVolumeCycle < volume)||(doVolumeDamping==0)){
            nextHitTime = micros() +  ((uint32_t)this->vibrationTimeDown);
            retract(1);
          }
          else{
            nextHitTime = micros() +  ((uint32_t)(vibrationTimeDown+vibrationTimeUp)*(ARAM_TOGGLE_ACTUATOR_VOLUME_DAMPING_CYCLES-volume));
            curVolumeCycle = 0;
          }
        }else{
          if((isScheduledToStop||hasCompletedStop)&&(actuatorMode==ARAM_TOGGLE_ACTUATOR_Actuator)){
            isScheduledToStop = 0;
            hasCompletedStop = 1;
            endMovement();
            return;
          }
          nextHitTime = micros() +  ((uint32_t)this->vibrationTimeUp);
          strike(1);
        }
      }
    }
  }
}

//schedule siwth of state in X milliseconds
void ARAM_TOGGLE_ACTUATOR::schedule(int time){
  this->nextScheduledSwitch = ((uint32_t)time)*MICROS_TO_MILIS;
  nextSwitch = micros() + nextScheduledSwitch;
}

#endif
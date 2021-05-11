
#Analog Sensors
uint16  WallSensorInMM              #Example: 34585
uint16  BatteryVoltageInmV          #Example: 16348
uint16  LeftDropInMM                #Example: 0
uint16  RightDropInMM               #Example: 0
int16   LeftMagSensor               #Example: 32768
int16   RightMagSensor              #Example: 32768
int16   UIButtonInmV                #Example: 3330
int16   VacuumCurrentInmA           #Example: 0
uint16  ChargeVoltInmV              #Example: 24024
int8    BatteryTemp0InC             #Example: 30
int8    BatteryTemp1InC             #Example: 28
int16   CurrentInmA                 #Example: 40
int16   SideBrushCurrentInmA        #Example: 0
int16   VoltageReferenceInmV        #Example: 1225
int16   AccelXInmG                  #Example: 36
int16   AccelYInmG                  #Example: 16
int16   AccelZInmG                  #Example: 1008
# the ones below are not supported due to lack of compatibility
# int8    XTemp0InC                   #Example: 28
# int8    XTemp1InC                   #Example: 28
# int8    NotConnected1               #Example: 0 - unknown
# int8    NotConnected2               #Example: 0 - unknown
# int8    NotConnected3               #Example: 0 - unknown

#Digital sensors
bool    SNSR_DC_JACK_CONNECT        #Example: 0 
bool    SNSR_DUSTBIN_IS_IN          #Example: 1
bool    SNSR_LEFT_WHEEL_EXTENDED    #Example: 0
bool    SNSR_RIGHT_WHEEL_EXTENDED   #Example: 0
bool    LSIDEBIT                    #Example: 0 
bool    LFRONTBIT                   #Example: 0
bool    RSIDEBIT                    #Example: 0 
bool    RFRONTBIT                   #Example: 0 


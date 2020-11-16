# Neato Help

   This document contains help documentation acquired from neato by running the script titled get_neato_help.py contained in the project at <https://github.com/brannonvann/neato>.

## Help Command Output from Neato

## Command help

    Help Strlen = 1840
    Help - Without any argument, this prints a list of all possible cmds.
    With a command name, it prints the help for that particular command
    Clean - Starts a cleaning by simulating press of start button.
    DiagTest - Executes different test modes. Once set, press Start button to engage. (Test modes are mutually exclusive.)
    GetAccel - Get the Accelerometer readings.
    GetAnalogSensors - Get the A2D readings for the analog sensors.
    GetButtons - Get the state of the UI Buttons.
    GetCalInfo - Prints out the cal info from the System Control Block.
    GetCharger - Get the diagnostic data for the charging system.
    GetDigitalSensors - Get the state of the digital sensors.
    GetErr - Get Error Message.
    GetLDSScan - Get scan packet from LDS.
    GetMotors - Get the diagnostic data for the motors.
    GetSchedule - Get the Cleaning Schedule. (24 hour clock format)
    GetTime - Get Current Scheduler Time.
    GetVersion - Get the version information for the system software and hardware.
    GetWarranty - Get the warranty validation codes.
    PlaySound - Play the specified sound in the robot.
    RestoreDefaults - Restore user settings to default.
    SetFuelGauge - Set Fuel Gauge Level.
    SetMotor - Sets the specified motor to run in a direction at a requested speed. (TestMode Only)
    SetTime - Sets the current day, hour, and minute for the scheduler clock.
    SetLED - Sets the specified LED to on,off,blink, or dim. (TestMode Only)
    SetIEC - Sets the IEC Cleaning Test parameters
    SetLCD - Sets the LCD to the specified display. (TestMode Only)
    SetLDSRotation - Sets LDS rotation on or off. Can only be run in TestMode.
    SetSchedule - Modify Cleaning Schedule.
    SetSystemMode - Set the operation mode of the robot. (TestMode Only)
    TestMode - Sets TestMode on or off. Some commands can only be run in TestMode.
    Upload - Uploads new program to the robot.
    
    
## Command help Help

    Help Strlen = 262
    Help - Without any argument, this prints a list of all possible cmds.
    With a command name, it prints the help for that particular command
    Cmd - (Optional) Next argument is command to show help for.
    If Cmd option not used, help gives list of all commands.
    
    
## Command help Clean

    Help Strlen = 420
    Clean - Starts a cleaning by simulating press of start button.
    House - (Optional) Equivalent to pressing 'Start' button once.
    Starts a house cleaning.
    (House cleaning mode is the default cleaning mode.)
    Spot - (Optional) Starts a spot clean.
    
    Width - (Optional) Spot Width in CM (100-500)(-1=use default).
    Height - (Optional) Spot Height in CM (100-500)(-1=use default).
    Stop - Stop Cleaning.
    
    
    
## Command help DiagTest

    Help Strlen = 2472
    DiagTest - Executes different test modes. Once set, press Start button to engage. (Test modes are mutually exclusive.)
    TestsOff - Stop Diagnostic Test and clears all diagnostic test modes. 
    
    DrivePath - Sets DrivePath TestMode. Press start button to start. Robot travels straight by commanded distance as path. Mutually exclusive with other diagtest modes. Use 'TestsOff' option to stop.
    
    DriveForever - Sets DriveForever TestMode. Press start button to start. Robot drives continuously. Mutually exclusive with other diagtest modes. Use 'TestsOff' option to stop.
    
    MoveAndBump - Sets Move and Bump TestMode. Press start button to start. Executes canned series of motions, but will react to bumps. Mutually exclusive with other diagtest modes.
    
    DropTest - Enables DropTest. Robot drives forward until a drop is detected. Mutually exclusive with other diagtest modes.
    
    AutoCycle - DropTest argument to enable automatic restart of the test. The robot will drive backwards and then forward until a drop is detected until the test is over.
    
    OneShot - Only executes test once.
    
    BrushOn - Turns on brush during test. May conflict with motor commands of test so use carefully!
    
    VacuumOn - Turns on vacuum during test. May conflict with motor commands of test so use carefully!
    
    LDSOn - Turns on LDS during test. May conflict with motor commands of test so use carefully!
    
    SideBrushOn - Turns on side brush during test. May conflict with motor commands of test so use carefully!
    
    AllMotorsOn - Turns on brush, vacuum, and lds during test. May conflict with motor commands of test so use carefully!
    
    DisablePickupDetect - Ignores pickup (wheel suspension). By default, pickup detect is enabled and stops the test.
    
    DrivePathDist - Distance in mm
    
    DriveForeverLeftDist - Use next arg to set left wheel dist for DriveForever test. Requires DriveForeverRightDist as well. The ratio of this value to DriveForeverRightDist determines turn radius.
    
    DriveForeverRightDist - Use next arg to set right wheel dist for DriveForever test. Requires DriveForeverLeftDist as well. The ratio of this value to DriveForeverLeftDist determines turn radius.
    
    DriveForeverSpeed - Use next arg to set turn speed of outer wheel for DriveForever test in mm/s.
    
    Speed - DropTest argument to set the robot speed in mm/s.
    
    BrushSpeed - DropTest argument to set the speed of the brush in rpm.
    
    
    
## Command help GetAccel

    Help Strlen = 188
    GetAccel - Get the Accelerometer readings.
    brief - Returns a single-line summary. Data order:
      Revision(0)
      Pitch
      Roll
      X Axis
      Y Axis
      Z Axis
    
    
    
## Command help GetAnalogSensors

    Help Strlen = 790
    GetAnalogSensors - Get the A2D readings for the analog sensors.
    raw - Return raw analog sensor values as milliVolts. 
    (Default is sensor values in native units of what they measure.)
    mainboard - Return raw analog sensor values as milliVolts from the mainboard. 
    (Default is sensor values in native units of what they measure.)
    stats - Return stats (avg,max,min,dev,cnt) of raw analog sensor values as milliVolts.
      (Implies 'raw' option)
    brief - Returns a single-line summary of default values. Data order:
      Revision(0)
      Wall Sensor MM
      Battery Voltage mV
      Left Drop MM
      Right Drop MM
      Left Mag
      Right Mag
      Vacuum Current mA
      Charge Voltage mV
      Battery Temp 0 C
      Battery Temp 1 C
      Current mA
    
    
    
## Command help GetButtons

    Help Strlen = 187
    GetButtons - Get the state of the UI Buttons.
    brief - Returns a single-line summary. Data order:
      Revision(0)
      Soft key
      Up
      Start
      Back
      Down
    
    
    
## Command help GetCalInfo

    Help Strlen = 69
    GetCalInfo - Prints out the cal info from the System Control Block.
    
    
## Command help GetCharger

    Help Strlen = 63
    GetCharger - Get the diagnostic data for the charging system.
    
    
## Command help GetDigitalSensors

    Help Strlen = 303
    GetDigitalSensors - Get the state of the digital sensors.
    brief - Returns a single-line summary. Data order:
      Revision(0)
      DC Jack
      Dustbin
      Left Wheel
      Right Wheel
      Left Side Bumper
      Left Front Bumper
      Right Side Bumper
      Right Front Bumper
    
    
    
## Command help GetErr

    Help Strlen = 70
    GetErr - Get Error Message.
    Clear - Dismiss the reported error.
    
    
## Command help GetLDSScan

    Help Strlen = 157
    GetLDSScan - Get scan packet from LDS.
    streamOn - Start streaming of raw (binary) LDS readings
    
    streamOff - Stop streaming of raw LDS readings
    
    
    
## Command help GetMotors

    Help Strlen = 596
    GetMotors - Get the diagnostic data for the motors.
    Brush - Return Brush Motor stats.
    Vacuum - Return Vacuum Motor stats.
    LeftWheel - Return LeftWheel Motor stats.
    RightWheel - Return RightWheel Motor stats.
    Laser - Return LDS Motor stats.
    Charger - Return Battery Charger stats.
    SideBrush - Return Side Brush stats.
    brief - Returns a single-line WHEEL ONLY summary. Data order:
      Revision(0)
      Left Wheel RPM
      Left Wheel Position MM
      Left Wheel Speed
      Right Wheel RPM
      Right Wheel Position MM
      Right Wheel Speed
    
    
    
## Command help GetSchedule

    Help Strlen = 170
    GetSchedule - Get the Cleaning Schedule. (24 hour clock format)
    Day - Day of the week to get schedule for. Sun=0,Sat=6.
     If not specified, then all days are given.
    
    
## Command help GetTime

    Help Strlen = 39
    GetTime - Get Current Scheduler Time.
    
    
## Command help GetVersion

    Help Strlen = 80
    GetVersion - Get the version information for the system software and hardware.
    
    
## Command help GetWarranty

    Help Strlen = 50
    GetWarranty - Get the warranty validation codes.
    
    
## Command help PlaySound

    Help Strlen = 810
    PlaySound - Play the specified sound in the robot.
    SoundID - Play the sound library entry specified by the number in the next argument.
    
    Legal values are:
    
       0 - Waking Up
       1 - Starting Cleaning
       2 - Cleaning Completed
       3 - Attention Needed
       4 - Backing up into base station
       5 - Base Station Docking Completed
       6 - Test Sound 1
       7 - Test Sound 2
       8 - Test Sound 3
       9 - Test Sound 4
       10 - Test Sound 5
       11 - Exploring
       12 - ShutDown
       13 - Picked Up
       14 - Going to sleep
       15 - Returning Home
       16 - User Canceled Cleaning
       17 - User Terminated Cleaning
       18 - Slipped Off Base While Charging
       19 - Alert
       20 - Thank You
    
    Stop - Stop playing sound.
    
    
## Command help RestoreDefaults

    Help Strlen = 53
    RestoreDefaults - Restore user settings to default.
    
    
## Command help SetFuelGauge

    Help Strlen = 86
    SetFuelGauge - Set Fuel Gauge Level.
    Percent - Fuel Gauge percent from 0 to 100
    
    
## Command help SetMotor

    Help Strlen = 1396
    SetMotor - Sets the specified motor to run in a direction at a requested speed. (TestMode Only)
    LWheelDist - Distance in millimeters to drive Left wheel
    		+/- 10000, pos = forward, neg = backward
    RWheelDist - Distance in millimeters to drive Right wheel
    		+/- 10000, pos = forward, neg = backward
    Speed - Speed in millimeters/second
    		0-300, required for wheel movements
    Accel - Acceleration in millimeters/second, 
    		0-300, used only for wheel movements, defaults to 'Speed'
    RPM - The following argument is the RPM of the motor.
    		0-10000, not used for wheels, but applied to all other motors
    Brush - Brush motor (Mutually exclusive with wheels and vacuum.)
    VacuumOn - Vacuum motor on (Mutually exclusive with VacuumOff)
    VacuumOff - Vacuum motor off (Mutually exclusive with VacuumOn)
    VacuumSpeed - Vacuum speed in percent (1-100)
    RWheelDisable - Disable Right Wheel motor
    LWheelDisable - Disable Left Wheel motor
    BrushDisable - Disable Brush motor
    RWheelEnable - Enable Right Wheel motor
    LWheelEnable - Enable Left Wheel motor
    BrushEnable - Enable Brush motor
    SideBrushEnable - Enable Side Brush Motor motor
    SideBrushDisable - Disable Side Brush Motor motor
    SideBrushOn - Enable the Side Brush
    SideBrushOff - Disable the Side Brush
    SideBrushPower - Side Brush maximum power in milliwatts
    
    
## Command help SetTime

    Help Strlen = 276
    SetTime - Sets the current day, hour, and minute for the scheduler clock.
    Day - Day of week value Sunday=0,Monday=1,... (required)
    Hour - Hour value 0..23 (required)
    Min - Minutes value 0..59 (required)
    Sec - Seconds value 0..59 (Optional, defaults to 0)
    
    
## Command help SetLED

    Help Strlen = 822
    SetLED - Sets the specified LED to on,off,blink, or dim. (TestMode Only)
    BacklightOn - LCD Backlight On  (mutually exclusive of BacklightOff)
    BacklightOff - LCD Backlight Off (mutually exclusive of BacklightOn)
    ButtonAmber - Start Button Amber (mutually exclusive of other Button options)
    ButtonGreen - Start Button Green (mutually exclusive of other Button options)
    LEDRed - Start Red LED (mutually exclusive of other Button options)
    LEDGreen - Start Green LED (mutually exclusive of other Button options)
    ButtonAmberDim - Start Button Amber Dim (mutually exclusive of other Button options)
    ButtonGreenDim - Start Button Green Dim (mutually exclusive of other Button options)
    ButtonOff - Start Button Off
    BlinkOn - Start the LED Blink
    BlinkOff - Stop the LED Blink
    
    
## Command help SetIEC

    Help Strlen = 318
    SetIEC - Sets the IEC Cleaning Test parameters
    FloorSelection - Next Arg is the floor type < carpet | hard >
    CarpetSpeed - Next Arg is test speed on carpet (10-300mm/s)
    HardSpeed - Next Arg is test speed on hard floors (10-300mm/s)
    Distance - Next Arg is test distance (200-4000 mm, default 1200)
    
    
## Command help SetLCD

    Help Strlen = 687
    SetLCD - Sets the LCD to the specified display. (TestMode Only)
    BGWhite - Fill LCD background with White
    BGBlack - Fill LCD background with Black
    HLine - Draw a horizontal line (in foreground color) at the following row.
    VLine - Draw a vertical line (in foreground color) at the following column.
    HBars - Draw alternating horizontal lines (FG,BG,FG,BG,...),
    across the whole screen.
    VBars - Draw alternating vertical lines (FG,BG,FG,BG,...),
    across the whole screen.
    FGWhite - Use White as Foreground (line) color
    FGBlack - Use Black as Foreground (line) color
    Contrast - Set the following value as the LCD Contrast value into NAND. 0..63
    
    
## Command help SetLDSRotation

    Help Strlen = 205
    SetLDSRotation - Sets LDS rotation on or off. Can only be run in TestMode.
    On - Turns LDS rotation on. Mutually exclusive with Off.
    
    Off - Turns LDS rotation off. Mutually exclusive with On.
    
    
    
## Command help SetSchedule

    Help Strlen = 525
    SetSchedule - Modify Cleaning Schedule.
    Day - Day of the week to schedule cleaning for. Sun=0,Sat=6. (required)
    Hour - Hour value 0..23 (required)
    Min - Minutes value 0..59 (required)
    House - Schedule to Clean whole house (default)
    (Mutually exclusive with None)
    None - Remove Scheduled Cleaning for specified day. Time is ignored.
    (Mutually exclusive with House)
    ON - Enable Scheduled cleanings (Mutually exclusive with OFF)
    OFF - Disable Scheduled cleanings (Mutually exclusive with ON)
    
    
## Command help SetSystemMode

    Help Strlen = 392
    SetSystemMode - Set the operation mode of the robot. (TestMode Only)
    Shutdown - Shut down the robot. (mutually exclusive of other options)
    Hibernate - Start hibernate operation.(mutually exclusive of other options)
    Standby - Start standby operation. (mutually exclusive of other options)
    PowerCycle - Power cycles the entire system. (mutually exclusive of other options)
    
    
## Command help TestMode

    Help Strlen = 201
    TestMode - Sets TestMode on or off. Some commands can only be run in TestMode.
    On - Turns Testmode on. Mutually exclusive with Off.
    
    Off - Turns Testmode off. Mutually exclusive with On.
    
    
    
## Command help Upload

    Help Strlen = 626
    Upload - Uploads new program to the robot.
    code - Upload file is the main application. (Mutually exclusive with sound, LDS)
    sound - Upload file is a sound module. (Mutually exclusive with code, LDS)
    LDS - Upload file is an LDS module. (Mutually exclusive with sound, code)
    mainboard - Upload file is mainboard module. (Mutually exclusive with sound, code and LDS)
    size - data size to upload to device.
    noburn - test option -- do NOT burn the flash after the upload.
    readflash - test option -- read the flash at the current region.
    reboot - Reset the robot after performing the upload.
    
    
## Command GetAccel

    Label,Value
    PitchInDegrees,  1.98
    RollInDegrees, -0.85
    XInG, 0.035
    YInG,-0.015
    ZInG, 1.011
    SumInG, 1.012
    
## Command GetAnalogSensors

    SensorName,Value
    WallSensorInMM,60,
    BatteryVoltageInmV,16348,
    LeftDropInMM,0,
    RightDropInMM,0,
    LeftMagSensor,32768,
    RightMagSensor,32768,
    UIButtonInmV,3330,
    VacuumCurrentInmA,0,
    ChargeVoltInmV,24024,
    BatteryTemp0InC,30,
    BatteryTemp1InC,28,
    CurrentInmA,40,
    SideBrushCurrentInmA,0,
    VoltageReferenceInmV,1225,
    AccelXInmG,36,
    AccelYInmG,-16,
    AccelZInmG,1008,
    
## Command GetButtons

    Button Name,Pressed
    BTN_SOFT_KEY,0
    BTN_SCROLL_UP,0
    BTN_START,0
    BTN_BACK,0
    BTN_SCROLL_DOWN,0
    
## Command GetCalInfo

    Parameter,Value
    LDSOffset,0
    XAccel,0
    YAccel,0
    ZAccel,0
    RTCOffset,0
    LCDContrast,17
    RDropMin,293
    RDropMid,168
    RDropMax,84
    LDropMin,295
    LDropMid,168
    LDropMax,68
    WallMin,721
    WallMid,265
    WallMax,140
    QAState,0
    CleaningTestSurface,carpet
    CleaningTestHardSpeed,200
    CleaningTestCarpetSpeed,100
    CleaningTestHardDistance,1200
    CleaningTestCarpetDistance,1200
    
## Command GetCharger

    Label,Value
    FuelPercent,93
    BatteryOverTemp,0
    ChargingActive,0
    ChargingEnabled,1
    ConfidentOnFuel,0
    OnReservedFuel,0
    EmptyFuel,0
    BatteryFailure,0
    ExtPwrPresent,1
    ThermistorPresent[0],1
    ThermistorPresent[1],1
    BattTempCAvg[0],30
    BattTempCAvg[1],28
    VBattV,16.34
    VExtV,24.03
    Charger_mAH,0
    
## Command GetDigitalSensors

    Digital Sensor Name, Value
    SNSR_DC_JACK_CONNECT,0
    SNSR_DUSTBIN_IS_IN,1
    SNSR_LEFT_WHEEL_EXTENDED,0
    SNSR_RIGHT_WHEEL_EXTENDED,0
    LSIDEBIT,0
    LFRONTBIT,0
    RSIDEBIT,0
    RFRONTBIT,0
    
## Command GetErr

## Command GetLDSScan

    AngleInDegrees,DistInMM,Intensity,ErrorCodeHEX
    0,0,0,0
    1,0,0,0
    2,0,0,0
    3,0,0,0
    4,0,0,0
    5,0,0,0
    6,0,0,0
    7,0,0,0
    8,0,0,0
    9,0,0,0
    10,0,0,0
    11,0,0,0
    12,0,0,0
    13,0,0,0
    14,0,0,0
    15,0,0,0
    16,0,0,0
    17,0,0,0
    18,0,0,0
    19,0,0,0
    20,0,0,0
    21,0,0,0
    22,0,0,0
    23,0,0,0
    24,0,0,0
    25,0,0,0
    26,0,0,0
    27,0,0,0
    28,0,0,0
    29,0,0,0
    30,0,0,0
    31,0,0,0
    32,0,0,0
    33,0,0,0
    34,0,0,0
    35,0,0,0
    36,0,0,0
    37,0,0,0
    38,0,0,0
    39,0,0,0
    40,0,0,0
    41,0,0,0
    42,0,0,0
    43,0,0,0
    44,0,0,0
    45,0,0,0
    46,0,0,0
    47,0,0,0
    48,0,0,0
    49,0,0,0
    50,0,0,0
    51,0,0,0
    52,0,0,0
    53,0,0,0
    54,0,0,0
    55,0,0,0
    56,0,0,0
    57,0,0,0
    58,0,0,0
    59,0,0,0
    60,0,0,0
    61,0,0,0
    62,0,0,0
    63,0,0,0
    64,0,0,0
    65,0,0,0
    66,0,0,0
    67,0,0,0
    68,0,0,0
    69,0,0,0
    70,0,0,0
    71,0,0,0
    72,0,0,0
    73,0,0,0
    74,0,0,0
    75,0,0,0
    76,0,0,0
    77,0,0,0
    78,0,0,0
    79,0,0,0
    80,0,0,0
    81,0,0,0
    82,0,0,0
    83,0,0,0
    84,0,0,0
    85,0,0,0
    86,0,0,0
    87,0,0,0
    88,0,0,0
    89,0,0,0
    90,0,0,0
    91,0,0,0
    92,0,0,0
    93,0,0,0
    94,0,0,0
    95,0,0,0
    96,0,0,0
    97,0,0,0
    98,0,0,0
    99,0,0,0
    100,0,0,0
    101,0,0,0
    102,0,0,0
    103,0,0,0
    104,0,0,0
    105,0,0,0
    106,0,0,0
    107,0,0,0
    108,0,0,0
    109,0,0,0
    110,0,0,0
    111,0,0,0
    112,0,0,0
    113,0,0,0
    114,0,0,0
    115,0,0,0
    116,0,0,0
    117,0,0,0
    118,0,0,0
    119,0,0,0
    120,0,0,0
    121,0,0,0
    122,0,0,0
    123,0,0,0
    124,0,0,0
    125,0,0,0
    126,0,0,0
    127,0,0,0
    128,0,0,0
    129,0,0,0
    130,0,0,0
    131,0,0,0
    132,0,0,0
    133,0,0,0
    134,0,0,0
    135,0,0,0
    136,0,0,0
    137,0,0,0
    138,0,0,0
    139,0,0,0
    140,0,0,0
    141,0,0,0
    142,0,0,0
    143,0,0,0
    144,0,0,0
    145,0,0,0
    146,0,0,0
    147,0,0,0
    148,0,0,0
    149,0,0,0
    150,0,0,0
    151,0,0,0
    152,0,0,0
    153,0,0,0
    154,0,0,0
    155,0,0,0
    156,0,0,0
    157,0,0,0
    158,0,0,0
    159,0,0,0
    160,0,0,0
    161,0,0,0
    162,0,0,0
    163,0,0,0
    164,0,0,0
    165,0,0,0
    166,0,0,0
    167,0,0,0
    168,0,0,0
    169,0,0,0
    170,0,0,0
    171,0,0,0
    172,0,0,0
    173,0,0,0
    174,0,0,0
    175,0,0,0
    176,0,0,0
    177,0,0,0
    178,0,0,0
    179,0,0,0
    180,0,0,0
    181,0,0,0
    182,0,0,0
    183,0,0,0
    184,0,0,0
    185,0,0,0
    186,0,0,0
    187,0,0,0
    188,0,0,0
    189,0,0,0
    190,0,0,0
    191,0,0,0
    192,0,0,0
    193,0,0,0
    194,0,0,0
    195,0,0,0
    196,0,0,0
    197,0,0,0
    198,0,0,0
    199,0,0,0
    200,0,0,0
    201,0,0,0
    202,0,0,0
    203,0,0,0
    204,0,0,0
    205,0,0,0
    206,0,0,0
    207,0,0,0
    208,0,0,0
    209,0,0,0
    210,0,0,0
    211,0,0,0
    212,0,0,0
    213,0,0,0
    214,0,0,0
    215,0,0,0
    216,0,0,0
    217,0,0,0
    218,0,0,0
    219,0,0,0
    220,0,0,0
    221,0,0,0
    222,0,0,0
    223,0,0,0
    224,0,0,0
    225,0,0,0
    226,0,0,0
    227,0,0,0
    228,0,0,0
    229,0,0,0
    230,0,0,0
    231,0,0,0
    232,0,0,0
    233,0,0,0
    234,0,0,0
    235,0,0,0
    236,0,0,0
    237,0,0,0
    238,0,0,0
    239,0,0,0
    240,0,0,0
    241,0,0,0
    242,0,0,0
    243,0,0,0
    244,0,0,0
    245,0,0,0
    246,0,0,0
    247,0,0,0
    248,0,0,0
    249,0,0,0
    250,0,0,0
    251,0,0,0
    252,0,0,0
    253,0,0,0
    254,0,0,0
    255,0,0,0
    256,0,0,0
    257,0,0,0
    258,0,0,0
    259,0,0,0
    260,0,0,0
    261,0,0,0
    262,0,0,0
    263,0,0,0
    264,0,0,0
    265,0,0,0
    266,0,0,0
    267,0,0,0
    268,0,0,0
    269,0,0,0
    270,0,0,0
    271,0,0,0
    272,0,0,0
    273,0,0,0
    274,0,0,0
    275,0,0,0
    276,0,0,0
    277,0,0,0
    278,0,0,0
    279,0,0,0
    280,0,0,0
    281,0,0,0
    282,0,0,0
    283,0,0,0
    284,0,0,0
    285,0,0,0
    286,0,0,0
    287,0,0,0
    288,0,0,0
    289,0,0,0
    290,0,0,0
    291,0,0,0
    292,0,0,0
    293,0,0,0
    294,0,0,0
    295,0,0,0
    296,0,0,0
    297,0,0,0
    298,0,0,0
    299,0,0,0
    300,0,0,0
    301,0,0,0
    302,0,0,0
    303,0,0,0
    304,0,0,0
    305,0,0,0
    306,0,0,0
    307,0,0,0
    308,0,0,0
    309,0,0,0
    310,0,0,0
    311,0,0,0
    312,0,0,0
    313,0,0,0
    314,0,0,0
    315,0,0,0
    316,0,0,0
    317,0,0,0
    318,0,0,0
    319,0,0,0
    320,0,0,0
    321,0,0,0
    322,0,0,0
    323,0,0,0
    324,0,0,0
    325,0,0,0
    326,0,0,0
    327,0,0,0
    328,0,0,0
    329,0,0,0
    330,0,0,0
    331,0,0,0
    332,0,0,0
    333,0,0,0
    334,0,0,0
    335,0,0,0
    336,0,0,0
    337,0,0,0
    338,0,0,0
    339,0,0,0
    340,0,0,0
    341,0,0,0
    342,0,0,0
    343,0,0,0
    344,0,0,0
    345,0,0,0
    346,0,0,0
    347,0,0,0
    348,0,0,0
    349,0,0,0
    350,0,0,0
    351,0,0,0
    352,0,0,0
    353,0,0,0
    354,0,0,0
    355,0,0,0
    356,0,0,0
    357,0,0,0
    358,0,0,0
    359,0,0,0
    ROTATION_SPEED,0.00
    
## Command GetMotors

    Parameter,Value
    Brush_RPM,0
    Brush_mA,0
    Vacuum_RPM,0
    Vacuum_mA,0
    LeftWheel_RPM,0
    LeftWheel_Load%,0
    LeftWheel_PositionInMM,0
    LeftWheel_Speed,0
    RightWheel_RPM,0
    RightWheel_Load%,0
    RightWheel_PositionInMM,0
    RightWheel_Speed,0
    Charger_mAH, 0
    SideBrush_mA,0
    
## Command GetSchedule

    Schedule is Disabled
    Sun 00:00 - None -
    Mon 08:15 H
    Tue 00:00 - None -
    Wed 08:15 H
    Thu 00:00 - None -
    Fri 08:15 H
    Sat 00:00 - None -
    
## Command GetTime

    Sunday 6:28:22
    
## Command GetVersion

    Component,Major,Minor,Build
    ModelID,-1,XV28,
    ConfigID,1,,
    Software,3,4,24079
    BatteryType,1,NIMH_12CELL,
    BlowerType,1,BLOWER_ORIG,
    BrushSpeed,1200,,
    BrushMotorType,1,BRUSH_MOTOR_ORIG,
    SideBrushType,1,SIDE_BRUSH_NONE,
    WheelPodType,1,WHEEL_POD_ORIG,
    DropSensorType,1,DROP_SENSOR_ORIG,
    MagSensorType,1,MAG_SENSOR_ORIG,
    WallSensorType,1,WALL_SENSOR_ORIG,
    Locale,1,LOCALE_USA,
    LDS Software,V2.6.15295,0000000000,
    LDS CPU,F2802x/c001,,
    MainBoard Vendor ID,505,,
    BootLoader Software,18119,P,p
    MainBoard Software,23179,1,
    MainBoard Boot,16219,
    MainBoard Version,4,0,
    ChassisRev,2,,
    UIPanelRev,1,,
    
## Command GetWarranty

    000ae7e8
    0123
    cc2dca46
    
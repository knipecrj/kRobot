'MMEDIT!!! Basic Version = Micromite_5.05.01
'MMEDIT!!! Port = COM6:38400:10,300
'MMEDIT!!! Device = Micromite_5.05.01
'Basic Version = Micromite_5.05.01
'Port = COM6:38400:10,300
'Device = Micromite_5.05.01
'Program: kRobot Turret Conrtol
'Source: Micromite MMBASIC V5.05
'Target: Microchip PIC32MX170F256B 

' I2C Intersystem Message Data Map: array msg(). This device is a peripheral  
' 00: VECTOR                              - [IN] Offset for data access - pointer - register selector
' 01: MODE                                - [IN] 0 = follow other commands, 1 = do calibtation
' 02: ANGLE                               - [IN] Move turret to angle in steps (0-96) = 0-360 degrees
' 03: ELEVATION                           - [IN] Move platform elevation. O deg is level
' 04: FIRE                                - [IN] 255 = FIRE!, any other number is OFF
' 05: ANGLE_STATUS                        - [OUT] Report determined rotation angle here
' 06: CAL_STATUS                          - [OUT] Status report: 0 = Not calibrated, 
'                                                                1 = Cal now, 
'                                                                2 = Sweep to Front
'                                                                3 = Cal done
' 07: SONIC_RANGE                         - [OUT] Holds result of last range value
' 08: TRIM_ELEVATION                      - [IN] Simple trim value for elevation (only positive!) 

' 30: WDOG:                               - [OUT]Check Alive counter (READ)
' 31: DUMMY                               -  not used

' Motor Stepping Sequence
' 1=ON 0=OFF (Drive TIP122 on)
' CW Direction (reverse for CCW)
' Step  Motor Connection Number
'Relative to STEPPER pinouts. Connections are PIN3=(6), PIN4=(4), PIN3=(3), PIN4=(1)
'STP PIN =1     3     4     6     (Decimal as a word)
' Step
' 0       1     0     1     0       10
' 0.5     0     0     1     0       2
' 1       0     1     1     0       6
' 1.5     0     1     0     0       4
' 2       0     1     0     1       5
' 2.5     0     0     0     1       1
' 3       1     0     0     1       9
' 3.5     1     0     0     0       8
'Use decimal values for 4 bit write PORT(3,4) = Decimal Number

' Code setup
option EXPLICIT               ' Declare and set-up all variable before use           
OPTION DEFAULT NONE           ' All variables need initalising before use
OPTION AUTORUN ON             ' Code to automaticall run on micro power restart

'Configuation contants
CONST RX_ELEMENTS = 31        ' I2C data buffer size (32 bytes / 0 index)
CONST FIRST_BYTE = 0          ' Waiting for first byte - I2C receive control
CONST SECOND_BYTE = 1         ' Waiting for 2nd byte 0 I2C receive control
CONST MSG_VECTOR = 0          ' msg element definition
CONST MSG_MODE = 1            ' msg element definition
CONST MSG_ANGLE = 2           ' msg element definition
CONST MSG_ELEVATION = 3       ' msg element definition
CONST MSG_FIRE = 4            ' msg element definition
CONST MSG_ANGLE_STATUS = 5    ' msg element definition
CONST MSG_CAL_STATUS = 6      ' msg element definition
CONST MSG_SONIC_RANGE = 7     ' msg element definition
CONST MSG_TRIM_ELEVATION = 8  ' msg element definition
CONST MSG_WDOG = 30           ' msg element definition

CONST PIN_ES_STOP = 25        ' PIC Pin Ref
CONST PIN_TURRET_LS = 24      ' PIC Pin Ref
CONST PIN_STEPPER_m1_1 = 3    ' PIC Pin Ref
CONST PIN_STEPPER_m1_2 = 4    ' PIC Pin Ref
CONST PIN_STEPPER_m2_1 = 5    ' PIC Pin Ref
CONST PIN_STEPPER_m2_2 = 6    ' PIC Pin Ref
CONST PIN_SOLN = 10           ' PIC Pin Ref
CONST PIN_WEAPON = 9          ' PIC Pin Ref
CONST PIN_SONIC_TRIG = 16     ' PIC Pin Ref (configured in function)
CONST PIN_SONIC_ECHO = 15     ' PIC Pin Ref (configured in function)
CONST PIN_REF_SENSOR_J5 = 22  ' PIC Pin Ref
CONST PIN_REF_SENSOR_J6 = 21  ' PIC Pin Ref

CONST stepsPer360 = 96                  ' Number of steps per rotation (based on 1/2 stepped unipolar - this one)

CONST NOT_CALIBRATED_YET = 0  ' Turret calibration state machine enumerations (0, 1, 2, 3)
CONST MOVING_TO_TURRET_LS = 1
CONST CALIBRATED_MOVING_TO_FRONT_FACE = 2
CONST TURRET_CALIBRATED = 3
CONST SOL_TO_MOVE_POS = 1               ' Energise to release turret latch
CONST SOL_TO_LOCK_POS = 0               ' De energise to latch turret
CONST SONIC_POLL_RATE = 20              ' Sonic Poll rate is 1 second (or 20 * 50ms)

CONST SERVO_M = 0.012                   ' To convert wheel steering angle to raw servo output (y = mx + c) [m]
CONST SERVO_C = 0.65                    ' To convert wheel steering angle to raw servo output (y = mx + c) [c]
CONST LATCH_COUNT = 5                   ' Latch count (counts of 50ms)
CONST MID_ROT_IN_STEPS = 48             ' Turret location updated based on OPT sensor (J5) - Mid postion
CONST FAR_CW_IN_STEPS = 96              ' Turret location updated based on OPT sensor (J6) - Far CW position


'Global variable declaration
DIM comState as INTEGER                 ' comState stores state for incomming data being 1st or 2nd byte
DIM msg(31) as integer                  ' Create I2C buffer (Intersystem Message Data Map array)
DIM rState AS INTEGER                   ' Used in state machine for I2C comms
DIM f as integer                        ' Good old fashioned variable used for for next loops
DIM actualStepPOS as INTEGER            ' Postion in steps
DIM demandStepPOS as INTEGER            ' Ref posstion in steps
DIM turretCalibratedStatus AS INTEGER   ' 0 = Not calibrated, 1 = calibrating, 2 = calibrated
DIM turretPosError AS INTEGER           ' Turrent postion error in steps
DIM sonicRangeInCm AS INTEGER           ' Integer version of sonic range finder result
DIM fServoPos as FLOAT                  ' Used for servo pos calulation

' initalise variables
DIM INTEGER stepData(7) = (10, 2, 6, 4, 5, 1, 9, 8) ' Create stepper phase array
DIM INTEGER curStep = 0                             ' The stepper motor has 8 steps per 1 phase cycle. It starts at 0 here
DIM INTEGER solLatchCount = 0                       ' Stepper solenoid turrent latch - delay 0.25s release (50 * 50ms)
DIM INTEGER sonicPollCount = 0                      ' Used to store Sonic Poll rate count ... 1 second (or 20 * 50ms)
DIM INTEGER j5_latch = 0                            ' Used for off-on event for J5 OPT sensor
DIM INTEGER j6_latch = 0                            ' Used for off-on event for J6 OPT sensor
DIM INTEGER j5_event = 0                            ' One shot for J5 OPT sensor (detected)
DIM INTEGER j6_event = 0                            ' One shot for J6 OPT sensor (detected)
DIM INTEGER ls_latch = 0                            ' Used for off-on event for LS sensor (not calibration routine)
DIM INTEGER ls_event = 0                            ' One shot for LS sensor (detected) (not calibration routine)

turretCalibratedStatus = NOT_CALIBRATED_YET         ' Dont know where the stepper is (turrent angle)
for f = 0 to 31                                     ' Reset I2C Buffer
  msg(f) = 0
next f
rState = FIRST_BYTE                                 ' Prime I2C comms state
actualStepPos = 0                                   ' Unknown turret position is in indexed state (on limit switch)

'Reset system, flush buffers, stabilise,  initialise one time calcs before loop start
REM

'PIC Setup
SETPIN PIN_ES_STOP, DIN                     ' PIC Pin set as Digital Input
SETPIN PIN_TURRET_LS, DIN                   ' PIC Pin set as Digital Input
SETPIN PIN_STEPPER_m1_1, DOUT               ' PIC PIN set as Digital Output
SETPIN PIN_STEPPER_m1_2, DOUT               ' PIC PIN set as Digital Output
SETPIN PIN_STEPPER_m2_1, DOUT               ' PIC PIN set as Digital Output
SETPIN PIN_STEPPER_m2_2, DOUT               ' PIC PIN set as Digital Output
SETPIN PIN_SOLN, DOUT                       ' PIC PIN set as Digital Output
SETPIN PIN_WEAPON, DOUT                     ' PIC PIN set as Digital Output
SETPIN PIN_REF_SENSOR_J5, DIN               ' PIC PIN set as Digital Output 
SETPIN PIN_REF_SENSOR_J6, DIN               ' PIC PIN set as Digital Output 


I2C SLAVE OPEN &H26, 0, 0, WriteD, ReadD    'Setup I2C - Address hex26
TIMER = 0                                   ' Reset the internal timer (ms from start)
PORT(3,4) = 0                               ' Stepper Outputs to all off
PIN(PIN_SOLN) = SOL_TO_LOCK_POS             ' Turret lock on at start
PIN(PIN_WEAPON) = 0                         ' Weapon off (probably best for now)

'MAIN PROGRAM LOOP
do

  IF PIN(PIN_ES_STOP) = 1 THEN 
  
    TurretControl()
    WeaponCheck()
    UpdateTurretElevation()
    ReferenceRotCheck()
    'PRINT "located " actualStepPOS
      
  ELSE
  
    'PRINT ("ES STOP!")
    PORT(3,4) = 0                             ' Turrent rotate stop (and all phases off)
    msg(MSG_FIRE) = 0                         ' Weapon safe (off)
    PIN(PIN_SOLN) = 0                         ' Turrent to latch - deengergise solenoid now
    
  
  ENDIF

  WdogUpdate                    ' Update the software w dog
  UpdateSonicEchoRange()        ' Test each scan, but only do every second - its a slow function.
  PAUSE 50                      ' TEST CODE _ REMOVE FOR PROD

loop                      
'END of Main Loop

'Interrupt for I2C Raspi WRITE Operation if address matches this device
'Data is received in 2 runs of 1 byte. This first in the index, the 2nd the data.
'To read from the PIC the Raspi sets the INDEX to point to the value it needs to read.
Sub ReadD                       ' Read means data is coming from Raspi and being read by the PIC

  LOCAL a as INTEGER            ' Temp store for integer conversion on byte data
  LOCAL b as INTEGER
  LOCAL s$ AS STRING            ' Temp store for incoming byte data (like char)
  LOCAL RCVD as INTEGER         ' Used by I2C for number of bytes received
  
 
  I2C SLAVE READ 2 , s$, RCVD    ' Read byte, data in s$, RCDV hold bytes received
  
  'PRINT "Length: " LEN(s$) " Bytes: " RCVD
  
  IF RCVD = 2 AND MM.I2C = 0 THEN ' We are on real data is here  

    a = ASC(LEFT$(s$,1))            ' Decode Char to integer (0-255)
    b = ASC (RIGHT$(s$, 1))         ' Decode Char to integer (0-255)
    
      'PRINT "1ST BYTE: ", a
      'PRINT "2ND BYTE: ", b
      
    msg(0) = a
    msg(msg(0)) = b
  
  ELSE
      REM - dump anything else - not required.

  END IF

End Sub


'Interrupt for I2C Raspi READ Operation if address matches this device
'Must do a write operation first to set INDEX to value it wants to read from
Sub WriteD                      ' Write means Raspi wants data and the PIC is writing to the Raspi

  I2C SLAVE WRITE 1, msg(msg(0))   ' Write 1 byte back to Master using the index value in Vector msg(0).
  'Print "READ: Index " msg(0), " Data: " msg(msg(0))
  'PRINT ""
End Sub


' Simple watchdog rolling counter for Raspi to detect if this chip is alive
SUB WdogUpdate

  if TIMER>1000 THEN  
    msg(MSG_WDOG) = msg(MSG_WDOG) + 1   ' Increment software watchdog
    TIMER = 0                           ' Reset timer (so this runs once per second)
    'PRINT msg(MSG_WDOG)
    
  ENDIF
  
  if msg(MSG_WDOG) > 255 THEN           ' Simple roll aroung to zero once 255 exceeded - keep value to 1 byte
    msg(MSG_WDOG) = 0
  ENDIF
  
    
END SUB


' Calibrate the turrent position.
' Rotates turret counter-clockwise until turrent limit switch is actuated
SUB goCalibrate()

  PRINT ("Calibrating")
  PIN(PIN_SOLN) = SOL_TO_MOVE_POS                                   ' Unlock Turret
  turretCalibratedStatus = MOVING_TO_TURRET_LS                      ' Update Turret Status
  WHILE PIN(PIN_TURRET_LS) = 1                                      ' Move CCW unit Turrent LS is reached.
        
        DecrementTurretCount()                                      ' Move on phase mark counter-clockwise
        PAUSE 100                                                   ' Sets speed of calibration sweep
  WEND
  turretCalibratedStatus = CALIBRATED_MOVING_TO_FRONT_FACE          ' We are on the limit switch - so move to front
  PRINT ("Moving to front")
 
  
  actualStepPOS = 0                                                 ' This is the set pos in phase steps (using curStep counter)
  
  FOR f = 0 to 48
    IncrementTurretCount()
    PAUSE 100
  NEXT f
  turretCalibratedStatus =  TURRET_CALIBRATED
  msg(MSG_ANGLE) = actualStepPOS                                    ' End of calibation - so update ref angle to 180 (where it is now)
  PORT(3,4) = 0                                                     ' Reset stepper pins (calibration over)
  PRINT ("Calibrated")
  ReleaseSoln()                                                     ' Move over - Lock Turret

END SUB


' Moves stepper phase counter one phase counter-clockwise
SUB DecrementTurretCount()

  IF PIN(PIN_TURRET_LS) = 1 THEN                                    ' True if ok to move, false if on hard stop (dont move!)
    curStep = curStep - 1
    'Print ("Stepping CCW")
    IF curStep < 0 THEN curStep = 7
    MoveTurret()                                                    ' Move the turret
    IF turretCalibratedStatus = TURRET_CALIBRATED THEN actualStepPos = actualStepPos - 1  
    msg(MSG_ANGLE_STATUS) = actualStepPos
    solLatchCount = LATCH_COUNT
  ENDIF  
    
END SUB

' Moves stepper phase counter one phase clockwise
SUB IncrementTurretCount()

  IF j6_latch = 0 THEN                                              ' j6_latch indicates OPT j6 has detected max CW travel. Stop!
    curStep = curStep + 1
    'Print ("Stepping CW")
    IF curStep > 7 THEN curStep = 0
    MoveTurret()                                                    ' Move the turret
    actualStepPos = actualStepPos + 1  
    msg(MSG_ANGLE_STATUS) = actualStepPos
    
    solLatchCount = LATCH_COUNT
  ENDIF
  
END SUB


' Moves turrent by updated output pins
SUB MoveTurret()
  IF PIN(PIN_ES_STOP) = 1 THEN                                    ' No moves if ES_STOP active *CK TOTO - will knock out of seq!!!
      PIN(PIN_SOLN) = SOL_TO_MOVE_POS                             ' Unlock turrent - there is a move in progress
      'PRINT("Unlock Turrent")
      PORT(3,4) = stepData(curStep)                              ' Set outputs for stepper motor
      'Print curStep
  ENDIF 
END SUB

' Turrent postion logic - calibrate or move to ref postion
SUB TurretControl()

  If turretCalibratedStatus = NOT_CALIBRATED_YET THEN             ' Kick off turret rotation calibration
    goCalibrate()
  ENDIF
  
  IF turretCalibratedStatus = TURRET_CALIBRATED THEN              ' Track to reference postion    

    demandStepPos = msg(MSG_ANGLE)                                ' Update ref postion
    
    turretPosError = demandStepPos - actualStepPOS
    
    'print msg(MSG_ANGLE), demandStepPos, actualStepPOS, turretPosError

    IF turretPosError >0 THEN                                     ' True if a move is required
      IncrementTurretCount()                                      ' Move CW - limit checks in function
    ELSEIF turretPosError <0 THEN
      DecrementTurretCount()                                      ' Move CCW - limit checks in function
    ELSE
      PORT(3,4) = 0                                               ' There is no pos error so no drive required
      ReleaseSoln()                                               ' Move over - Lock Turret    
    ENDIF
          
  ENDIF

END SUB



'Turrent latch delay (0.25 secs = 50*50ms)
SUB ReleaseSoln()

  IF solLatchCount > 0 THEN solLatchCount = solLatchCount - 1
  
  'Print solLatchCount
  IF solLatchCount = 0 THEN
    PIN(PIN_SOLN) = SOL_TO_LOCK_POS                             ' Move over - Lock Turret    
    'PRINT ("Lock")
  END IF
 

END Sub


' Check if a weapon fire is required
SUB WeaponCheck()

  IF msg(MSG_FIRE) = 255 THEN
  
    'PRINT("FIRE")
  
    Pin(PIN_WEAPON) = 1                                           ' Weapon fire
  
  ELSE
  
    PIN(PIN_WEAPON) = 0                                           ' Weapon safe
  
  ENDIF

END SUB


'Run sonic range finder located on turret
SUB UpdateSonicEchoRange()

  sonicRangeInCm = 0                                              ' An impossible result - so assume this is gargbage
  sonicPollCount = sonicPollCount + 1                             ' Inc counter (occurs every 50ms)
  IF sonicPollCount > SONIC_POLL_RATE THEN
    sonicPollCount = 0

    sonicRangeInCm  = DISTANCE(PIN_SONIC_TRIG, PIN_SONIC_ECHO)    ' Call embedded C function

    IF sonicRangeInCm > 0 THEN 
      msg(MSG_SONIC_RANGE) = sonicRangeInCm                       ' If relevant update ISM data for PI
    ELSE
      msg(MSG_SONIC_RANGE) = sonicRangeInCm 
    END IF
  END IF
    
END SUB


'Turret Elevation control. Calibrated for 0deg to 45 deg inclination.
SUB UpdateTurretElevation()

  fServoPos = ( msg(MSG_ELEVATION) *  SERVO_M ) + SERVO_C + msg(MSG_TRIM_ELEVATION)    'Calc Servo pos
  SERVO 2, fServoPos                                             ' Drive Servo to location required 

END SUB


'Uses Optical Prox Sensort QRD1114 to recal turrent in travel
SUB ReferenceRotCheck()

  IF j5_latch = 0 THEN
    If Pin(PIN_REF_SENSOR_J5) = 0 THEN                                  ' Turrent is at front 180 pos
      j5_latch = 1                                                      ' Latch true while detector senses
      j5_event = 1                                                      ' Event is a one-shot on detection only
    END IF
  ELSE
    j5_event=0
    IF PIN(PIN_REF_SENSOR_J5) = 1 THEN
      j5_latch = 0    
    END IF
  END IF

  IF j6_latch = 0 THEN
    If Pin(PIN_REF_SENSOR_J6) = 0 THEN                                  ' Turrent is fully clockwise
      j6_latch = 1                                                      ' Latch true while detector senses
      j6_event = 1                                                      ' Event is a one-shot on detection only
    END IF
  ELSE
    j6_event=0
    IF PIN(PIN_REF_SENSOR_J6) = 1 THEN
      j6_latch = 0    
    END IF
  END IF
  
  IF ls_latch = 0 THEN
    If Pin(PIN_TURRET_LS) = 0 THEN                                      ' Turrent fully counter clockwise (hardstop)
      ls_latch = 1                                                      ' Latch true while detector senses
      ls_event = 1                                                      ' Event is a one-shot on detection only
    END IF
  ELSE
    ls_event=0
    IF PIN(PIN_TURRET_LS) = 1 THEN
      ls_latch = 0    
    END IF
  END IF  
  
  IF j5_event THEN actualStepPos = MID_ROT_IN_STEPS                     ' One-shot update intenal steps counter (96 steps = 360 deg, 180 deg is forward)
  IF j6_event THEN actualStepPos = FAR_CW_IN_STEPS
  iF ls_event THEN actualStepPos = 0
  
END SUB


CFunction DISTANCE(integer, integer) float
0000002D 27BDFFF8 AFBF0004 00852023 03E42021 ACC40000 8FBF0004 03E00008
27BD0008 27BDFFE0 AFBF001C 00002021 3C059D00 24A50040 27A60010 0411FFF1
00000000 8FA30010 3C029D00 8C4200BC 3C049D00 24840310 0040F809 00832021
8FBF001C 03E00008 27BD0020 000410C0 00041A00 00621823 00031180 00431823
00641821 00031980 3C029D00 8C420000 3C047735 34849400 8C420000 0082001B
004001F4 00002012 0064001B 008001F4 03E00008 00001012
27BDFFD8 AFBF0024 AFB40020 AFB3001C AFB20018 AFB10014 AFB00010 00809021
3C029D00 8C420000 8C430000 3C020098 34429680 0062102B 10400003 00A08821
0411FFCA 00000000 8E220000 14400003 3C109D00 8E420000 AE220000 8E020010
8E240000 24050002 0040F809 2406000E 8E02001C 8E440000 0040F809 24050005
8E020010 8E440000 24050008 0040F809 00003021 8E02001C 8E440000 0040F809
24050006 8E020004 0040F809 24040014 8E02001C 8E440000 0040F809 24050005
8E020004 0040F809 24040032 8E020010 8E240000 24050002 0040F809 2406000E
8E020004 0040F809 24040032 0411FFB1 24040005 00409821 0000A021 40944800
00008021 10000005 3C129D00 40104800 0270102B 1440001E 24050002 8E420020
0040F809 8E240000 1440FFF8 24040064 10000025 00000000 40104800 0270102B
14400013 24050002 8E420020 0040F809 8E240000 1040FFF8 24040020 1000001F
00000000 40104800 0270102B 14400008 24050001 8E420020 0040F809 8E240000
1440FFF8 00000000 1000001A 3C119D00 00052823 3C029D00 8C420080 00A02021
0040F809 00052FC3 8FBF0024 8FB40020 8FB3001C 8FB20018 8FB10014 8FB00010
03E00008 27BD0028 0411FF7A 00000000 00409821 1000FFDC 3C129D00 0411FF75
00000000 00409821 40944800 1000FFE1 3C129D00 8E320064 8E220000 3C037735
34639400 8C420000 0062001B 004001F4 00001012 72028002 24021696 0202001B
004001F4 00002012 8E220080 0040F809 00002821 00408021 8E220080 2404000A
0040F809 00002821 02002021 0240F809 00402821 1000FFD5 8FBF0024
20555043 65657073 6F742064 6F6C206F 000A0D77
End CFunction
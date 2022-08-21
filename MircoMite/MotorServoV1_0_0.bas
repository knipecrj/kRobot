'MMEDIT!!! Basic Version = Micromite_5.05.01
'MMEDIT!!! Port = COM6:38400:10,300
'MMEDIT!!! Device = Micromite_5.05.01
'Program: kRobot Motor-Servo Control Drive and Steering
'Source: Micromite MMBASIC V5.05
'Target: Microchip PIC32MX170F256B 

' I2C Intersystem Message Data Map: array msg(). This device is a peripheral  
' 00: VECTOR                              - [IN] Offset for data access - pointer - register selector
' 01: ANGLE: WHEEL LHS(PORT) ANGLE        - [IN] 0 = dont move, 90 is face front. 45 is 45 deg left, 135 is 45 deg right.
' 02: ANGLE: WHEEL RHS(STARBOARD)ANGLE    - [IN] same as above
' 03: SPEED: WHEEL LHS FWD                - [IN] 0 = zero speed, 100 = max speed
' 04: SPEED: WHEEL RHS FWD                - [IN] same as above
' 05: SPEED: WHEEL LHS REV                - [IN] 0 = zero speed, 100 = max speed
' 06: SPEED: WHEEL RHS REV                - [IN] same as above

' 07: TRIM ANGLE WHEEL LHS                - [IN] TRIM OFFSET (Centered around 90) in deg
' 08: TRIM ANGLE WHEEL RHS                - [IN] TRIM OFFSET (Centered around 90) in deg
' 
' 30: WDOG:                               - [OUT]Check Alive counter (READ)
' 31: DUMMY                               -  not used

' General Note - 1 refers to RHS, 2 to LHS
 
' Code setup
option EXPLICIT               ' Declare and set-up all variable before use           
OPTION DEFAULT NONE           ' All variables need initalising before use
OPTION AUTORUN ON             ' Code to automaticall run on micro power restart
  
  
'Configuation contants
CONST RX_ELEMENTS = 31        ' I2C data buffer size (32 bytes / 0 index)
CONST FIRST_BYTE = 0          ' Waiting for first byte - I2C receive control
CONST SECOND_BYTE = 1         ' Waiting for 2nd byte 0 I2C receive control
CONST PIN_DRV1_DIR = 2        ' PIC Pin Ref (Drive 2 is 
CONST PIN_DRV1_NOT_EN = 25    ' PIC Pin Ref
CONST PIN_DRV2_DIR = 23       ' PIC Pin Ref
CONST PIN_DRV2_NOT_EN = 22    ' PIC Pin Ref
CONST PIN_ES_STOP = 21        ' PIC Pin Ref
CONST MSG_VECTOR = 0          ' msg element definition
CONST MSG_ANG_WHL_LHS = 1     ' msg element definition
CONST MSG_ANG_WHL_RHS = 2     ' msg element definition
CONST MSG_WHL_SP_FWD_LHS = 3  ' msg element definition
CONST MSG_WHL_SP_FWD_RHS = 4  ' msg element definition
CONST MSG_WHL_SP_REV_LHS = 5  ' msg element definition
CONST MSG_WHL_SP_REV_RHS = 6  ' msg element definition
CONST MSG_TRIM_ANG_LHS = 7    ' msg element definition
CONST MSG_TRIM_ANG_RHS = 8    ' msg element definition
CONST MSG_WDOG = 30           ' msg element definition
CONST SERVO_M = 0.007446809   ' To convert wheel steering angle to raw servo output (y = mx + c) [m]
CONST SERVO_C = 0.754482429   ' To convert wheel steering angle to raw servo output (y = mx + c) [c]
CONST ENABLE_ACTIVELOW = 0    ' Make zero to turn on as it is active low
CONST DISABLE_ACTIVELOW = 1   ' Make 1 to turn off as it is active low
CONST SPEED_FACTOR = 1        ' Just in case scaling is required


'Global variable declaration
DIM comState as INTEGER       ' comState stores state for incomming data being 1st or 2nd byte
DIM msg(31) as integer        ' Create I2C buffer (Intersystem Message Data Map array)
DIM rState AS INTEGER         ' Used in state machine for I2C comms
DIM f as integer              ' Good old fashioned variable used for for next loops
DIM fServoPosLHS as FLOAT     ' Used for servo pos calulation
DIM fServoPosRHS as FLOAT     ' Used for servo pos calulation
DIM SpeedLHS AS FLOAT         ' Used for speed % duty cycle result. 0 = stop, 100 = max
DIM SpeedRHS as FLOAT         ' Used for speed % duty cycle result. 0 = stop, 100 = max


'Initalise variables
for f = 0 to 31             ' Reset I2C Buffer
  msg(f) = 0
next f
msg(MSG_ANG_WHL_LHS) = 90   ' Default wheel facing forward (90 deg)
msg(MSG_ANG_WHL_RHS) = 90   ' Default wheel facing forward (90 deg)
rState = FIRST_BYTE 


'Reset system, flush buffers, stabilise,  initialise one time calcs before loop start
REM


'PIC Setup
SETPIN PIN_DRV1_DIR, DOUT       ' PIC Pin set as Digit
SETPIN PIN_DRV1_NOT_EN, DOUT    ' PIC Pin set as Digital Output
SETPIN PIN_DRV2_DIR, DOUT       ' PIC Pin set as Digital Output
SETPIN PIN_DRV2_NOT_EN, DOUT    ' PIC Pin set as Digital Output
SETPIN PIN_ES_STOP, DIN         ' PIC Pin set as Digital Input
I2C SLAVE OPEN &H25, 0, 0, WriteD, ReadD  'Setup I2C - Address hex24

pin(PIN_DRV1_NOT_EN) = DISABLE_ACTIVELOW      'Drive 1 disabled - disconnect outputs
PIN(PIN_DRV2_NOT_EN) = DISABLE_ACTIVELOW      'Drive 2 disabled - disconnect outputs

TIMER = 0                       ' Reset the internal timer (ms from start)


'MAIN PROGRAM LOOP
do


  IF PIN(PIN_ES_STOP) = 1 THEN
    pin(PIN_DRV1_NOT_EN) = ENABLE_ACTIVELOW
    pin(PIN_DRV2_NOT_EN) = ENABLE_ACTIVELOW
    
  
    ' Servo postion calulation and physical control
    fServoPosLHS = ( msg(MSG_ANG_WHL_LHS) *  SERVO_M ) + SERVO_C + msg(MSG_TRIM_ANG_LHS)    'Calc Servo pos
    fServoPosRHS = ( msg(MSG_ANG_WHL_RHS) *  SERVO_M ) + SERVO_C + msg(MSG_TRIM_ANG_RHS)
  
    ' Values are Pulse Widths in ms , Typically 0.8ms = -90 deg, 2.2ms = +90 deg (Pulse every 20ms)
    'print fServoPosLHS, fServoPosRHS
    SERVO 1, fServoPosLHS, fServoPosRHS       ' Send Servos do postions required (LHS = Servo2, RHS = Servo1)

    ' Motor LHS control (Motor 2)
    if msg(MSG_WHL_SP_FWD_LHS)>0 THEN                         ' If this is a speed forward then
      PIN(PIN_DRV2_DIR) = 1                                 ' Enable forward DIR pin
      SpeedLHS = SPEED_FACTOR * msg(MSG_WHL_SP_FWD_LHS)     ' Calcuate PWM duty on cycle in %
      msg(MSG_WHL_SP_REV_LHS) = 0                           ' Make sure the opposite direction message is null. Dominant direction. 
    ELSE  
      PIN(PIN_DRV2_DIR) = 0                                 ' or enable reverse direction
      SpeedLHS = SPEED_FACTOR * msg(MSG_WHL_SP_REV_LHS)     ' Calcuate PWM duty on cycle in % 
    END IF
    
    ' Motor RHS control (Motor 1)    
    if msg(MSG_WHL_SP_FWD_RHS)>0 THEN                         ' If this is a speed forward then
      PIN(PIN_DRV1_DIR) = 1                                 ' Enable forward DIR pin
      SpeedRHS = SPEED_FACTOR * msg(MSG_WHL_SP_FWD_RHS)     ' Calcuate PWM duty on cycle in %
      msg(MSG_WHL_SP_REV_RHS) = 0                           ' Make sure the opposite direction message is null. Dominant direction.
    ELSE
      PIN(PIN_DRV1_DIR) = 0                                 ' or enable reverse direction  
      SpeedRHS = SPEED_FACTOR * msg(MSG_WHL_SP_REV_RHS)     ' Calcuate PWM duty on cycle in %
    END IF
        
    PWM 2, 1000, SpeedRHS, SpeedLHS                         ' Set the PWM channel 2 (pins 26 and 22 for RHS-Motor1 and LHS-Motor2), 1000Hz
  
  ELSE

    print "ESTOP"
    pin(PIN_DRV1_NOT_EN) = DISABLE_ACTIVELOW      ' Drive 1 disabled - disconnect outputs
    PIN(PIN_DRV2_NOT_EN) = DISABLE_ACTIVELOW      ' Drive 2 disabled - discoinnect outputs
    msg(MSG_WHL_SP_FWD_LHS) = 0                   ' Zero all speed setpoints in ISM array
    msg(MSG_WHL_SP_REV_LHS) = 0                   ' Zero all speed setpoints in ISM array
    msg(MSG_WHL_SP_FWD_RHS) = 0                   ' Zero all speed setpoints in ISM array
    msg(MSG_WHL_SP_REV_RHS) = 0                   ' Zero all speed setpoints in ISM array
        
  ENDIF
       
'  WATCHDOG 1000                 ' PIC Wdog, if this does not run every 1 sec the chip resets and restarts.
  
  WdogUpdate                    ' Update the m
  

  PAUSE 1000                    ' TEST CODE _ REMOVE FOR PROD    
     
loop                      
'END of Main Loop


'Interrupt for I2C Raspi WRITE Operation if address matches this device
'Data is received in 2 runs of 1 byte. This first in the index, the 2nd the data.
'To read from the PIC the Raspi sets the INDEX to point to the value it needs to read.
Sub ReadD                       ' Read means data is coming from Raspi and being read by the PIC

  LOCAL n as INTEGER            ' Temp store for integer conversion on byte data
  LOCAL s$ AS STRING            ' Temp store for incoming byte data (like char)
  LOCAL RCVD as INTEGER         ' Used by I2C for number of bytes received
  LOCAL rOp as INTEGER          ' Set-up flag for read or write operation
 
  I2C SLAVE READ 1, s$, RCVD    ' Read byte, data in s$, RCDV hold bytes received
  n = Asc(s$)                   ' Bodge - I2C Maximite receives bytes (like a char). Integer conversion here

  If (comState  = FIRST_BYTE) Then
    msg(0) = n                  ' First byte is index - store in command buffer 00 - Vector
    comState = SECOND_BYTE      ' Set-up to receive 2nd byte

    If msg(0) = 0 Then
      rOp = 1                   ' This is the set-up for a read operation
    Else
      rOp = 0                   ' This is not the set-up of a read operation
    EndIf

  Else
                                ' SECOND_BYTE has come in and stored in "n"
    msg(msg(0)) = n             ' Put data value in msg() element pointed to by msg(0) from first byte
    comState = FIRST_BYTE       ' Reset - one byte write complete by Raspi.

    If rOp = 0 Then
      Print "WRITE: Index: " msg(0),"Data: "  msg(msg(0))
    Else
      Print "Read Op Set-up for element: " msg(0)
    EndIf

  EndIf

End Sub


'Interrupt for I2C Raspi READ Operation if address matches this device
'Must do a write operation first to set INDEX to value it wants to read from
Sub WriteD                      ' Write means Raspi wants data and the PIC is writing to the Raspi

  I2C SLAVE WRITE 1, msg(msg(0))   ' Write 1 byte back to Master using the index value in Vector msg(0).
  Print "READ: Index " msg(0), "Data: " msg(msg(0))

End Sub


' Simple watchdog rolling counter for Raspi to detect if this chip is alive
SUB WdogUpdate

  if TIMER>1000 THEN  
    msg(MSG_WDOG) = msg(MSG_WDOG) + 1   ' Increment software watchdog
    TIMER = 0                           ' Reset timer (so this runs once per second)
    PRINT msg(MSG_WDOG)
  ENDIF
  
  if msg(MSG_WDOG) > 255 THEN           ' Simple roll aroung to zero once 255 exceeded - keep value to 1 byte
    msg(MSG_WDOG) = 0
  ENDIF
  
END SUB
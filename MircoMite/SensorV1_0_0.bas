'MMEDIT!!! Basic Version = Micromite_5.05.01
'MMEDIT!!! Port = COM7:38400:10,300
'MMEDIT!!! Device = Micromite_5.05.01
'Basic Version = PicoMote V5.07.03
'Port = COM6:38400:10,300
'Device = PicoMite firmware V5.07.03
'Program: kRobot Sensor Data Processing
'Source: PicoMite MMBASIC
'Target: Raspberry Pi Pico (1st Version) 

' I2C Intersystem Message Data Map: array msg(). This device is a peripheral  
' 00: VECTOR                      - [IN] Offset for data access - pointer - register selector
' 01: COMMAND                     - [IN/OUT] B7=x,B6=x,B5=x,B4=x,B3=GO_TO_WAR,B2=DO_OPERATE,B1=E_STOP_CMD,B0=DO_CALIBRATE
' 02: DATA_TEXT                   - [IN] DATA
' 03: GEN_STATUS                  - [OUT] B7=x,B6=x,B5=x,B4=E_STOP,B3=CMPS_OK,B2=FRONT_TOO_CLOSE,B1=REAR_TOO_CLOSE,B0=GPS_OK
' 04: MSG_ODOMETER_MSB            - [OUT] Wheel rotate counter
' 05: MSG_ODOMETER_LSB            - [OUT] Wheel rotate counter
' 06: GPS LONG_MSB                - [OUT] Longitude to 16 bits 
' 07: GPS LONG_LSB                - [OUT] Longitude to 16 bits 
' 08: GPS LAT_MSB                 - [OUT] Latitude to 16 bits
' 09: GPS LAT_LSB                 - [OUT] Latitude to 16 bits
' 10: GPS: ALTITUDE               - [OUT] Altitude 0M to 255M max
' 11: GPS: TRACK                  - [OUT] Latitude 0 to 255: 0 to 360deg
' 12: GPS: SATELLITES             - [OUT] Count
' 13: GPS: SPEED                  - [OUT] Speed in knots 0 to 255 max
' 14: GPS: YEAR (2 digits)        - [OUT] last 2 digits
' 15: GPS: MONTH                  - [OUT] month     
' 16: GPS: DAY                    - [OUT] day of month
' 17: GPS: HOUR                   - [OUT] hour in 24 hr format
' 18: GPS: MIN                    - [OUT] mins
' 19: GPS: SECONDS                - [OUT] secs
' 20: COMPASS: DIR (BYTE)         - [OUT] Magnetic heading 0 - 255 (not degrees!)
' 21: COMPASS : PITCH (BYTE)      - [OUT] Pitch -90 0 + 90 deg
' 22: COMPASS : ROLL (BYTE)       - [OUT] Roll -90 0 + 90 deg
' 23: COMPASS : TEMP (BYTE)       - [OUT] Deg C
' 24: COMPASS : CALIBRATION (BYTE)- [OUT] Contains calib data = 255 for fully calibrated
' 25: 11V LIPO BUS                - [OUT] Voltage in V * 10
' 26: 5.0V BUS                    - [OUT] Voltage in V * 10
' 27: 6.0V BUS 1                  - [OUT] Voltage in V * 10
' 28: 6.0V BUS 2                  - [OUT] Voltage in V * 10
' 29: IR FRONT DISTANCE : CM      - [OUT] Front distance in CM to object
' 20: IR REAR DISTANCE : CM       - [OUT] Front distance in CM to onject
' 31: WDOG                        - [OUT] Rolling Watchdog count


' Code setup
option EXPLICIT               ' Declare and set-up all variable before use           
OPTION DEFAULT NONE           ' All variables need initalising before use
OPTION AUTORUN ON             ' Code to automaticall run on micro power restart

'Configuation contants
CONST RX_ELEMENTS = 31            ' I2C data buffer size (32 bytes / 0 index)
CONST MSG_VECTOR = 0              ' msg element definition
CONST MSG_MODE = 1                ' msg element definition  
CONST MSG_DATA_TEXT = 2           ' msg element definition 
CONST MSG_GEN_STATUS = 3          ' msg element definition 
CONST MSG_ODOMETER_MSB = 4        ' msg element definition 
CONST MSG_ODOMETER_LSB = 5        ' msg element definition
CONST MSG_GPS_LONG_MSB = 6        ' msg element definition
CONST MSG_GPS_LONG_LSB = 7        ' msg element definition
CONST MSG_GPS_LAT_MSB = 8         ' msg element definition
CONST MSG_GPS_LAT_LSB = 9         ' msg element definition
CONST MSG_GPS_ALTITUDE = 10       ' msg element definition
CONST MSG_GPS_TRACK = 11          ' msg element definition
CONST MSG_GPS_SATELLITES = 12     ' msg element definition
CONST MSG_GPS_SPEED = 13          ' msg element definition
CONST MSG_GPS_YEAR = 14           ' msg element definition
CONST MGS_GPS_MONTH = 15          ' msg element definition
CONST MSG_GPS_DAY = 16            ' msg element definition
CONST MSG_GPS_HOUR = 17           ' msg element definition
CONST MSG_GPS_MIN = 18            ' msg element definition
CONST MSG_GPS_SECONDS = 19        ' msg element definition
CONST MSG_CMPS_DIR = 20           ' msg element definition
CONST MSG_CMPS_PITCH = 21         ' msg element definition
CONST MSG_CMPS_ROLL = 22          ' msg element definition
CONST MSG_TEMP = 23               ' msg element definition
CONST MSG_CMPS_CAL = 24           ' msg element definition
CONST MSG_11V = 25                ' msg element definition
CONST MSG_5V = 26                 ' msg element definition
CONST MSG_6V_B1 = 27              ' msg element definition
CONST MGS_6B_B2 = 28              ' msg element definition
CONST MSG_FRONT_DIST = 29         ' msg element definition
CONST MSG_REAR_DIST = 30          ' msg element definition
CONST MSG_WDOG = 31               ' msg element definition

CONST PIN_WHEEL_ENC_A2 = 9        ' AKA GPIO6
CONST PIN_WHEEL_ENC_A1 = 10       ' AKA GPI07
CONST PIN_COMP_TX_COM2 = 11       ' AKA GPIO - UART1 TX (Not connected) 
CONST PIN_COMP_RX_COM2 = 12       ' AKA GPIO - UART1 RX
CONST PIN_ES_STOP = 15            ' AKA GPIO11
CONST PIN_COMP_TX_COM1 = 16       ' AKA GPIO12 - UART0 TX
CONST PIN_COMP_RX_COM1 = 17       ' AKA GPIO13 - UART0 RX

CONST PIN_I2C2SDA = 19            ' AKA GPI014
CONST PIN_I2C2SCL = 20            ' AKA GPIO15

CONST PIN_LED_RED = 22            ' AKA GPIO17
CONST PIN_A_MPLEX_A = 24          ' AKA GPIO18 (start of analog multiplex control)
CONST PIN_A_MPLEX_B = 25          ' AKA GPIO19 (start of analog multiplex control)
CONST PIN_A_MPLEX_C = 26          ' AKA GPIO20 (start of analog multiplex control)
CONST PIN_LED_YELLOW = 27         ' AKA GPIO21
CONST PIN_LED_GREEN = 29          ' AKA GPIO22
CONST PIN_IR_FRONT_AN = 32        ' AKA GPIO27_ADC1
CONST PIN_IR_REAR_AN = 31         ' AKA GPIO26_ADC0
CONST PIN_MUTLIPLEX_AN = 34       ' AKA GPIO28_ADC2

CONST VOLTAGE_SCAN_RATE_SECS = 10  ' How often to scan voltages in seconds
CONST GPS_SCAN_RATE_SECS = 5       ' How often the GPS is updated
CONST TOO_CLOSE = 5               ' cm -  Below this you will collide


CONST read_version_8bit = &H11                  ' For CMPS12 interface
CONST read_bearing_8bit = &H12                  ' For CMPS12 interface
CONST read_pitch_8bit = &H14                    ' For CMPS12 interface
CONST read_roll_8bit = &H15                     ' For CMPS12 interface
CONST read_temperature_16bits = &H22            ' For CMPS12 interface
CONST read_get_calibration_state_8bit = &H24    ' For CMPS12 interface

'Global variable declaration
DIM comState as INTEGER                 ' comState stores state for incomming data being 1st or 2nd byte
DIM msg(31) as integer                  ' Create I2C buffer (Intersystem Message Data Map array)
DIM f as integer                        ' Good old fashioned variable used for for next loops
DIM green_state AS INTEGER              ' LED state 1= ON 0=OFF
DIM yellow_state AS INTEGER             ' LED state 1= ON 0=OFF
DIM red_state AS INTEGER                ' LED state 1= ON 0=OFF
DIM kDcalc as FLOAT                     ' Used in distance calcs
DIM xDcalc AS FLOAT                     ' Used in distance calcs
DIM rawFrontDistance AS FLOAT           ' Holds AtoD conversion for Front
DIM rawRearDistance AS FLOAT            ' Holds AtoD conversion for Rear
DIM rawMuliPlexInput AS FLOAT           ' Holds AtoD conversion from Multiplexer
DIM scanVLatch AS INTEGER               ' Ensures Mutiplexer runs on time period only
DIM scanGPSRate AS INTEGER              ' Ensures Mutiplexer runs on time period only
DIM rawODOcount AS INTEGER              ' Holds counter for wheel movement via encoder. +ve is forward
DIM gpsOK AS INTEGER                    ' Holds general GPS OK status
DIM gpsDate as STRING                   ' GPS date as a string
DIM gpsTime AS STRING                   ' GPS date as a string
DIM gpsAlt AS FLOAT                     ' GPS Altitued in meters
DIM gpsSats AS INTEGER                  ' GPS Satellites locked onto
DIM gpsLong AS FLOAT                    ' GPS longitude as a float
DIM gpsLat AS FLOAT                     ' GPS latitude as a float
DIM gpsTrack AS FLOAT                   ' GPS track as a float in degrees
DIM gpsSpeed AS FLOAT                   ' GPS speed in knots as a float
DIM topLine AS STRING                   ' Top line of text for OLED (builder)
DIM bottomLine AS STRING                ' Bottom line of text for OLED (builder)
DIM topText AS STRING                   ' Top line text for display
DIM bottomText AS STRING                ' Bottom line for display
DIM textStateChk AS INTEGER             ' used to determine text and format input from PI
DIM textForTopLine AS INTEGER           ' Default text for top line
DIM colState AS INTEGER                 ' 6=Red, 7=Blue, 8=Green, 9=White
DIM fontState AS INTEGER                ' 0=Default, 1=Small

' Do convert ANIN Volts to actual volts (Physical is FSD = 1.65V)
' 11, 5, 6, 6V respectively - others not set)
DIM FLOAT anCal(7) = (6.6666, 3.030303, 3.636364, 3.636364, 1.0, 1.0, 1.0, 1.0)

' Set up for CMPS12 read - cmREAD holds addresses of data, cmpee
DIM INTEGER cmpREAD(4) = (read_bearing_8bit, read_pitch_8bit, read_roll_8bit, read_temperature_16bits, read_get_calibration_state_8bit)
DIM INTEGER cmpFlags(4) = (0, 0, 0, 0, 0)
DIM INTEGER cmpCounter = 0
DIM STRING serial_data = ""


' initalise variables
kDcalc = 10.157                              ' Curve fit parameters for d=k/(v+x) 
xDcalc = 0.084                               ' Where d is distance in cm, v is the AIN value

for f = 0 to 31                         ' Reset I2C message array
  msg(f) = 0
next f

green_state = 0                         ' State set
yellow_state = 0                        ' State set
red_state = 0                           ' State set

scanVLatch = 0                          ' State logic - for scanning voltages - slowly
scanGPSRate = 0                         ' State logic for scanning GPS
rawODOcount = 0                         ' Reset the ODOmeter value (raw counts)

textStateChk = 0                        ' No information or information processed if zero
bottomLine = ""                         ' Reset OLED text
topLine = ""                            ' Reset OLED text
topText = "......."                     ' Dummy data
bottomText = "+++++++"                  ' Dummy data
textForTopLine = 1                      ' Default text for top line
colState = 7                            ' Make blue
fontState = 0                           ' Make default

'Reset system, flush buffers, stabilise,  initialise one time calcs before loop start
REM

'PIC Setup
SETPIN PIN_ES_STOP, DIN                           ' PIC Pin set as Digital Input

SETPIN PIN_IR_FRONT_AN, AIN                       ' PIC Pin set as Analog Input
SETPIN PIN_IR_REAR_AN, AIN                        ' PIC Pin set as Analog Input
SETPIN PIN_MUTLIPLEX_AN, AIN                      ' PIC Pin set as Analog Input

SETPIN PIN_LED_RED, DOUT                          ' PIC PIN set as Digital Output
SETPIN PIN_LED_YELLOW, DOUT                       ' PIC PIN set as Digital Output
SETPIN PIN_LED_GREEN, DOUT                        ' PIC PIN set as Digital Output
SETPIN PIN_A_MPLEX_A, DOUT                        ' PIC PIN set as Digital Output
SETPIN PIN_A_MPLEX_B, DOUT                        ' PIC PIN set as Digital Output
SETPIN PIN_A_MPLEX_C, DOUT                        ' PIC PIN set as Digital Output

SETPIN PIN_WHEEL_ENC_A1, INTH, EndcoderEvent         ' PIC PIN for Encoder set as Digtal Input, Interrupt
SETPIN PIN_WHEEL_ENC_A2, DIN                         ' PIC Pin set as Digital Input

SETPIN PIN_ES_STOP, DIN                           ' Set Emergency Stop line as Input (true = OK state)

SETPIN PIN_I2C2SDA, PIN_I2C2SCL, I2C2             ' I2C2 Pin Set-up
I2C2 SLAVE OPEN &H28, WriteD, ReadD               ' Setup I2C - Address hex28

SETPIN PIN_COMP_RX_COM1, PIN_COMP_TX_COM1, COM1   ' Assign pins to serial port UART0 - COM1
OPEN "COM1:9600, 256, SerInt, 1" as #1            ' Open COM1 port with 256byte buffer and interrupt "SerInt"

SETPIN PIN_COMP_RX_COM2, PIN_COMP_TX_COM2, COM2   ' Assign pins to serial port UART1 - COM2
OPEN "COM2:9600" AS GPS                           ' Open COM2 for GPS data


' SPECIAL CODE - this will set-up LCD. Check with OPTION LIST command!
REM OPTION SYSTEM SPI GP2, GP3, GP0               ' Run in command line (not in prog)     
REM OPTION LCDPANEL ST7735S, L, GP01, GP04, GP05  ' Run in command line (not in proj)

COLOUR RGB(WHITE), RGB(BLUE)                ' Default font colour
FONT 1, 3                                   ' Default font (8x12)
CLS RGB(BLUE)                               ' Make blue to start


TIMER = 0                                   ' Reset the internal timer (ms from start)


'MAIN PROGRAM LOOP
do

  UpdateStates                              ' Update State Machings and scan IO
  ScanDistances                             ' Update IR distances
  ScanVoltageBusses                         ' Update Voltage Readings (slow - once a sec) 
  CalculateDistanceMoved                    ' Uses rawODOcount to set MSG_ODOMETER_MSB & LBS msgs 
  GetCompassData                            ' Do CMPS12 readings
  GetGPSData                                ' Get the GPS data time, date, long, lat, satelites
  GenStatus                                 ' Update Status Byte
  TextProcess                               ' Update the OLED text if required

  WdogUpdate                                ' Update the software w dog
  
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
  
 
  I2C2 SLAVE READ 2 , s$, RCVD    ' Read byte, data in s$, RCDV hold bytes received
  
  PRINT "Length: " LEN(s$) " Bytes: " RCVD
  
  IF RCVD = 2 AND MM.I2C = 0 THEN ' We are on real data is here  

    a = ASC(LEFT$(s$,1))            ' Decode Char to integer (0-255)
    b = ASC (RIGHT$(s$, 1))         ' Decode Char to integer (0-255)
    
      PRINT "1ST BYTE: ", a
      PRINT "2ND BYTE: ", b
      
    msg(0) = a
    msg(msg(0)) = b
  
  ELSE
      REM - dump anything else - not required.

  END IF

End Sub


'Interrupt for I2C Raspi READ Operation if address matches this device
'Must do a write operation first to set INDEX to value it wants to read from
Sub WriteD                      ' Write means Raspi wants data and the PIC is writing to the Raspi

  I2C2 SLAVE WRITE 1, msg(msg(0))   ' Write 1 byte back to Master using the index value in Vector msg(0).
  Print "READ: Index " msg(0), " Data: " msg(msg(0))
  PRINT ""
End Sub

' Interrupt on low to high of wheel rotation encoder (PIN_WHL_ENC_A1)
' If PIN_WHL_ENC_A2 is high - going forwards else in reverse
SUB EndcoderEvent

  IF PIN(PIN_WHEEL_ENC_A2) = 1 THEN             ' If this is high event is called when travelling going forward
    rawODOcount = rawODOcount + 1
  ELSE
    rawODOcount = rawODOcount - 1         ' Or you are moving in reverse
  END IF

END SUB


' Simple watchdog rolling counter for Raspi to detect if this chip is alive
SUB WdogUpdate

  if TIMER>1000 THEN  
    msg(MSG_WDOG) = msg(MSG_WDOG) + 1   ' Increment software watchdog
    TIMER = 0                           ' Reset timer (so this runs once per second)
      
    'Temp display code - ck move to change function later CK ***TODO***  
    TEXT 0, 0, topText
    TEXT 0, 40, bottomText

    'DataDumpPrint
    
  ENDIF
  
  if msg(MSG_WDOG) > 255 THEN           ' Simple roll aroung to zero once 255 exceeded - keep value to 1 byte
    msg(MSG_WDOG) = 0
  ENDIF
  

      
END SUB


' Update cyclick output states
SUB UpdateStates

  ' TEST CODE BLOCK
  if msg(MSG_WDOG) MOD 2 = 0 THEN
    red_state = 1
  ELSE
    red_state = 0
  ENDIF
  
  if gpsOK THEN                         ' Amber of GPS OK
    yellow_state = 1
  ELSE
    yellow_state = 0
  ENDIF
  
  if msg(MSG_CMPS_CAL)  = 255 THEN      ' Green if Compass if full calibrated
    green_state = 1
  ELSE
    green_state = 0
  ENDIF
  
    'END OF TEST CODE BLOCK
  
  PIN(PIN_LED_RED) = red_state
  PIN(PIN_LED_YELLOW) = yellow_state
  PIN(PIN_LED_GREEN) = green_state

END SUB


' Convert IR distance converters to cm and store in msg array
SUB ScanDistances

  rawFrontDistance = PIN(PIN_IR_FRONT_AN)                           ' A to D
  rawRearDistance = PIN(PIN_IR_REAR_AN)                             ' A to D
  
  'AIN is not linear - following inverse rule of v = 1/d hence the maths below
  'Curve fit parameters for d=k/(v+x) where d is distance in cm, v is the AIN value
  'Values seen on AIN: 4cm to 30cm were 2.46 and 0.25
  
  msg(MSG_FRONT_DIST) = FIX(kDcalc / (rawFrontDistance + xDCalc))   ' Convert to cm
  msg(MSG_REAR_DIST) = FIX(kDcalc / (rawRearDistance + xDCalc))     ' Conver to cm
  
  If msg(MSG_FRONT_DIST)>255 THEN msg(MSG_FRONT_DIST) = 255        ' Clamp to 8 bit max
  If msg(MSG_REAR_DIST)>255 THEN msg(MSG_REAR_DIST) = 255          ' Clamp to 8 bit max

END SUB


' Update voltage readings (calibrated here for V * 10 into an INTEGER)
SUB ScanVoltageBusses

  'PORT(PIN_A_MPLEX_A, 3) = 0
  'rawMuliPlexInput = PIN(34)
  'PRINT rawMuliPlexInput
  

  LOCAL i as INTEGER                                                ' Iterator
  LOCAL patt as INTEGER                                             ' Holds Scan Patten to address mplex
  
  ' Runs at period set
  IF (msg(MSG_WDOG) MOD VOLTAGE_SCAN_RATE_SECS = 0) AND (scanVLatch = 0) THEN ' Runs at period specced
  
  ' Used Analog multiplex switch: X0=>LIPO 11V, X1=>5V, X2=>6VBUS2, X4=>6VBUS1
  ' Other pins not used at this time
    scanVLatch = 1                                                  ' Run only once of correct time sec
  
    FOR i = 0 to 3
    
      'PRINT i, "SCANNING"
    
      'Set output pattern to analog mutlip. + brief delay
      patt = i
      if patt = 3 then patt = 4                                     ' Fix as X0, X1, X2, X4 (X3 missing now)
      PORT(PIN_A_MPLEX_A , 3) = patt                                ' Select analog
      PAUSE 50                                                      ' Settle delay (required tuning)
      rawMuliPlexInput = PIN(PIN_MUTLIPLEX_AN)                      ' Read analog
      msg(MSG_11V+i) = FIX(10*(rawMuliPlexInput * anCal(i)))        ' Convert analog (volts * 10) for Integer
      
    NEXT i
    
    ' Do 3.3V check and store (nb this is and internal pin)

  ENDIF
  
  ' Reset latch for next event
  IF (msg(MSG_WDOG) MOD VOLTAGE_SCAN_RATE_SECS > 0) THEN scanVLatch = 0

END SUB


' Out of interrupt calculation to update MSG_ODOMETER_MSB and MSG_ODOMETER_LSB
' CK Note TOTO - may overrun on distance -32768 to +32767 - maybe clamp or reset to zero? CK TODO
SUB CalculateDistanceMoved

  LOCAL INTEGER tempd = rawODOcount 
  LOCAL S As STRING 
  LOCAL B1 AS INTEGER
  LOCAL B2 AS INTEGER
  
  ' Uses 2s complement conversion to 2 bytes
  S = Bin2str$(INT16, tempd, BIG)                                   ' Function where the magic happens
  B1 = Asc(Left$(S, 1))                                             ' B1 holds high byte
  B2 = Asc(Right$(S, 1))                                            ' B2 holds low byte

  'Print "High Byte: " B1
  'Print "Low Byte: " B2

  msg(MSG_ODOMETER_MSB) = B1                                            ' Update msg for Ras PI
  msg(MSG_ODOMETER_LSB) = B2

END SUB


SUB GetCompassData

  IF cmpFlags(cmpCounter) = 0 THEN                                  ' Checks if op has been already done
    
    PRINT #1, CHR$(cmpREAD(cmpCounter))                             ' Issues request to CMPS12
    cmpFlags(cmpCounter) = 1                                        ' Flags request made - await interrupt response
  
  ENDIF  
  
  IF serial_data <>"" THEN                                           ' Interrupt has loaded var with data if true
    
    msg(MSG_CMPS_DIR+cmpCounter) = ASC(serial_data)                      ' Store byte data
    'PRINT "Serial Data:", cmpCounter, msg(CMPS_DIR+cmpCounter)
    serial_data = ""                                                 ' Reset for next item to read
    cmpFlags(cmpCounter) = 0                                         ' Reset latch for next operation
    cmpCounter = cmpCounter + 1                                      ' Set-up for next read
    IF cmpCounter > 4 THEN cmpCounter = 0                            ' Reset for start of next set of reads
    
  ENDIF

END SUB 


' Interrupt routine - captures serial data from CMPS12 returned
SUB SerInt

  LOCAL temp AS String
  
  temp = INPUT$(10, #1)                                             ' Retrieve data from serial buffer
  serial_data = temp
  temp = ""

END SUB


' Process GPS data from COM2
SUB GetGPSData

  IF GPS(VALID) THEN                                                ' True with valid data              
  
    'Only run peridocally - set by GPS_SCAN_RATE_SECS
    IF (msg(MSG_WDOG) MOD GPS_SCAN_RATE_SECS = 0) AND (scanGPSRate = 0) THEN 
    
      scanGPSRate = 1                                               ' One shot on time event
    
      ' Code below does what is says on the tin. These are MMBasic functions
      gpsOK = 1
      gpsTime = GPS(TIME)
      gpsDate = GPS(DATE)
      gpsSats = GPS(SATELLITES)
      gpsALT = GPS(ALTITUDE)
      gpsLong = GPS(LONGITUDE)
      gpsLat = GPS(LATITUDE)
      gpsTrack = GPS(TRACK)
      gpsSpeed = GPS(SPEED)
      'PRINT gpsTime, gpsDate, gpsSats, gpsALT, gpsLong, gpsLat ,gpsTrack ,gpsSpeed 
      
      EncodeMsgGPSMessages
      
    END IF

  ELSE
      gpsOK = 0
  ENDIF
  
  ' Reset latch for next event
  IF (msg(MSG_WDOG) MOD GPS_SCAN_RATE_SECS  > 0) THEN scanGPSRate = 0

END SUB

' Process GPS messages for msg array
SUB EncodeMsgGPSMessages

  LOCAL iHour as INTEGER
  LOCAL iMin AS INTEGER
  LOCAL iSec AS INTEGER
  LOCAL iDay as INTEGER
  LOCAL iMonth AS INTEGER
  LOCAL iYear AS INTEGER
  LOCAL iTrack AS INTEGER
  LOCAL iSpeed AS INTEGER
  LOCAL iAlt AS INTEGER
  LOCAL iLong AS INTEGER
  LOCAL iLat AS INTEGER
  LOCAL SLong As STRING
  LOCAL sLat AS STRING 
   
  'PRINT gpsDate
  iHour = VAL(LEFT$(gpsTime,2))
  iMin = VAL(MID$(gpsTime, 4, 2))
  iSec = VAL(RIGHT$(gpsTime,2))
  'PRINT iHour, iMin, iSec
  iDay = VAL(LEFT$(gpsDate,2))
  iMonth = VAL(MID$(gpsDate, 4, 2))
  iYear = VAL(RIGHT$(gpsDate,2))        ' Take only the last 2 digits
  'PRINT iDay, iMonth, iYear
  iTrack = FIX((gpsTrack/360.0)*255.0)  ' Scale 0-360 to 0-255
  iSpeed = FIX(iSpeed)                  ' In knots - unlikey be be above 255. TODO CK CHECK
  iAlt = FIX(gpsALT)                    ' In meters - unlikley to be above 255. TODO CK CHECK
  
  iLong = FIX(gpsLong*100)              ' Good for 2 digits
  sLong = Bin2str$(INT16, iLong, BIG)   ' 2's compiment - good to +-32000 ish
  
  iLat = FIX(gpsLat*100)                ' Good for 2 digits
  sLat = Bin2str$(INT16, iLat, BIG)     ' 2's compiment - good to +-32000 ish
  
  msg(MSG_GPS_YEAR) = iYear
  msg(MGS_GPS_MONTH) = iMonth
  msg(MSG_GPS_DAY) = iDay
  msg(MSG_GPS_HOUR) = iHour
  msg(MSG_GPS_MIN) = iMin
  msg(MSG_GPS_SECONDS) = iSec
  msg(MSG_GPS_SATELLITES) = gpsSats
  msg(MSG_GPS_TRACK) = iTrack
  msg(MSG_GPS_ALTITUDE) = iAlt
  
  msg(MSG_GPS_LONG_MSB) = Asc(Left$(sLong, 1))  ' B1 holds high byte
  msg(MSG_GPS_LONG_LSB) = Asc(Right$(sLong, 1)) ' B2 holds low byte  
  msg(MSG_GPS_LAT_MSB) = Asc(Left$(sLat, 1))  ' B1 holds high byte
  msg(MSG_GPS_LAT_LSB) = Asc(Right$(sLat, 1)) ' B2 holds low byte 

END SUB



' Set-up 03: GEN_STATUS: B7=x,B6=x,B5=x,B4=E_STOP,B3=CMPS_OK,B2=FRONT_TOO_CLOSE,B1=REAR_TOO_CLOSE,B0=GPS_OK
SUB GenStatus

  msg(MSG_GEN_STATUS) = 0

  'Collision avoidance for front
  IF msg(MSG_FRONT_DIST) <= TOO_CLOSE THEN
    msg(MSG_GEN_STATUS) = msg(MSG_GEN_STATUS) OR &B00000100
  ELSE
    msg(MSG_GEN_STATUS) = msg(MSG_GEN_STATUS) AND &B11111011
  ENDIF
  'print msg(MSG_GEN_STATUS)
  
  'Collision avoidance for rear
  IF msg(MSG_REAR_DIST) <= TOO_CLOSE THEN
    msg(MSG_GEN_STATUS) = msg(MSG_GEN_STATUS) OR &B00000010
  ELSE
    msg(MSG_GEN_STATUS) = msg(MSG_GEN_STATUS) AND &B11111101
  ENDIF
  'print msg(MSG_GEN_STATUS)
  
  IF gpsOK = 1 THEN
    msg(MSG_GEN_STATUS) = msg(MSG_GEN_STATUS) OR &B00000001
  ELSE
    msg(MSG_GEN_STATUS) = msg(MSG_GEN_STATUS) AND &B11111110
  ENDIF
  'print msg(MSG_GEN_STATUS)
  
  IF msg(MSG_CMPS_CAL) = 255  THEN
    msg(MSG_GEN_STATUS) = msg(MSG_GEN_STATUS) OR &B00001000
  ELSE
    msg(MSG_GEN_STATUS) = msg(MSG_GEN_STATUS) AND &B11110111
  ENDIF
  'print msg(MSG_GEN_STATUS)
  
  'Estop is normally 1 if ok. Estops are pull low when active
  IF PIN(PIN_ES_STOP) = 0 THEN
    msg(MSG_GEN_STATUS) = msg(MSG_GEN_STATUS) OR &B00010000        'Make Estop detect as active as 1
  ELSE
    msg(MSG_GEN_STATUS) = msg(MSG_GEN_STATUS) AND &B11101111
  ENDIF
  'print msg(MSG_GEN_STATUS)
  'PRINT

END SUB


' Process code for text display.
' Text from PI on msg(MSG_DATA_TEXT)
' 0 means ready for more
' 32 .. 127 is the ASCII char
' 1 end of sentance - pump to screen
' 2 text for top line
' 3 text for bottom line
' 4 Font 1 (8x12) Standard
' 5 Font 8 (6x8) Smaller Font
' 6 Background Red
' 7 Background Blue
' 8 Backgroung Green
' 9 Backgrount White
SUB TextProcess

  LOCAL dt AS INTEGER
  dt = msg(MSG_DATA_TEXT)
  'print dt
  
  IF dt > 0 THEN                                      ' It is not zero so lets go to work
  
    PRINT dt
  
    SELECT CASE dt
      
      CASE 1                                          ' End of sentance so update screen
      
        IF fontState = 0 THEN                         ' Set font size
          Font 1, 3 
        ELSE
          FONT 1, 2                                     
        ENDIF
        
        IF colState = 6 THEN                            'Set Colours
          COLOUR RGB(WHITE), RGB(RED)
          CLS RGB(RED)
        ENDIF
        
        IF colState = 7 THEN
          COLOUR RGB(WHITE), RGB(BLUE)
          CLS RGB(BLUE)
        ENDIF
        
        IF colState = 8 THEN
          COLOUR RGB(WHITE), RGB(GREEN)
          CLS RGB(GREEN)
        ENDIF
        
        IF colState = 9 THEN
          COLOUR RGB(BLACK), RGB(WHITE)
          CLS RGB(WHITE)
        ENDIF
          
        IF topLine <> "" THEN topText = topLine         ' Write message
        IF bottomLine <>"" THEN bottomText = bottomLine ' Write message
        
        bottomLine = ""
        topLine = ""
              
      CASE 2
        textForTopLine = 1                              ' Command for top line text set
      CASE 3
        textForTopLine = 0                              ' Command for bottom line text set
      
      CASE 4
        fontState = 0                                   ' Command for normal font
      CASE 5
        fontState = 1                                   ' Command for small font
      
      CASE 6
        colState = 6                                    ' Command for colour change
      CASE 7
        colState = 7 
      CASE 8
        colState = 8 
      CASE 9
        colState = 9
         
    END SELECT
    
    IF dt >=32 AND dt<=127 THEN                       ' This is a character received
    
      IF textForTopLine = 1 THEN                      ' Updated either top or bottom line
        topLine = topLine + CHR$(dt)
      ELSE
        bottomLine = bottomLine + CHR$(dt)
      END IF
            
    END IF
        
    msg(MSG_DATA_TEXT) = 0                            ' Reset so PI can send next char or command
      
  END IF

END SUB
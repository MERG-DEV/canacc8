;     TITLE   "CANACC5 or CANACC8 source for combined SLiM / FLiM node for CBUS"

; Code is the same for both CANACC5 and CANACC8 except for the module ID

; filename CANACC8_v2s.asm  (RH) 14/06/15 
; Has CANACC8 module ID



; based on CANACC5_v2n. Now includes feedback events and startup options.
; Feedback etc only settable via FLiM 

; Original  SLiM / FLiM version  19/11/09
; this code is for 18F2480 
; Uses 4 MHz resonator and PLL for 16 MHz clock
; The setup timer is TMR3. Used during self enumeration.
; CAN bit rate of 125 Kbits/sec
; Standard frame only

; DIL switch sequence for SLiM

; 1 Output select LSB
; 2 Output select
; 3 Output select MSB
; 4 Polarity
; 5 Learn
; 6 Unlearn /reset

;Flash timer is TMR0.  

;node number release frame <0x51><NN hi><NN lo>
;keep alive frame  <0x52><NN hi><NN lo>
;set learn mode <0x53><NN hi><NN lo>
;out of learn mode <0x54><NN hi><NN lo>
;clear all events <0x55><NN hi><NN lo>  Valid only if in learn mode
;read no. of events left <0x56><NN hi><NN lo>
;set event in learn mode  <0xD2><EN1><EN2><EN3><EN4><EVI><EV>  uses EV indexing
;The EV sent will overwrite the existing one
;read event in learn mode <0xB2><EN1><EN2><EN3><EN4><EVI>
;unset event in learn mode <0x95><EN1><EN2><EN3><EN4>
;reply to 0xB2. <0xD3><EN1><EN2><EN3><EN4><EVI><EV>
;Also sent if attempt to read / write too many EVs. Returns with EVI = 0 in that case

;read node parameters <0x10> Only works in setup mode. Sends string of 7 bytes as 
;<0xEF><para1><para2><para3><para4><para5><para6><para7>

;this code assumes a three byte EV. EVI, EV2 and EV3. EV3 used for feedback 

;EV1 sets which outputs are active  (1 in each bit position is active)
;EV2 sets the polarity of each active output. A 1 bit is reverse.
;EV3 sets the feedback event. Format follows CANSERVO8C.

;NV1 to NV8 used for output timers as before.

; Uses NV9 for feedback delay
; NV10 and 11 for startup conditions. NV12 spare

; 128 Events possible. 8 feedback events. Just ON or OFF for position for each output.
; Output states can be polled and there is a SoD function.

; CANACC5 v2p has feedback and polling. This is a update on rev 2n.  (no rev O)
; This code is identical to CANACC8 rev v2m except for the module ID.   

; 24/05/15 Version 2r Beta 1. Fix bug in reval to set ENidx and EVidx correctly (RH)
; Now release version 2r. The version for both the CANACC5 and CANACC8 is now the same.
; v2sBeta 1 EEPROM layout mo0dified to make it compatable with earlier versions
;     EVstart now move back to 0xF00086
; Released as v2s  22/06/15  MB


;end of comments for CANACC5 / 8

; This is the bootloader section

;*  Filename Boot2.asm  30/10/09

;*************************************************************** * * * * * * * * * * * * * * ;*
;*  CBUS bootloader

;*  Based on the Microchip botloader 'canio.asm' tho which full acknowledgement is made.
;*  Relevant information is contained in the Microchip Application note AN247

;*
;* Basic Operation:
;* The following is a CAN bootloader designed for PIC18F microcontrollers
;* with built-in CAN such as the PIC18F458. The bootloader is designed to
;* be simple, small, flexible, and portable.
;*
;
;
;*
;* Commands:
;* Put commands received from source (Master --> Slave)
;* The count (DLC) can vary.
;* XXXXXXXXXXX 0 0 8 XXXXXXXX XXXXXX00 ADDRL ADDRH ADDRU RESVD CTLBT SPCMD CPDTL CPDTH
;* XXXXXXXXXXX 0 0 8 XXXXXXXX XXXXXX01 DATA0 DATA1 DATA2 DATA3 DATA4 DATA5 DATA6 DATA7
;*


;*
;* ADDRL - Bits 0 to 7 of the memory pointer.
;* ADDRH - Bits 8 - 15 of the memory pointer.
;* ADDRU - Bits 16 - 23 of the memory pointer.
;* RESVD - Reserved for future use.
;* CTLBT - Control bits.
;* SPCMD - Special command.
;* CPDTL - Bits 0 - 7 of 2s complement checksum
;* CPDTH - Bits 8 - 15 of 2s complement checksum
;* DATAX - General data.
;*
;* Control bits:
;* MODE_WRT_UNLCK-Set this to allow write and erase operations to memory.
;* MODE_ERASE_ONLY-Set this to only erase Program Memory on a put command. Must be on 64-byte
;*  boundary.
;* MODE_AUTO_ERASE-Set this to automatically erase Program Memory while writing data.
;* MODE_AUTO_INC-Set this to automatically increment the pointer after writing.
;* MODE_ACK-Set this to generate an acknowledge after a 'put' (PG Mode only)
;*
;* Special Commands:
;* CMD_NOP      0x00  Do nothing
;* CMD_RESET    0x01  Issue a soft reset after setting last EEPROM data to 0x00
;* CMD_RST_CHKSM  0x02  Reset the checksum counter and verify
;* CMD_CHK_RUN    0x03  Add checksum to special data, if verify and zero checksum
;* CMD_BOOT_TEST  0x04  Just sends a message frame back to verify boot mode.

;*  Modified version of the Microchip code by M Bolton  30/10/09
;
; The user program must have the folowing vectors

; User code reset vector  0x0800
; User code HPINT vector  0x0808
; user code LPINT vector  0x0818

; Checksum is 16 bit addition of all programmable bytes.
; User sends 2s complement of addition at end of program in command 0x03 (16 bits only)

;**********************************************************************************

; 
; Assembly options
  LIST  P=18F2480,r=hex,N=75,C=120,T=OFF

  include   "p18f2480.inc"
  include   "cbuslib/cbusdefs.inc"
  
  ;definitions  for ACC8   Change these to suit hardware.
  
S_PORT  equ PORTA ;setup switch  Change as needed
S_BIT equ 2

LEARN   equ 1 ;learn switch in port A
POL   equ 5 ;pol switch in port B
UNLEARN equ 0 ;unlearn / setup  in port A

LED_PORT equ  PORTB  ;change as needed
LED2  equ   7 ;PB7 is the green LED on the PCB
LED1  equ   6 ;PB6 is the yellow LED on the PCB


CMD_ON    equ 0x90  ;on event
CMD_OFF equ 0x91  ;off event
CMD_REQ equ 0x92  ;request event

SCMD_ON equ 0x98
SCMD_OFF  equ 0x99
SCMD_REQ  equ 0x9A
OPC_PNN equ 0xB6

OLD_EN_NUM  equ .32   ;old number of allowed events
EN_NUM  equ .128
HASH_SZ equ .8
EV_NUM  equ 3   ;number of allowed EVs per event

Modstat equ 1   ;address in EEPROM

;module types - returned in the Flags parameter



RQNN  equ 0xbc  ; response to OPC_QNN - provisional

MAN_NO      equ MANU_MERG    ;manufacturer number
MAJOR_VER   equ 2
MINOR_VER   equ "S"
MODULE_ID   equ MTYP_CANACC8   ; id to identify this type of module
EVT_NUM     equ EN_NUM           ; Number of events
EVperEVT    equ EV_NUM           ; Event variables per event
NV_NUM      equ .12             ; Number of node variables
NODEFLGS    equ PF_CONSUMER + PF_PRODUCER + PF_BOOT
CPU_TYPE    equ P18F2480
BETA    equ 0



; definitions used by bootloader

#define MODE_SELF_VERIFY  ;Enable self verification of written data (undefine if not wanted)

#define HIGH_INT_VECT 0x0808  ;HP interrupt vector redirect. Change if target is different
#define LOW_INT_VECT  0x0818  ;LP interrupt vector redirect. Change if target is different.
#define RESET_VECT  0x0800  ;start of target
#define CAN_CD_BIT  RXB0EIDL,0  ;Received control / data select bit
#define CAN_PG_BIT  RXB0EIDL,1  ;Received PUT / GET bit
#define CANTX_CD_BIT  TXB0EIDL,0  ;Transmit control/data select bit
#define CAN_TXB0SIDH  B'10000000' ;Transmitted ID for target node
#define CAN_TXB0SIDL  B'00001000'
#define CAN_TXB0EIDH  B'00000000' ;
#define CAN_TXB0EIDL  B'00000100'
#define CAN_RXF0SIDH  B'00000000' ;Receive filter for target node
#define CAN_RXF0SIDL  B'00001000'
#define CAN_RXF0EIDH  B'00000000'
#define CAN_RXF0EIDL  B'00000111'
#define CAN_RXM0SIDH  B'11111111' ;Receive masks for target node
#define CAN_RXM0SIDL  B'11101011'
#define CAN_RXM0EIDH  B'11111111'
#define CAN_RXM0EIDL  B'11111000'
#define CAN_BRGCON1   B'00000011' ;CAN bit rate controls. As for other CBUS modules
#define CAN_BRGCON2   B'10011110'
#define CAN_BRGCON3   B'00000011'
#define CAN_CIOCON    B'00100000' ;CAN I/O control  
; ************************************************************ ** * * * * * * * * * * * * * * *
; ************************************************************ ** * * * * * * * * * * * * * * *
#ifndef EEADRH    
#define EEADRH  EEADR+ 1  
#endif      
#define TRUE  1 
#define FALSE 0 
#define WREG1 PRODH ; Alternate working register
#define WREG2 PRODL 
#define MODE_WRT_UNLCK  _bootCtlBits, 0 ; Unlock write and erase
#define MODE_ERASE_ONLY _bootCtlBits, 1 ; Erase without write
#define MODE_AUTO_ERASE _bootCtlBits, 2 ; Enable auto erase before write
#define MODE_AUTO_INC _bootCtlBits, 3 ; Enable auto inc the address
#define MODE_ACK    _bootCtlBits, 4 ; Acknowledge mode
#define ERR_VERIFY    _bootErrStat, 0 ; Failed to verify if set
#define CMD_NOP     0x00  
#define CMD_RESET   0x01  
#define CMD_RST_CHKSM 0x02  
#define CMD_CHK_RUN   0x03
#define CMD_BOOT_TEST   0x04  



;set config registers. These are common to bootloader and ACC8. 

; note. there seem to be differences in the naming of the CONFIG parameters between
; versions of the p18F2480.inf files

  CONFIG  FCMEN = OFF, OSC = HSPLL, IESO = OFF
  CONFIG  PWRT = ON,BOREN = BOHW, BORV=0
  CONFIG  WDT=OFF
  CONFIG  MCLRE = ON
  CONFIG  LPT1OSC = OFF, PBADEN = OFF
  CONFIG  DEBUG = OFF
  CONFIG  XINST = OFF,LVP = OFF,STVREN = ON,CP0 = OFF
  CONFIG  CP1 = OFF, CPB = OFF, CPD = OFF,WRT0 = OFF,WRT1 = OFF, WRTB = OFF
  CONFIG  WRTC = OFF,WRTD = OFF, EBTR0 = OFF, EBTR1 = OFF, EBTRB = OFF
  

; processor uses  4 MHz. Resonator with HSPLL to give a clock of 16MHz

;********************************************************************************
; RAM addresses used by boot. can also be used by application.

  CBLOCK 0
  _bootCtlMem
  _bootAddrL    ; Address info
  _bootAddrH    
  _bootAddrU    
  _unused0    ;(Reserved)
  _bootCtlBits  ; Boot Mode Control bits
  _bootSpcCmd   ; Special boot commands
  _bootChkL   ; Chksum low byte fromPC
  _bootChkH   ; Chksum hi byte from PC    
  _bootCount    
  _bootChksmL   ; 16 bit checksum
  _bootChksmH   
  _bootErrStat  ;Error Status flags
  ENDC
  
  ; end of bootloader RAM


;****************************************************************
; define RAM storage for ACC8
  
  CBLOCK  0   ;file registers - access bank
          ;interrupt stack for low priority
          ;hpint uses fast stack
  W_tempL
  St_tempL
  Bsr_tempL
  PCH_tempH   ;save PCH in hpint
  PCH_tempL   ;save PCH in lpint (if used)
  Fsr_temp0L
  Fsr_temp0H 
  Fsr_temp1L
  Fsr_temp1H 
  Fsr_temp2L
  Fsr_temp2H
  Fsr_fb0L
  Fsr_fb0H 
  Fsr_fb1L
  Fsr_fb1H 
  
  TempCANCON
  TempCANSTAT
  TempINTCON
  TempECAN
  CanID_tmp ;temp for CAN Node ID
  IDtemph   ;used in ID shuffle
  IDtempl
  NN_temph    ;node number in RAM
  NN_templ
  ENtemp1   ;number of events
  
  IDcount   ;used in self allocation of CAN ID.
  
  Mode    ;for FLiM / SLiM etc
  Count   ;counter for loading
  Count1
  Count2
  Countfb
  Keepcnt   ;keep alive counter
  Latcount  ;latency counter
  Datmode   ;flag for data waiting and other states
  Temp    ;temps
  Temp1
  Dlc     ;data length

  

  

  Match   ;match flag
  ENcount   ;which EN matched
  ENcount1  ;temp for count offset
  ENend   ;last  EN number
  ENtemp    ;holds current EN pointer
  EVtemp    ;holds current EV pointer
  EVtemp1 
  EVtemp2   ;holds current EV qualifier
  EVtemp3   ;holds copy of Rx0d0 during ev_set routine
  EVtempp   ;temp used in poll routine
  Last    ;used in update of current state

  Rollfb    ;rolling bit for feedback events
  Op_fb   ;output number for feedback indexing
  Fbcount   ;counter for feedback bit

  
  Tx1con      ;start of transmit frame  1
  Tx1sidh
  Tx1sidl
  Tx1eidh
  Tx1eidl
  Tx1dlc
  Tx1d0
  Tx1d1
  Tx1d2
  Tx1d3
  Tx1d4
  Tx1d5
  Tx1d6
  Tx1d7
  
    ;***************************************************************
  Timout    ;used in timer routines
  Timbit    ;
  Timset    ;
  Timtemp
  OpBits    ;data for loading into PORTC after receiving an event
  OnBits    ;bits to turn on efter event
  OffBits   ;bits to turn off after event
  OpNum   ; output number 

  Roll    ;rolling bit for enum
  
  Fsr_tmp1Le  
  Fsr_tmp1He  
  
  ;variables used by Flash Ram event handling

  evaddrh     ; event data ptr
  evaddrl
  prevadrh    ; previous event data ptr
  prevadrl
  nextadrh    ; next event data ptr
  nextadrl
  htaddrh     ; current hash table ptr
  htaddrl
  htidx     ; index of current hash table entry in EEPROM
  hnum      ; actual hash number
  freadrh     ; current free chain address
  freadrl
  initFlags   ; used in intialising Flash from EEPROM events
  Saved_Fsr0L   ; used in rdfbev routine
  Saved_Fsr0H

  ev_opc      ;safe buffer
  ev0
  ev1
  ev2
  ev3
  EVidx   ; EV index from learn cmd
  EVdata    ; EV data from learn cmd

  ENidx   ; event index from commands which access events by index
  CountFb0  ; counters used by Flash handling
  CountFb1

; Timer control values, 1 per output
  T1      
  T2
  T3
  T4
  T5
  T6
  T7
  T8  

  E1
  E2
  E3
  E4
  E5
  E6
  E7
  E8

  Tmr3h   ;used in feedback delay
  Nv2     ;start position
  Nv3     ;run or not
  Nv4     ;not used yet
    
  ENDC
  
  CBLOCK    0x80
  T1Copy      ;reload timer registers for each output
  T2Copy
  T3Copy      ;these variable are only accessed indirectly
  T4Copy      ;in the lpint routine.
  T5Copy
  T6Copy
  T7Copy
  T8Copy    
    
  Enum0   ;bits for new enum scheme. Only accessed indirectly
  Enum1
  Enum2
  Enum3
  Enum4
  Enum5
  Enum6
  Enum7
  Enum8
  Enum9
  Enum10
  Enum11
  Enum12
  Enum13
  
  ENDC
  
  CBLOCK 0x100    ;bank 1
  ; 64 bytes of event data - the quanta size for updating Flash
  evt00       ; Event number - 4 bytes
  evt01
  evt02
  evt03
  next0h        ; next entry in list
  next0l
  prev0h        ; previous entry in list
  prev0l
  ev00        ; event variables - upto 8
  ev01
  ev02
  ev03
  ev04
  ev05
  ev06
  ev07
  
  evt10       ; Event number - 4 bytes
  evt11
  evt12
  evt13
  next1h        ; next entry in list
  next1l
  prev1h        ; previous entry in list
  prev1l
  ev10        ; event variables - upto 8
  ev11
  ev12
  ev13
  ev14
  ev15
  ev16
  ev17
  
  evt20       ; Event number - 4 bytes
  evt21
  evt22
  evt23
  next2h        ; next entry in list
  next2l
  prev2h        ; previous entry in list
  prev2l
  ev20        ; event variables - upto 8
  ev21
  ev22
  ev23
  ev24
  ev25
  ev26
  ev27
  
  evt30       ; Event number - 4 bytes
  evt31
  evt32
  evt33
  next3h        ; next entry in list
  next3l
  prev3h        ; previous entry in list
  prev3l
  ev30        ; event variables - upto 8
  ev31
  ev32
  ev33
  ev34
  ev35
  ev36
  ev37
  
  ENDC
    
  CBLOCK  0x200   ;bank 2
  EN1         ;start of EN ram
  EN1a
  EN1b
  EN1c
  
  EN2
  EN2a
  EN2b
  EN2c
  
  ENDC
  
  CBLOCK  0x280   ;bank 2
  EV1         ;start of EV ram
  ENDC
  
  CBLOCK 0x2C0
  NV_temp
  ENDC

;****************************************************************
; This is the bootloader
; ***************************************************************************** 
;_STARTUPCODE 0x00
  ORG 0x0000
; *****************************************************************************
  bra _CANInit
  bra _StartWrite
; ***************************************************************************** 
;_INTV_H CODE 0x08
  ORG 0x0008
; *****************************************************************************

  goto  HIGH_INT_VECT

; ***************************************************************************** 
;_INTV_L CODE 0x18
  ORG 0x0018
; *****************************************************************************

  goto  LOW_INT_VECT 

; ************************************************************** 
; Code start
; **************************************************************
  ORG 0x0020
;_CAN_IO_MODULE CODE
; ************************************************************ ** * * * * * * * * * * * * * * * 
; Function: VOID _StartWrite(WREG _eecon_data)
;PreCondition: Nothing
;Input: _eecon_data
;Output: Nothing. Self write timing started.
;Side Effects: EECON1 is corrupted; WREG is corrupted.
;Stack Requirements: 1 level.
;Overview: Unlock and start the write or erase sequence to protected
; memory. Function will wait until write is finished.
;
; ************************************************************ ** * * * * * * * * * * * * * * *
_StartWrite
  movwf   EECON1
  btfss   MODE_WRT_UNLCK  ; Stop if write locked
  return
  movlw   0x55  ; Unlock
  movwf    EECON2 
  movlw  0xAA 
  movwf    EECON2
  bsf  EECON1, WR ; Start the write
  nop
  btfsc   EECON1, WR  ; Wait (depends on mem type)
  bra $ - 2
  return
; ************************************************************ ** * * * * * * * * * * * * * * *

; Function: _bootChksm _UpdateChksum(WREG _bootChksmL)
;
; PreCondition: Nothing
; Input: _bootChksmL
; Output: _bootChksm. This is a static 16 bit value stored in the Access Bank.
; Side Effects: STATUS register is corrupted.
; Stack Requirements: 1 level.
; Overview: This function adds a byte to the current 16 bit checksum
; count. WREG should contain the byte before being called.
;
; The _bootChksm value is considered a part of the special
; register set for bootloading. Thus it is not visible. ;
;*************************************************************** * * * * * * * * * * * *
_UpdateChksum:
  addwf _bootChksmL,  F ; Keep a checksum
  btfsc STATUS, C
  incf  _bootChksmH,  F
  return
;************************************************************ ** * * * * * * * * * * * * * * *
;
; Function: VOID _CANInit(CAN,  BOOT)
;
; PreCondition: Enter only after a reset has occurred.
; Input: CAN control information, bootloader control information ; Output: None.
; Side Effects: N/A. Only run immediately after reset.
; Stack Requirements: N/A
; Overview: This routine is technically not a function since it will not
; return when called. It has been written in a linear form to
; save space.Thus 'call' and 'return' instructions are not
; included, but rather they are implied. ;
; This routine tests the boot flags to determine if boot mode is
; desired or normal operation is desired. If boot mode then the
; routine initializes the CAN module defined by user input. It
; also resets some registers associated to bootloading.
;
; ************************************************************ ** * * * * * * * * * * * * * * *
_CANInit:
  clrf  EECON1
  setf  EEADR ; Point to last location of EEDATA
  setf  EEADRH
  bsf EECON1, RD  ; Read the control code
  incfsz EEDATA, W

  goto  RESET_VECT


  clrf  _bootSpcCmd   ; Reset the special command register
  movlw   0x1C    ; Reset the boot control bits
  movwf   _bootCtlBits 
  movlb d'15'   ; Set Bank 15
  bcf   TRISB, CANTX  ; Set the TX pin to output 
  movlw   CAN_RXF0SIDH  ; Set filter 0
  movwf   RXF0SIDH
  movlw   CAN_RXF0SIDL 
  movwf   RXF0SIDL
  comf  WREG    ; Prevent filter 1 from causing a receive event





  movwf RXF1SIDL  ;   
  movlw CAN_RXF0EIDH  
  movwf RXF0EIDH  
  movlw CAN_RXF0EIDL  
  movwf RXF0EIDL  
  movlw CAN_RXM0SIDH  ; Set mask
  movwf RXM0SIDH  
  movlw CAN_RXM0SIDL  
  movwf RXM0SIDL  
  movlw CAN_RXM0EIDH  
  movwf RXM0EIDH  
  movlw CAN_RXM0EIDL  
  movwf RXM0EIDL  
  movlw CAN_BRGCON1 ; Set bit rate
  movwf BRGCON1 
  movlw CAN_BRGCON2 
  movwf BRGCON2 
  movlw CAN_BRGCON3 
  movwf BRGCON3 
  movlw CAN_CIOCON  ; Set IO
  movwf CIOCON  
  
  clrf  CANCON  ; Enter Normal mode
  movlw B'00001110'
  movwf ADCON1
  bcf TRISB,7
  bcf TRISB,6
  bsf PORTB,7   ;green LED on
  bsf PORTB,6   ;yellow LED on


; ************************************************************ ** * * * * * * * * * * * * * * * 
; This routine is essentially a polling loop that waits for a
; receive event from RXB0 of the CAN module. When data is
; received, FSR0 is set to point to the TX or RX buffer depending
; upon whether the request was a 'put' or a 'get'.
; ************************************************************ ** * * * * * * * * * * * * * * * 
_CANMain
  
  bcf RXB0CON, RXFUL  ; Clear the receive flag
_wait clrwdt      ; Clear WDT while waiting   
  btfss   RXB0CON, RXFUL  ; Wait for a message  
  bra _wait



_CANMainJp1
  lfsr  0, RXB0D0
  movf  RXB0DLC, W 
  andlw   0x0F
  movwf   _bootCount 
  movwf   WREG1
  bz  _CANMain 
_CANMainJp2       ;?
  


; ************************************************************** * * * * * * * * * * * * * * * 
; Function: VOID _ReadWriteMemory()
;
; PreCondition:Enter only after _CANMain().
; Input: None.
; Output: None.
; Side Effects: N/A.
; Stack Requirements: N/A
; Overview: This routine is technically not a function since it will not
; return when called. It has been written in a linear form to
; save space.Thus 'call' and 'return' instructions are not
; included, but rather they are implied.
;This is the memory I/O engine. A total of eight data bytes are received and decoded. In addition two control bits are received, put/get and control/data.
;A pointer to the buffer is passed via FSR0 for reading or writing. 
;The control register set contains a pointer, some control bits and special command registers.
;Control
;<PG><CD><ADDRL><ADDRH><ADDRU><_RES_><CTLBT>< SPCMD><CPDTL><CPDTH>
;Data
;<PG>< CD>< DATA0>< DATA1>< DATA2>< DATA3>< DATA4>< DATA5>< DATA6>< DATA7>
;PG bit:  Put = 0, Get = 1
;CD bit:  Control = 0, Data = 1

; ************************************************************ ** * * * * * * * * * * * * * * *
_ReadWriteMemory:
  btfsc CAN_CD_BIT  ; Write/read data or control registers
  bra _DataReg
; ************************************************************ ** * * * * * * * * * * * * * * * ; This routine reads or writes the bootloader control registers,
; then executes any immediate command received.
_ControlReg
  lfsr  1, _bootAddrL   ;_bootCtlMem
_ControlRegLp1

  movff   POSTINC0, POSTINC1 
  decfsz  WREG1, F
  bra _ControlRegLp1

; ********************************************************* 
; This is a no operation command.
  movf  _bootSpcCmd, W    ; NOP Command
  bz  _CANMain
; bz  _SpecialCmdJp2    ; or send an acknowledge

; ********************************************************* 
; This is the reset command.
  xorlw   CMD_RESET   ; RESET Command 
  btfss   STATUS, Z
  bra   _SpecialCmdJp4
  setf  EEADR   ; Point to last location of EEDATA
  setf  EEADRH
  clrf  EEDATA    ; and clear the data (FF for now)
  movlw   b'00000100' ; Setup for EEData
  rcall   _StartWrite
  bcf   PORTB,6   ;yellow LED off
  reset
; *********************************************************
; This is the Selfcheck reset command. This routine 
; resets the internal check registers, i.e. checksum and 
; self verify.
_SpecialCmdJp4
  movf  _bootSpcCmd, W 
  xorlw   CMD_RST_CHKSM
  bnz   _SpecialCmdJp1
  clrf  _bootChksmH
  clrf  _bootChksmL
  bcf   ERR_VERIFY    
  clrf  _bootErrStat
  bra   _CANMain
; RESET_CHKSM Command
; Reset chksum
; Clear the error verify flag

;This is the Test and Run command. The checksum is
; verified, and the self-write verification bit is checked. 
; If both pass, then the boot flag is cleared.
_SpecialCmdJp1
  movf  _bootSpcCmd, W    ; RUN_CHKSM Command
  xorlw   CMD_CHK_RUN 
  bnz _SpecialCmdJp3
  movf  _bootChkL, W  ; Add the control byte
  addwf  _bootChksmL, F
  bnz _SpecialCmdJp2
  movf  _bootChkH, W 
  addwfc  _bootChksmH, F
  bnz _SpecialCmdJp2
  btfsc   ERR_VERIFY    ; Look for verify errors
  bra _SpecialCmdJp2

  bra   _CANSendOK  ;send OK message


_SpecialCmdJp2

  bra _CANSendNOK ; or send an error acknowledge


_SpecialCmdJp3
  movf  _bootSpcCmd, W    ; RUN_CHKSM Command
  xorlw   CMD_BOOT_TEST 
  bnz _CANMain
  bra _CANSendBoot

; ************************************************************** * * * * * * * * * * * * * * * 
; This is a jump routine to branch to the appropriate memory access function.
; The high byte of the 24-bit pointer is used to determine which memory to access. 
; All program memories (including Config and User IDs) are directly mapped. 
; EEDATA is remapped.
_DataReg
; *********************************************************
_SetPointers
  movf  _bootAddrU, W ; Copy upper pointer
  movwf   TBLPTRU
  andlw   0xF0  ; Filter
  movwf   WREG2
  movf  _bootAddrH, W ; Copy the high pointer
  movwf   TBLPTRH
  movwf   EEADRH
  movf  _bootAddrL, W ; Copy the low pointer
  movwf   TBLPTRL
  movwf  EEADR
  btfss   MODE_AUTO_INC ; Adjust the pointer if auto inc is enabled
  bra _SetPointersJp1
  movf  _bootCount, W ; add the count to the pointer
  addwf  _bootAddrL, F 
  clrf  WREG
  addwfc   _bootAddrH, F 
  addwfc   _bootAddrU, F 

_SetPointersJp1     ;?

_Decode
  movlw   0x30
  cpfslt  WREG2
  bra _DecodeJp1



  bra _PMEraseWrite

_DecodeJp1
  movf  WREG2,W
  xorlw   0x30
  bnz _DecodeJp2



  bra _CFGWrite 
_DecodeJp2
  movf  WREG2,W 
  xorlw 0xF0
  bnz _CANMain
  bra _EEWrite

f 

; Program memory < 0x300000
; Config memory = 0x300000
; EEPROM data = 0xF00000
  
; ************************************************************ ** * 
; ************************************************************** * 
; Function: VOID _PMRead()
; VOID _PMEraseWrite ()
;
; PreCondition:WREG1 and FSR0 must be loaded with the count and address of
; the source data.
; Input: None.
; Output: None.
; Side Effects: N/A.
; Stack Requirements: N/A
; Overview: These routines are technically not functions since they will not
; return when called. They have been written in a linear form to
; save space.Thus 'call' and 'return' instructions are not
; included, but rather they are implied.
;These are the program memory read/write functions. Erase is available through control flags. An automatic erase option is also available.
; A write lock indicator is in place to ensure intentional write operations.
;Note: write operations must be on 8-byte boundaries and must be 8 bytes long. Also erase operations can only occur on 64-byte boundaries.
; ************************************************************ ** * * * * * * * * * * * * * * *



_PMEraseWrite:
  btfss   MODE_AUTO_ERASE
  bra _PMWrite
_PMErase:
  movf  TBLPTRL, W
  andlw b'00111111'
  bnz _PMWrite
_PMEraseJp1
  movlw b'10010100' 
  rcall   _StartWrite 
_PMWrite:
  btfsc   MODE_ERASE_ONLY


  bra _CANMain 

  movf  TBLPTRL, W
  andlw b'00000111'
  bnz _CANMain 
  movlw   0x08
  movwf WREG1

_PMWriteLp1         ; Load the holding registers
  movf  POSTINC0, W 
  movwf   TABLAT
  rcall  _UpdateChksum  ; Adjust the checksum
  tblwt*+
  decfsz   WREG1, F
  bra _PMWriteLp1

#ifdef MODE_SELF_VERIFY 
  movlw  0x08
  movwf   WREG1 
_PMWriteLp2
  tblrd*-     ; Point back into the block
  movf  POSTDEC0, W 
  decfsz   WREG1, F
  bra _PMWriteLp2
  movlw  b'10000100'  ; Setup writes
  rcall _StartWrite   ; Write the data
  movlw   0x08
  movwf   WREG1
_PMReadBackLp1
  tblrd*+     ; Test the data
  movf  TABLAT, W 
  xorwf   POSTINC0, W
  btfss STATUS, Z
  bsf ERR_VERIFY 
  decfsz  WREG1, F
  bra _PMReadBackLp1  ; Not finished then repeat
#else
  tblrd*-     ; Point back into the block
         ; Setup writes
  movlw   b'10000100'   ; Write the data
  rcall   _StartWrite   ; Return the pointer position
  tblrd*+
#endif

  bra _CANMain


; ************************************************************** * * * * * * * * * * * * * * *
 ; Function: VOID _CFGWrite()
; VOID _CFGRead()
;
; PreCondition:WREG1 and FSR0 must be loaded with the count and address of the source data. 
; Input: None.
; Output: None.
; Side Effects: N/A.
; Stack Requirements: N/A
; Overview: These routines are technically not functions since they will not
; return when called. They have been written in a linear form to
; save space. Thus 'call' and 'return' instructions are not
; included, but rather they are implied.
;
; These are the Config memory read/write functions. Read is
; actually the same for standard program memory, so any read
; request is passed directly to _PMRead.
;
; ************************************************************ ** * * * * * * * * * * * * * * *
_CFGWrite

#ifdef MODE_SELF_VERIFY   ; Write to config area
  movf  INDF0, W    ; Load data
#else
  movf  POSTINC0, W
#endif
  movwf   TABLAT
  rcall   _UpdateChksum ; Adjust the checksum
  tblwt*      ; Write the data
  movlw b'11000100' 
  rcall   _StartWrite
  tblrd*+     ; Move the pointers and verify
#ifdef MODE_SELF_VERIFY 
  movf  TABLAT, W 
  xorwf   POSTINC0, W

#endif
  decfsz  WREG1, F
  bra _CFGWrite ; Not finished then repeat

  bra _CANMain 



; ************************************************************** * * * * * * * * * * * * * * * 
; Function: VOID _EERead()
; VOID _EEWrite()
;
; PreCondition:WREG1 and FSR0 must be loaded with the count and address of
 ;  the source data.
; Input:  None.
; Output: None.
; Side Effects: N/A.
; Stack Requirements: N/A
; Overview: These routines are technically not functions since they will not
; return when called. They have been written in a linear form to
; save space. Thus 'call' and 'return' instructions are not
; included, but rather they are implied.
;
; This is the EEDATA memory read/write functions.
;
; ************************************************************ ** * * * * * * * * * * * * * * *


_EEWrite:

#ifdef MODE_SELF_VERIFY
  movf  INDF0, W
#else
  movf  POSTINC0, W 
#endif

  movwf   EEDATA
  rcall   _UpdateChksum 
  movlw b'00000100' 
  rcall  _StartWrite

#ifdef MODE_SELF_VERIFY 
  clrf  EECON1
  bsf EECON1, RD
  movf  EEDATA, W 
  xorwf   POSTINC0, W
  btfss STATUS, Z
  bsf ERR_VERIFY
#endif

  infsnz   EEADR, F 
  incf  EEADRH, F 
  decfsz  WREG1, F
  bra _EEWrite


  bra _CANMain 
  

; Read the data

; Adjust EEDATA pointer
; Not finished then repeat
; Load data
; Adjust the checksum 
; Setup for EEData
; and write
; Read back the data ; verify the data ; and adjust pointer
; Adjust EEDATA pointer
; Not finished then repeat

; ************************************************************** * * * * * * * * * * * * * * *
; Function: VOID _CANSendAck()
; VOID _CANSendResponce ()
;
; PreCondition:TXB0 must be preloaded with the data.
; Input: None.
; Output: None.
; Side Effects: N/A.
; Stack Requirements: N/A
; Overview: These routines are technically not functions since they will not
; return when called. They have been written in a linear form to
; save space. Thus 'call' and 'return' instructions are not
; included, but rather they are implied. ;
; These routines are used for 'talking back' to the source. The
; _CANSendAck routine sends an empty message to indicate
; acknowledgement of a memory write operation. The
; _CANSendResponce is used to send data back to the source. ;
; ************************************************************ ** * * * * * * * * * * * * * * *



_CANSendMessage
  btfsc   TXB0CON,TXREQ 
  bra $ - 2
  movlw   CAN_TXB0SIDH 
  movwf   TXB0SIDH
  movlw   CAN_TXB0SIDL 
  movwf   TXB0SIDL
  movlw   CAN_TXB0EIDH 
  movwf   TXB0EIDH  

  movlw CAN_TXB0EIDL
  movwf TXB0EIDL
  bsf CANTX_CD_BIT
  btfss CAN_CD_BIT 
  bcf CANTX_CD_BIT
  bsf TXB0CON, TXREQ
      bra  _CANMain ; Setup the command bit

_CANSendOK        ;send OK message 
  movlw 1     ;a 1 is OK
  movwf TXB0D0
  movwf TXB0DLC
  bra   _CANSendMessage
  
_CANSendNOK       ;send not OK message
  clrf  TXB0D0    ;a 0 is not OK
  movlw 1
  movwf TXB0DLC
  bra   _CANSendMessage

_CANSendBoot
  movlw 2     ;2 is confirm boot mode
  movwf TXB0D0
  movlw 1
  movwf TXB0DLC
  bra   _CANSendMessage
    
; Start the transmission

;   End of bootloader

;************************************************************************************************************
;
;   start of ACC8 program code

    ORG   0800h
loadadr   
    nop           ;for debug
    goto  setup

    ORG   0808h
    goto  hpint     ;high priority interrupt
    
    ORG   0810h     ;node type parameters
myName  db  "ACC5   "

    ORG   0818h 
    goto  lpint     ;low priority interrupt

    ORG   0820h

nodeprm     db  MAN_NO, MINOR_VER, MODULE_ID, EVT_NUM, EVperEVT, NV_NUM 
      db  MAJOR_VER,NODEFLGS,CPU_TYPE,PB_CAN    ; Main parameters
            dw  RESET_VECT     ; Load address for module code above bootloader
            dw  0           ; Top 2 bytes of 32 bit address not used
            db  0,0,0,0,CPUM_MICROCHIP,BETA
sparprm     fill 0,prmcnt-$ ; Unused parameter space set to zero

PRMCOUNT    equ sparprm-nodeprm ; Number of parameter bytes implemented

             ORG 0838h

prmcnt      dw  PRMCOUNT    ; Number of parameters implemented
nodenam     dw  myName      ; Pointer to module type name
            dw  0 ; Top 2 bytes of 32 bit address not used


PRCKSUM     equ MAN_NO+MINOR_VER+MODULE_ID+EVT_NUM+EVperEVT+NV_NUM+MAJOR_VER+NODEFLGS+CPU_TYPE+PB_CAN+HIGH myName+LOW myName+HIGH loadadr+LOW loadadr+PRMCOUNT+CPUM_MICROCHIP+BETA


cksum       dw  PRCKSUM     ; Checksum of parameters


;*******************************************************************

    ORG   0840h     ;start of program
; 
;
;   high priority interrupt. Used for CAN receive and transmit error.

hpint   movwf W_tempL       
    movff STATUS,St_tempL
    movff CANCON,TempCANCON
    movff CANSTAT,TempCANSTAT
  
    movff PCLATH,PCH_tempH    ;save PCLATH
    clrf  PCLATH
  
    movff FSR0L,Fsr_temp0L    ;save FSR0
    movff FSR0H,Fsr_temp0H
    movff FSR1L,Fsr_temp1L    ;save FSR1
    movff FSR1H,Fsr_temp1H
    
    

    movlw 0x0A      ;for relocated code
    movwf PCLATH
    movf  TempCANSTAT,W     ;Jump table
    andlw B'00001110'
    addwf PCL,F     ;jump
    bra   back
    bra   errint      ;error interrupt
    bra   back
    bra   back
    bra   back
    bra   back      ;only receive interrupts used (now in ECAN)
    bra   back
    bra   back
    

    

    
    ;error routine here. Only acts on lost arbitration  
errint  movlb .15         ;change bank      
    btfss TXB1CON,TXLARB
    bra   errbak        ;not lost arb.
  
    movf  Latcount,F      ;is it already at zero?
    bz    errbak
    decfsz  Latcount,F
    bra   errbak
    bcf   TXB1CON,TXREQ
    movlw B'00111111'
    andwf TXB1SIDH,F      ;change priority
txagain bsf   TXB1CON,TXREQ   ;try again
          
errbak    bcf   RXB1CON,RXFUL
    movlb 0
    bcf   RXB0CON,RXFUL   ;ready for next
    
    bcf   COMSTAT,RXB0OVFL  ;clear overflow flags if set
    bcf   COMSTAT,RXB1OVFL    
    bra   back1


    
back  bcf   RXB0CON,RXFUL ;ready for next
  
  
back1 clrf  PIR3      ;clear all flags
    movf  CANCON,W
    andlw B'11110001'
    iorwf TempCANCON,W
    
    movwf CANCON
    movff PCH_tempH,PCLATH
    movff Fsr_temp0L,FSR0L    ;recover FSR0
    movff Fsr_temp0H,FSR0H

    movff Fsr_temp1L,FSR1L    ;recover FSR1
    movff Fsr_temp1H,FSR1H
    movf  W_tempL,W
    movff St_tempL,STATUS 
    
    retfie  

;now handled as a subroutine      
    
isRTR btfsc Datmode,1   ;setup mode?
    return          ;back
    btfss Mode,1      ;FLiM?
    return
    movlb .15
isRTR1  btfsc TXB2CON,TXREQ 
    bra   isRTR1    
    bsf   TXB2CON,TXREQ ;send ID frame - preloaded in TXB2

    movlb 0
    return


;**************************************************************
;
;
;   low priority interrupt. Used by output timer overflow. Every 10 millisecs.
; 

lpint movwf W_tempL       ;used for output timers
    movff STATUS,St_tempL
    movff BSR,Bsr_tempL
    movff FSR0H, Fsr_temp0H
    movff FSR0L, Fsr_temp0L
    movff FSR1H, Fsr_temp1H
    movff FSR1L, Fsr_temp1L

    movlb 0
    
    btfss PIR1,TMR1IF     ;is it timer 1?
    bra   tmr3

lp0   movlw 0xE0        ;Timer 1 lo byte. (adjust if needed)
    movwf TMR1L       ;reset timer 1
    clrf  PIR1        ;clear all timer flag
    
lp1   clrf  Timout
    clrf  Timbit        ;rolling bit for testing which timer
    
    movf  T1,W
    bz    doT2
    decfsz  T1,F
    bra   doT2
    bsf   Timout,0      ;set bits in Timout if it needs to go off
    movff T1Copy, T1
doT2    
    movf  T2,W
    bz    doT3
    decfsz  T2,F
    bra   doT3
    bsf   Timout,1
    movff T2Copy, T2
doT3
    movf  T3,W
    bz    doT4
    decfsz  T3,F
    bra   doT4
    bsf   Timout,2
    movff T3Copy, T3
    
doT4
    movf  T4,W
    bz    doT5
    decfsz  T4,F
    bra   doT5
    bsf   Timout,3
    movff T4Copy, T4
doT5    
    movf  T5,W
    bz    doT6
    decfsz  T5,F
    bra   doT6
    bsf   Timout,4
    movff T5Copy, T5

doT6  movf  T6,W
    bz    doT7
    decfsz  T6,F
    bra   doT7
    bsf   Timout,5
    movff T6Copy, T6
doT7
    movf  T7,W
    bz    doT8
    decfsz  T7,F
    bra   doT8
    bsf   Timout,6
    movff T7Copy, T7
doT8
    movf  T8,W
    bz    doFlags
    decfsz  T8,F
    bra   doFlags
    bsf   Timout,7
    movff T8Copy, T8
    
doFlags
    tstfsz  Timout
    bra   off           ;turn off outputs
    bra   lpend         ;nothing to do
    
off   movf  Timout,w
    xorwf PORTC         ; set outputs
    bra   lpend

tmr3  bcf   PIR2,TMR3IF       ;clear flag
    bcf   PIE2,TMR3IE
    movlw 0x09
    movwf PCLATH
    movf  Op_fb,W         ;output  number
    rlncf WREG
    addwf PCL           ;state table jump
    bra   fb1
    bra   fb2
    bra   fb3
    bra   fb4
    bra   fb5
    bra   fb6
    bra   fb7
    bra   fb8

fb1   movf  Rollfb,W
    andwf EVtemp,W        ;is this output enabled?
    bnz   fb_ev1          ;yes, send feedback event for OP1
fb1b  incf  Op_fb
fb2   rlncf Rollfb,F
    movf  Rollfb,W
    andwf EVtemp,W        ;is this output enabled?
    bnz   fb_ev2          ;yes, send feedback event for OP2
fb2b  incf  Op_fb
fb3   rlncf Rollfb,F
    movf  Rollfb,W
    andwf EVtemp,W        ;is this output enabled?
    bnz   fb_ev3          ;yes, send feedback event for OP3
fb3b  incf  Op_fb
fb4   rlncf Rollfb,F
    movf  Rollfb,W
    andwf EVtemp,W        ;is this output enabled?
    bnz   fb_ev4          ;yes, send feedback event for OP4
fb4b  incf  Op_fb
fb5   rlncf Rollfb,F
    movf  Rollfb,W
    andwf EVtemp,W        ;is this output enabled?
    bnz   fb_ev5          ;yes, send feedback event for OP5
fb5b  incf  Op_fb
fb6   rlncf Rollfb,F
    movf  Rollfb,W
    andwf EVtemp,W        ;is this output enabled?
    bnz   fb_ev6          ;yes, send feedback event for OP6
fb6b  incf  Op_fb
fb7   rlncf Rollfb,F
    movf  Rollfb,W
    andwf EVtemp,W        ;is this output enabled?
    bnz   fb_ev7          ;yes, send feedback event for OP7
fb7b  incf  Op_fb
fb8   rlncf Rollfb,F
    movf  Rollfb,W
    andwf EVtemp,W        ;is this output enabled?
    bnz   fb_ev8          ;yes, send feedback event for OP8
fb8b  nop   
    
lpend
    movff Fsr_temp0H, FSR0H
    movff Fsr_temp0L, FSR0L
    movff Fsr_temp1H, FSR1H
    movff Fsr_temp1L, FSR1L
    movff Bsr_tempL,BSR
    movf  W_tempL,W
    movff St_tempL,STATUS 
    retfie  

fb_ev1  btfss E1,7    ;any FB event?
    bra   fb1b    ;no
    movf  Rollfb,W  ;get bit
    andwf Last,W    ;on or off?
    bnz   fb1_on
    btfsc E1,6    ;polarity reversed?
    bra   fb1_on
    bra   fb1_off
fb1_on  btfsc E1,6    ;polarity reversed?
    bra   fb1_off
    call  fbev_on   ;send fedback event
    bra   fb_out    ;done
fb1_off call  fbev_off
    bra   fb_out

fb_ev2  btfss E2,7    ;any FB event?
    bra   fb2b    ;no
    movf  Rollfb,W  ;get bit
    andwf Last,W    ;on or off?
    bnz   fb2_on
    btfsc E2,6    ;polarity reversed?
    bra   fb2_on
    bra   fb2_off
fb2_on  btfsc E2,6    ;polarity reversed?
    bra   fb2_off
    call  fbev_on   ;send fedback event
    bra   fb_out    ;done
fb2_off call  fbev_off
    bra   fb_out

fb_ev3  btfss E3,7    ;any FB event?
    bra   fb3b    ;no
    movf  Rollfb,W  ;get bit
    andwf Last,W    ;on or off?
    bnz   fb3_on
    btfsc E3,6    ;polarity reversed?
    bra   fb3_on
    bra   fb3_off
fb3_on  btfsc E3,6    ;polarity reversed?
    bra   fb3_off
    call  fbev_on   ;send fedback event
    bra   fb_out    ;done
fb3_off call  fbev_off
    bra   fb_out

fb_ev4  btfss E4,7    ;any FB event?
    bra   fb4b    ;no
    movf  Rollfb,W  ;get bit
    andwf Last,W    ;on or off?
    bnz   fb4_on
    btfsc E4,6    ;polarity reversed?
    bra   fb4_on
    bra   fb4_off
fb4_on  btfsc E4,6    ;polarity reversed?
    bra   fb4_off
    call  fbev_on   ;send fedback event
    bra   fb_out    ;done
fb4_off call  fbev_off
    bra   fb_out

fb_ev5  btfss E5,7    ;any FB event?
    bra   fb5b    ;no
    movf  Rollfb,W  ;get bit
    andwf Last,W    ;on or off?
    bnz   fb5_on
    btfsc E5,6    ;polarity reversed?
    bra   fb5_on
    bra   fb5_off
fb5_on  btfsc E5,6    ;polarity reversed?
    bra   fb5_off
    call  fbev_on   ;send fedback event
    bra   fb_out    ;done
fb5_off call  fbev_off
    bra   fb_out

fb_ev6  btfss E6,7    ;any FB event?
    bra   fb6b    ;no
    movf  Rollfb,W  ;get bit
    andwf Last,W    ;on or off?
    bnz   fb6_on
    btfsc E6,6    ;polarity reversed?
    bra   fb6_on
    bra   fb6_off
fb6_on  btfsc E6,6    ;polarity reversed?
    bra   fb6_off
    call  fbev_on   ;send fedback event
    bra   fb_out    ;done
fb6_off call  fbev_off
    bra   fb_out

fb_ev7  btfss E7,7    ;any FB event?
    bra   fb7b    ;no
    movf  Rollfb,W  ;get bit
    andwf Last,W    ;on or off?
    bnz   fb7_on
    btfsc E7,6    ;polarity reversed?
    bra   fb7_on
    bra   fb7_off
fb7_on  btfsc E7,6    ;polarity reversed?
    bra   fb7_off
    call  fbev_on   ;send fedback event
    bra   fb_out    ;done
fb7_off call  fbev_off
    bra   fb_out

fb_ev8  btfss E8,7    ;any FB event?
    bra   fb8b    ;no
    movf  Rollfb,W  ;get bit
    andwf Last,W    ;on or off?
    bnz   fb8_on
    btfsc E8,6    ;polarity reversed?
    bra   fb8_on
    bra   fb8_off
fb8_on  btfsc E8,6    ;polarity reversed?
    bra   fb8_off
    call  fbev_on   ;send fedback event
    bra   lpend       ;no more
fb8_off call  fbev_off
    bra   lpend       

fb_out  incf  Op_fb   ;end except for last one
    movf  Tmr3h,W
    clrf  TMR3L   ;reload timer
    movwf TMR3H
    bsf   PIE2,TMR3IE ;re-enable interrupt
    bra   lpend

  



                        

;*********************************************************************

; main waiting loop

main  btfsc Datmode,0   ;busy?
    bra   main_OK
    btfsc COMSTAT,7   ;look for CAN input. 
    bra   getcan
    bra   main_OK 

getcan  movf  CANCON,W
    andlw B'00001111'
    movwf TempCANCON
    movf  ECANCON,W
    andlw B'11110000'
    iorwf TempCANCON,W
    movwf ECANCON
    btfsc RXB0SIDL,EXID   ;ignore extended frames here
    bra   no_can
    btfss RXB0DLC,RXRTR   ;is it RTR?
    bra   get_1
    call  isRTR
    bra   no_can
    
get_1           
    
get_3 movf  RXB0DLC,F
    bnz   get_2     ;ignore zero length frames
    bra   no_can 
get_2 call  copyev      ;save buffer
    bsf   Datmode,0   ;valid message frame  
    bra   main_OK

no_can  bcf   RXB0CON,RXFUL

main_OK   btfsc Mode,1      ;is it SLiM?
    bra   mainf     ;no

mains             ;is SLiM

    btfss PIR2,TMR3IF   ;flash timer overflow?
    bra   nofl_s      ;no SLiM flash
    btg   PORTB,7     ;toggle green LED
    bcf   PIR2,TMR3IF
nofl_s  bra   noflash       ;main1
    
; here if FLiM mde

mainf btfss INTCON,TMR0IF   ;is it flash?
    bra   noflash
    btfss Datmode,2
    bra   nofl1
    
    btg   PORTB,6     ;flash yellow LED
    
nofl1 bcf   INTCON,TMR0IF
    btfss Datmode,3   ;running mode
    bra   noflash
    decfsz  Keepcnt     ;send keep alive?
    bra   noflash
    movlw .10
    movwf Keepcnt
    movlw 0x52
;   call  nnrel     ;send keep alive frame (works OK, turn off for now)

noflash btfsc S_PORT,S_BIT  ;setup button?
    bra   main3
    movlw .100
    movwf Count
    clrf  Count1
    clrf  Count2
wait  decfsz  Count2
    goto  wait
    btfss Datmode,2
    bra   wait2
    btfss INTCON,TMR0IF   ;is it flash?
    bra   wait2
    btg   PORTB,6     ;flash LED
    bcf   INTCON,TMR0IF
wait2 decfsz  Count1
    goto  wait
    btfsc S_PORT,S_BIT
    bra   main4     ;not held long enough
    decfsz  Count
    goto  wait
    btfss Mode,1      ;is it in FLiM?
    bra   go_FLiM
    clrf  Datmode     ;back to virgin

    bcf   PORTB,6     ;yellow off
    
    bsf   PORTB,7     ;Green LED on
    clrf  INTCON      ;interrupts off
    movlw 1
    movwf IDcount     ;back to start
    movlw Modstat
    movwf EEADR
    movlw   0
    call  eewrite     ;status to reset
    movlw 0x51      ;send node release frame
    call  nnrel
    clrf  NN_temph
    clrf  NN_templ
wait1 btfss S_PORT,S_BIT
    bra   wait1     ;wait till release
    call  ldely
    btfss S_PORT,S_BIT
    bra   wait1
  
    
    movlw LOW NodeID      ;put NN back to 0000
    movwf EEADR
    movlw 0
    call  eewrite
    incf  EEADR
    movlw 0
    call  eewrite 
    btfss Mode,1
    bra   main5       ;FLiM setup
    movlw Modstat
    movwf EEADR
    movlw 0
    call  eewrite       ;mode back to SLiM
    clrf  Datmode
    bcf   Mode,1
    bcf   PORTB,6
    bsf   PORTB,7       ;green LED on
  
    movlw B'11000000'
    movwf INTCON
    goto  main        ;

main5 movlw Modstat
    movwf EEADR
    movlw 1
    call  eewrite       ;mode to FLiM in EEPROM
    bsf   Mode,1        ;to FLiM
    call  self_en       ;self enumerate routine
    bcf   Datmode,1
    call  nnack       ;send request for NN
    bsf   Datmode,2

    bra   main1


main4 
    btfss Datmode,2
    bra   mset2
    bcf   Datmode,2
    bsf   PORTB,6     ;LED on
    movlw 0x52
    call  nnrel
    movlw Modstat
    movwf EEADR
    movlw B'00001000'
    movwf Datmode     ;normal
    call  eewrite
    bra   main3
    
mset2 bsf   Datmode,2
    call  self_en
    bcf   Datmode,1
    call  nnack
    bra   main1

main3 btfss Datmode,1   ;setup mode ?
    bra   main1
;   call  self_en

    bcf   Datmode,1   ;out of setup
    bsf   Datmode,2   ;wait for NN

  
;   
;   bsf   PORTB,7     ;on light
    bra   main1     ;continue normally

go_FLiM bsf   Datmode,1   ;FLiM setup mode
    bcf   PORTB,7     ;green off
    bra   wait1
    
    

; common to FLiM and SLiM   
  
  
main1 
    btfss Datmode,0   ;any new CAN frame received?
    bra   main
    
    bra   packet      ;yes
;   bra   do        ;look for inputs

;********************************************************************



unset ;bsf  Datmode,5   ;unlearn this event
    ;bra  go_on
    btfss Datmode,4
    bra   main2     ;prevent error message
    bsf   Datmode,5
    call  copyev
    bra   learn2
    
readEV  btfss Datmode,4
    bra   main2     ;prevent error message
    call  copyev
    movf  EVidx,w     ;check EV index
    bz    rdev1
    decf  EVidx
    movlw EV_NUM
    cpfslt  EVidx
rdev1 bra   noEV1
    bsf   Datmode,6
    bra   learn2

evns1 call  thisNN        ;read event numbers
    sublw 0
    bnz   evns3
    call  evnsend
    bra   main2
evns3 goto  notNN

reval call  thisNN        ;read event numbers
    sublw 0
    bnz   notNNx
    movff RXB0D3, ENidx
    movff RXB0D4, EVidx
;   movff Rx0d3,ENidx
;   movff Rx0d4,EVidx
    call  evsend
    bra   main2
notNNx  goto  notNN

go_on_x goto  go_on

params  btfss Datmode,2   ;only in setup mode
    bra   main2
    call  parasend
    bra   main2

name
    btfss Datmode,2   ;only in setup mode
    bra   main2
    call  namesend
    bra   main2
      
doQnn
    movf  NN_temph,w    ;respond if NN is not zero
    addwf NN_templ,w
    btfss STATUS,Z
    call  whoami
    bra   main2

short clrf  ev0
    clrf  ev1
    bra   go_on
        
setNVx  goto  setNV
readNVx goto  readNV
readENx goto  readEN
unsetx  goto  unset


    
;********************************************************************
                ;main packet handling is here
                ;add more commands for incoming frames as needed
    
packet  movlw CMD_ON      ;only ON, OFF and request events supported
    subwf ev_opc,W  
    bz    go_on_x
    movlw CMD_OFF
    subwf ev_opc,W
    bz    go_on_x
    movlw CMD_REQ
    subwf ev_opc,W
    bz    go_on_x
    movlw SCMD_ON
    subwf ev_opc,W
    bz    short
    movlw SCMD_OFF
    subwf ev_opc,W
    bz    short
    movlw SCMD_REQ
    subwf ev_opc,W
    bz    go_on_x
    
    movlw 0x5C      ;reboot
    subwf ev_opc,W
    bz    reboot
    movlw 0x73
    subwf ev_opc,W
    bz    para1a      ;read individual parameters
    btfss Mode,1      ;FLiM?
    bra   main2
    movlw 0x42      ;set NN on 0x42
    subwf ev_opc,W
    bz    setNN
    movlw 0x0d      ; QNN
    subwf ev_opc,w
    bz    doQnn
    movlw 0x10      
    subwf ev_opc,W
    bz    params      ;read node parameters
    movlw 0x11
    subwf ev_opc,w
    bz    name      ;read module name
    
    movlw 0x53      ;set to learn mode on 0x53
    subwf ev_opc,W
    bz    setlrn1   
    movlw 0x54      ;clear learn mode on 0x54
    subwf ev_opc,W
    bz    notlrn1
    movlw 0x55      ;clear all events on 0x55
    subwf ev_opc,W
    bz    clrens1
    movlw 0x56      ;read number of events left
    subwf ev_opc,W
    bz    rden
    movlw 0x71      ;read NVs
    subwf ev_opc,W
    bz    readNVx
    movlw 0x96      ;set NV
    subwf ev_opc,W
    bz    setNVx
    movlw 0xD2      ;is it set event?
    subwf ev_opc,W
    bz    chklrn1     ;do learn
    movlw 0x95      ;is it unset event
    subwf ev_opc,W      
    bz    unsetx
    movlw 0xB2      ;read event variables
    subwf ev_opc,W
    bz    readEV
  
    movlw 0x57      ;is it read events
    subwf ev_opc,W
    bz    readENx
    movlw 0x72
    subwf ev_opc,W
    bz    rdENi_1     ;read event by index
    movlw 0x58
    subwf ev_opc,W
    bz    evns
    movlw 0x9C        ;read event variables by EN#
    subwf ev_opc,W
    bz    reval
    movlw 0x5D
    subwf ev_opc,W
    bz    enum1
    movlw 0x75      ;force new CAN_ID
    subwf ev_opc,W
    bz    newID1
    bra   main2

enum1 goto  enum
newID1  goto  newID
clrens1 goto  clrens
notlrn1 goto  notlrn
chklrn1 goto  chklrn
rdENi_1 goto  readENi
setlrn1 goto  setlrn

  
evns  goto  evns1
    bra   main2
    
reboot  btfss Mode,1      ;FLiM?
    bra   reboots
    call  thisNN
    sublw 0
    bnz   notNN
    
reboot1 movlw 0xFF
    movwf EEADR
    movlw 0xFF
    call  eewrite     ;set last EEPROM byte to 0xFF
    reset         ;software reset to bootloader

reboots
    movf  ev0,W
    addwf ev1,W
    bnz   notNN
    bra   reboot1 
  
para1a  btfss Mode, 1
    bra   para1s
    call  thisNN      ;read parameter by index
    sublw 0
    bnz   notNN
    call  para1rd
    bra   main2
    
para1s
    movf  ev0,W
    addwf ev1,W
    bnz   notNN
    call  para1rd
    bra   main2
      
main2 bcf   RXB0CON,RXFUL
    bcf   Datmode,0
    goto  main      ;loop
    
setNN btfss Datmode,2   ;in NN set mode?
    bra   main2     ;no
    call  putNN     ;put in NN
    bcf   Datmode,2
    bsf   Datmode,3
    movlw .10
    movwf Keepcnt     ;for keep alive
    movlw 0x52
    call  nnrel     ;confirm NN set
    bsf   LED_PORT,LED1 ;LED ON
    bcf   LED_PORT,LED2
    bra   main2

newID call  thisNN
    sublw 0
    bnz   notNN
    movff ev3,IDcount

    call  here2       ;put in as if it was enumerated
    movlw 0x52
    call  nnrel       ;acknowledge new CAN_ID
    goto  main2
    
sendNN  btfss Datmode,2   ;in NN set mode?
    bra   main2     ;no
    movlw 0x50      ;send back NN
    movwf Tx1d0
    movlw 3
    movwf Dlc
    call  sendTX
    bra   main2

rden  goto  rden1
  
setlrn  call  thisNN
    sublw 0
    bnz   notNN
    bsf   Datmode,4
    bsf   LED_PORT,LED1     ;LED on
    bra   main2

notlrn  call  thisNN
    sublw 0
    bnz   notNN
    bcf   Datmode,4
notln1    ;leave in learn mode
    bcf   Datmode,5

    bra   main2
clrens  call  thisNN
    sublw 0
    bnz   notNN
    btfss Datmode,4
    bra   clrerr
    call  initevdata  ;clear flash
    call  clreepr   ;clear EEPROM of EVs
    call  evcopy    ;refresh RAM (all to 00)
    movlw 0x59
    call  nnrel   ;send WRACK
    bra   notln1
    
notNN bra   main2

clrerr  movlw 2     ;not in learn mode
    goto  errmsg

    
chklrn  btfss Datmode,4   ;is in learn mode?
    bra   main2     ;j if not
    call  copyev
    movf  EVidx,w     ;check EV index
    bz    noEV1
    decf  EVidx
    movlw EV_NUM
    cpfslt  EVidx
    bra   noEV1
    bra   learn2
    
noEV1
    movlw 6
    goto  errmsg

readENi call  thisNN      ;read event by index
    sublw 0
    bnz   notNN
    call  enrdi
    bra   main2

enum  call  thisNN
    sublw 0
    bnz   notNN1
    call  self_en
    movlw 0x52
    call  nnrel     ;send confirm frame
    bcf   RXB0CON,RXFUL
    movlw B'00001000'   ;back to normal running
    movwf Datmode
    goto  main2
notNN1  goto  notNN
  


go_on call  copyev
    btfss Mode,1      ;FLiM?
    bra   go_on_s
    
go_on1  call  enmatch
    sublw 0
    bz    do_it
    bra   main2     ;not here

go_on_s btfss PORTA,LEARN
    bra   learn2      ;is in learn mode
    bra   go_on1

paraerr movlw 3       ;error not in setup mode
    goto  errmsg

setNV call  thisNN
    sublw 0
    bnz   notNN     ;not this node
    call  putNV
    call  evcopy      ;update NV copy in RAM
    bra   main2

readNV  call  thisNN
    sublw 0
    bnz   notNN     ;not this node
    call  getNV
    bra   main2

readEN  call  thisNN
    sublw 0
    bnz   notNN
    call  enread
    bra   main2
    
do_it
    call  rdfbev
    movff POSTINC0, EVtemp
    movff POSTINC0, EVtemp2
    movff POSTINC0, EVtemp3
    call  ev_set      ;do it -  for consumer action
    
    bra   main2
      
rden1 call  thisNN
    sublw 0
    bnz   notNN
    call  rdFreeSp
    bra   main2   
    
learn1
    bra   learn2
    
learn2  call  enmatch     ;is it there already?
    sublw   0
    bz    isthere
    btfsc Mode,1      ;FLiM?
    bra   learn3
    btfss PORTA,UNLEARN ;if unset and not here
    bra   l_out2      ;do nothing else 
    call  learnin     ;put EN into stack and RAM
    sublw 0
    bz    lrnend
    movlw 4
    goto  errmsg1     ;too many
    
    ;here if FLiM
learn3  btfsc Datmode,6   ;read EV?
    bra   rdbak1      ;not here
    btfsc Datmode,5   ;if unset and not here
    bra   l_out1      ;do nothing else 
    
learn4  movlw 2
    subwf EVidx,W
    bnz   learn5
    call  dn_teach

learn5  call  learnin     ;put EN into stack and RAM
    sublw 0
    bz    lrnend

    movlw 4
    goto  errmsg2 
    
rdbak1  movlw 5       ;no match
    goto  errmsg2
    
lrnend  ;movlw  0x59
    ;call nnrel
    bra   l_out2
                
isthere
    btfsc Mode,1
    bra   isthf     ;j if FLiM mode
    btfsc PORTA,UNLEARN ;is it here and unlearn...
    bra   dolrn
    call  unlearn     ;...goto unlearn  
    bra   l_out1
      
isthf
    btfsc Datmode, 6    ;is it read back
    bra   rdbak
    btfss Datmode,5   ;FLiM unlearn?
    bra   dolrn
    call  rdfbev      ;is unlearn
    movlw 2
    movff PLUSW0,EVtemp3
    call  ev_del      ;delete EV 
    call  unlearn     ;delete event from flash
    movlw 0x59
    call  nnrel
    bra   l_out1

dolrn movlw 2
    subwf EVidx,W
    bnz   do_lrn1
    call  dn_teach
do_lrn1 call  learnin
  
    bra   lrnend
    
rdbak
    call  rdfbev
    movff EVidx,Tx1d5   ;Index for readout  
    incf  Tx1d5,F     ;add one back
    movf  EVidx,w
    movff PLUSW0,Tx1d6
    movlw 0xD3        ;readback of EVs
    movwf Tx1d0
    movff ev0,Tx1d1
    movff ev1,Tx1d2
    movff ev2,Tx1d3
    movff ev3,Tx1d4
    movlw 7
    movwf Dlc
    call  sendTXa 
    bra   l_out1

l_out bcf   Datmode,4

l_out1  bcf   Datmode,6
l_out2  bcf   RXB0CON,RXFUL
    bcf   Datmode,0
    
    clrf  PCLATH
    goto  main2

noEV  movlw 6       ;invalid EV#
    goto  errmsg2

    
;***************************************************************************
;   main setup routine
;*************************************************************************

setup lfsr  FSR0, 0     ; clear 128 bytes of ram
nextram clrf  POSTINC0
    btfss FSR0L, 7
    bra   nextram 
    
    clrf  INTCON      ;no interrupts yet
    clrf  ADCON0      ;turn off A/D, all digital I/O
    movlw B'00001111'
    movwf ADCON1
    
    ;port settings will be hardware dependent. RB2 and RB3 are for CAN.
    ;set S_PORT and S_BIT to correspond to port used for setup.
    ;rest are hardware options
    
  
    movlw B'00000111'   ;Port A  PA0 and PA1 inputs for SLiM compatibility. PA2 is setup PB
    movwf TRISA     ;
    movlw B'00111011'   ;RB2 = CANTX, RB3 = CANRX,  
                ;RB6,7 for debug and ICSP and LEDs
                ;PORTB has pullups enabled on inputs
    movwf TRISB
    bcf   LED_PORT,LED2
    bcf   LED_PORT,LED1
    bsf   PORTB,2     ;CAN recessive
    movlw B'00000000'   ;Port C  set to outputs.
    movwf TRISC
    clrf  PORTC
  
    
; next segment is essential.
    
    bsf   RCON,IPEN   ;enable interrupt priority levels
    clrf  BSR       ;set to bank 0
    clrf  EECON1      ;no accesses to program memory  
    clrf  Datmode
    clrf  Latcount

     
    bsf   CANCON,7    ;CAN to config mode
    movlw B'10110000'
    movwf ECANCON 
    bsf   ECANCON,5   ;CAN mode 2 
    movf  ECANCON,W
    movwf TempECAN 

    movlb .15       ;change bank
    clrf  BSEL0     ;8 frame FIFO
    clrf  RXB0CON
    clrf  RXB1CON
    clrf  B0CON
    clrf  B1CON
    clrf  B2CON
    clrf  B3CON
    clrf  B4CON
    clrf  B5CON
    movlb 0
    movlw B'00000011'   ;set CAN bit rate at 125000 for now
    movwf BRGCON1
    movlw B'10011110'   ;set phase 1 etc
    movwf BRGCON2
    movlw B'00000011'   ;set phase 2 etc
    movwf BRGCON3
    movlw B'00100000'
    movwf CIOCON      ;CAN to high when off
    movlw B'00100100'   ;B'00100100'
    movwf RXB0CON     ;enable double buffer of RX0

    


    
mskload lfsr  0,RXM0SIDH    ;Clear masks, point to start
mskloop clrf  POSTINC0    
    movlw LOW RXM1EIDL+1    ;end of masks
    cpfseq  FSR0L
    bra   mskloop
    
    clrf  CANCON      ;out of CAN setup mode
    clrf  CCP1CON
    movlw B'10000100'
    movwf T0CON     ;set Timer 0 for LED flash
    
    movlw B'10100001'   ;Timer 1 control.16 bit write, 1MHz tick
    movwf T1CON     ;Timer 1 is for output duration
    movlw 0xB1
    movwf TMR1H     ;set timer hi byte
    movlw 0xE0
    movwf TMR1L     ;set for 20ms tick
        
    clrf  Tx1con
    movlw B'00100011'
    movwf IPR3      ;high priority CAN RX and Tx error interrupts(for now)
    clrf  IPR1      ;all peripheral interrupts are low priority
    clrf  IPR2
    clrf  PIE2
    movlw B'00000001'
    movwf PIE1      ;enable interrupt for timer 1


;next segment required
    
    movlw B'00000001'
    movwf IDcount     ;set at lowest value for starters
    
    clrf  INTCON2     ;
    clrf  INTCON3     ;
    

    movlw B'00100000'   ;B'00100011'  Rx0 and RX1 interrupt and Tx error
                
    movwf PIE3
  
    clrf  PIR1
    clrf  PIR2

    bcf   RXB0CON,RXFUL   ;ready for next
    bcf   COMSTAT,RXB0OVFL  ;clear overflow flags if set
    bcf   COMSTAT,RXB1OVFL
    clrf  PIR3      ;clear all flags
  
    call  copyEVs     ;set up flash ram if not already done
    clrf  Mode      ;test for setup mode
    movlw Modstat     ;get setup status
    movwf EEADR
    call  eeread
    movwf Datmode
    sublw 0       ;not set yet
    bnz   setid
    bra   slimset     ;wait for setup PB
  
    
setid bsf   Mode,1      ;flag FLiM
    call  newid_f     ;put ID into Tx1buf, TXB2 and ID number store
    
seten_f call  evcopy      ;initialise EVs etc to RAM
    movlw 0x90      ;startup event is ON
    movwf ev_opc
    movff Nv3,EVtemp    ;which outputs to move?
    movff Nv2,EVtemp2
    comf  Nv2,W     ;off or last?
    andwf Last,W
    comf  WREG
    iorwf EVtemp2
    clrf  EVtemp3     ;no feedback
    call  ev_set      ;set outputs
    movlw B'11000000'
    movwf INTCON      ;enable interrupts
    bcf   LED_PORT,LED2
    bsf   LED_PORT,LED1     ;Yellow LED on.

    bcf   Datmode,0
    goto  main

slimset bcf   Mode,1
    clrf  NN_temph
    clrf  NN_templ
    ;test for clear all events
    btfss PORTA,LEARN   ;ignore the clear if learn is set
    goto  seten
    btfss PORTA,UNLEARN
    call  initevdata      ;clear all events if unlearn is set during power up
seten call  evcopy      ;initialise EVs etc to RAM
    movlw B'11000000'
    movwf INTCON      ;enable interrupts
    bcf   PORTB,6
    bsf   PORTB,7     ;RUN LED on. Green for SLiM
    goto  main




    
;****************************************************************************
;   start of subroutines

;   Do an event.  arrives with EVs in EVtemp and EVtemp2, Feedback EV in EVtemp3
;   Updates output state in EEPROM as well.

ev_set  ;movff  ev_opc, EVtemp3
    btfsc ev_opc,1    ;is it a request? bit set if yes
    bra   poll
    movlw B'00000011'
    andwf EVtemp3,W
    xorlw 3       ;is a SoD?
    bz    sod       ;yes
    movff EVtemp,EVtemp1
    movff EVtemp2, OffBits
    comf  EVtemp1,W
    andwf PORTC,W     ;mask unaffected outputs
    movwf EVtemp1     ;unaffected outputs
    movf  EVtemp,W
    xorwf EVtemp2,F   ;change polarity if needed
    btfss ev_opc,0      ;on or off?
    bra   ev_on
    movf  EVtemp,W
    xorwf EVtemp2,W
    iorwf EVtemp1,W
    movwf OpBits
    bra   doNvs
          
ev_on movf  EVtemp,W
    andwf EVtemp2,W
    iorwf EVtemp1,W
    movwf OpBits
    
doNvs movlw LOW NVnow   ; get current state
    movwf EEADR
    call  eeread
    movwf Last
    comf  EVtemp, W   ;mask bits
    andwf Last
    movf  OpBits,W
    iorwf Last,W
    movwf Last      ;save for feedback
    call  eewrite     ;write back to EEPROM
    movlw 1
    movwf Rollfb      ;set rolling bit for feedback events
    clrf  Op_fb     ;output index
    movf  Tmr3h,W     ;get feedback delay time
    sublw 0xFF
    movwf TMR3H
    clrf  TMR3L
  

    
    bcf   PIE1, 0     ; inhibit timer1 interupts
    nop
    clrf  OpNum
    movlw 1
    movwf Roll
    lfsr  FSR0, T1
    lfsr  FSR1, T1Copy
    comf  OffBits,w
    andwf EVtemp,w
    movwf OnBits
nxtop
    movf  Roll,w
    andwf EVtemp,w
    bz    no_op     ;j if bit not affected
    movlw LOW NVstart
    addwf OpNum,w
    movwf EEADR
    call  eeread      ;get NV
    movwf EVtemp1
    bcf   WREG,7
    tstfsz  WREG
    bra   pulse
    bra   nopulse
pulse             ;pulse action
    incf  WREG
    movwf Temp
    btfss EVtemp1,7   ;test repeat pulse flag
    bra   onepulse    ;j if not repeat
    btfsc ev_opc,0    ;chk for on command
    bra   stop_p      ;j if off event, turn off pulsing
    movf  OpNum,w
    movff Temp, PLUSW0  ; set up for repeat pulses
    movff Temp, PLUSW1
    bra   no_op   

onepulse  
    btfsc ev_opc,0
    bra   off_event
    movf  Roll,w
    andwf OffBits,w
    bnz   nopulse
    bra   do_on
off_event
    movf  Roll,w
    andwf OffBits,w
    bz    nopulse
do_on 
    movf  OpNum,w
    movff Temp, PLUSW0
    clrf  PLUSW1
    bra   no_op
    
stop_p  comf  Roll, w
    andwf OpBits  
nopulse             ;normal action so no timers
    movf  OpNum,w
    clrf  PLUSW0
    clrf  PLUSW1
no_op
    incf  OpNum
    rlcf  Roll
    bnc   nxtop 
    comf  EVtemp, w
    andwf PORTC
    movf  OpBits,w
    iorwf PORTC
    bsf   PIE1,0      ; allow timer 1 interupts
    bsf   PIE2,TMR3IE   ;enable interrupts on timer 3
    bsf   T3CON,TMR3ON  ;start feedback timer
    return  

sod   movlw 1
    movwf Rollfb      ;set rolling bit for feedback events
    clrf  Op_fb     ;output index
    movf  Tmr3h,W     ;get feedback delay time
    sublw 0xFF
    movwf TMR3H
    clrf  TMR3L
    setf  EVtemp      ;all outputs
    bsf   PIE2,TMR3IE   ;enable interrupts on timer 3
    bsf   T3CON,TMR3ON  ;start feedback timer
    return    

poll  movff FSR0L,Fsr_fb0L  ;in case a poll during a flashing
    movff FSR0H,Fsr_fb0H
    movff FSR1L,Fsr_fb1L
    movff FSR1H,Fsr_fb1H
    btfss EVtemp3,7   ;is it a feedback event? can't poll if not
    return
    movlw B'00011100'
    andwf EVtemp3,W   ;mask all but OP number
    movwf EVtempp     ;save count
    movlw 4
    movwf Countfb     ;transfer 4 bytes of event
    lfsr  FSR1,Tx1d1
    lfsr  FSR0,ev0
    
poll1 movff POSTINC0,POSTINC1
    decfsz  Countfb
    bra   poll1
    movf  EVtempp,W
    rrncf WREG
    rrncf WREG      ;get OP number
    movwf Countfb
    incf  Countfb     ;add 1
    clrf  Rollfb
    bsf   Rollfb,0    ;rolling bit
poll2 decfsz  Countfb     ;this OP ?
    bra   poll3
    bra   poll4
poll3 rlncf Rollfb
    bra   poll2
poll4 movf  Rollfb,W    ;get bit
    andwf Last,W      ;is this OP on or off?
    bnz   poll_on
    btfsc ev_opc,3    ;is it a short?
    bra   s_off
    movlw 0x94      ;long off
    movwf Tx1d0
    bra   poll_ol     ;output a response
s_off movlw 0x9E
    movwf Tx1d0
    bra   poll_os   ;output a short 
poll_on btfsc ev_opc,3    ;is it a short?
    bra   s_on
    movlw 0x93      ;long off
    movwf Tx1d0
    bra   poll_ol     ;output a response
s_on  movlw 0x9D
    movwf Tx1d0
poll_os movlw 5
    movwf Dlc
    call  sendTX
    bra   polbak
poll_ol movlw 5
    movwf Dlc
    call  sendTXa
polbak  movff Fsr_fb0L,FSR0L  ;in case a poll during a flashing
    movff Fsr_fb0H,FSR0H
    movff Fsr_fb1L,FSR1L
    movff Fsr_fb1H,FSR1H
  
    return    


;   Send contents of Tx1 buffer via CAN TXB1

sendTX1 lfsr  FSR0,Tx1con
    lfsr  FSR1,TXB1CON
    
    movlb .15       ;check for buffer access
ldTX2 btfsc TXB1CON,TXREQ ; Tx buffer available...?
    bra   ldTX2     ;... not yet
    movlb 0
    
ldTX1 movf  POSTINC0,W
    movwf POSTINC1  ;load TXB1
    movlw Tx1d7+1
    cpfseq  FSR0L
    bra   ldTX1

    
    movlb .15       ;bank 15
tx1test btfsc TXB1CON,TXREQ ;test if clear to send
    bra   tx1test
    bsf   TXB1CON,TXREQ ;OK so send
    
tx1done movlb 0       ;bank 0
    return          ;successful send

    
;*********************************************************************
;   put in NN from command

putNN movff ev0,NN_temph
    movff ev1,NN_templ
    movlw LOW NodeID
    movwf EEADR
    movf  ev0,W
    call  eewrite
    incf  EEADR
    movf  ev1,W
    call  eewrite
    movlw Modstat
    movwf EEADR
    movlw B'00001000'   ;Module status has NN set
    call  eewrite
    return

;***************************************************************************  

newid_f movlw LOW CANid     ;put in stored ID. FLiM mode
    movwf EEADR
    bsf   EECON1,RD
    movf  EEDATA,W
    movwf CanID_tmp     
    call  shuffle
    movlw B'11110000'
    andwf Tx1sidh
    movf  IDtemph,W   ;set current ID into CAN buffer
    iorwf Tx1sidh     ;leave priority bits alone
    movf  IDtempl,W
    movwf Tx1sidl     ;only top three bits used
    movlw LOW NodeID
    movwf EEADR
    call  eeread
    movwf NN_temph      ;get stored NN
    incf  EEADR
    call  eeread
    movwf NN_templ  
    
    movlb .15       ;put ID into TXB2 for enumeration response to RTR
new_1 btfsc TXB2CON,TXREQ
    bra   new_1
    clrf  TXB2SIDH
    movf  IDtemph,W
    movwf TXB2SIDH
    movf  IDtempl,W
    movwf TXB2SIDL
    movlw 0xB0
    iorwf TXB2SIDH    ;set priority
    clrf  TXB2DLC     ;no data, no RTR
    movlb 0
  
    return

;*********************************************************************    


    
nnack movlw 0x50      ;request frame for new NN or ack if not virgin
nnrel movwf Tx1d0
    movff NN_temph,Tx1d1
    movff NN_templ,Tx1d2
    movlw 3
    movwf Dlc
    call  sendTX
    return



    
;*****************************************************************************
;
;   shuffle for standard ID. Puts 7 bit ID into IDtemph and IDtempl for CAN frame
shuffle movff CanID_tmp,IDtempl   ;get 7 bit ID
    swapf IDtempl,F
    rlncf IDtempl,W
    andlw B'11100000'
    movwf IDtempl         ;has sidl
    movff CanID_tmp,IDtemph
    rrncf IDtemph,F
    rrncf IDtemph,F
    rrncf IDtemph,W
    andlw B'00001111'
    movwf IDtemph         ;has sidh
    return

;*********************************************************************************

;   reverse shuffle for incoming ID. sidh and sidl into one byte.

shuffin movff RXB0SIDL,IDtempl
    swapf IDtempl,F
    rrncf IDtempl,W
    andlw B'00000111'
    movwf IDtempl
    movff RXB0SIDH,IDtemph
    rlncf IDtemph,F
    rlncf IDtemph,F
    rlncf IDtemph,W
    andlw B'01111000'
    iorwf IDtempl,W     ;returns with ID in W
    return
;************************************************************************************
;   
eeread  bcf   EECON1,EEPGD  ;read a EEPROM byte, EEADR must be set before this sub.
    bcf   EECON1,CFGS   ;returns with data in W
    bsf   EECON1,RD
    movf  EEDATA,W
    return

;**************************************************************************
eewrite movwf EEDATA      ;write to EEPROM, EEADR must be set before this sub.
    bcf   EECON1,EEPGD  ;data to write in W
    bcf   EECON1,CFGS
    bsf   EECON1,WREN
    movff INTCON,TempINTCON
    
    clrf  INTCON  ;disable interrupts
    movlw 0x55
    movwf EECON2
    movlw 0xAA
    movwf EECON2
    bsf   EECON1,WR
eetest  btfsc EECON1,WR
    bra   eetest
    bcf   PIR2,EEIF
    bcf   EECON1,WREN
    movff TempINTCON,INTCON   ;reenable interrupts
    
    return  
    
;***************************************************************


;*********************************************************************
;   send a CAN frame
;   entry at sendTX puts the current NN in the frame - for producer events
;   entry at sendTXa neeeds Tx1d1 and Tx1d2 setting first
;   Latcount is the number of CAN send retries before priority is increased
;   the CAN-ID is pre-loaded in the Tx1 buffer 
;   Dlc must be loaded by calling source to the data length value
    
sendTX  movff NN_temph,Tx1d1
    movff NN_templ,Tx1d2

sendTXa movf  Dlc,W       ;get data length
    movwf Tx1dlc
    movlw B'00001111'   ;clear old priority
    andwf Tx1sidh,F
    movlw B'10110000'
    iorwf Tx1sidh     ;low priority
    movlw .10
    movwf Latcount
    call  sendTX1     ;send frame
    return      

;**************************************************************************

;   check if command is for this node

thisNN  movf  NN_temph,W
    subwf ev0,W
    bnz   not_NN
    movf  NN_templ,W
    subwf ev1,W
    bnz   not_NN
    retlw   0     ;returns 0 if match
not_NN  retlw 1
              
;**********************************************************************
;   loads ENs from EEPROM to RAM for fast access
;   shifts all 32 even if less are used

en_ram  movlw OLD_EN_NUM
    movwf Count     ;number of ENs allowed 
    
    bcf   STATUS,C    ;clear carry
    rlncf Count,F     ;double it
    rlncf Count,F     ;double again
    lfsr  FSR0,EN1    ;set FSR0 to start of ram buffer
    movlw LOW ENstart     ;load ENs from EEPROM to RAM
    movwf EEADR
enload  bsf   EECON1,RD   ;get first byte
    movf  EEDATA,W
    movwf POSTINC0
    incf  EEADR
    decfsz  Count,F
    bra   enload
    
ev_ram  movlw OLD_EN_NUM    ;now copy original EVs to RAM
    movwf Count     ;number of ENs allowed 
    bcf   STATUS,C
    rlncf Count     ; 2 EVs per event
    lfsr  FSR0, EV1
    movlw LOW EVstart
    movwf EEADR
ev_load
    bsf   EECON1,RD   ;get first byte
    movf  EEDATA,W
    movwf POSTINC0
    incf  EEADR
    decfsz  Count,F
    bra   ev_load
    
    return  
    
    
;   clears all stored events

enclear movlw OLD_EN_NUM * 6 + 2    ;number of locations in EEPROM
    movwf Count
    movlw LOW ENindex
    movwf EEADR
enloop  movlw 0
    call  eewrite
    incf  EEADR
    decfsz  Count
    bra   enloop
    ;now clear the ram
    movlw OLD_EN_NUM * 4
    movwf Count
    lfsr  FSR0, EN1
ramloop clrf  POSTINC0
    decfsz  Count
    bra   ramloop
    return  
    
;************************************************************

getop movlw B'00010011'   ;get DIP switch setting for output
    andwf PORTB,W
    movwf Temp
    movwf Temp1
    rrncf Temp1,F
    rrncf Temp1,W
    andlw B'00000100'
    iorwf Temp,W
    
    andlw B'00000111'   ;mask
    movwf Temp
    movlw 1
    movwf EVtemp
getop1  movf  Temp,F      ;is it zero?
    bz    getop2
    rlncf EVtemp,F    ;put rolling bit into EVtemp
    decf  Temp,F
    bra   getop1
getop2  return


#include "cbuslib/evhndlr.asm"


;**************************************************************************
;   send node parameter bytes (7 maximum)

parasend  
    movlw 0xEF
    movwf Tx1d0
    movlw LOW nodeprm
    movwf TBLPTRL
    movlw 8
    movwf TBLPTRH   ;relocated code
    lfsr  FSR0,Tx1d1
    movlw 7
    movwf Count
    bsf   EECON1,EEPGD
    
para1 tblrd*+
    movff TABLAT,POSTINC0
    decfsz  Count
    bra   para1
    bcf   EECON1,EEPGD  
    movlw 8
    movwf Dlc
    call  sendTXa
    return

;**************************************************************************
;   send module name - 7 bytes

namesend  
    movlw 0xE2
    movwf Tx1d0
    movlw LOW myName
    movwf TBLPTRL
    movlw HIGH myName
    movwf TBLPTRH   ;relocated code
    lfsr  FSR0,Tx1d1
    movlw 7
    movwf Count
    bsf   EECON1,EEPGD
    
name1 tblrd*+
    movff TABLAT,POSTINC0
    decfsz  Count
    bra   name1
    bcf   EECON1,EEPGD  
    movlw 8
    movwf Dlc
    call  sendTXa
    return
    


;**********************************************************

;   send individual parameter

;   Index 0 sends no of parameters

para1rd movf  ev2,w
    sublw 0
    bz    numParams
    movlw PRMCOUNT
    movff ev2, Temp
    decf  Temp
    cpfslt  Temp
    bra   pidxerr
    movlw 0x9B
    movwf Tx1d0
    movlw 7   ;FLAGS index in nodeprm
    cpfseq  Temp
    bra   notFlags      
    call  getflags
    movwf Tx1d4
    bra   addflags
notFlags  
    movlw .14
    cpfseq  Temp
    bra   nxtparam
    call  getId1
    movwf Tx1d4
    bra   addflags
nxtparam
    movlw .15
    cpfseq  Temp
    bra   paramrd
    call  getId2
    movwf Tx1d4
    bra   addflags
paramrd
    movlw LOW nodeprm
    movwf TBLPTRL
    movlw HIGH nodeprm
    movwf TBLPTRH   ;relocated code
    clrf  TBLPTRU
    decf  ev2,W
    addwf TBLPTRL
    bsf   EECON1,EEPGD
    tblrd*
    movff TABLAT,Tx1d4
addflags            
    movff ev2,Tx1d3
    movlw 5
    movwf Dlc
    call  sendTX
    return  
  
numParams
    movlw 0x9B
    movwf Tx1d0
    movlw PRMCOUNT
    movwf Tx1d4
    movff ev2,Tx1d3
    movlw 5
    movwf Dlc
    call  sendTX
    return
    
pidxerr
    movlw .10
    call  errsub
    return
    
getflags    ; create flags byte
    movlw PF_CONSUMER
    btfsc Mode,1
    iorlw 4   ; set bit 2
    movwf Temp
    bsf   Temp,3    ;set bit 3, we are bootable
    movf  Temp,w
    return
    
;**********************************************************

; returns Node Number, Manufacturer Id, Module Id and Flags

whoami
    call  ldely   ;wait for other nodes
    movlw OPC_PNN
    movwf Tx1d0
    movlw MAN_NO    ;Manufacturer Id
    movwf Tx1d3
    movlw MODULE_ID   ; Module Id
    movwf Tx1d4
    call  getflags
    movwf Tx1d5
    movlw 6
    movwf Dlc
    call  sendTX
    return
      
;***********************************************************
;
;getDevId returnd DEVID2 and DEVID1 in PRODH and PRODL

getId1
  call  getProdId
  movf  PRODL,w
  return
  
getId2
  call  getProdId
  movf  PRODH,w
  return

getProdId

  movlw 0x3F
  movwf TBLPTRU
  movlw 0xFF
  movwf TBLPTRH
  movlw 0xFE
  movwf TBLPTRL
  bsf   EECON1, EEPGD
  tblrd*+
  movff TABLAT, PRODL
  tblrd*
  movff TABLAT, PRODH
  return
  
;***********************************************************

; error message send

errmsg  call  errsub
    goto  main2 
errmsg1 call  errsub
    goto  l_out2
errmsg2 call  errsub
    goto  l_out1

errsub  movwf Tx1d3   ;main eror message send. Error no. in WREG
    movlw 0x6F
    movwf Tx1d0
    movlw 4
    movwf Dlc
    call  sendTX
    return

;*********************************************************
;   a delay routine
      
dely  movlw .10
    movwf Count1
dely2 clrf  Count
dely1 decfsz  Count,F
    goto  dely1
    decfsz  Count1
    bra   dely2
    return    
    
;****************************************************************

;   longer delay

ldely movlw .100
    movwf Count2
ldely1  call  dely
    decfsz  Count2
    bra   ldely1
    
    return

;**************************************************************************

putNV movlw NV_NUM + 1    ;put new NV in EEPROM and the NV ram.
    cpfslt  ev2
    return
    movf  ev2,W
    bz    no_NV
    decf  WREG      ;NVI starts at 1
    addlw LOW NVstart
    movwf EEADR
    movf  ev3,W
    call  eewrite 
        
    movlw 0x59
    call  nnrel     ; send WRACK
  

    
no_NV return

;************************************************************************

getNV movlw NV_NUM + 1    ;get NV from EEPROM and send.
    cpfslt  ev2
    bz    no_NV1
    movf  ev2,W
    bz    no_NV1
    decf  WREG      ;NVI starts at 1
    addlw LOW NVstart
    movwf EEADR
    call  eeread
    movwf Tx1d4     ;NV value
getNV1  movff ev2,Tx1d3   ;NV index
getNV2  movff ev0,Tx1d1
    movff ev1,Tx1d2
    movlw 0x97      ;NV answer
    movwf Tx1d0
    movlw 5
    movwf Dlc
    call  sendTXa
    return

no_NV1  clrf  Tx1d3     ;if not valid NV
    clrf  Tx1d4
    bra   getNV2

nv_rest movlw 8
    movwf Count
    movlw LOW NVstart
    movwf EEADR
nv_rest1 
    movlw 0
    call  eewrite
    incf  EEADR
    decfsz  Count
    bra   nv_rest1
    
    return


;**********************************************************************

;   self enumeration as separate subroutine

self_en movff FSR1L,Fsr_tmp1Le  ;save FSR1 just in case
    movff FSR1H,Fsr_tmp1He 
;   movlw B'11000000'
;   movwf INTCON      ;start interrupts if not already started
    bsf   Datmode,1   ;set to 'setup' mode
    movlw .14
    movwf Count
    lfsr  FSR0, Enum0
clr_en
    clrf  POSTINC0
    decfsz  Count
    bra   clr_en
    bcf   PIE2,TMR3IE   ;disable interrupts for self-en
    movlw 0x3C      ;set T3 to 1 mSec (may need more?)
    movwf TMR3H
    movlw 0xAF
    movwf TMR3L
    movlw B'10110001'
    movwf T3CON     ;enable timer 3
    bcf   PIR2,TMR3IF
    
    movlb .15
    movlw B'10111111'   ;fixed node, default ID  
    movwf TXB1SIDH
    movlw B'11100000'
    movwf TXB1SIDL
    movlw B'01000000'   ;RTR frame
    movwf TXB1DLC
rtr_snd btfsc TXB1CON,TXREQ
    bra   rtr_snd
    bsf   TXB1CON,TXREQ
rtr_go  btfsc TXB1CON,TXREQ   ;wait till sent
    bra   rtr_go
    clrf  TXB1DLC       ;no more RTR frames
    movlb 0
    
  

  
    
  

self_en1  btfsc PIR2,TMR3IF   ;setup timer out?
    bra   en_done
    btfsc COMSTAT,7   ;look for CAN input. 
    bra   getcan1
    bra   self_en1    ;no CAN
  

getcan1 movf  CANCON,W
    andlw B'00001111'
    movwf TempCANCON
    movf  ECANCON,W
    andlw B'11110000'
    iorwf TempCANCON,W
    movwf ECANCON
    btfsc RXB0SIDL,EXID   ;ignore extended frames here
    bra   no_can1
    
    
en_1  btfss Datmode,1     ;setup mode?
    bra   no_can1
    movf  RXB0DLC,F
    bnz   no_can1       ;only zero length frames
    call  setmode
    bra   no_can1 

no_can1 bcf   RXB0CON,RXFUL
    bra   self_en1      ;loop till timer out 

en_done bcf   T3CON,TMR3ON  ;timer off
    bcf   PIR2,TMR3IF   ;clear flag


    clrf  IDcount
    incf  IDcount,F     ;ID starts at 1
    clrf  Roll
    bsf   Roll,0
    lfsr  FSR1,Enum0      ;set FSR to start
here1 incf  INDF1,W       ;find a space
    bnz   here
    movlw 8
    addwf IDcount,F
    incf  FSR1L
    bra   here1
here  movf  Roll,W
    andwf INDF1,W
    bz    here2
    rlcf  Roll,F
    incf  IDcount,F
    bra   here
here2 movlw .100        ;limit to ID
    cpfslt  IDcount
    bra   segful        ;segment full
    
here3 movlw LOW CANid   ;put new ID in EEPROM
    movwf EEADR
    movf  IDcount,W
    call  eewrite
    movf  IDcount,W
    call  newid_f     ;put new ID in various buffers

      
    movff Fsr_tmp1Le,FSR1L  ;
    movff Fsr_tmp1He,FSR1H 
    return  0         

segful  movlw 7   ;segment full, no CAN_ID allocated
    call  errsub
    setf  IDcount
    bcf   IDcount,7
    bra   here3

;;****************************************************************
;
setmode tstfsz  RXB0DLC
    return        ;only zero length frames for setup
    
    swapf RXB0SIDH,W      ;get ID into one byte
    rrcf  WREG
    andlw B'01111000'     ;mask
    movwf Temp
    swapf RXB0SIDL,W
    rrncf WREG
    andlw B'00000111'
    iorwf Temp,W
    movwf IDcount       ;has current incoming CAN_ID

    lfsr  FSR1,Enum0      ;set enum to table
enum_st clrf  Roll        ;start of enum sequence
    bsf   Roll,0
    movlw 8
enum_1  cpfsgt  IDcount
    bra   enum_2
    subwf IDcount,F     ;subtract 8
    incf  FSR1L       ;next table byte
    bra   enum_1
enum_2  dcfsnz  IDcount,F
    bra   enum_3
    rlncf Roll,F
    bra   enum_2
enum_3  movf  Roll,W
    iorwf INDF1,F
    bcf   RXB0CON,RXFUL   ;clear read
    return
;*******************************************************

copyev    ; copy event data to safe buffer
    movff RXB0D0, ev_opc
    movff RXB0D1, ev0
    movff RXB0D2, ev1
    movff RXB0D3, ev2
    movff RXB0D4, ev3
    movff RXB0D5, EVidx   ; only used by learn and some read cmds
    movff RXB0D6, EVdata    ; only used by learn cmd
    movlw SCMD_ON
    subwf RXB0D0,W
    bz    short1
    movlw SCMD_OFF
    subwf RXB0D0,W
    bz    short1
    movlw SCMD_REQ
    subwf RXB0D0,W
    bz    short1
    return
short1  clrf  ev0         ;for short events, clear ev0 and ev1
    clrf  ev1
    return

;********************************************************

; teach position DNs. Takes settings in EV3, held in EVdata

dn_teach    btfss EVdata,7    ;is it a DN setting?
    return      ;no
    
    

dn_store  movff EVdata,EVtemp
    movf  EVdata,W
    andlw B'00011100' ;mask all except address
    addlw LOW ENstart
    movwf EEADR
    movf  ev0,W
    call  eewrite
    incf  EEADR
    movf  ev1,W
    call  eewrite
    incf  EEADR
    movf  ev2,W
    call  eewrite
    incf  EEADR
    movf  ev3,W
    call  eewrite

    movf  EVtemp,W
    andlw B'00011100' ;mask all except address
    rrncf WREG
    rrncf WREG
;   andlw B'00000111' ;mask  bits. 8 EVs
    addlw LOW EVstart ;now put in EVs 
    
    movwf EEADR   ;set address in EEPROM
    movf  EVtemp,W
    call  eewrite   ;write EV value
    movff EVtemp,EVdata   ;restore EVdata
    call  evcopy    ;update RAM copy
    return

;**********************************************************



; copy EV3s from EEPROM to RAM and Timer value

evcopy  movlw 8
    movwf Count
    movlw LOW EVstart
    movwf EEADR
    lfsr  FSR1,E1
evcopy1 call  eeread
    movwf POSTINC1
    incf  EEADR
    decfsz  Count
    bra   evcopy1
    movlw LOW NVstart+8
    movwf EEADR
    movlw 4
    movwf Count
    lfsr  FSR1,Tmr3h
evcopy2 call  eeread
    movwf POSTINC1
    incf  EEADR
    decfsz  Count
    bra   evcopy2
    movlw LOW NVnow
    movwf EEADR
    call  eeread
    movwf Last
    return

;***************************************************************

;   send feedback events

fbev_on movlw 0x90    ;0n
    movwf Tx1d0   ;put in buffer
    bra   fbev1
fbev_off movlw  0x91    ;0ff
    movwf Tx1d0   ;put in buffer
fbev1 lfsr  FSR0,Tx1d1  ;load event into buffer
    movlw LOW ENstart
    movwf EEADR
    movf  Op_fb,W   ;get output number
    rlncf WREG
    rlncf WREG    ;4 bytes per event
    addwf EEADR   ;offset
    movlw 4
    movwf Countfb   ;counter for 4 bytes
    movlw 5
    movwf Dlc     ;for sending
fb_loop call  eeread
    movwf POSTINC0
    incf  EEADR
    decfsz  Countfb
    bra   fb_loop
    movf  Tx1d1,F ;check for short event
    bnz   f_long
    movf  Tx1d2,F
    bnz   f_long
    bsf   Tx1d0,3   ;set for short
    call  sendTX    ;add NN
    return
f_long  call  sendTXa   ;don't change NN
    return
    
;**************************************************************
;
; deletes a feedback event by clearing  EV3 in EEPROM
; arrives with EV3 in EVtemp3 

ev_del  movf  EVtemp3,W
    btfss WREG,7    ;is it a response event?
    return
    andlw B'00011100' ;mask bits
    rrncf WREG
    rrncf WREG
    movwf EVtemp
    addlw LOW EVstart ;now put in EVs 
    
    movwf EEADR   ;set address in EEPROM
    clrf  WREG    ;set to zero
    call  eewrite
    call  evcopy    ;copy to RAM
    return
  
;*************************************************************
;   clear all EEPROM EV3s if delete all events

clreepr movlw LOW EVstart   ;clear EV3 set
    movwf EEADR
    movlw 8
    movwf Count
clree2  clrf  WREG
    call  eewrite
    incf  EEADR
    decfsz  Count
    bra   clree2
    return

;******************************************************************

    
  ORG   0x3000
evdata        

;************************************************************************

;************************************************************************   
  ORG 0xF00000      ;EEPROM data. Defaults
  
CANid de  B'01111111',0 ;CAN id default and module status
NodeID  de  0,0     ;Node ID
ENindex de  0,0   ;points to next available EN number (in lo byte)
          ;free space in hi byte

  ORG 0xF00006

ENstart ;feedback events stored here. Room for 8 four byte events, one per output.

  ORG 0xF00086  ;for compatability with earlier versions
    
    ;event variables stored here. set to zero initially
    
EVstart de  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0   ;allows for 2 EVs per event

  
  ORG 0xF000C6
    
hashtab de  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
hashnum de  0,0,0,0,0,0,0,0

FreeCh  de  0,0
;must keep aligmant unchanged
spare de  0,0,0,0,0,0

  
NVstart de  0,0,0,0,0,0,0,0
    de 0x0A,0x00,0xFF,0xFF      ;4 NVs for node variables
                    ;NV9 is feedback delay. In 0.5mSec intervals approx.
                    ;NV10 startup position. Bit set is OFF end. 
                    ;bit  clear is now go to last saved position
                    ;NV11 is move on startup. Bit set is move.
                    ;NV12 not used yet




NVnow de  0x00,0            ;holds current or last positions
                    ;default is all off


    ORG 0xF000FE
    de    0,0   ;for boot load

    end


;************   

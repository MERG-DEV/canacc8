    TITLE   "Source for CAN accessory decoder using CBUS"
; filename CANACC8e.asm

; a simple 8 output consumer node for SLiM model only
; same code can be used for CANACC5

; CAN rate at 125000 for now
; Tested 24/09/07 Seems OK
; Added clear all if UNLEARN during reset
; Changed switch sequence to match CANACC4

; DIL switch sequence

; 1 Output select LSB
; 2 Output select
; 3 Output select MSB
; 4 Polarity
; 5 Learn
; 6 Unlearn /reset

; added RUN LED switch. No full diagnostics yet.
; added short event handling for 'many' producers.
; corrected polarity handling.
; increase in number of events
; LED flash when events full
; Seems OK  03/05/08
; added clear of PORTC on setup to prevent outputs being set on power up.
; modified CONFIG and LOW so no warnings
; correction to EV change sequence for already learned events. (Bug fix)  22/08/08
; corection to RXB1CON and RXF1SIDL so it rejects extended frames. 28/10/09  (Rev e)





; 
; Assembly options
  LIST  P=18F2480,r=hex,N=75,C=120,T=ON

  include   "p18f2480.inc"

; set config registers

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

;original CONFIG settings left here for reference
  
; __CONFIG  _CONFIG1H,  B'00100110' ;oscillator HS with PLL
; __CONFIG  _CONFIG2L,  B'00001110' ;brown out voltage and PWT  
; __CONFIG  _CONFIG2H,  B'00000000' ;watchdog time and enable (disabled for now)
; __CONFIG  _CONFIG3H,  B'10000000' ;MCLR enable  
; __CONFIG  _CONFIG4L,  B'10000001' ;B'10000001'  for   no debug
; __CONFIG  _CONFIG5L,  B'00001111' ;code protection (off)  
; __CONFIG  _CONFIG5H,  B'11000000' ;code protection (off)  
; __CONFIG  _CONFIG6L,  B'00001111' ;write protection (off) 
; __CONFIG  _CONFIG6H,  B'11100000' ;write protection (off) 
; __CONFIG  _CONFIG7L,  B'00001111' ;table read protection (off)  
; __CONFIG  _CONFIG7H,  B'01000000' ;boot block protection (off)



; processor uses 4 MHz resonator but clock is 16 MHz.

;**************************************************************************
;definitions

LEARN   equ 1 ;learn switch in port A
POL   equ 5 ;pol switch in port B
SETUP equ 0 ;unlearn / setup jumper in port A
CMD_ON  equ 0x90  ;on event
CMD_OFF equ 0x91  ;off event
SCMD_ON equ 0x98
SCMD_OFF  equ 0x99
EN_NUM  equ .32   ;number of allowed events



;****************************************************************
; define RAM storage
  
  CBLOCK  0   ;file registers - access bank
          ;interrupt stack for low priority
          ;hpint uses fast stack
  W_tempL
  St_tempL
  Bsr_tempL
  PCH_tempH   ;save PCH in hpint
  PCH_tempL   ;save PCH in lpint
  Fsr_temp0L
  Fsr_temp0H 
  Fsr_temp1L
  Fsr_temp1H 
  TempCANCON
  TempCANSTAT
  Datmode     ;flag for data waiting 
  Count     ;counter for loading
  Count1
  
  
  
  
  
  
  Temp      ;temps
  Temp1
  
  
  
  
  

  ENDC
  

  
  CBLOCK  h'60' ;rest of bank 0
  
  Rx0con      ;start of receive packet 0
  Rx0sidh
  Rx0sidl
  Rx0eidh
  Rx0eidl
  Rx0dlc
  Rx0d0
  Rx0d1
  Rx0d2
  Rx0d3
  Rx0d4
  Rx0d5
  Rx0d6
  Rx0d7
  
  
  
  
  
  
  
  
    
  
  Cmdtmp    ;command temp for number of bytes in frame jump table
  
  DNindex   ;holds number of allowed DNs
  Match   ;match flag
  DNcount   ;which DN matched?
  ENcount   ;which EN matched
  ENcount1  ;temp for count offset
  ENend   ;last  EN number
  ENtemp
  EVtemp    ;holds current EV
  EVtemp1 
  EVtemp2   ;holds current EV qualifier
  EVtemp3 
  Mask
  Shift
  Shift1
  
  
  Eadr    ;temp eeprom address
  
  
  
  
  
  
  

    
  ;*************************************************************
  Intemp    ;used in input test
  Intemp1
  Input   ;holds input state for change detection
  Incount   ;used in input scan
  Inbit   ;which input bit
  Debcount  ;debounce counter
  Inmode    ;all digital or 4 analog
  Outtmp    ;used to sort out output
  Togmode   ;outputs to toggle
  ;
  ;****************************************************************
  
  
    
  ENDC
  
  CBLOCK  0x100   ;bank 1
  EN1         ;start of EN ram
  EN1a
  EN1b
  EN1c
  
  EN2
  EN2a
  EN2b
  EN2c
  
  ENDC
  CBLOCK  0x200   ;bank 2
  EV1         ;start of EV ram
  ENDC
  

;****************************************************************
;
;   start of program code

    ORG   0000h
    nop           ;for debug
    goto  setup

    ORG   0008h
    goto  hpint     ;high priority interrupt

    ORG   0018h 
    goto  lpint     ;low priority interrupt


;*******************************************************************

    ORG   0020h     ;start of program
; 
;
;   high priority interrupt. Used for CAN receive and transmit error.

hpint movff CANCON,TempCANCON
    movff CANSTAT,TempCANSTAT
  
    movff PCLATH,PCH_tempH    ;save PCLATH
    clrf  PCLATH
  
    movff FSR0L,Fsr_temp0L    ;save FSR0
    movff FSR0H,Fsr_temp0H
    movff FSR1L,Fsr_temp1L    ;save FSR1
    movff FSR1H,Fsr_temp1H
    movlw 0
    movwf PCLATH          ;for jump
  
    movf  TempCANSTAT,W     ;Jump table
  
    andlw B'00001110'
    addwf PCL,F     ;jump
    bra   back
    bra   back      
    bra   back
    bra   back
    bra   back
    bra   rxb1int     ;only receive interrupts used
    bra   rxb0int
    bra   back
    
rxb1int bcf   PIR3,RXB1IF   ;uses RB0 to RB1 rollover so may never use this
                ;may need bank switch?
  
    lfsr  FSR0,Rx0con   ;
    bsf   Datmode,0
    goto  access
    
rxb0int bcf   PIR3,RXB0IF
    
    lfsr  FSR0,Rx0con
    bsf   Datmode,0
    goto  access
    


access  movf  CANCON,W
    andlw B'11110001'
    movwf CANCON
    movf  TempCANSTAT,W
    andlw B'00001110'
    iorwf CANCON
    lfsr  FSR1,RXB0CON  ;this is switched bank
load  movff POSTINC1,POSTINC0
    movlw 0x6E      ;end of access buffer lo byte
    cpfseq  FSR1L
    bra   load    
    
back  bcf   RXB0CON,RXFUL ;ready for next
  
back1 movlw B'00000000'
    andwf PIR3      ;clear any other flags
    movf  CANCON,W
    andlw B'11110001'
    iorwf TempCANCON,W
    
    movwf CANCON
    movff PCH_tempH,PCLATH
    movff Fsr_temp0L,FSR0L    ;recover FSR0
    movff Fsr_temp0H,FSR0H

    movff Fsr_temp1L,FSR1L    ;recover FSR1
    movff Fsr_temp1H,FSR1H

    
    retfie  1       ;use shadow registers



;**************************************************************
;
;
;   low priority interrupt. 
; 

lpint   retfie  
            

;*********************************************************************


    


main  btfss PIR2,TMR3IF   ;flash timer overflow?
    bra   noflash
    btg   PORTB,6     ;toggle LED
    bcf   PIR2,TMR3IF
noflash btfss Datmode,0   ;any new CAN frame received?
    bra   main
    bra   packet
    
  
    
  
  
                ;main packet handling is here
    
packet  movlw CMD_ON  ;only ON and OFF events supported
    subwf Rx0d0,W
    bz    go_on
    movlw CMD_OFF
    subwf Rx0d0,W
    bz    go_on
    movlw SCMD_ON
    subwf Rx0d0,W
    bz  short
    movlw SCMD_OFF
    subwf Rx0d0,W
    bz  short
  
          
main2 bcf   Datmode,0
    goto  main      ;loop




short clrf  Rx0d1
    clrf  Rx0d2     
    
go_on btfss PORTA,LEARN
    bra   learn1      ;is in learn mode
    call  enmatch
    sublw 0
    bz    do_it
    bra   main2     ;not here
    
do_it 
    call  ev_set      ;do it
    bra   main2
    
    
learn1  call  enmatch     ;is it there already?
    sublw   0
    bz    isthere
    btfss PORTA,SETUP   ;if unset and not here
    bra   l_out     ;do nothing else 
    call  learnin     ;put EN into stack and RAM
    sublw 0
    bz    new_EV      ;OK
    bra   l_out     ;too many     
isthere btfss PORTA,SETUP   ;is it here and unlearn,goto unlearn
    bra   unlearn     ;else modify EVs
    bra   mod_EV
  

    
    
    
  
  
new_EV  movlw LOW ENindex+1   ;here if a new event
    movwf EEADR
    bsf   EECON1,RD
    decf  EEDATA,W
    movwf ENcount       ;recover EN counter
mod_EV  rlncf ENcount,W     ;two byte values
    addlw LOW EVstart     ;point to EV
    movwf EEADR
    bsf   EECON1,RD
    call  getop       ;get switch. value in EVtemp
    movf  EVtemp,W
        
    iorwf EEDATA,W
    movwf Temp
    call  eewrite       ;put back EV value  
    incf  EEADR
    bsf   EECON1,RD
    btfsc PORTB,POL     ;test polarity
    bra   shift3
    movf  EVtemp,W
        
    iorwf EEDATA,W
    movwf EVtemp2
shift4  call  eewrite       ;put back EV qual value 
    movff Temp,EVtemp
    call  ev_set      ;try it
    bra   l_out
shift3  comf  EVtemp,W    ;clear the POL bit
    andwf EEDATA,W
    movwf EVtemp2
    bra   shift4  

l_out bcf   Datmode,0
    clrf  PCLATH
    goto  main2
                ;unlearn an EN. 
unlearn movlw LOW ENindex+1   ;get number of events in stack
    movwf EEADR
    bsf   EECON1,RD
    
    movff EEDATA,ENend
    movff EEDATA,ENtemp
    rlncf ENend,F     ;ready for end value
    rlncf ENend,F
    movlw LOW ENstart
    addwf ENend,F     ;end now points to next past end in EEPROM
    movlw 4
    addwf ENend,F
    rlncf ENcount,F   ;Double the counter for two bytes
    rlncf ENcount,F   ;Double the counter for two bytes
    movlw LOW ENstart + 4
    addwf ENcount,W
    movwf EEADR
un1   bsf   EECON1,RD
    movf  EEDATA,W    ;get byte
    decf  EEADR,F
    decf  EEADR,F
    decf  EEADR,F
    decf  EEADR,F
    call  eewrite     ;put back in
    movlw 5
    addwf EEADR,F
    movf  ENend,W
    cpfseq  EEADR
    bra   un1
    
    rrncf ENcount,F   ;back to double bytes
    rlncf ENtemp,F
    movlw LOW EVstart
    addwf ENtemp,F
    movlw 2
    addwf ENtemp,F
    movlw LOW EVstart + 2
    addwf ENcount,W
    movwf EEADR
un2   bsf   EECON1,RD
    movf  EEDATA,W    ;get byte
    decf  EEADR,F
    decf  EEADR,F
    call  eewrite     ;put back in
    movlw 3
    addwf EEADR,F
    movf  ENtemp,W
    cpfseq  EEADR
    bra   un2
    movlw LOW ENindex+1
    movwf EEADR
    bsf   EECON1,RD
    movf  EEDATA,W
    movwf Temp
    decf  Temp,W
    call  eewrite     ;put back number in stack less 1
    call  en_ram      ;rewrite RAM stack
    bcf   T3CON,TMR3ON  ;flash timer off
    bcf   PIR2,TMR3IF
    bcf   PORTB,6     ;LED off
    bra   l_out
    

    
;***************************************************************************
;   main setup routine
;*************************************************************************
setup clrf  INTCON      ;no interrupts yet
    clrf  ADCON0      ;ADC is off
    movlw B'00001111'   ;set Port A to all digital for now
    movwf ADCON1
    movlw B'00000011'   ;Port A 0 is unlearn, 1 is polarity
    movwf TRISA     ;
    movlw B'00111011'   ;RB0,1 logic inputs,  RB2 = CANTX, RB3 = CANRX, RB4,5 are logic 
            ;input - RB6,7 for debug, ICSP and diagnostics LEDs
    movwf TRISB
    clrf  PORTB
    bsf   PORTB,2     ;CAN recessive
    movlw B'00000000'   ;Port C drives the outputs 
    movwf TRISC
    clrf  PORTC     ;all outputs off

    
    bsf   RCON,IPEN   ;enable interrupt priority levels
    clrf  BSR       ;set to bank 0
    clrf  EECON1      ;no accesses to program memory  
    clrf  Datmode

    clrf  ECANCON     ;CAN mode 0 for now
     
    bsf   CANCON,7    ;CAN to config mode
    movlw B'00000011'   ;set CAN bit rate at 125000 for now
    movwf BRGCON1
    movlw B'10011110'   ;set phase 1 etc
    movwf BRGCON2
    movlw B'00000011'   ;set phase 2 etc
    movwf BRGCON3
    movlw B'00100000'
    movwf CIOCON      ;CAN to high when off
    movlw B'00100100'
    movwf RXB0CON     ;enable double buffer of RX0
    movlb .15
    movlw B'00100000'
    movwf RXB1CON
    clrf  RXF0SIDL
    clrf  RXF1SIDL
    movlb 0
    
mskload lfsr  FSR0,RXM0SIDH   ;Clear masks, point to start
mskloop clrf  POSTINC0    
    movlw LOW RXM1EIDL+1    ;end of masks
    cpfseq  FSR0L
    bra   mskloop
    
    
  
    clrf  CANCON      ;out of CAN setup mode
    movlw B'10110000'
    movwf T3CON     ;set T3 for LED flash
    
    
    movlw B'00000011'
    movwf IPR3      ;high priority CAN RX  interrupts(for now)
    clrf  IPR1      ;all peripheral interrupts are low priority
    clrf  IPR2
    clrf  PIE2
    
        ;test for clear all events
    btfss PORTA,LEARN   ;ignore the clear if learn is set
    goto  seten
    btfss PORTA,SETUP
    call  enclear     ;clear all events if unlearn is set during power up
seten call  en_ram      ;put events in RAM
    
    
    clrf  INTCON2     ;enable port B pullups 
    clrf  INTCON3     ;just in case
    
  
    movlw B'00000011'   ;Rx0 and RX1 interrupt 
    movwf PIE3  
    
  
  
    clrf  PIR1
    clrf  PIR2
    clrf  PIR3      ;clear all flags
    bcf   RXB0CON,RXFUL
    movlw B'10000000'
    movwf INTCON      ;enable interrupts
    bsf PORTB,7     ;turn on RUN LED.   

main4 goto  main  
    
;****************************************************************************
;   start of subroutines    


;********************************************************************
;   Do an event.  arrives with EV in EVtemp and EVtemp2

ev_set  movff EVtemp,EVtemp1
    comf  EVtemp1,W
    andwf PORTC,W     ;mask unaffected outputs
    movwf EVtemp1 
    movf  EVtemp,W
    xorwf EVtemp2,F   ;change polarity if needed
    btfss Rx0d0,0     ;on or off?
    bra   ev_on
    movf  EVtemp,W
    xorwf EVtemp2,W
    iorwf EVtemp1,W
    movwf PORTC
    return      
ev_on movf  EVtemp,W
    andwf EVtemp2,W
    iorwf EVtemp1,W
    movwf PORTC
    return      

          
    

    





;*************************************************************************************

;   
eeread  bcf   EECON1,EEPGD  ;read a EEPROM byte, EEADR must be set before this sub.
    bcf   EECON1,CFGS
    bsf   EECON1,RD
    movf  EEDATA,W
    return

;**************************************************************************
eewrite movwf EEDATA      ;write to EEPROM, EEADR must be set before this sub.
    bcf   EECON1,EEPGD
    bcf   EECON1,CFGS
    bsf   EECON1,WREN
    
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
    movlw B'11000000'
    movwf INTCON    ;reenable interrupts
    
    return  
    



;**************************************************************************
;
;   EN match. Compares EN (in Rx0d1, Rx0d2, Rx0d3 and Rx0d4) with stored ENs
;   If match, returns with W = 0
;   The matching number is in ENcount. The corresponding EV is in EVtemp and EVtemp2
;
enmatch lfsr  FSR0,EN1  ;EN ram image
    movlw LOW ENindex+1 ;
    movwf EEADR
    bsf   EECON1,RD
    movff EEDATA,Count
    movf  Count,F
  
    bz    en_out    ;if no events set, do nothing
    clrf  ENcount
  
    
ennext  clrf  Match
    movf  POSTINC0,W
    cpfseq  Rx0d1
    incf  Match
    movf  POSTINC0,W
    cpfseq  Rx0d2
    incf  Match
    movf  POSTINC0,W
    cpfseq  Rx0d3
    incf  Match
    movf  POSTINC0,W
    cpfseq  Rx0d4
    incf  Match
    tstfsz  Match
    bra   en_match
    rlncf ENcount,W   ;get EVs
    addlw LOW EVstart   
    movwf EEADR
    bcf   EEADR,0   ;multiple of 2
    bsf   EECON1,RD
    movff EEDATA,EVtemp ;EV
    incf  EEADR
    bsf   EECON1,RD
    movff EEDATA,EVtemp2  ;EV qualifier
    
    retlw 0     ;is a match
en_match  
    movf  Count,F
    bz    en_out
    decf  Count,F
    incf  ENcount,F
    bra   ennext
en_out  retlw 1     ;no match 
      
;**************************************************************************

getop movlw B'00010011'   ;get DIP switch setting for output
    andwf PORTB,W
    movwf Temp
    movwf Temp1
    rrncf Temp1,F
    rrncf Temp1,W
    andlw B'00000100'
    iorwf Temp,W
    
    andlw B'00000111'   ;mask
    movwf Shift
    movlw 1
    movwf EVtemp
shift movf  Shift,F     ;is it zero?
    bz    shift2
    rlncf EVtemp,F    ;put rolling bit into EVtemp
    decf  Shift,F
    bra   shift
shift2  return

    


;*********************************************************

;   learn input of EN

learnin btfss PORTA,SETUP     ;don't do if unlearn
    return
    movlw LOW ENindex+1
    movwf EEADR
    bsf   EECON1,RD
    movff EEDATA,ENcount    ;hold pointer
    movlw EN_NUM
    cpfslt  ENcount
    retlw 1           ;too many
    lfsr  FSR0,EN1      ;point to EN stack in RAM
    
    rlncf ENcount,F     ;double it
    rlncf ENcount,F     ;double again
    movf  ENcount,W
    movff Rx0d1,PLUSW0    ;put in RAM stack
    addlw 1
    movff Rx0d2,PLUSW0
    addlw 1
    movff Rx0d3,PLUSW0
    addlw 1
    movff Rx0d4,PLUSW0
    movlw LOW ENstart
    addwf ENcount,W
    movwf EEADR
    movf  Rx0d1,W       ;get EN hi byte
    call  eewrite
    incf  EEADR
    movf  Rx0d2,W
    call  eewrite
    incf  EEADR
    movf  Rx0d3,W
    call  eewrite
    incf  EEADR
    movf  Rx0d4,W
    call  eewrite
    
    
    movlw LOW ENindex+1
    movwf EEADR
    bsf   EECON1,RD
    movf  EEDATA,W
    addlw 1         ;increment for next
    movwf Temp        
    call  eewrite       ;put back
    movlw EN_NUM        ;is it full now?
    subwf Temp,W
    bnz   notful
    bsf   T3CON,TMR3ON    ;set for flash
    retlw 1
notful  retlw 0
    
;**********************************************************************
;   loads ENs from EEPROM to RAM for fast access
;   shifts all 32 even if less are used

en_ram  movlw EN_NUM
    movwf Count     ;number of ENs allowed 
    
    bcf   STATUS,C    ;clear carry
    rlncf Count,F     ;double it
    rlncf Count,F     ;double again
    lfsr  FSR0,EN1    ;set FSR0 to start of ram buffer
    movlw LOW ENstart     ;load ENs from EEPROM to RAM
    movwf EEADR
enload  bsf   EECON1,RD   ;get first byte
    movff EEDATA,POSTINC0
    incf  EEADR
    decfsz  Count,F
    bra   enload
    return
    
;******************************************************************

;   clears all stored events

enclear movlw EN_NUM * 6 + 2    ;number of locations in EEPROM
    movwf Count
    movlw LOW ENindex
    movwf EEADR
enloop  movlw 0
    call  eewrite
    incf  EEADR
    decfsz  Count
    bra   enloop
    return  
    
;*********************************************************************
;   a delay routine
      
dely  movlw .10
    movwf Count1
dely2 clrf  Count
dely1 decfsz  Count,F
    goto  dely1
    decfsz  Count1
    bra   dely2
    return    
    
;************************************************************************   
  ORG 0xF00000      ;EEPROM data. Defaults
  

    
ENindex de  0,0   ;points to next available EN number (only hi byte used)

  ORG 0xF00002

ENstart 
    
  ORG 0xF00082
  
    
EVstart de  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
    de  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
    de  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
    de  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
    de  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
    de  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
    de  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
    de  0,0,0,0,0,0,0,0,0,0,0,0,0,0
    
    end

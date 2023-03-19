; NABU 4K ROM
; P/N: 90020060 RevA
; recreated by D. Hunter Feb - Mar 2023
;

; I/O ports
CTRL_REG	EQU	000h		; 74LS273
PSG_DATA	EQU	040h		; AY-3-8910
PSG_ADDR	EQU	041h
HCCA		EQU	080h		; TR1863 UART
KB_DATA		EQU	090h		; 8251A UART
KB_STAT		EQU	091h
VDP_DATA	EQU	0A0h		; TMS9918A
VDP_ADDR	EQU	0A1h

; PSG registers
PSG_R7_ENABLE	EQU	007h
PSG_R14_INT_MASK EQU	00Eh
PSG_R15_STATUS	EQU	00Fh

BEGIN		EQU	140Fh		; START OF LOADED PROGRAM
RAM_TEST_START	EQU	2000h		; START AFTER 8K ROM 
VRAM		EQU	0FC40h		; VIDEO RAM MIRROR? START 
STACK		EQU	0FFEEh		; STACK TOP + 1

; INTERNAL VARIABLES
CTRL_BITS	EQU	0FFEEh		; CURRENT CONTROL REGISTER SETTINGS
POST_FLG	EQU	0FFEFh		; POWER ON SELF TEST ERROR FLAG
ERR_CNT		EQU	0FFF0h		; ERROR COUNTS FOR EACH TEST
BOOT_FLG	EQU	0FFF6h		; COLD BOOT FLAG
TEST_NUM	EQU	0FFF7h		; CURRENT SELF TEST NUMBER
SIG_STAT	EQU	0FFF8h		; SIGNAL STATUS
DSP_STR		EQU	0FFF9h		; DISPLAY CODE STRING MEMORY FFF9h - FFFDh
LOAD_FLG:	EQU	0FFFEh		; SET TO 5AA5h IF PROGRAM LOADED


; NETWORK ADAPTOR CONSTANTS
NA_ESCAPE	EQU	10h		; ESCAPE SEQUENCE
NA_RESET	EQU	80h		; RESET CONNECTION
NA_SET_STATUS	EQU	81h		; SET STATUS
NA_GET_STATUS	EQU	82h		; GET STATUS
NA_STARTUP	EQU	83h		; START UP CONNECTION
NA_REQUEST	EQU	84h		; PACKET REQUEST
NA_CHANGE	EQU	85h		; CHANGE CHANNEL
NA_UNAUTH	EQU	90h		; UNAUTHORIZED ACCOUNT
NA_AUTH		EQU	91h		; AUTHORIZED ACCOUNT
NA_NO_SIGNAL	EQU	9Fh		; NO CABLE SIGNAL
NA_SIGNAL	EQU	1Fh		; CABLE SIGNAL LOCK
NA_DONE		EQU	0E1h		; TRANSFER COMPLETE
NA_CONFIRM	EQU	0E4h		; CONFIRMED MESSAGE

; STATUS MESSAGES
STAT_SIG	EQU	01h		; SIGNAL OK
STAT_READY	EQU	05h		; READY
STAT_GOOD	EQU	06h		; GOOD
STAT_NA_RDY	EQU	1Eh		; ADAPTOR READY
STAT_CONN	EQU	8Fh		; CONNECTED
;
; ACK = ESCAPE, STAT_GOOD
; FINISHED = ESCAPE, NA_DONE
; PACKET SIZE = 1024 BYTES  = HEADER (16 BYTES) + PAYLOAD (991 BYTES) + FOOTER/CRC (2 BYTES)

INV_RESP	EQU	02h		; INVALID RESPONSE
OK_RESP		EQU	00h		; ALL IS GOOD RESPONSE


; INTERRUPT MASKS
INT_HRCV	EQU	80h		; HCCA RECEIVE CHARACTER
INT_HSND	EQU	40h		; HCCA SEND CHARACTER
INT_KBD		EQU	20h		; KEYBOARD 

; CONTROL REGISTER (PORT 0) BITS
PAUSE		EQU	020h		; D5 - YELLOW LED (1 = ON)
ALERT		EQU	010h		; D4 - RED LED (1 = ON)
CHECK		EQU	008h		; D3 - GREEN LED (1 = ON)
STROBE		EQU	004h		; D2 - STROBE DATA TO PRINTER (1 = STROBE LOW)
TV_SW		EQU	002h		; D1 - VIDEO TO TV (1 = NABU, 0 = CABLE)
ROM_SEL		EQU	001h		; D0 - 0 = ROM SELECT, 1 = RAM


	ORG     0000h			; START AT RESET VECTOR

        ; COLD BOOT
COLD_BOOT:
	LD      A,TV_SW
        LD      (CTRL_BITS),A	; SAVE CONTROL BIT SETTING
        OUT     (CTRL_REG),A	; SELECT NABU FOR SCREEN
        LD      SP,STACK	; SET STACK POINTER
	
        LD      HL,VDP_INIT_TBL	; POINT TO VDP INITIALIZATION TABLE
        LD      B,08h			
        LD      C,VDP_ADDR
VDP_INIT:
	OUTI			; INITIALIZE VDP
        LD      A,B
        OR      80h
        OUT     (VDP_ADDR),A
        AND     7Fh             
        JR      NZ,VDP_INIT
	
        LD      HL,POST_FLG	; CLEAR 10 BYTES ABOVE THE STACK
        LD      DE,ERR_CNT
        LD      BC,0009h
        XOR     A		; CLEAR ACC
        LD      (HL),A		; RESET POST FLAG
        LDIR
	
        CALL    PSG_PORTB	; SET PSG TO PORT B
	
        XOR     A
        CALL    KB_CTRL		; INITIALIZE 8251 
        CALL    KB_CTRL
        CALL    KB_CTRL
        CALL    KB_CTRL
        CALL    KB_CTRL
        LD      A,40h           ; RESET 8251
        CALL    KB_CTRL
        LD      A,4Eh           ; 1 STOP BIT, 8 BITS, 64X CLOCK
        CALL    KB_CTRL
        LD      A,04h		; ENABLE RECEIVE
        CALL    KB_CTRL
	
        CALL    LOAD_VDP_TABLE
        CALL    CHK_LOAD	; PROGRAM LOADED BEFORE?
        JP      Z,START_HCCA	; YES, SKIP SELF TEST
        LD      A,3Ah           ; TURN ALL LEDS ON
        OUT     (CTRL_REG),A
	
	; WAIT FOR COMMUNICATION FROM KEYBOARD
        LD      H,0Ah
        LD      C,INT_KBD       ; ENABLE KB INTERRUPT
        LD      DE,0E000h	; TIMEOUT VALUE
        LD      B,01h
WAIT_KB:  
	CALL    DEC_AND_TEST_DE
        JR      Z,WAIT_KB1	; NOT TIMED OUT
        DJNZ    WAIT_KB1
        JR      ROM_CHECK	; KEYBOARD OK, MOVE ON

WAIT_KB1:  
	CALL    SET_INT_MASK
        JR      Z,WAIT_KB
        IN      A,(KB_DATA)
        CP      31h             ; '1'
        JR      Z,PRESS_1
        CP      3Dh             ; '='
        JR      NZ,WAIT_KB
        BIT     0,H
        JR      Z,WAIT_KB
WAIT_KB2:  
	DEC     H
        JR      Z,WAIT_KB3	; GIVE UP TRYING
        JR      WAIT_KB

PRESS_1:  
	BIT     0,H
        JR      NZ,WAIT_KB
        JR      WAIT_KB2

WAIT_KB3:  
	LD      HL,POST_FLG
        INC     (HL)		; INCREMENT DEBUG FLAG TO INDICATE ERROR
	
        ; ROM CHECK BY CALCULATING AND VERIFYING CHECKSUM
	; ALSO THE PLACE FOR RE-STARTING
	; TEST # 0
ROM_CHECK:  
	LD      A,(BOOT_FLG)	; COLD BOOT?  (CLEARED ABOVE, NOT ZERO IF RESTARTING)
        BIT     0,A
        JR      NZ,ROM_CHK1
        CALL    SET_LED
        OUT     (CTRL_REG),A	; SET LEDS
ROM_CHK1:  
	XOR     A
        LD      (LOAD_FLG+1),A	; CLEAR LOAD FLAG
        LD      IX,CHKSUM-1	; IX = ROM POINTER START BELOW CHECKSUM
        LD      DE,0FFFFh	; DE = -1
        XOR     A
        LD      H,A		; CLEAR H,L = CHECKSUM
        LD      L,A
        LD      B,A		; CLEAR B
ROM_CHK2:  
	LD      C,(IX+00h)	; GET ROM VALUE
        ADD     HL,BC		; ADD TO RUNNING CHECKSUM
        ADD     IX,DE		; DECREMENT ROM POINTER
        JR      C,ROM_CHK2	; DONE?
	
        LD      A,(CHKSUM)	; YES, COMPARE STORED CHECKSUM WITH H,L
        CP      L
        JR      NZ,ROM_CHK3
        LD      A,(CHKSUM+1)
        SUB     H
        JR      Z,ROM_CHK4
ROM_CHK3:  
	CALL    FLASH_ALERT	; ROM ERROR
ROM_CHK4:
	CALL    RESP_BEEP

	; TEST VIDEO DISPLAY PROCESSOR
	; TEST # 1
        XOR     A
        OUT     (VDP_ADDR),A
        LD      A,40h           ; WRITE TO VDP RAM AT 0000h
        OUT     (VDP_ADDR),A
        XOR     A		; FIRST VALUE IS 0
        LD      DE,0001h
        LD      IX,0C000h
VDP_TST_PREP:
	OUT     (VDP_DATA),A	; PREPARE VDP RAM (2K WORTH) FOR TESTING
        INC     A		; INCREMENT VALUE STORED
        ADD     IX,DE
        JR      NC,VDP_TST_PREP
	
	; TEST VDP RAM	
VDP_RAM_TEST:
        LD      IX,0C000h
        XOR     A		; CLEAR REGISTERS
        LD      C,A
        LD      B,A
        LD      H,A
        LD      L,A
VTEST1: 
	LD      A,L
        OUT     (VDP_ADDR),A
        LD      A,H
        OUT     (VDP_ADDR),A
        IN      A,(VDP_DATA)	; READ FROM VDP RAM AT (HL)
        CP      C
        JR      NZ,VTEST2       ; IS THE MEMORY CLEARED?
        LD      A,L		; YES, 
        OUT     (VDP_ADDR),A
        LD      A,H
        OR      40h             ; WRITE TO VDP RAM AT (HL)
        OUT     (VDP_ADDR),A
        LD      A,C
        CPL
        OUT     (VDP_DATA),A	; WRITE ALL ONES
        LD      A,L
        OUT     (VDP_ADDR),A
        LD      A,H
        OUT     (VDP_ADDR),A
        IN      A,(VDP_DATA)	; GET BACK
        CPL
        CP      C		; VERIFY
        JR      Z,VTEST3	; IT'S GOOD
VTEST2: CALL    FLASH_ALERT	; IT'S NOT, INDICATE ERROR
VTEST3: INC     C
        INC     HL
        LD      DE,0001h
        ADD     IX,DE
        JR      NC,VTEST1
        CALL    LOAD_VDP_TABLE	; RESTORE VDP SETTINGS (ERASED BY TEST)
        CALL    SHOW_PN		; SHOW THE PART NUMBER
        CALL    RESP_BEEP	; AND BEEP
	
	; TEST RAM 
	; START AFTER THE ROM (ASSUMING 8K) AND UP TO THE STACK (2000h - FFEDh)
	; NOTE: IT DOESN'T TEST THE RAM FROM 1000h TO 2000h WHICH IS EITHER RAM (4k) OR ROM (8k)
 	; TEST # 2
RAM_TEST:	
        XOR     A
        LD      IX,RAM_TEST_START
        LD      HL,2100h
        LD      DE,0001h
RTEST1: LD      (IX+00h),A      ; FILL WITH INCREMENTING VALUES
        INC     A
        INC     IX
        ADD     HL,DE
        JR      NC,RTEST1
	
        XOR     A
        LD      C,A
        LD      HL,2100h
        LD      IX,RAM_TEST_START
        LD      B,A
RTEST2: LD      A,(IX+00h)
        CP      C
        JR      NZ,RTEST3
        CPL
        LD      (IX+00h),A
        LD      A,(IX+00h)
        CPL
        CP      C
        JR      Z,RTEST4
RTEST3: CALL    FLASH_ALERT	; SHOW ERROR
RTEST4: INC     IX
        INC     C
        ADD     HL,DE
        JR      NC,RTEST2
        CALL    RESP_BEEP
	
	; TEST PROGRAMMABLE SOUND GENERATOR (PSG)
	; TEST # 3

	; CLEAR PSG REGISTERS	
        LD      B,10h
        LD      C,00h
CLRPSG: LD      A,B
        DEC     A
        OUT     (PSG_ADDR),A 
        LD      A,C
        OUT     (PSG_DATA),A
        DJNZ    CLRPSG
	
	; GENERATE A SERIES OF TONES
GEN_TONES:
        LD      C,00h		; C = COUNT
GTONE1: LD      B,0FFh
GTONE2: LD      A,C
        OUT     (PSG_ADDR),A
        LD      A,B
        OUT     (PSG_DATA),A	; GENERATE TONE 
        IN      A,(PSG_DATA)	; GET TONE BACK
        CALL    CHK_TONE
        CALL    NZ,FLASH_ALERT	; INDICATE ERROR IF IT OCCURRED
        SRL     B		; SHIFT B FOR DURATION
        JR      C,GTONE2
        INC     C		; NEXT TONE
        LD      A,14
        CP      C		; LAST TONE?
        JR      NZ,GTONE1	; NO, CONTINUE
        JR      CHK_KB
	
        ; CHECK SOUND GENERATOR TONE
	; TONE NUMBER IN C
	; DURATION IN B
CHK_TONE:  
	LD      HL,TONE_TBL
        LD      E,C
        LD      D,00h		; SELECT TONE NUMBER
        ADD     HL,DE
        AND     (HL)		; MASK WITH TONE
        LD      E,A
        LD      A,B
        AND     (HL)
        CP      E		; SET ERROR FLAG
        RET

TONE_TBL: 
	DEFB    0FFh		; TONE VALUES
        DEFB    0Fh
        DEFB    0FFh
        DEFB    0Fh
        DEFB    0FFh
        DEFB    0Fh
        DEFB    1Fh
        DEFB    0FFh
        DEFB    1Fh
        DEFB    1Fh
        DEFB    1Fh
        DEFB    0FFh
        DEFB    0FFh
        DEFB    0Fh

        ; KEYBOARD TEST
	; TEST # 4
	
        ; CHECK FOR KEYBOARD ERRORS
CHK_KB:  
	CALL    RESP_BEEP
        CALL    PSG_PORTB
        LD      DE,0000h
        LD      B,05h		; NUMBER OF TRIES
CHK_KB1:  
	LD      C,INT_KBD
        CALL    SET_INT_MASK
        JR      Z,CHK_KB3
        IN      A,(KB_DATA)
        CP      95h
        JR      NC,CHK_KB3      ; KB POWER UP RESET?
        CP      91h
        JR      C,CHK_KB3       ; KB RAM FAULT
        CP      94h
        JR      Z,WDOG		; GOT KB WATCHDOG, CONTINUE
				; WATCHDOG IS APPROX. EVERY 3.7 SECONDS
CHK_KB2:  
	CALL    FLASH_ALERT	; KEYBOARD FAILURE, FLASH LEDS
	
WDOG:  	CALL    RESP_BEEP
        JR      START_HCCA	; NEXT TEST
	
CHK_KB3:  
	DEC     DE
        LD      A,D
        OR      E
        JR      NZ,CHK_KB1	; LONG TIME OUT CHECK
        DJNZ    CHK_KB1		; CHECK NUMBER OF TRIES
        JR      CHK_KB2		; GIVE UP AND QUIT

        ; ADAPTOR TEST
	; TEST # 5
	
        ; GET A CHARACTER FROM THE NETWORK (HCCA)
START_HCCA:
	IN      A,(HCCA)
        CALL    SEND_START	; SEND STARTUP SEQUENCE
        JR      Z,START1	
        CALL    FLASH_ALERT	; IF ERROR, INDICATE ON LEDS
        JR      START2

        ; START FAILED
ST_FAIL:  
	LD      A,05h		; RESET TEST NUMBER FOR ADAPTOR
        LD      (TEST_NUM),A	; SET TEST_NUM TO 05 (HCCA)
        LD      (LOAD_FLG),A	; RESET LOAD FLAG
        CALL    FLASH_ALERT
        JR      START_HCCA	; TRY AGAIN

	; START WAS GOOD
START1:  
	LD      C,NA_GET_STATUS
        CALL    OUT_HCCA	; REQUEST STATUS FROM ADAPTOR
        JR      NZ,ST_FAIL
        LD      A,STAT_SIG
	
        OUT     (HCCA),A	; REQUEST SIGNAL STATUS
        CALL    WAIT_FOR_HCCA
        IN      A,(HCCA)	; GET SIGNAL STATUS BACK
        JR      NZ,ST_FAIL
	
        LD      (SIG_STAT),A	; PUT SIGNAL STATUS IN SIG_STAT
        LD      C,NA_ESCAPE
        CALL    CHK_HCCA	; EXPECT ESCAPE, THEN
        JR      NZ,ST_FAIL
	
        LD      C,NA_DONE
        CALL    CHK_HCCA	; EXPECT DONE
        JR      NZ,ST_FAIL	; IF ZERO, RECEIVED "FINISHED" (ESCAPE,DONE) RESPONSE
	
	
START2:  
	CALL    CHK_LOAD	; PROGRAM LOADED?
        CALL    NZ,RESP_BEEP	; NO, SOUND BEEP
        LD      A,(POST_FLG)
        OR      A		; ERROR FLAG?
        JP      NZ,ROM_CHECK	; YES, RESTART TESTS ON ERROR
; *** THIS WOULD BE A GOOD PLACE FOR A JUMP TO A MONITOR OR OTHER CODE
        LD      A,(BOOT_FLG)	; CHECK COLD BOOT FLAG
        BIT     0,A
        JR      NZ,START3	; WARM BOOT, JUMP AROUND TV SWITCH
        LD      A,TV_SW
        LD      (CTRL_BITS),A	; SET TV SWITCH FOR NABU
        OUT     (CTRL_REG),A
	
START3:  
	LD      HL,SIG_STAT	; BRING BACK SIGNAL STATUS RECEIVED
        BIT     7,(HL)		; GET UPPER BIT OF SIGNAL STATUS
        JR      Z,CHAN_OK	; SIGNAL GOOD IF BIT 7 IS ZERO
	
NEWCODE:
	LD      HL,MSG_CHAN	; OTHERWISE, PROMPT FOR THE USER CHANNEL CODE
        CALL    DSP_MSG
NEWCOD1:  
	LD      HL,DSP_STR	; NEW CODE WILL GO IN DSP_STR
        LD      DE,02C7h	; SCREEN ADDRESS TO ECHO CODE ENTRY
        CALL    GET_HEX
        LD      HL,DSP_STR	; POINT TO DIGIT DISPLAY STRING
        XOR     A
        LD      C,A		; CLEAR C
        LD      B,04h		; GET 4 HEX DIGITS AND STORE 
				; IN SEQUENTIAL MEMORY LOCATIONS
NEWCOD2:  
	LD      A,(HL)		; GET NIBBLE FROM MEMORY
        BIT     0,B
        JR      Z,NEWCOD3	; DONE?
        SLA     A		; NO, SHIFT CODE LEFT
        BIT     4,A
        JR      Z,NEWCOD3
        RES     4,A
        INC     A
NEWCOD3:  
	ADD     A,C		; ADD NEW NIBBLE TO ONE BROUGHT FROM MEMORY
        LD      C,A
        INC     HL
        DJNZ    NEWCOD2		; RECEIVED 4 CHARACTERS?
        AND     0Fh
        CP      (HL)
        JR      Z,NEWCOD4	; VALID CODE?
	
        LD      HL,MSG_RETYPE	; NO, PROMPT FOR RE-TYPE
        CALL    DSP_MSG
        LD      C,90h
        LD      DE,0E000h
        CALL    BEEP
        JR      NEWCOD1

NEWCOD4:  
	LD      HL,DSP_STR	; POINT TO NEW CODE
        LD      B,04h
        LD      DE,0000h
NEWCOD5:  
	LD      A,(HL)		; GET CODE NIBBLE
        SLA     E
        RL      D
        SLA     E
        RL      D
        SLA     E
        RL      D
        SLA     E
        RL      D		; SHIFT DE OVER BY 4
        ADD     A,E		; ADD IN NIBBLE TO DE
        LD      E,A
        INC     HL		; NEXT CODE 
        DJNZ    NEWCOD5		; 4 NIBBLES = 1 WORD
        PUSH    DE		; SAVE
        LD      C,NA_CHANGE
        CALL    OUT_HCCA	; SEND NEW CHANGE COMMAND TO HCCA
        JP      NZ,ST_FAIL
	
        POP     DE		; GET CODE
        LD      A,D
        OUT     (HCCA),A	; SEND MSB
        LD      C,INT_HSND
        CALL    SET_INT_MASK
        LD      A,E
        OUT     (HCCA),A	; SEND LSB
        CALL    EXP_CONF	; CHECK FOR CONFIRMATION BACK
        JP      NZ,ST_FAIL	; NO, TRY AGAIN

	; HAVE A VALID CHANNEL CODE
CHAN_OK:
	LD      C,NA_SET_STATUS	; SEND TO HCCA: NA_SET_STATUS, STAT_CONN, STAT_READY
        CALL    OUT_HCCA	
        JP      NZ,ST_FAIL
	
        LD      A,STAT_CONN	; INDICATE CONNECTED
        OUT     (HCCA),A
        LD      C,INT_HSND
        CALL    SET_INT_MASK
        LD      A,STAT_READY	; AND READY
        OUT     (HCCA),A
        CALL    EXP_CONF
        JP      NZ,ST_FAIL
	
        LD      A,STAT_READY
        LD      (SIG_STAT),A	; PUT READY IN SIG_STAT  (ADAPTOR READY)
	
        LD      HL,0000h
        LD      DE,0000h
        LD      BC,1000h
        LDIR			; COPY ROM CODE TO RAM UNDERNEATH
	
        LD      A,(CTRL_BITS)
        SET     0,A
        OUT     (CTRL_REG),A	; DISABLE ROM AND SWITCH TO RAM
				; *** NOW RUNNING IN RAM
	
        LD      HL,MSG_WAIT	; SEND WAIT MESSAGE
        CALL    DSP_MSG

	; READY TO GET PAK # 000001

	;
	; RAM LAYOUT FOR RECEIVING PACKETS / PROGRAMS
	;
	; ADDRESS	START	DESCRIPTION
	; 1000h		00	PAK ID BITS 23-16
	; 1001h		00	PAK ID BITS 15-8
	; 1002h		01	PAK ID BITS 7-0
	; 1003h		00	SEGMENT NUMBER
	; 1004h		0B	LSB OF PACKET STORAGE ADDRESS
	; 1005h		10	MSB OF PACKET STORAGE ADDRESS
	; 1006h		XX	LSB OF PAYLOAD LENGTH (991 MAX)
	; 1007h		XX	MSB OF PAYLOAD LENGTH
	; 1008h		0D	LSB OF RAM WRITE ADDRESS
	; 1009h		14	MSB OF RAM WRITE ADDRESS
	; 100Ah		XX	LAST PACKET FLAG IN BIT 4 ?
	; 100Bh		XX	START OF PACKET (1024 BYTES)
	;  ---
	; 140Ah		XX	END OF PACKET
	; 140Bh		00	LSB OF LAST ADDRESS FOR DATA
	; 140Ch		00	MSB OF LAST ADDRESS FOR DATA
	; 140Dh		XX	LSB OF DATA SIZE OF PROGRAM?
	; 140Eh		XX	MSB OF DATA SIZE OF PROGRAM?
	; 140Fh		XX	STARTING POINT OF PROGRAM
	;
	; PACKET = 16 BYTES HEADER, 991 BYTES PAYLOAD, 2 BYTES FOOTER (CRC?)
	;
	; HEADER:
	;	PAK ID:		USB MSB LSB	24 BIT PAK ID
	;	SEGMENT:	XX		SEGMENT NUMBER
	;	OWNER:		01h
	;	TIER:		75h 0FFh 0FFh 0FFh
	;	UNKNOWN:	7Fh 80h
	;	TYPE:		XX		00h (NORMAL) OR 10h (LAST SEGMENT)
	;	SEGMENT:	LSB MSB
	;	OFFSET: 	MSB LSB
	;

        ; LOAD A PACKET FROM HCCA, COPY TO RAM
	; (RUNNING OUT OF RAM NOW)
LOAD_PKT:  
	XOR     A
        LD      (1000h),A
        LD      (1001h),A
        LD      (1003h),A
        INC     A
        LD      (1002h),A	; 1000h -> 1002h = 00,00,01 (PAK ID)  1003h = 00 (SEGMENT)
        LD      BC,100Bh
        LD      (1004h),BC	; 1004h = 100Bh (PACKET START ADDRESS)
        LD      BC,140Dh
        LD      (1008h),BC	; 1008h = 140Dh	(WRITE ADDRESS)
        XOR     A
        LD      (140Bh),A	; 140Bh = 0	(LAST ADDRESS)
        LD      (140Ch),A	; 140Ch = 0
        XOR     A
        OUT     (PSG_ADDR),A 
        OUT     (PSG_DATA),A 	; RESET PSG
        CALL    INIT_PSG
	
LDPKT1: CALL    RCV_PKT		; RECEIVE A PACKET FROM THE HCCA
        JP      NZ,ST_FAIL	; SUCCESSFUL?
	
        XOR     A		; YES
        OUT     (PSG_ADDR),A
        LD      A,(1003h)
        SLA     A
        SLA     A		; TIMES 4
        CPL			; INVERT
        OUT     (PSG_DATA),A	; INCREMENTING TONE PITCH
        LD      HL,(1004h)
        LD      BC,000Bh
        ADD     HL,BC		; HL = (1004h) + 11
        LD      A,(HL)		; OFFSET INTO PACKET RECEIVED
        LD      (100Ah),A	; LAST PACKET FLAG?
        LD      HL,(1006h)
        LD      BC,-18
        ADD     HL,BC		; HL = PAYLOAD COUNT FOR PACKET
        PUSH    HL
        POP     BC		; NUMBER OF BYTES TO COPY
        LD      HL,(140Bh)
        ADD     HL,BC
        LD      (140Bh),HL	; LAST ADDRESS OF DATA
        LD      HL,(1004h)
        LD      DE,0010h
        ADD     HL,DE
        LD      DE,(1008h)	; OFFSET 16 BYTES INTO PAYLOAD
        LDIR			; COPY PAYLOAD
	
        LD      (1008h),DE
        LD      A,(100Ah)
        BIT     4,A		; CHECK IF LAST PACKET
        JR      NZ,LOAD_DONE	; IT IS, WRAP THINGS UP
        LD      HL,1003h
        INC     (HL)		; INCREMENT SEGMENT NUMBER
        JR      LDPKT1 

	; SET FLAG TO INDICATE PROGRAM LOAD COMPLETE
	; TOP TWO BYTES = 5AA5h
LOAD_DONE:
	LD      A,0A5h		; SET LOAD INDICATOR
        LD      (LOAD_FLG),A
        LD      A,5Ah
        LD      (LOAD_FLG+1),A
	
        LD      A,PSG_R7_ENABLE
        OUT     (PSG_ADDR),A    
        LD      A,7Fh           ; ENABLE PORTB
        OUT     (PSG_DATA),A
        JP      BEGIN		; RUN PROGRAM LOADED


	;*** SUBROUTINES ***
	; DECREMENT DE, RETURN 1 IF ZERO, 0 IF IT IS NOT
DEC_AND_TEST_DE:
	DEC     DE
        LD      A,D
        OR      E
        JR      Z,DECTST1	; A = 0
        XOR     A
        DEC     A		; A = -1
DECTST1:  
	INC     A		; 1 = DE IS ZERO, 0 = DE IS NOT ZERO
        RET

        ; RECEIVE A CHARACTER FROM HCCA, AND CHECK THAT IT EQUALS REG C
CHK_HCCA:  
	PUSH    BC
        CALL    WAIT_FOR_HCCA	; WAIT FOR CHARACTER FROM HCCA
        POP     BC
        RET     NZ		; TIMED OUT, EXIT
        IN      A,(HCCA)	; GET CHARACTER
        SUB     C		
        RET     Z		; RETURN 0 IF IT MATCHES
        LD      A,INV_RESP	; ELSE RETURN 2
        RET

        ; OUTPUT A CHARACTER FROM REG C TO HCCA, VERIFY RESPONSE IS ACK (NA_ESCAPE, STAT_GOOD)
OUT_HCCA:  
	LD      A,C
        OUT     (HCCA),A	; SEND CHARACTER
        LD      C,NA_ESCAPE
        CALL    CHK_HCCA	; REPLY IS ESCAPE?
        RET     NZ		; NO, EXIT
        LD      C,STAT_GOOD	; YES, LOOK FOR STATUS GOOD (ACK MESSAGE)
        JR      CHK_HCCA

	; SEND 4 STARTUP BYTES TO ADAPTOR
SEND_START:	
	LD      B,04h
SND_STRT1:  
	LD      A,NA_STARTUP	; STARTUP BYTE
        PUSH    AF		; MOVE STACK TO KEEP IT CLEAN
        OUT     (HCCA),A	; SEND STARTUP
SND_STRT2:  
	POP     AF		; CLEAN UP STACK
        LD      C,NA_ESCAPE
        LD      HL,SND_STRT4
SND_STRT3:
	PUSH    HL		; PUSH RETURN ADDRESS ON STACK
        CALL    CHK_HCCA	; CHECK FOR RESPONSE ACK (NA_ESCAPE, STAT_GOOD) THEN CONFIRMED
        CP      INV_RESP		
        JR      Z,SND_STRT2	; INVALID RESPONSE, TRY AGAIN
        CP      OK_RESP
        RET     Z		; GOT THE RIGHT RESPONSE, RETURN TO ADDRESS PUSHED
        POP     AF		; NOPE, CLEAN UP STACK
        DJNZ    SND_STRT1	; TRY IT AGAIN
        XOR     A
        INC     A		; SET A TO 1 TO INDICATE FAILURE
        RET

SND_STRT4:  
	LD	C,STAT_GOOD
	LD	HL,SND_STRT5	; PUSH RETURN ADDRESS ON STACK
	JR	SND_STRT3
SND_STRT5:
	LD	C,NA_CONFIRM
	POP	HL		; CLEAN UP STACK
	JR	SND_STRT3

	; RECEIVE A PACKET FROM THE HCCA
RCV_PKT:  
	EXX			;* INITIALIZE ALTERNATE REGISTERS FOR CRC
        XOR     A		;* CLEAR A
        DEC     A		;* A = 0FFh
        LD      D,A		;* DE = 0FFFFh
        LD      E,A		;*
        EXX
	
        LD      C,NA_REQUEST
        CALL    OUT_HCCA	; REQUEST PAK #
        RET     NZ
        LD      B,04h
        LD      HL,1003h	
RCVP1:  LD      C,INT_HSND	; SEND SEGMENT AND PAK ID, LOWEST BYTE FIRST
        CALL    SET_INT_MASK
        JR      Z,RCVP1
        LD      A,(HL)
        OUT     (HCCA),A	; SEND BYTE
        DEC     L		; SEND FROM 1003h -> 1000h
        DJNZ    RCVP1
	
        LD      C,NA_CONFIRM
        CALL    CHK_HCCA	; GET CONFIRMATION?
        RET     NZ
	
        LD      B,18h		; NOT SURE WHAT THIS IS ABOUT
        LD      DE,0001h
        LD      HL,0000h
RCVP2:  ADD     HL,DE
        JR      NC,RCVP3
        DJNZ    RCVP3
	
        LD      HL,MSG_WRONG	; SOMETHING WENT HORRIBLY WRONG
        CALL    DSP_MSG
        POP     BC
        JP      LOAD_PKT

RCVP3:  LD      C,INT_HRCV	; WAIT FOR BYTE FROM HCCA
        CALL    SET_INT_MASK
        JR      Z,RCVP2
        IN      A,(HCCA)
        SUB     NA_AUTH		; RECEIVE AUTHORIZATION?
        RET     NZ		; NO, RETURN
	
        LD      A,NA_ESCAPE	; YES
        OUT     (HCCA),A	; SEND ESCAPE (START OF ACK)
RCVP4:  LD      C,INT_HSND
        CALL    SET_INT_MASK
        JR      Z,RCVP4
	
        LD      C,INT_HRCV
        CALL    SET_INT_MASK	; CHANGE TO RECEIVE INTERRUPT
	
        LD      HL,(1004h)	; GET PACKET START ADDRESS
        LD      BC,0000h
        RES     0,E		; DE = 0000h
        LD      A,STAT_GOOD
        OUT     (HCCA),A	; SEND GOOD (END OF ACK)
RCVP5:  PUSH    DE
        LD      DE,0FFFFh
RCVP6:  LD      A,PSG_R15_STATUS
        OUT     (PSG_ADDR),A
        IN      A,(PSG_DATA)
        BIT     0,A		; BYTE RECEIVED?
        JR      NZ,RCVP7
        CALL    DEC_AND_TEST_DE
        JR      NZ,RCVP11	; TIMED OUT?
        JR      RCVP6

RCVP7:  POP     DE
        IN      A,(HCCA)	; GET BYTE FROM HCCA
        CP      NA_ESCAPE
        JR      NZ,RCVP9	; GOT ESCAPE CODE?
        BIT     0,E
        JR      Z,RCVP8		; ALREADY CLEARED?
        RES     0,E		; NO, CLEAR FLAG
        LD      (HL),A		; SAVE BYTE
        CALL    CALC_CRC
        INC     HL		; NEXT ADDRESS
        INC     BC		; INCREMENT COUNT
        JR      RCVP5

RCVP8:  SET     0,E		; YES, SET ESCAPE RECEIVED FLAG
        JR      RCVP5

RCVP9:  BIT     0,E		; ESCAPE FLAG SET?
        JR      NZ,RCVP10
        LD      (HL),A		; SAVE BYTE
        CALL    CALC_CRC
        INC     HL		; NEXT ADDRESS
        INC     BC		; INCREMENT COUNT
        JR      RCVP5 

RCVP10: LD      (1006h),BC	; ESCAPE RECEIVED PRIOR
        CP      NA_DONE
        JP      NZ,RCV_PKT	; GET DONE?  (THEN FINISHED WITH PACKET)
        EXX			;* YES
        LD      A,E		;*
        CP      0Fh		;* CHECK FOR FINAL CRC
        JP      NZ,RCV_PKT	;*
        LD      A,D		;*
        CP      1Dh		;*
        JP      NZ,RCV_PKT	;*
        XOR     A		;* CLEAR A
        RET			;* RETURN (WHY IS THERE NOT ANOTHER EXX?)

RCVP11: POP     DE		; TIME OUT, EXIT FROM PACKET RECEIVE
        POP     DE
        JP      ST_FAIL

        ; SEND HIGH OR LOW TONE RESPONSE
	; AND DISPLAY MESSAGE
RESP_BEEP:	
	LD      C,50h           ; LOW TONE
        LD      HL,BOOT_FLG
        BIT     1,(HL)		; SEND SHORT BEEP?
        JR      Z,RBEEP1        ; YES
        LD      C,90h		; NO, HIGH TONE
RBEEP1: LD      DE,0CC00h
        CALL    BEEP		; SEND TONE C FOR DELAY DE
        RES     1,(HL)		; CLEAR VAR A BIT 1
        LD      HL,8000h
        LD      DE,0001h
RBEEP2: ADD     HL,DE
        JR      NC,RBEEP2       ; PAUSE
	
        LD      A,(TEST_NUM)	; GET TEST NUMBER
        LD      C,A		; SAVE IN C
        SUB     05h		; HCCA TEST?
        JR      Z,RBEEP3        ; SKIP IF HCCA TEST
        ADD     A,06h		; N - 5 + 6 = N + 1
RBEEP3: LD      (TEST_NUM),A	; INC TEST NUMBER IF < 5, ELSE CLEAR IT
        LD      A,(BOOT_FLG)	; CHECK COLD BOOT FLAG
        BIT     0,A
        JP      NZ,SHOW_MSG	; IF WARM BOOT, SKIP SETTING LEDS
        CALL    SET_LED		; SET LEDS 
        OUT     (CTRL_REG),A
        LD      (CTRL_BITS),A
        JP      SHOW_MSG	; SHOW MESSAGE FOR TEST NUMBER

        ; FLASH ALERT LED
	; TO INDICATE TEST FAILURE AND INCREMENT TEST FAILURE COUNTS
FLASH_ALERT:	
	PUSH    AF
        PUSH    DE
        PUSH    HL
        LD      A,(BOOT_FLG)	; CHECK COLD BOOT FLAG
        BIT     0,A
        JR      NZ,FLSH1	; SKIP SETTING LEDS IF WARM BOOT
        CALL    SET_LED
        OR      ALERT		; SET ALERT LED
        OUT     (CTRL_REG),A
        LD      (CTRL_BITS),A	; SET LED(S)
	
FLSH1:  LD      A,03h
        LD      (BOOT_FLG),A	; UPDATE COLD BOOT FLAG TO INDICATE 1ST BOOT HAS OCCURRED
        LD      HL,ERR_CNT	; POINT TO ERROR COUNTS
        LD      D,00h
        LD      A,(TEST_NUM)	; GET TEST NUMBER
        LD      E,A
        ADD     HL,DE		; INDEX INTO ERR_CNT VARIABLES
        INC     (HL)		; INCREMENT THE NUMBER OF ERRORS FOR THAT TEST
        JR      NZ,FLSH2
        DEC     (HL)		; KEEP FROM WRAPPING AROUND TO ZERO
FLSH2:  POP     HL
        POP     DE
        POP     AF
        RET

        ; SET THE LEDS BASED ON THE VALUE IN TEST_NUM
SET_LED:
	LD      HL,LED_TBL
        LD      A,(TEST_NUM)	; ERROR CODE?
        LD      E,A
        LD      D,00h
        ADD     HL,DE
        LD      A,(HL)		; GET LED ENCODING FROM TABLE
        RET

LED_TBL:
	DEFB    2Ah             ; 0: PAUSE AND CHECK
        DEFB    0Ah		; 1: CHECK
        DEFB    2Ah             ; 2: PAUSE AND CHECK
        DEFB    0Ah		; 3: CHECK
        DEFB    22h             ; 4: PAUSE
        DEFB    02h		; 5: ALL LEDS OFF

;--------------------------------------------
; Load character patterns in to pattern table
; in VRAM, including custom characters needed
; to display logo
;--------------------------------------------
        ; LOAD VDP WITH FONT DATA
LOAD_VDP_TABLE:
	XOR     A
        OUT     (VDP_ADDR),A
        LD      A,41h           ; WRITE TO VRAM STARTING AT 0100
        OUT     (VDP_ADDR),A
        LD      C,VDP_DATA		
        LD      D,04h		; 5 PAGES OF DATA
        LD      HL,VDP_TABLE
LD_VDP_LP:  
	LD      B,0FFh		; 256 BYTES AT A TIME
        OTIR
        DEC     D
        JR      NZ,LD_VDP_LP
	
	; INITIALIZE VIDEO MEMORY TO SPACES
CLEAR_SCREEN:	
        XOR     A
        OUT     (VDP_ADDR),A
        LD      A,48h           ; WRITE TO VRAM STARTING AT 0800
        OUT     (VDP_ADDR),A
        LD      A,20h           ; BLANK
        LD      HL,VRAM
        LD      DE,0001h
CLEAR1:  
	OUT     (VDP_DATA),A
        ADD     HL,DE
        JR      NC,CLEAR1
	
	; DISPLAY THE NABU LOGO
DSP_LOGO:
        LD      B,07h		; NUMBER OF ROWS
        LD      HL,LOGO_DATA	
        LD      DE,000Fh
LOGO1:  PUSH    BC
        PUSH    HL
        PUSH    DE
        CALL    DSP_MSG		; SEND ROW
        POP     DE
        POP     HL
        POP     BC
        ADD     HL,DE
        DJNZ    LOGO1
        RET

        
        ; ENABLE PORT B OF PSG
PSG_PORTB:
	LD      A,PSG_R7_ENABLE
        OUT     (PSG_ADDR),A 
        LD      A,7Fh
        OUT     (PSG_DATA),A  
        RET

        ; WRITE TO KEYBOARD (8251) CONTROL PORT
KB_CTRL:  
	OUT     (KB_STAT),A
        NOP			; PAUSE
        NOP
        NOP
        NOP
        NOP
        RET

	; CHECK FOR PROGRAM LOADED FLAG	(5AA5h AT LOAD_FLAG)
CHK_LOAD:
	LD      HL,LOAD_FLG
        LD      A,(HL)
        SUB     0A5h
        RET     NZ
        INC     HL
        LD      A,(HL)
        SUB     5Ah 		; 0 = PROGRAM IS LOADED
        RET

        ; SHOW THE PART NUMBER
SHOW_PN:  
	LD      C,05h
        PUSH    BC
SHOW1:  CALL    SHOW_MSG	; SHOW ADAPTOR ERROR IF IT OCCURRED
        POP     BC
        DEC     C
        JP      M,SHOW2		; SHOW_MSG RETURNED ZERO?
        PUSH    BC		; NO, KEEP GOING
        JR      SHOW1

SHOW2:  LD      A,(POST_FLG)
        OR      A
        RET     Z		; ERROR?
        LD      HL,(ROM_STR)	; YES, SHOW ROM PART NUMBER
        LD      (DSP_STR),HL	; 
        LD      A,(ROM_STR+2)	; 
        LD      (DSP_STR+2),A	; 
        LD      DE,(ROMPN)	; MOVE ROM PART NUMBER TO DISPLAY DIGIT STRING
        LD      (DSP_STR+3),DE	; DSP_STR = 02 5A 03 36 30
        LD      HL,DSP_STR
        JP      DSP_MSG		; DISPLAY ROM PART NUMBER ON SCREEN

        ; SEND STRING TO VDP
DSP_MSG:  
	LD      B,(HL)		; GET COUNT OF CHARACTERS IN MESSAGE
        INC     HL
        LD      A,(HL)
        OUT     (VDP_ADDR),A	; LOWER BYTE OF VDP RAM ADDRESS
        INC     HL
        LD      A,(HL)
        ADD     A,48h           ; 
        OUT     (VDP_ADDR),A	; UPPER BYTE OF VDP RAM ADDRESS
        INC     HL
        LD      C,VDP_DATA
        OTIR			; WRITE DATA TO VDP RAM
        RET

	; GET CHANNEL HEX CODE FROM USER - 4 DIGITS AND <CR> OR <YES>
	; DE = VDP ADDRESS
	; HL = STORE ADDRESS
	; B = NUMBER OF DIGITS + 1
GET_HEX:  
	LD      B,05h		; NUMBER OF DIGITS TO RECEIVE + 1
GETDIG:  
	IN      A,(KB_STAT)
        BIT     1,A
        JR      Z,GETDIG	; WAIT FOR KEY
        IN      A,(KB_DATA)	; GET KEY
        PUSH    AF		; SAVE A COPY ON THE STACK
        SUB     30h
        JR      C,GETHX5        ; < '0' CONTROL CHAR?
        CP      0Ah
        JR      C,GETHX2        ; > '9'
        SUB     07h		; YES, CONVERT TO HEX A-F
        CP      0Ah
        JR      C,GETHX5        ; NOT A-F?
        CP      10h		
        JR      C,GETHX2        ; > F?
        SUB     20h             ; YES, LOWER CASE?
        CP      0Ah
        JR      C,GETHX5        ; a-f?
        CP      10h
        JR      NC,GETHX5       ; NO, CHECK FOR OTHER CHARACTERS
GETHX2: LD      C,A		; SAVE IN C
        LD      A,B
        OR      A
        JR      NZ,GETHX3       ; DONE?
        POP     AF		; CLEAN UP STACK
        JR      GETDIG          ; GET ANOTHER KEY

GETHX3: LD      A,E		; VALID KEY?
        OUT     (VDP_ADDR),A
        LD      A,D
        OR      48h
        OUT     (VDP_ADDR),A
        POP     AF
        CP      5Bh             ; LOWER CASE ?
        JR      C,GETHX4	
        SUB     20h             ; YES, CHANGE TO UPPER CASE
GETHX4: OUT     (VDP_DATA),A	; ECHO CHARACTER TO SCREEN
        LD      (HL),C		; STORE IN MEMORY
        INC     HL		; INCREMENT ADDRESSES
        INC     DE
        DEC     B		; DECREMENT COUNT
        JR      GETDIG		; GET ANOTHER CHARACTER

GETHX5: POP     AF
        CP      0Dh
        JR      Z,GETHX6	; <CR>?
        CP      0E7h
        JR      NZ,DEL_CHAR	; <YES> KEY?
GETHX6: LD      A,B
        OR      A
        JR      NZ,GETDIG	; DONE ?
        RET


	; DELETE CHARACTER ON SCREEN AND BACK UP SCREEN POSITION
DEL_CHAR:  
	CP      7Fh             ; DELETE KEY?
        JR      Z,DELCH1        
        CP      0E1h		; BACK ARROW?
        JR      NZ,GETDIG	; NO, IGNORE CHARACTER AND GET ANOTHER
DELCH1: LD      A,B		; GET CHARACTER COUNT
        CP      05h		; AT START?
        JR      Z,GETDIG        ; YES, IGNORE
        DEC     DE		; NO, BACK UP POINTERS
        DEC     HL
        INC     B
        LD      A,E
        OUT     (VDP_ADDR),A
        LD      A,D
        OR      48h             
        OUT     (VDP_ADDR),A	; POINT TO CHARACTER ON SCREEN
        LD      A,20h           
        OUT     (VDP_DATA),A	; AND BLANK IT OUT
        JR      GETDIG          ; CONTINUE ON

	; VALID CHANNEL CODE ACKNOWLEDGEMENT
EXP_CONF:  
	LD      C,NA_CONFIRM
        JP      CHK_HCCA	; EXPECT CONFIRMATION


	; OUTPUT HEX CODE TO SCREEN OR ERROR MESSAGES
	; C = ERROR COUNT OFFSET
SHOW_MSG:  
	LD      A,(POST_FLG)
        OR      A
        JR      Z,CHK_ERR	; POST ERROR?
	
        LD      A,02h		; YES, 
        LD      (DSP_STR),A	; 2 CHARACTERS TO DISPLAY
        LD      DE,40		; DE = 40 COLUMNS PER ROW
        LD      B,C
        INC     B
        LD      HL,012Bh	; MIDDLE COLUMN OF LINE 8
SHMS1:  ADD     HL,DE
        DJNZ    SHMS1		; OFFSET ROW
        LD      (DSP_STR+1),HL	; ADDRESS IN VAR_E,VAR_F
        LD      HL,ERR_CNT
        ADD     HL,BC		; BC = ROW OFFSET
        LD      DE,DSP_STR+3	; POINT TO STRING MEMORY START
        CALL    TOASCII		; CONVERT TO ASCII HEX
        LD      HL,DSP_STR	; STRING TO DISPLAY AT DSP_STR
        JP      DSP_MSG

CHK_ERR:  			; CHECK ERROR COUNTS
	LD      HL,ERR_CNT
        LD      B,00h
        ADD     HL,BC
        LD      A,(HL)		; GET ERROR COUNT
        OR      A
        RET     Z		; CHECK IF ERROR COUNT IS > 0
	
        LD      HL,ERR_MSGS
        LD      DE,0013h	; LENGTH OF EACH ERROR MESSAGE
        INC     C
CHKER1: DEC     C
        JP      Z,DSP_MSG	; SHOW ERROR MESSAGE
        ADD     HL,DE		; POINT TO NEXT ERROR MESSAGE
        JR      CHKER1


        ; CONVERT BYTE IN (HL) TO ASCII PAIR, STORE IN (DE)
TOASCII:
	LD      A,(HL)
        SRL     A
        SRL     A
        SRL     A
        SRL     A
        CALL    HEXTOASC
        INC     DE
        LD      A,(HL)
        AND     0Fh

        ; CONVERT 4 BIT NIBBLE TO ASCII
HEXTOASC:  
	ADD     A,'0'
        CP      3Ah
        JR      C,HEX1		; IF > 9, CONVERT TO A-F
        ADD     A,07h
HEX1:   LD      (DE),A		; STORE IN (DE)
        RET


        ; ENABLE INTERRUPT MASK FROM REGISTER C
SET_INT_MASK:
	LD      A,PSG_R14_INT_MASK
        OUT     (PSG_ADDR),A	     ; ENABLE OUTPUT REGISTER
        LD      A,C
        OUT     (PSG_DATA),A 	     ; SET MASK
        LD      A,PSG_R15_STATUS
        OUT     (PSG_ADDR),A
        IN      A,(PSG_DATA)         ; READ INTERRUPT STATUS
        BIT     0,A
        RET


	; USE ALTERNATE REGISTER SET TO CALCULATE THE CRC
	; DE' IS THE CRC, BC' IS THE INDEX TO THE TABLE
	; NOTE: XOR CRC WITH 0FFFFh TO GET THE 2 BYTE CRC AT END OF PACKET
CALC_CRC:  
	PUSH    AF
        EXX			;* SWAP BC, DE AND HL WITH ALTERNATES
        XOR     D		;* ACC = ACC XOR D'
        LD      C,A		;*
        LD      B,00h		;* BC' = ACC
        SLA     C		;*
        RL      B		;* BC' = BC' * 2
        LD      D,E		;* D' = E'
        LD      IY,CRC_TBL	;* POINT TO CRC_TBL
        ADD     IY,BC		;* INDEX INTO TABLE BY BC'*2
        LD      A,(IY+00h)      ;* GET FIRST BYTE INTO E
        LD      E,A		;*
        LD      A,(IY+01h)	;*
        XOR     D		;* XOR SECOND BYTE WITH D'
        LD      D,A		;* AND PUT BACK IN D'
        EXX
        POP     AF
        RET


        ; WAIT FOR CHARACTER FROM HCCA
WAIT_FOR_HCCA:
	LD      DE,0FFFFh
WAIT1:  CALL    DEC_AND_TEST_DE		; TIMEOUT COUNTER
        RET     NZ
        LD      C,INT_HRCV
        CALL    SET_INT_MASK		; ENABLE HCCA RECEIVE INTERRUPT
        JR      Z,WAIT1			; EXIT WHEN INTERRUPT OCCURS
        XOR     A
        RET

	; INITIALIZE PSG
INIT_PSG:
	LD      A,PSG_R7_ENABLE
        OUT     (PSG_ADDR),A 
        LD      A,7Eh           
        OUT     (PSG_DATA),A         ; ENABLE PORT B
        LD      A,08h
        OUT     (PSG_ADDR),A
        LD      A,06h
        OUT     (PSG_DATA),A         ; AMPLITUDE 6
        LD      A,01h
        OUT     (PSG_ADDR),A         ; 
        DEC     A
        OUT     (PSG_DATA),A         ; TONE A UPPER BYTE = 0
        RET


        ; SEND TONE IN C OUT PSG FOR DURATION IN DE
BEEP:
	PUSH    AF
        PUSH    HL
        PUSH    BC
        PUSH    DE
        CALL    INIT_PSG
        XOR     A
        OUT     (PSG_ADDR),A
        LD      A,C
        OUT     (PSG_DATA),A	; SET TONEA  = C
	
        LD      HL,0001h
        LD      B,04h
        EX      DE,HL
BEEP1:  ADD     HL,DE
        JR      NC,BEEP1 
        POP     HL
        PUSH    HL
        DJNZ    BEEP1
        LD      A,PSG_R7_ENABLE
        OUT     (PSG_ADDR),A
        LD      A,7Fh
        OUT     (PSG_DATA),A	; PORT B ENABLE, TONE OFF
        POP     DE
        POP     BC
        POP     HL
        POP     AF
        RET

VDP_INIT_TBL:
	DEFB    0F5h		; REG 7 = WHITE ON LIGHT BLUE
        DEFB    00h		; REG 6 = SPRITE BASE ADDRESS = 0000h
        DEFB    00h		; REG 5 = SPRITE ATTR ADDRESS = 0000h
        DEFB    00h		; REG 4 = PATTERN GENERATOR ADDRESS = 0000h
        DEFB    00h		; REG 3 = COLOR TABLE BASE ADDRESS = 0000h
        DEFB    02h		; REG 2 = NAME TABLE ADDRESS = 0800h
        DEFB    0D0h		; REG 1 = 4K RAM, ENABLE DISPLAY, ENABLE INT, TEXT MODE
        DEFB    00h		; REG 0 = TEXT MODE
	
; MESSAGE STRINGS ARE STORED AS:
; CHARACTER_COUNT, VDP RAM LSB, VDP RAM MSB, STRING
	
MSG_CHAN:	
        DEFB	21h,0ABh,02h
        DEFB    "PLEASE TYPE IN CHANNEL CODE      "
MSG_WRONG:
	DEFB	34h,21h,03h
	DEFB	"SEE "
        DEFB    22h             ; '"'
        DEFB    "IF SOMETHING GOES WRONG"
        DEFB    22h             ; '"'
        DEFB    " IN        OWNERS GUIDE"
MSG_WAIT:	
        DEFB    0Bh,21h,03h
        DEFB    "PLEASE WAIT"
	
ERR_MSGS:	
MSG_ROM_FAIL:			; ERROR 0
	DEFB	10h,49h,01h
        DEFB    "ROM FAILURE     "
MSG_VID_FAIL:			; ERROR 1
        DEFB    10h,71h,01h
        DEFB    "VIDEO FAILURE   "
MSG_RAM_FAIL:			; ERROR 2
        DEFB    10h,99h,01h
        DEFB    "RAM FAILURE     "
MSG_PSG_FAIL:			; ERROR 3
        DEFB    10h,0C1h,01h
        DEFB    "SOUND FAILURE   "
MSG_KB_FAIL:			; ERROR 4
        DEFB    10h,0E9h,01h
        DEFB    "KEYBOARD FAILURE"
MSG_ADP_FAIL:			; ERROR 5
        DEFB    10h,11h,02h
        DEFB    "ADAPTOR FAILURE "
MSG_RETYPE:
	DEFB	21h,0ABh,02h
        DEFB    "RE-TYPE CHANNEL CODE             "

; COUNT AND ADDRESS FOR ROM PART NUMBER ON SCREEN
ROM_STR:  
	DEFB    02h,5Ah,03h
; LOGO DATA USING RE-MAPPED CHARACTERS
LOGO_DATA:
	DEFB    0Ch,0Dh,00h
        DEFB    "[\\\\\\\\\\]"		; ROW 1
        DEFB	0Ch,35h,00h
        DEFB    "^^^^^^^^^^^^"		; ROW 2
        DEFB    0Ch,5Dh,00h
        DEFB    "_`abcdefghij"		; ROW 3
        DEFB    0Ch,85h,00h
        DEFB    "klmnopqrstuv"		; ROW 4
        DEFB    0Ch,0ADh,00h
        DEFB    "wxyz{|}~"		; ROW 5
        DEFB    7Fh,80h,81h,82h
        DEFB    0Ch,0D5h,00h
        DEFB    83h,83h,83h,83h,83h,83h,83h,83h,83h,83h,83h,83h ; ROW 6
        DEFB    0Ch,0FDh,00h
        DEFB    84h,85h,85h,85h,85h,85h,85h,85h,85h,85h,85h,86h
	
	; VDP CHARACTER TABLE STARTING AT 0100h
VDP_TABLE:
        DEFB    00h,00h,00h,00h,00h,00h,00h,00h ; SPACE 20h
        DEFB    10h,10h,10h,10h,10h,00h,10h,00h ; !
        DEFB    28h,28h,00h,00h,00h,00h,00h,00h ; "
	DEFB    28h,28h,7Ch,28h,7Ch,28h,28h,00h ; #
        DEFB    38h,54h,50h,38h,14h,54h,38h,00h ; $
        DEFB    60h,64h,08h,10h,20h,6Ch,0Ch,00h ; %
        DEFB    10h,28h,28h,30h,50h,4Ch,7Ch,00h ; &
        DEFB    30h,30h,10h,60h,00h,00h,00h,00h ; '
        DEFB    10h,20h,40h,40h,40h,20h,10h,00h ; (
        DEFB    40h,20h,10h,10h,10h,20h,40h,00h ; )
        DEFB    00h,54h,38h,7Ch,38h,54h,10h,00h ; *
        DEFB    00h,00h,10h,10h,7Ch,10h,10h,00h ; +
        DEFB    00h,00h,00h,00h,30h,30h,10h,60h ; ,
        DEFB    00h,00h,00h,00h,38h,00h,00h,00h ; -
        DEFB    00h,00h,00h,00h,00h,18h,18h,00h ; .
        DEFB    04h,04h,08h,18h,30h,20h,40h,00h ; /
        DEFB    38h,44h,4Ch,54h,64h,44h,38h,00h ; 0
        DEFB    10h,30h,10h,10h,10h,10h,38h,00h ; 1
        DEFB    30h,48h,48h,18h,30h,20h,78h,00h ; 2
        DEFB    30h,48h,08h,10h,08h,48h,30h,00h ; 3
        DEFB    10h,30h,30h,50h,50h,78h,10h,00h ; 4
        DEFB    78h,40h,50h,68h,08h,48h,30h,00h ; 5
        DEFB    30h,28h,40h,70h,68h,48h,30h,00h ; 6
        DEFB    78h,48h,08h,10h,30h,20h,20h,00h ; 7
        DEFB    30h,48h,48h,30h,48h,48h,30h,00h ; 8
        DEFB    30h,48h,48h,38h,08h,50h,30h,00h ; 9
        DEFB    00h,00h,30h,30h,00h,30h,30h,00h ; :
        DEFB    00h,00h,30h,30h,00h,30h,10h,40h ; ;
        DEFB    08h,10h,20h,40h,20h,10h,08h,00h ; <
        DEFB    00h,00h,00h,78h,00h,78h,00h,00h ; =
        DEFB    40h,20h,10h,08h,10h,20h,40h,00h ; >
        DEFB    30h,48h,48h,10h,20h,20h,00h,20h ; ?
        DEFB    40h,38h,04h,38h,08h,38h,48h,34h ; @
        DEFB    10h,28h,44h,44h,7Ch,44h,44h,00h ; A
        DEFB    78h,44h,44h,78h,44h,44h,78h,00h ; B
        DEFB    38h,44h,40h,40h,40h,44h,38h,00h ; C
        DEFB    70h,48h,44h,44h,44h,48h,70h,00h ; D
        DEFB    7Ch,40h,40h,70h,40h,40h,7Ch,00h ; E
        DEFB    7Ch,40h,40h,70h,40h,40h,40h,00h ; F
	DEFB    38h,44h,44h,40h,5Ch,44h,3Ch,00h ; G
        DEFB    44h,44h,44h,7Ch,44h,44h,44h,00h ; H
        DEFB    38h,10h,10h,10h,10h,10h,38h,00h ; I
        DEFB    1Ch,08h,08h,08h,48h,48h,30h,00h ; J
        DEFB    48h,48h,50h,70h,50h,48h,4Ch,00h ; K
        DEFB    40h,40h,40h,40h,40h,40h,7Ch,00h ; L
        DEFB    6Ch,54h,54h,44h,44h,44h,44h,00h ; M
        DEFB    44h,64h,64h,54h,54h,4Ch,4Ch,00h ; N
        DEFB    38h,44h,44h,44h,44h,44h,38h,00h ; O
        DEFB    78h,44h,44h,78h,40h,40h,40h,00h ; P
        DEFB    38h,44h,44h,44h,54h,48h,34h,00h ; Q
        DEFB    78h,48h,48h,78h,50h,48h,4Ch,00h ; R
        DEFB    38h,44h,40h,38h,04h,44h,38h,00h ; S
        DEFB    7Ch,10h,10h,10h,10h,10h,10h,00h ; T
        DEFB    44h,44h,44h,44h,44h,44h,38h,00h ; U
        DEFB    44h,44h,44h,28h,28h,28h,10h,00h ; V
        DEFB    44h,44h,44h,44h,54h,54h,28h,00h ; W
        DEFB    44h,44h,28h,10h,28h,44h,44h,00h ; X
        DEFB    44h,44h,44h,38h,10h,10h,10h,00h ; Y
        DEFB    7Ch,0Ch,18h,10h,20h,60h,7Ch,00h ; Z
; 
; 
; REMAPPED CHARACTERS FOR NABU logo
        DEFB    00h,00h,00h,3Ch,7Ch,00h,00h,0FCh ; [ 5Bh     ; top left corner
        DEFB    0FCh,00h,00h,0FCh,0FCh,00h,00h,0FCh ; \ 5Ch     ; top middle 10
        DEFB    00h,00h,00h,0F0h,0F8h,00h,00h,0FCh ; ] 5Dh     ; top right corner
        DEFB    0FCh,0FCh,00h,0FCh,0FCh,0FCh,0FCh,00h ; ^ 5Eh     ; 2nd row
        DEFB    0FCh,0FCh,0F0h,0F0h,0F0h,0F0h,0F0h,0F0h ; _ 5Fh     ; 1st row of NABU
        DEFB    0FCh,0FCh,0Ch,0Ch,0Ch,04h,04h,04h ; `
        DEFB    0FCh,0FCh,0E0h,0E0h,0E0h,0E0h,0E0h,0E0h ; a
        DEFB    0FCh,0FCh,7Ch,7Ch,7Ch,7Ch,7Ch,7Ch ; b
        DEFB    0FCh,0FCh,80h,80h,80h,18h,18h,18h ; c
        DEFB    0FCh,0FCh,7Ch,7Ch,7Ch,3Ch,3Ch,3Ch ; d
        DEFB    0FCh,0FCh,80h,80h,80h,80h,80h,80h ; e
        DEFB    0FCh,0FCh,00h,00h,00h,60h,70h,70h ; f
        DEFB    0FCh,0FCh,78h,38h,18h,08h,08h,08h ; g
        DEFB    0FCh,0FCh,04h,04h,04h,04h,04h,04h ; h
        DEFB    0FCh,0FCh,0C0h,0C0h,0C0h,0C0h,0C0h,0C0h ; i
        DEFB    0FCh,0FCh,3Ch,3Ch,3Ch,3Ch,3Ch,3Ch ; j
        DEFB    0F0h,0F0h,0F0h,0F0h,0F0h,0F0h,0F0h,0F0h ; k 6Bh     ;2nd row of NABU
        DEFB    00h,00h,20h,30h,30h,38h,38h,3Ch ; l
        DEFB    0E0h,0E0h,0E0h,60h,60h,60h,20h,20h ; m
        DEFB    78h,78h,78h,70h,70h,70h,60h,60h ; n
        DEFB    18h,3Ch,3Ch,3Ch,3Ch,00h,00h,00h ; o
        DEFB    1Ch,1Ch,1Ch,0Ch,0Ch,0Ch,04h,04h ; p
        DEFB    80h,80h,80h,80h,80h,80h,80h,80h ; q
        DEFB    60h,60h,00h,04h,00h,60h,60h,70h ; r
        DEFB    18h,38h,78h,0F8h,78h,38h,18h,08h ; s
        DEFB    04h,04h,04h,04h,04h,04h,04h,04h ; t
        DEFB    0C0h,0C0h,0C0h,0C0h,0C0h,0C0h,0C0h,0C0h ; u
        DEFB    3Ch,3Ch,3Ch,3Ch,3Ch,3Ch,3Ch,3Ch ; v
        DEFB    0F0h,0F0h,0F0h,0F0h,0F0h,0FCh,0FCh,00h ; w 77h     ; 3rd row of NABU
        DEFB    3Ch,3Ch,3Ch,3Ch,3Ch,0FCh,0FCh,00h ; x
        DEFB    00h,00h,80h,80h,80h,0FCh,0FCh,00h ; y
        DEFB    60h,40h,40h,40h,40h,0FCh,0FCh,00h ; z
        DEFB    00h,18h,18h,3Ch,3Ch,0FCh,0FCh,00h ; {
        DEFB    04h,00h,00h,00h,00h,0FCh,0FCh,00h ; |
        DEFB    80h,80h,80h,80h,80h,0FCh,0FCh,00h ; }
        DEFB    70h,60h,00h,00h,00h,0FCh,0FCh,00h ; ~
        DEFB    08h,08h,1Ch,3Ch,7Ch,0FCh,0FCh,00h ; (DEL) 7Fh 127d
        DEFB    04h,00h,00h,80h,0C0h,0FCh,0FCh,00h ; 
        DEFB    0C0h,80h,00h,00h,04h,0FCh,0FCh,00h ; 
        DEFB    3Ch,3Ch,7Ch,0FCh,0FCh,0FCh,0FCh,00h ; 
        DEFB    0FCh,0FCh,0FCh,0FCh,00h,0FCh,0FCh,0FCh ; 83h       ; 6th row
        DEFB    00h,00h,7Ch,3Ch,00h,00h,00h,00h ; 84h       ; bottom left corner
        DEFB    00h,00h,0FCh,0FCh,00h,00h,0FCh,00h ; 85h       ; bottom middle 10
        DEFB    00h,00h,0F8h,0F0h,00h,00h,00h,00h ; 86h       ; bottom right corner
; 
; 
; LOOK UP TABLE FOR PACKET CRC
CRC_TBL:
        DEFB    00h,00h,21h,10h,42h,20h,63h,30h 	
        DEFB    84h,40h,0A5h,50h,0C6h,60h,0E7h,70h 	
        DEFB    08h,81h,29h,91h,4Ah,0A1h,6Bh,0B1h
        DEFB    8Ch,0C1h,0ADh,0D1h,0CEh,0E1h,0EFh,0F1h
        DEFB    31h,12h,10h,02h,73h,32h,52h,22h 
        DEFB    0B5h,52h,94h,42h,0F7h,72h,0D6h,62h
        DEFB    39h,93h,18h,83h,7Bh,0B3h,5Ah,0A3h
        DEFB    0BDh,0D3h,9Ch,0C3h,0FFh,0F3h,0DEh,0E3h
        DEFB    62h,24h,43h,34h,20h,04h,01h,14h
        DEFB    0E6h,64h,0C7h,74h,0A4h,44h,85h,54h 
        DEFB    6Ah,0A5h,4Bh,0B5h,28h,85h,09h,95h 
        DEFB    0EEh,0E5h,0CFh,0F5h,0ACh,0C5h,8Dh,0D5h 
        DEFB    53h,36h,72h,26h,11h,16h,30h,06h 
        DEFB    0D7h,76h,0F6h,66h,95h,56h,0B4h,46h 
        DEFB    5Bh,0B7h,7Ah,0A7h,19h,97h,38h,87h 
        DEFB    0DFh,0F7h,0FEh,0E7h,9Dh,0D7h,0BCh,0C7h 
        DEFB    0C4h,48h,0E5h,58h,86h,68h,0A7h,78h 
        DEFB    40h,08h,61h,18h,02h,28h,23h,38h 
        DEFB    0CCh,0C9h,0EDh,0D9h,8Eh,0E9h,0AFh,0F9h 
        DEFB    48h,89h,69h,99h,0Ah,0A9h,2Bh,0B9h 
        DEFB    0F5h,5Ah,0D4h,4Ah,0B7h,7Ah,96h,6Ah 
        DEFB    71h,1Ah,50h,0Ah,33h,3Ah,12h,2Ah 
        DEFB    0FDh,0DBh,0DCh,0CBh,0BFh,0FBh,9Eh,0EBh 
        DEFB    79h,9Bh,58h,8Bh,3Bh,0BBh,1Ah,0ABh 
        DEFB    0A6h,6Ch,87h,7Ch,0E4h,4Ch,0C5h,5Ch 
        DEFB    22h,2Ch,03h,3Ch,60h,0Ch,41h,1Ch 
        DEFB    0AEh,0EDh,8Fh,0FDh,0ECh,0CDh,0CDh,0DDh 
        DEFB    2Ah,0ADh,0Bh,0BDh,68h,8Dh,49h,9Dh 
        DEFB    97h,7Eh,0B6h,6Eh,0D5h,5Eh,0F4h,4Eh 
        DEFB    13h,3Eh,32h,2Eh,51h,1Eh,70h,0Eh 
        DEFB    9Fh,0FFh,0BEh,0EFh,0DDh,0DFh,0FCh,0CFh 
        DEFB    1Bh,0BFh,3Ah,0AFh,59h,9Fh,78h,8Fh 
        DEFB    88h,91h,0A9h,81h,0CAh,0B1h,0EBh,0A1h 
        DEFB    0Ch,0D1h,2Dh,0C1h,4Eh,0F1h,6Fh,0E1h 
        DEFB    80h,10h,0A1h,00h,0C2h,30h,0E3h,20h 
        DEFB    04h,50h,25h,40h,46h,70h,67h,60h 
        DEFB    0B9h,83h,98h,93h,0FBh,0A3h,0DAh,0B3h 
        DEFB    3Dh,0C3h,1Ch,0D3h,7Fh,0E3h,5Eh,0F3h 
        DEFB    0B1h,02h,90h,12h,0F3h,22h,0D2h,32h 
        DEFB    35h,42h,14h,52h,77h,62h,56h,72h 
        DEFB    0EAh,0B5h,0CBh,0A5h,0A8h,95h,89h,85h 
        DEFB    6Eh,0F5h,4Fh,0E5h,2Ch,0D5h,0Dh,0C5h 
        DEFB    0E2h,34h,0C3h,24h,0A0h,14h,81h,04h 
        DEFB    66h,74h,47h,64h,24h,54h,05h,44h 
        DEFB    0DBh,0A7h,0FAh,0B7h,99h,87h,0B8h,97h 
        DEFB    5Fh,0E7h,7Eh,0F7h,1Dh,0C7h,3Ch,0D7h 
        DEFB    0D3h,26h,0F2h,36h,91h,06h,0B0h,16h 
        DEFB    57h,66h,76h,76h,15h,46h,34h,56h 
        DEFB    4Ch,0D9h,6Dh,0C9h,0Eh,0F9h,2Fh,0E9h 
        DEFB    0C8h,99h,0E9h,89h,8Ah,0B9h,0ABh,0A9h 
        DEFB    44h,58h,65h,48h,06h,78h,27h,68h 
        DEFB    0C0h,18h,0E1h,08h,82h,38h,0A3h,28h 
        DEFB    7Dh,0CBh,5Ch,0DBh,3Fh,0EBh,1Eh,0FBh 
        DEFB    0F9h,8Bh,0D8h,9Bh,0BBh,0ABh,9Ah,0BBh 
        DEFB    75h,4Ah,54h,5Ah,37h,6Ah,16h,7Ah 
        DEFB    0F1h,0Ah,0D0h,1Ah,0B3h,2Ah,92h,3Ah 
        DEFB    2Eh,0FDh,0Fh,0EDh,6Ch,0DDh,4Dh,0CDh 
        DEFB    0AAh,0BDh,8Bh,0ADh,0E8h,9Dh,0C9h,8Dh 
        DEFB    26h,7Ch,07h,6Ch,64h,5Ch,45h,4Ch 
        DEFB    0A2h,3Ch,83h,2Ch,0E0h,1Ch,0C1h,0Ch 
        DEFB    1Fh,0EFh,3Eh,0FFh,5Dh,0CFh,7Ch,0DFh 
        DEFB    9Bh,0AFh,0BAh,0BFh,0D9h,8Fh,0F8h,9Fh 
        DEFB    17h,6Eh,36h,7Eh,55h,4Eh,74h,5Eh 
        DEFB    93h,2Eh,0B2h,3Eh,0D1h,0Eh,0F0h,1Eh 
	
	; 646 BYTES OF FREE SPACE
	
	ORG	0FFAh
	DEFB	0Bh		; LAST ADDRESS OF LOADED PROGRAM?
	DEFB	14h
ROMPN:  DEFB    30h             ; ROM PART NUMBER: ASCII '60'
        DEFB    36h
CHKSUM: DEFB    0CFh		; ROM CHECKSUM: SUM OF BYTES 0000h-0FFDh
L0FFF:  DEFB    3Bh

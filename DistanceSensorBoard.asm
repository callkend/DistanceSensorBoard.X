;====Kendall Callister I2C Sensor==========================================

    LIST  P=16F1788

;====PIC CONFIG=================================================================
    
    #include "p16f1788.inc"

; CONFIG1
; __config 0xC9E4
 __CONFIG _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_OFF & _MCLRE_ON & _CP_OFF & _CPD_OFF & _BOREN_OFF & _CLKOUTEN_OFF & _IESO_OFF & _FCMEN_OFF
; CONFIG2
; __config 0xDEFF
 __CONFIG _CONFIG2, _WRT_OFF & _VCAPEN_OFF & _PLLEN_OFF & _STVREN_ON & _BORV_LO & _LPBOR_OFF & _LVP_OFF

 ;====END PIC CONFIG============================================================
 
;=====CONSTANTS=================================================================
ID1	EQU "I"	    ;Set the ID of the device to INF for infrared sensor
ID2	EQU "N"	    ;//
ID3	EQU "F"	    ;/
ID4	EQU D'3'    ;Set the number of sensors connected to the device
;=====END CONSTANTS=============================================================
 
 ;====RAM LOCATIONS=============================================================
SENSOR1	EQU 0X20
SENSOR2	EQU 0X21
SENSOR3 EQU 0X22
 
ADCRES	EQU 0X23
I2CFLAG	EQU 0X24
IDFLAG	EQU 0X25
 ;====END RAM LOCATIONS=========================================================
 
 ;====MEMORY ORG================================================================
 	    ORG	    0X00	
	    GOTO    SETUP	;Setup Vector 0X00
	    ORG	    0X004	
	    GOTO    INTERRUPT	;Interrupt Vector 0X04
	    GOTO    MAINBEGIN	;Main Vector 0X05
 ;====END MEMORY ORG============================================================
 
 ;====SETUP=====================================================================
 SETUP
 ;-------OSCILLATOR SETUP-------------------------------------------------------
    BANKSEL OSCCON	    ;INTERNAL OSCILATOR CONFIGURED TO 32MHZ
    BSF	    OSCCON,SPLLEN   ;/      
    BSF	    OSCCON,IRCF3    ;/
    BSF	    OSCCON,IRCF2    ;/
    BSF	    OSCCON,IRCF1    ;/
    BSF	    OSCCON,IRCF0    ;/
    BCF	    OSCCON,SCS1	    ;/
    BCF	    OSCCON,SCS0	    ;/
;--------OSCILLATOR SETUP-------------------------------------------------------
    
;------Port Setup---------------------------------------------------------------
    BANKSEL PORTA
    CLRF    PORTA
    CLRF    PORTB
    CLRF    PORTC
    BANKSEL TRISA
    MOVLW   B'00000111'
    MOVWF   TRISA
    CLRF    TRISB
    CLRF    TRISC
    BANKSEL ANSELA
    MOVLW   B'00000111'
    MOVWF   ANSELA
    CLRF    ANSELB
    CLRF    ANSELC
    
;------End of Port Setup--------------------------------------------------------
    
;------ALTERNATE PIN SETUP------------------------------------------------------  
    BANKSEL APFCON1		;SETTING UP ALTERNATE TRANSMIT AND RECEIVE 
    BCF	    APFCON1,TXSEL	;////
    BCF	    APFCON1,RXSEL	;///
    BSF	    APFCON1,SCKSEL	;//
    BSF	    APFCON1,SDISEL	;/
;------ALTERNATE PIN SETUP END--------------------------------------------------
    
;------Personal Memory Setup----------------------------------------------------
    BANKSEL SENSOR1
    MOVLW   0XFF
    MOVWF   SENSOR1
    MOVWF   SENSOR2
    MOVWF   SENSOR3
    CLRF    ADCRES
    CLRF    I2CFLAG
    MOVLW   0X01
    MOVWF   IDFLAG
;------End Personal Memory Setup------------------------------------------------
    
;------Indirect Addressing Setup------------------------------------------------
    BANKSEL FSR0L
    MOVLW   LOW(SENSOR1)
    MOVWF   FSR0L
;------End Indirect Addressing Setup--------------------------------------------
    
;------ADC Setup----------------------------------------------------------------
    BANKSEL ADCON2	    ;SELECT BANK 0 FOR ADCON0
    BCF	    ADCON2,TRIGSEL3 ;TRIGGER DISABLED
    BCF	    ADCON2,TRIGSEL2 ;///
    BCF	    ADCON2,TRIGSEL1 ;//
    BCF	    ADCON2,TRIGSEL0 ;/	
    BSF	    ADCON2,CHSN3    ;ADC NEGATIVE REFERENCE
    BSF	    ADCON2,CHSN2    ;///
    BSF	    ADCON2,CHSN1    ;//
    BSF	    ADCON2,CHSN0    ;/
    
    BANKSEL		ADCON1		;SELECT BANK FOR ADCON1
    BCF	    ADCON1,ADFM	    ;SIGN MAGNITUDE FORMAT	
    BCF	    ADCON1,ADCS2    ;FOSC/32
    BSF	    ADCON1,ADCS1    ;//
    BCF	    ADCON1,ADCS0    ;/	
    BCF	    ADCON1,ADNREF   ;VSS SET AS REFERENCE 
    BCF	    ADCON1,ADPREF0  ;VDD SET AS REFERENCE
    BCF	    ADCON1,ADPREF1  ;/
	
    BANKSEL ADCON0	    ;SELECT BANK 0 FOR ADCON0
    BCF	    ADCON0,GO	    ;CLEAR BIT SO ADC DOES NOT START A CONVERSION AT STARTUP
    BSF	    ADCON0,ADRMD    ;10 BIT RESULT
    BCF	    ADCON0,CHS4	    ;SELECT PORT A PIN 0 
    BCF	    ADCON0,CHS3	    ;////
    BCF	    ADCON0,CHS2	    ;///
    BCF	    ADCON0,CHS1	    ;//
    BCF	    ADCON0,CHS0	    ;/
    BSF	    ADCON0,ADON	    ;/TURN THE A/D CONVERTOR ON
    
    BANKSEL PIR1
    BCF	    PIR1,ADIF
;------End of ADC Setup---------------------------------------------------------
    
;------Timer 2 Setup------------------------------------------------------------
    BANKSEL T2CON
    BCF	    T2CON,T2CONPS3
    BCF	    T2CON,T2CONPS2
    BCF	    T2CON,T2CONPS1
    BSF	    T2CON,T2CONPS0
    BSF	    T2CON,T2CKPS1
    BCF	    T2CON,T2CKPS0
    
    BANKSEL PR2
    MOVLW   D'250'
    MOVWF   PR2
;------Timer 2 End of Setup-----------------------------------------------------    
    
;------I2C SETUP----------------------------------------------------------------
    ;Slave Address needs to be verified to make sure that it working correctly
    BANKSEL EEADRL
    MOVLW   0X00
    MOVWF   EEADRL
    BCF	    EECON1,CFGS
    BCF	    EECON1,EEPGD
    BSF	    EECON1,RD
    LSLF    EEDATL,0		;Grab the Slave Address if there is one from EEPROM
    
    BANKSEL SSPADD
    BTFSC   STATUS,Z		;Check to see if the EEPROM has data
    MOVLW   0X20		;Default Slave Address 0X10
    MOVWF   SSPADD		;SLAVE MODE ADDRESS
    
    BANKSEL SSPCON1
    BSF	    SSPCON1, CKP	;Enable Clock
    BSF	    SSPCON1, SSPEN	;ENABLE SERIAL PORT FOR I2C
    BCF	    SSPCON1, SSPM3	; 0110 I2C Slave Mode
    BSF	    SSPCON1, SSPM2	;--/
    BSF	    SSPCON1, SSPM1	;-/
    BCF	    SSPCON1, SSPM0	;/
    
    BANKSEL	SSPCON2
    BCF	    SSPCON2, GCEN	;GENERAL CALL DISABLE BIT
    BCF	    SSPCON2,SEN		;Disable Clock streching

    BANKSEL	SSPSTAT
    BSF	    SSPSTAT,SMP		;SET SLEW RATE TO 100K FOR I2C
    BSF	    SSPSTAT,CKE		;ENABLE SMBUS SPECIFIC INPUTS

;------END I2C SETUP------------------------------------------------------------
    
;------PIE1 REGISTER------------------------------------------------------------					
    BANKSEL PIE1		;Select Bank 1
    BSF	    PIE1,ADIE		;Enable ADC Interrupt
    BSF	    PIE1,SSP1IE		;(MSSP) Interrupt Disable
    BSF	    PIE1,TMR2IE		;Enable Timer 2 interrupt
;------END PIE1 REGISTER--------------------------------------------------------
    BANKSEL PIR1
    BCF	    PIR1,SSP1IF		;Clear(MSSP) Interrupt Flag	
    BCF	    PIR1,TMR2IF		;Clear Timer2 Interrupt Flag
    
    BANKSEL T2CON
    BSF	    T2CON,TMR2ON	;Start Timer2
    BSF	    INTCON,PEIE		;Enable peripheral interrupts
    BSF	    INTCON,GIE		;Enable Global interrupts
    GOTO    MAINBEGIN
      
;======End of Setup=============================================================    

;=====INTERRUPT=================================================================

INTERRUPT
    
    BANKSEL PIR1
    BTFSC   SSP1IF
    CALL    I2CHANDLER
    BTFSC   ADIF
    CALL    ADCHANDLER
    BTFSC   TIMER2
    CALL    T2HANDLER
    
    RETFIE
    
;=====END INTERRUPT=============================================================
    
;======I2C HANDLER==============================================================
I2CHANDLER		
    BCF	    PIR1,SSP1IF		;Clear MSSP Flag
    
    BANKSEL SSPSTAT
    BTFSS   D_NOT_A
    GOTO    RADDRESS
    
    BTFSS   SSPSTAT,R_NOT_W
    GOTO    WRITE
    
    BANKSEL I2CFLAG
    BTFSC   I2CFLAG,0	    ;Check to see if sensor data was called for
    GOTO    READSENSOR	    ;Goto a code that will send the sensor data
    BTFSC   I2CFLAG,2	    ;Check to see if the ID is wanted
    GOTO    READID	    ;Goto the code that sends the ID TAG
    
          
RADDRESS
    BANKSEL SSPBUF
    MOVF    SSPBUF,0
    RETURN
    
    
    
WRITE
    BANKSEL I2CFLAG
    BTFSC   I2CFLAG,1
    GOTO    NEWADD
    BANKSEL SSPBUF
    
NEWADD	;ADD CODE TO SET SLAVE ADDRESS TO RECIEVED BYTE
    RETURN
    
;------READ SENSORS------------------------------------------------------------- I think clock release still needs to be added after data is loaded
READSENSOR
    BANKSEL SSPBUF
    MOVF    INDF0,0
    MOVWF   SSPBUF
    BANKSEL FSR0L_SHAD
    INCF    FSR0L_SHAD,1
    RETURN
    
;------END READ SENSORS---------------------------------------------------------   
    
;------READID------------------------------------------------------------------- I think clock release still needs to be added after data is loaded
READID
    BANKSEL IDFLAG
    BTFSC   IDFLAG,0
    GOTO    SENDI
    BTFSC   IDFLAG,1
    GOTO    SENDN
    BTFSC   IDFLAG,2
    GOTO    SENDF
    BTFSC   IDFLAG,3
    GOTO    SENDNUM
    
SENDI
    BANKSEL SSPBUF
    MOVLW   ID1
    MOVWF   SSPBUF
    BANKSEL IDFLAG
    LSLF    IDFLAG,1
    RETURN
    
SENDN
    BANKSEL SSPBUF
    MOVLW   ID2
    MOVWF   SSPBUF
    BANKSEL IDFLAG
    LSLF    IDFLAG,1
    RETURN
    
SENDF
    BANKSEL SSPBUF
    MOVLW   ID3
    MOVWF   SSPBUF
    BANKSEL IDFLAG
    LSLF    IDFLAG,1
    RETURN
    
SENDNUM
    BANKSEL SSPBUF
    MOVLW   ID4
    MOVWF   SSPBUF
    BANKSEL IDFLAG
    MOVLW   0X01
    MOVWF   IDFLAG
    RETURN  
;------END IDREAD---------------------------------------------------------------
    
READCALL
    BANKSEL I2CFLAG
    BSF	    I2CFLAG,0
    BANKSEL FSR0L_SHAD
    MOVLW   LOW(SENSOR1)
    MOVWF   FSR0L_SHAD
ADDCALL
    BANKSEL I2CFLAG
    BSF	    I2CFLAG,1
IDCALL
    BANKSEL I2CFLAG
    BSF	    I2CFLAG,2
    
MAINBEGIN
    
    
    
    
ORG 0X300   
I2CTABLE
    GOTO    READCALL	;Tells the PIC that the sensors data is wanted
    GOTO    ADDCALL	;Tells the PIC that the slave address will change
    GOTO    IDCALL	;Tells the PIC that the ID will be read
    END
    
;====MAIN END===================================================================
    
    

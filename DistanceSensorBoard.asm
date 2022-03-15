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
SLVADD  EQU 0X26
PFLAG   EQU 0X27
TCOUNT  EQU 0X28
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
    MOVLW   B'11000000'
    MOVWF   TRISB
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
    BSF	    APFCON1,SCKSEL	;// Changed for the ant board test to port b
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
    BANKSEL ADCON2	        ;SELECT BANK 0 FOR ADCON0
    BCF	    ADCON2,TRIGSEL3 ;TRIGGER DISABLED
    BCF	    ADCON2,TRIGSEL2 ;///
    BCF	    ADCON2,TRIGSEL1 ;//
    BCF	    ADCON2,TRIGSEL0 ;/	
    BSF	    ADCON2,CHSN3    ;ADC NEGATIVE REFERENCE
    BSF	    ADCON2,CHSN2    ;///
    BSF	    ADCON2,CHSN1    ;//
    BSF	    ADCON2,CHSN0    ;/
    
    BANKSEL	ADCON1		    ;SELECT BANK FOR ADCON1
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
    BCF	    T2CON,T2OUTPS3
    BCF	    T2CON,T2OUTPS2
    BCF	    T2CON,T2OUTPS1
    BSF	    T2CON,T2OUTPS0
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
    ;BTFSC   STATUS,Z		;Check to see if the EEPROM has data
    MOVLW   0X20		;Default Slave Address 0X10
    MOVWF   SSPADD		;SLAVE MODE ADDRESS
    
    BANKSEL SSPCON1
    BCF	    SSPCON1, SSPM3	; 0110 I2C Slave Mode
    BSF	    SSPCON1, SSPM2	;--/
    BSF	    SSPCON1, SSPM1	;-/
    BCF	    SSPCON1, SSPM0	;/
    BSF	    SSPCON1, SSPEN	;ENABLE SERIAL PORT FOR I2C
    BSF	    SSPCON1, CKP	;Enable Clock
    
    
    BANKSEL SSPCON2
    BCF	    SSPCON2, GCEN	;GENERAL CALL DISABLE BIT
    BCF	    SSPCON2,SEN		;Disable Clock streching
    
    BANKSEL SSPCON3
    BCF	    SSPCON3,PCIE
    BCF	    SSPCON3,SCIE
    BSF	    SSPCON3,BOEN
    BCF	    SSPCON3,AHEN
    BCF	    SSPCON3,DHEN

    BANKSEL SSPSTAT
    BCF	    SSPSTAT,SMP		;SET SLEW RATE TO 100K FOR I2C
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
    BTFSC   PIR1,SSP1IF
    CALL    I2CHANDLER
    BTFSC   PIR1,ADIF
    CALL    ADCHANDLER
    BTFSC   PIR1,TMR2IF
    CALL    T2HANDLER
    
    RETFIE
    
;=====END INTERRUPT=============================================================
    
;======I2C HANDLER==============================================================
I2CHANDLER		
    BCF	    PIR1,SSP1IF		;Clear MSSP Flag
    
    BANKSEL SSPSTAT         
    BTFSS   SSPSTAT,D_NOT_A ;Check to see if recieved byte was data or address
    GOTO    RADDRESS        ;If address is recieved clear the buffer
    
    BTFSS   SSPSTAT,R_NOT_W ;Check to see if a write or a read is requested
    GOTO    WRITE           ;Go to a sub that handles writes
              
RADDRESS                    ;Clear SSPBUF to prevent an overwrite error
    BANKSEL SSPBUF
    MOVF    SSPBUF,0
    
    BANKSEL I2CFLAG
    BTFSC   I2CFLAG,0	    ;Check to see if sensor data was called for
    GOTO    READSENSOR	    ;Goto a code that will send the sensor data
    BTFSC   I2CFLAG,2	    ;Check to see if the ID is wanted
    GOTO    READID	        ;Goto the code that sends the ID TAG
    RETURN
    

;-----Write -----------------------------------------------------------------
WRITE
    BANKSEL I2CFLAG         
    BTFSC   I2CFLAG,1       ;Check to see if the a new address will be writen
    GOTO    NEWADD          ;Run sub to set new slave address
    MOVLW   0X03            ;Set the PCL LATH to table location
    MOVWF   PCLATH          ;/
    BANKSEL SSPBUF
    MOVF    SSPBUF,0        ;The recieved data will be used to make a calcultated
    GOTO    I2CTABLE        ;jump in the table.
    

NEWADD	;ADD CODE TO SET SLAVE ADDRESS TO RECIEVED BYTE
    BCF     I2CFLAG,1   ;Clear the flag that called the sub
    BANKSEL SSPBUF      
    MOVF    SSPBUF,0    ;Set Slave address to recieved Byte
    BANKSEL SLVADD      ;//
    MOVWF   SLVADD      ;/
    BSF     PFLAG,0     ;Set a personal flag to run sub in main
    RETURN

;-----Write End---------------------------------------------------------------
    
;------READ SENSORS------------------------------------------------------------- I think clock release still needs to be added after data is loaded
READSENSOR
    BANKSEL SSPBUF
    MOVF    INDF0,0     ;Move the data of the selected sensor into the SSPBUF
    MOVWF   SSPBUF      ;/
    BSF     SSPCON1,CKP ;Release SCL allowing for the data to be clocked out
    BANKSEL FSR0L_SHAD
    INCF    FSR0L_SHAD,1;Increament to the next sensor to be read
    RETURN
    
;------END READ SENSORS---------------------------------------------------------   
    
;------READID------------------------------------------------------------------- I think clock release still needs to be added after data is loaded
READID
    BANKSEL IDFLAG
    BSF	    PORTB,3  
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
    BSF     SSPCON1,CKP
    BANKSEL IDFLAG
    LSLF    IDFLAG,1
    RETURN
    
SENDN
    BANKSEL SSPBUF
    MOVLW   ID2
    MOVWF   SSPBUF
    BSF     SSPCON1,CKP
    BANKSEL IDFLAG
    LSLF    IDFLAG,1
    RETURN
    
SENDF
    BANKSEL SSPBUF
    MOVLW   ID3
    MOVWF   SSPBUF
    BSF     SSPCON1,CKP
    BANKSEL IDFLAG
    LSLF    IDFLAG,1
    RETURN
    
SENDNUM
    BANKSEL SSPBUF
    MOVLW   ID4
    MOVWF   SSPBUF
    BSF     SSPCON1,CKP
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
    RETURN
ADDCALL
    BANKSEL I2CFLAG
    BSF	    I2CFLAG,1
    RETURN
IDCALL
    BANKSEL I2CFLAG
    BSF	    I2CFLAG,2
    RETURN

;======I2C HANDLER END==========================================================

;======ADC HANDLER==============================================================
ADCHANDLER
    BCF     PIR1,ADIF   ;Clear ADC Flag
    BANKSEL ADRESH      
    MOVF    ADRESH,0    ;Grab 8 MSBs of ADC result
    BANKSEL ADCRES      
    MOVWF   ADCRES      ;Save ADC result
    MOVLW   0X04        ;Increment Analog Channel
    ADDWF   ADCON0,1    ;/
    MOVLW   0X7C        ;Mask off the Analog channel select in ADCON0
    ANDWF   ADCON0,0    ;/
    SUBLW   0X0C        ;Check to see if AN3 is selected
    BTFSC   STATUS,Z    ;/
    CALL    RESETADC    ;If AN3 is selected run sub to select AN0
    RETURN

RESETADC
    BCF     ADCON0,CHS0 ;Select analog channel 0
    BCF     ADCON1,CHS1 ;/
    RETURN
;======ADC HANDLER END==========================================================

;======TIMER2 HANDLER===========================================================
T2HANDLER
    BCF     PIR1,TMR2IF     ;Clear Timer2 Flag
    INCF    TCOUNT,1        ;Increment Timer2 Counter
    MOVLW   D'16'           ;Check to see if 16ms has passed since counter started
    SUBWF   TCOUNT,0        ;//
    BTFSC   STATUS,Z        ;/
    CALL    ADC0            ;Start AN0 ADC if 16ms has passed
    MOVLW   D'32'           ;Check to see if 32ms has passed since counter started
    SUBWF   TCOUNT,0        ;//
    BTFSC   STATUS,Z        ;/
    CALL    ADC1            ;Start AN0 ADC if 32ms has passed
    MOVLW   D'48'           ;Check to see if 48ms has passed since counter started
    SUBWF   TCOUNT,0        ;//
    BTFSC   STATUS,Z        ;/
    CALL    ADC2            ;Start AN0 ADC if 48ms has passed. The Sharp IR sensor
    RETURN                  ;takes 48ms max to take a new measuremnt. The timer is
                            ;reset every 48ms because of this.
ADC0
    BANKSEL ADCON0          
    BSF     ADCON0,GO       ;Start ADC for AN0
    BANKSEL FSR1L_SHAD      
    MOVLW   LOW(SENSOR1)    ;Set memory location for the next result
    MOVWF   FSR1L_SHAD      ;/
    RETURN

ADC1
    BANKSEL ADCON0
    BSF     ADCON0,GO       ;Start ADC for AN1
    BANKSEL FSR1L_SHAD
    INCF    FSR1L_SHAD,1    ;Set memory location for the next result
    RETURN

ADC2
    BANKSEL ADCON0
    BSF     ADCON0,GO       ;Start ADC for AN2
    BANKSEL FSR1L_SHAD
    INCF    FSR1L_SHAD,1    ;Set memory location for the next result
    BANKSEL TCOUNT
    CLRF    TCOUNT          ;Reset Timer Counter
    RETURN
;======TIMER2 HANDLER END=======================================================

;----WRITE ADDRESS--------------------------------------------------------------
WRITEADD
    BCF     PFLAG,0     ;Clear the Flag that was set

    BANKSEL EEADRL      
    MOVLW   0X00        ;Slave address is placed in EEPROM address 0x00
    MOVWF   EEADRL
    BANKSEL SLVADD
    MOVF    SLVADD,0    ;Move slave address that is to be saved into EEDAT
    BANKSEL EEDATL      ;//
    MOVWF   EEDATL      ;/
    BCF     EECON1,CFGS ;Deselect configuration space
    BCF     EECON1,EEPGD;Point to Data Memory
    BSF     EECON1,WREN ;Enable Write

    BCF     INTCON,GIE  ;Disable interrupts

    MOVLW   0X55        ;Charge Pump
    MOVWF   EECON2      ;///
    MOVLW   0XAA        ;//
    MOVWF   EECON2      ;/
    BSF     EECON1,WR   ;Write the value into the address

    BSF     INTCON,GIE  ;Re enable interrupts
    BCF     EECON1,WREN ;Disable Write
    BTFSC   EECON1,WR   ;Wait until write is finished
    GOTO    $-2         ;/
    BANKSEL SLVADD      ;Set the Slave address of the PIC to the new value that
    MOVF    SLVADD,0    ;was saved into the EEPROM
    BANKSEL SSPADD      ;//
    MOVWF   SSPADD      ;/
    RETURN
;----WRITE ADDRESS END---------------------------------------------------------
    
;====MAIN======================================================================
MAINBEGIN
    BANKSEL PFLAG
    BTFSC   PFLAG,0
    CALL    WRITEADD
    MOVF    I2CFLAG,0
    MOVWF   PORTB

    GOTO    MAINBEGIN
    
    
    
    
    ORG 0X300   
I2CTABLE
    ADDWF   PCL,1
    GOTO    READCALL;Tells the PIC that the sensors data is wanted
    ORG 0X321
    GOTO    ADDCALL	;Tells the PIC that the slave address will change
    ORG 0X33D
    GOTO    IDCALL	;Tells the PIC that the ID will be read
    END
    
;====MAIN END===================================================================
    
    

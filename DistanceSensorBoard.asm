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
SENSOR1	    EQU 0X20
SENSOR2	    EQU 0X21
SENSOR3	    EQU 0X22
 
I2CFLAG	    EQU 0X23
IDFLAG	    EQU 0X24
SLVADD	    EQU 0X25
PFLAG	    EQU 0X26
TCOUNT	    EQU 0X27
  
ADCRESL	    EQU 0X28
ADCRESH	    EQU 0X29
DIVIDENDL   EQU 0X2A
DIVIDENDH   EQU 0X2B
DIVISORL    EQU 0X2C
DIVISORH    EQU	0X2D
REMAINH	    EQU 0X2E
REMAINL	    EQU 0X2F
ADCCOUNT    EQU	0X30
 ;====END RAM LOCATIONS=========================================================
 
 ;====MEMORY ORG================================================================
 	ORG	    0X00	
	GOTO    SETUP	    ;Setup Vector 0X00
	ORG	0X004	
	GOTO    INTERRUPT   ;Interrupt Vector 0X04
	GOTO    MAINBEGIN   ;Main Vector 0X05
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
    BANKSEL PORTA	    ;Set ports to 0
    CLRF    PORTA	    ;///
    CLRF    PORTB	    ;//
    CLRF    PORTC	    ;/
    BANKSEL TRISA	    ;Set RA0 - RA2 to inputs rest to outputs
    MOVLW   B'00000111'	    ;//
    MOVWF   TRISA	    ;/
    MOVLW   B'11000000'	    ;Set RB6 - RB7 to inputs for I2C
    MOVWF   TRISB	    ;
    CLRF    TRISC	    ;Set PortC to be outputs
    BANKSEL ANSELA	    ;Set RA0 - RA2 as analog inputs
    MOVLW   B'00000111'	    ;//
    MOVWF   ANSELA	    ;/
    CLRF    ANSELB	    ;PortB Digital I/O
    CLRF    ANSELC	    ;PortC Digital I/O
    
;------End of Port Setup--------------------------------------------------------
    
;------ALTERNATE PIN SETUP------------------------------------------------------  
    BANKSEL APFCON1	    ;SETTING UP ALTERNATE TRANSMIT AND RECEIVE 
    BCF	    APFCON1,TXSEL   ;//// 
    BCF	    APFCON1,RXSEL   ;///
    BSF	    APFCON1,SCKSEL  ;// Changed for the ant board test to port b!!!!!!!!!
    BSF	    APFCON1,SDISEL  ;/
;------ALTERNATE PIN SETUP END--------------------------------------------------
    
;------Personal Memory Setup----------------------------------------------------
    BANKSEL SENSOR1	    ;Set default sensor value to 0xFF
    MOVLW   0XFF	    ;////
    MOVWF   SENSOR1	    ;///
    MOVWF   SENSOR2	    ;//
    MOVWF   SENSOR3	    ;/
    CLRF    ADCRESH	    ;Make sure flags and result are empty to prevent errors
    CLRF    I2CFLAG	    ;//
    CLRF    PFLAG	    ;/
    MOVLW   0X01	    ;Set the ID flag to 0x01
    MOVWF   IDFLAG	    ;/
    MOVLW   0X20	    ;Used to set the default address if no address in the
    MOVWF   SLVADD	    ;EEPROM
;------End Personal Memory Setup------------------------------------------------
    
;------Indirect Addressing Setup------------------------------------------------
    BANKSEL FSR0L	    ;Set the indirect addressing to the start location
    MOVLW   LOW(SENSOR1)    ;//
    MOVWF   FSR0L	    ;/
;------End Indirect Addressing Setup--------------------------------------------
    
;------ADC Setup----------------------------------------------------------------
    BANKSEL ADCON2	    ;SELECT BANK 0 FOR ADCON0
    BCF	    ADCON2,TRIGSEL3 ;TRIGGER DISABLED
    BCF	    ADCON2,TRIGSEL2 ;///
    BCF	    ADCON2,TRIGSEL1 ;//
    BCF	    ADCON2,TRIGSEL0 ;/	
    BSF	    ADCON2,CHSN3    ;ADC NEGATIVE CHANNEL
    BSF	    ADCON2,CHSN2    ;///
    BSF	    ADCON2,CHSN1    ;//
    BSF	    ADCON2,CHSN0    ;/
    
    BANKSEL ADCON1	    ;SELECT BANK FOR ADCON1
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
    BCF	    PIR1,ADIF	    ;Clear ADC flag
;------End of ADC Setup---------------------------------------------------------
    
;------Timer 2 Setup------------------------------------------------------------
    BANKSEL T2CON	    ;Set Timer 2 to trigger every 1ms
    BCF	    T2CON,T2OUTPS3  ;/////////
    BCF	    T2CON,T2OUTPS2  ;////////
    BCF	    T2CON,T2OUTPS1  ;///////
    BSF	    T2CON,T2OUTPS0  ;//////
    BSF	    T2CON,T2CKPS1   ;/////
    BCF	    T2CON,T2CKPS0   ;////
    BANKSEL PR2		    ;///
    MOVLW   D'250'	    ;//
    MOVWF   PR2		    ;/
;------Timer 2 End of Setup-----------------------------------------------------    
    
;------I2C SETUP----------------------------------------------------------------
    BANKSEL EEADRL	    
    MOVLW   0X01	    ;Read the data location 0x01 to determine if the PIC
    MOVWF   EEADRL	    ;has been powered on before to prevent writing the
    BCF	    EECON1,CFGS	    ;default address every time
    BCF	    EECON1,EEPGD    ;/////
    BSF	    EECON1,RD	    ;////
    MOVLW   0X55	    ;///
    SUBWF   EEDATL,0	    ;//
    BTFSS   STATUS,Z	    ;/
    CALL    STARTADD        ;Write the default address to EEPROM if no data

    BANKSEL EEADRL	    
    MOVLW   0X00	    ;Read the Slave address from the EEPROM
    MOVWF   EEADRL	    ;////
    BCF	    EECON1,CFGS	    ;///
    BCF	    EECON1,EEPGD    ;//
    BSF	    EECON1,RD	    ;/
    LSLF    EEDATL,0	    ;Slave address is left shifted to be in the right spot
    BANKSEL SSPADD	    
    MOVWF   SSPADD	    ;SLAVE MODE ADDRESS
    
    BANKSEL SSPCON1
    BCF	    SSPCON1, SSPM3  ; 0110 I2C Slave Mode
    BSF	    SSPCON1, SSPM2  ;///
    BSF	    SSPCON1, SSPM1  ;//
    BCF	    SSPCON1, SSPM0  ;/
    BSF	    SSPCON1, SSPEN  ;ENABLE SERIAL PORT FOR I2C
    BSF	    SSPCON1, CKP    ;Enable Clock
    
    
    BANKSEL SSPCON2
    BCF	    SSPCON2,GCEN    ;GENERAL CALL DISABLE BIT
    BCF	    SSPCON2,SEN	    ;Disable Clock streching
    
    BANKSEL SSPCON3
    BCF	    SSPCON3,PCIE    ;Disable stop condition interrupt
    BCF	    SSPCON3,SCIE    ;Disable start condition interrupt
    BSF	    SSPCON3,BOEN    ;Ignore the state of SSPOV to always !ACK
    BCF	    SSPCON3,AHEN    ;Address holding disabled
    BCF	    SSPCON3,DHEN    ;Data holding disabled

    BANKSEL SSPSTAT
    BCF	    SSPSTAT,SMP	    ;SET SLEW RATE TO 400K FOR I2C
    BSF	    SSPSTAT,CKE	    ;ENABLE SMBUS SPECIFIC INPUTS

;------END I2C SETUP------------------------------------------------------------
    
;------PIE1 REGISTER------------------------------------------------------------					
    BANKSEL PIE1	    ;Select Bank 1
    BSF	    PIE1,ADIE	    ;Enable ADC Interrupt
    BSF	    PIE1,SSP1IE	    ;(MSSP) Interrupt Disable
    BSF	    PIE1,TMR2IE	    ;Enable Timer 2 interrupt
;------END PIE1 REGISTER--------------------------------------------------------
    BANKSEL PIR1
    BCF	    PIR1,SSP1IF	    ;Clear(MSSP) Interrupt Flag	
    BCF	    PIR1,TMR2IF	    ;Clear Timer2 Interrupt Flag
    
    BANKSEL T2CON
    BSF	    T2CON,TMR2ON    ;Start Timer2
    BSF	    INTCON,PEIE	    ;Enable peripheral interrupts
    BSF	    INTCON,GIE	    ;Enable Global interrupts
    GOTO    MAINBEGIN

STARTADD
BANKSEL	    EEADRL      
    MOVLW   0X01	    ;First Start data is place in EEPROM address 0x01
    MOVWF   EEADRL	    ;/
    MOVLW   0X55	    ;Move Start code to EEPROM 0x01
    BANKSEL EEDATL	    ;//
    MOVWF   EEDATL	    ;/
    BCF     EECON1,CFGS	    ;Deselect configuration space
    BCF     EECON1,EEPGD    ;Point to Data Memory
    BSF     EECON1,WREN	    ;Enable Write

    MOVLW   0X55	    ;Charge Pump
    MOVWF   EECON2	    ;///
    MOVLW   0XAA	    ;//
    MOVWF   EECON2	    ;/
    BSF     EECON1,WR	    ;Write the value into the address

    BCF     EECON1,WREN	    ;Disable Write
    BTFSC   EECON1,WR	    ;Wait until write is finished
    GOTO    $-2		    ;/

    BANKSEL EEADRL      
    MOVLW   0X00	    ;Slave address is placed in EEPROM address 0x00
    MOVWF   EEADRL
    MOVLW   0x10	    ;Move slave address that is to be saved into EEDAT
    MOVWF   EEDATL	    ;/
    BCF     EECON1,CFGS	    ;Deselect configuration space
    BCF     EECON1,EEPGD    ;Point to Data Memory
    BSF     EECON1,WREN	    ;Enable Write

    MOVLW   0X55	    ;Charge Pump
    MOVWF   EECON2	    ;///
    MOVLW   0XAA	    ;//
    MOVWF   EECON2	    ;/
    BSF     EECON1,WR	    ;Write the value into the address

    BCF     EECON1,WREN	    ;Disable Write
    BTFSC   EECON1,WR	    ;Wait until write is finished
    GOTO    $-2		    ;/
    RETURN
      
;======End of Setup=============================================================    

;=====INTERRUPT=================================================================

INTERRUPT
    BANKSEL PIR1	    ;Shadow registers handle Working save and Status save
    BTFSC   PIR1,SSP1IF	    ;Test SSP interrupt for I2C
    CALL    I2CHANDLER	    ;Handle I2C interrupt
    BTFSC   PIR1,ADIF	    ;Test ADC interrupt
    CALL    ADCHANDLER	    ;Handle ADC interrupt
    BTFSC   PIR1,TMR2IF	    ;Test Timer 2 interrupt
    CALL    T2HANDLER	    ;Handle Timer2 interrupt
    
    RETFIE		    ;Return from interrupt
    
;=====END INTERRUPT=============================================================
    
;======I2C HANDLER==============================================================
I2CHANDLER		
    BCF	    PIR1,SSP1IF	    ;Clear MSSP Flag
    
    BANKSEL SSPSTAT         
    BTFSS   SSPSTAT,D_NOT_A ;Check to see if recieved byte was data or address
    GOTO    RADDRESS        ;If address is recieved clear the buffer
    
    BTFSS   SSPSTAT,R_NOT_W ;Check to see if a write or a read is requested
    GOTO    WRITE           ;Go to a sub that handles writes
			    ;A read will be handled by the Raddress sub
RADDRESS                    ;Clear SSPBUF to prevent an overwrite error
    BANKSEL SSPBUF	    ;//
    MOVF    SSPBUF,0	    ;/
			    
    BANKSEL I2CFLAG
    BTFSC   I2CFLAG,0	    ;Check to see if sensor data was called for
    GOTO    READSENSOR	    ;Goto a code that will send the sensor data
    BTFSC   I2CFLAG,2	    ;Check to see if the ID is wanted
    GOTO    READID	    ;Goto the code that sends the ID TAG
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
    

NEWADD			
    BCF     I2CFLAG,1	    ;Clear the flag that called the sub
    BANKSEL SSPBUF      
    MOVF    SSPBUF,0	    ;Set Slave address to recieved Byte
    BANKSEL SLVADD	    ;//
    MOVWF   SLVADD	    ;/
    BSF     PFLAG,0	    ;Set a personal flag to run sub in main
    RETURN

;-----Write End---------------------------------------------------------------
    
;------READ SENSORS------------------------------------------------------------- I think clock release still needs to be added after data is loaded
READSENSOR
    BANKSEL SSPBUF
    MOVF    INDF0,0	    ;Move the data of the selected sensor into the SSPBUF
    MOVWF   SSPBUF	    ;/
    BSF     SSPCON1,CKP	    ;Release SCL allowing for the data to be clocked out
    BANKSEL FSR0L_SHAD
    INCF    FSR0L_SHAD,1    ;Increament to the next sensor to be read
    RETURN
    
;------END READ SENSORS---------------------------------------------------------   
    
;------READID-------------------------------------------------------------------
READID
    BANKSEL IDFLAG	
    BTFSC   IDFLAG,0	    ;Test if the I needs to be sent
    GOTO    SENDI	    ;Send I
    BTFSC   IDFLAG,1	    ;Test if the N needs to be sent
    GOTO    SENDN	    ;Send N
    BTFSC   IDFLAG,2	    ;Test if the F needs to be sent
    GOTO    SENDF	    ;Send F
    BTFSC   IDFLAG,3	    ;Test if the number of sensors needs to be sent
    GOTO    SENDNUM	    ;Send the numbers of sensor attached
    
SENDI			    ;Send the first letter of the Identification
    BANKSEL SSPBUF	    
    MOVLW   ID1		    ;Move I into SSPBUF
    MOVWF   SSPBUF	    ;/
    BSF     SSPCON1,CKP	    ;Release Master Clock to finish read
    BANKSEL IDFLAG	    
    LSLF    IDFLAG,1	    ;Raise flag to send next piece of data
    RETURN
    
SENDN			    ;Send the second letter if the Identification
    BANKSEL SSPBUF	    
    MOVLW   ID2		    ;Move N into SSPBUF
    MOVWF   SSPBUF	    ;/
    BSF     SSPCON1,CKP	    ;Release Master Clock to finish read
    BANKSEL IDFLAG
    LSLF    IDFLAG,1	    ;Raise flag to send next piece of data
    RETURN
    
SENDF			    ;This is a really good comment
    BANKSEL SSPBUF	
    MOVLW   ID3		    ;Send the third letter of the identification
    MOVWF   SSPBUF	    ;/
    BSF     SSPCON1,CKP	    ;Release Master Clock to finish read
    BANKSEL IDFLAG
    LSLF    IDFLAG,1	    ;Raise flag to send next piece of data
    RETURN
    
SENDNUM
    BANKSEL SSPBUF
    MOVLW   ID4		    ;Send the number of sensors
    MOVWF   SSPBUF	    ;/
    BSF     SSPCON1,CKP	    ;Release Master Clock to finish read
    BANKSEL IDFLAG
    MOVLW   0X01	    ;Reset the flag back the star of the ID
    MOVWF   IDFLAG
    RETURN  
;------END IDREAD---------------------------------------------------------------
    
READCALL
    BANKSEL I2CFLAG	    
    BSF	    I2CFLAG,0	    ;Set flag that the Sensors data is wanted
    BANKSEL FSR0L_SHAD	    
    MOVLW   LOW(SENSOR1)    ;Set the indirect addressing to the first sensor
    MOVWF   FSR0L_SHAD	    ;/
    RETURN
ADDCALL
    BANKSEL I2CFLAG	    
    BSF	    I2CFLAG,1	    ;Set a flag that the Slave Address is to be changed
    RETURN
IDCALL
    BANKSEL I2CFLAG
    BSF	    I2CFLAG,2	    ;Set a flag that the ID of the board is needed
    RETURN

;======I2C HANDLER END==========================================================

;======ADC HANDLER==============================================================
ADCHANDLER
    BCF     PIR1,ADIF	    ;Clear ADC Flag
    BANKSEL ADRESH      
    MOVF    ADRESH,0	    ;Grab 8 MSBs of ADC result
    BANKSEL ADCRESH      
    MOVWF   ADCRESH	    ;Save ADC Result high byte
    BANKSEL ADRESL
    MOVLW   B'11000000'	    ;Mask off the signed bit on ADC ResultL
    ANDWF   ADRESL,0	    ;/
    BANKSEL ADCRESL	    
    MOVWF   ADCRESL	    ;Save ADC Result low byte
    BANKSEL ADCON0
    MOVLW   0X04	    ;Increment Analog Channel
    ADDWF   ADCON0,1	    ;/
    MOVLW   0X7C	    ;Mask off the Analog channel select in ADCON0
    ANDWF   ADCON0,0	    ;/
    SUBLW   0X0C	    ;Check to see if AN3 is selected
    BTFSC   STATUS,Z	    ;/
    CALL    RESETADC	    ;If AN3 is selected run sub to select AN0
    BANKSEL PFLAG
    BSF     PFLAG,1	    ;Set a flag to handle the ADC in main
    RETURN

RESETADC
    BCF     ADCON0,CHS0	    ;Select analog channel 0
    BCF     ADCON0,CHS1	    ;/ 
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
    LSLF    SLVADD,0    ;was saved into the EEPROM
    BANKSEL SSPADD      ;//
    MOVWF   SSPADD      ;/
    RETURN
;----WRITE ADDRESS END----------------------------------------------------------

;----ADC CONVERTER--------------------------------------------------------------
ADCCONVERT
    BCF	    PFLAG,1	;Clear ADC flag
    MOVLW   D'6'	;Shift ADC result 6 times to right justify 10-bit result
    MOVWF   ADCCOUNT	;//////
RJUSTIFY		;/////
    LSRF    ADCRESH,1	;////
    RRF	    ADCRESL,1	;///
    DECFSZ  ADCCOUNT	;//
    GOTO    RJUSTIFY	;/
    
    MOVF    ADCRESH,0	;Set the Denominator as the ADC result
    MOVWF   DIVISORH	;///
    MOVF    ADCRESL,0	;//
    MOVWF   DIVISORL	;/
    
    MOVLW   0X17	;The Nominator is Constant value that brings result in
    MOVWF   DIVIDENDH	;line with the inverse distance curve of the sensor.
    MOVLW   0XA2	;//
    MOVWF   DIVIDENDL	;/
    
    CALL    DIVISION	;Call function to do division to get distance from ADC
			;result
			
    ;Y=6050/X This is the possible math solution needs to be verified also two byte math :(
    
    MOVF    DIVIDENDL,0	;Move the result of division into Sensors Registers
    MOVWF   INDF1
    
    RETURN
;----ADC CONVERTER END----------------------------------------------------------
    
;----Division-------------------------------------------------------------------
DIVISION
    CLRF    REMAINH	;Clear the remainder and and carry bit before division
    CLRF    REMAINL	;operation
    BCF	    STATUS,C	;/
    
    MOVLW   D'17'	;17 loops for 16-bit division
    MOVWF   ADCCOUNT	;/
    
DIVIDELOOP
    RLF	    DIVIDENDL,1	;Rotate the Nominator Left with carry
    RLF	    DIVIDENDH,1	;/
    DECFSZ  ADCCOUNT,1	;Decerment Loop Counter	exit if done
    GOTO    $+2		;Don't Exit Division function
    RETURN		;Division is done
    
    RLF	    REMAINL,1	;Rotate the Remainder Left with carry
    RLF	    REMAINH,1	;/
    MOVF    DIVISORL,0	;Subtract Denominator from the remainder
    SUBWF   REMAINL,1	;///
    MOVF    DIVISORH,0	;//
    SUBWFB  REMAINH,1	;/
    BTFSS   STATUS,C	;Check to see if result was Negative
    GOTO    $+3		;If result is negative skip 3 lines
    BSF	    STATUS,C	;Set Carry bit
    GOTO    DIVIDELOOP	;Restart Loop
    
    MOVF    DIVISORL,0	;Add the Denominator back to the remainder
    ADDWF   REMAINL,1	;///
    MOVF    DIVISORH,0	;//
    ADDWFC  REMAINH,1	;/
    BCF	    STATUS,C	;Clear Carry bit
    GOTO    DIVIDELOOP	;Restart Division Loop
;----Division end---------------------------------------------------------------
    
;====MAIN=======================================================================
MAINBEGIN
    BANKSEL PFLAG
    BTFSC   PFLAG,0
    CALL    WRITEADD
    BANKSEL PFLAG
    BTFSC   PFLAG,1
    CALL    ADCCONVERT
    GOTO    MAINBEGIN
    
    
    
    
    ORG 0X300   
I2CTABLE
    ADDWF   PCL,1	    ;Do calculated jump to raise I2C flag 
    GOTO    READCALL	    ;Tells the PIC that the sensors data is wanted
    ORG 0X323
    GOTO    ADDCALL	    ;Tells the PIC that the slave address will change
    ORG 0X33D
    GOTO    IDCALL	    ;Tells the PIC that the ID will be read
    END
    
;====MAIN END===================================================================
    
    

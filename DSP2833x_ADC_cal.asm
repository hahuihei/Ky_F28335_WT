;; TI File $Revision: /main/1 $
;; Checkin $Date: July 30, 2007   10:29:23 $
;;###########################################################################
;;
;; FILE:    ADC_cal.asm
;;
;; TITLE:   2833x Boot Rom ADC Cal routine.
;;
;; Functions:
;;
;;     _ADC_cal - Copies device specific calibration data into ADCREFSEL and ADCOFFTRIM registers 
;; Notes:
;;
;;###########################################################################
;; $TI Release: 2833x/2823x Header Files V1.32 $
;; $Release Date: June 28, 2010 $
;;###########################################################################

    .def _ADC_cal
	.asg "0x711C",   ADCREFSEL_LOC

;-----------------------------------------------
; _ADC_cal
;-----------------------------------------------
;-----------------------------------------------
; This is the ADC cal routine.This routine is programmed into 
; reserved memory by the factory. 0xAAAA and 0xBBBB are place- 
; holders for calibration data.  
;The actual values programmed by TI are device specific. 
;
; This function assumes that the clocks have been
; enabled to the ADC module.
;-----------------------------------------------

;    .sect ".adc_cal"
    .sect "ramfuncs"
    
_ADC_cal
    MOVW  DP,   #ADCREFSEL_LOC >> 6
    MOV   @28,  #0x20ab    ;0x1d1e    ;AAAA            ; actual value may not be 0xAAAA
    MOV   @29,  #0x0009    ;0x01fe    ;BBBB             ; actual value may not be 0xBBBB
    LRETR
;eof ----------

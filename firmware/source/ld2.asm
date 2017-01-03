;**********************************
;
;     Light dimmer v2.0
;     =================
;
;**********************************
; Source: www.elektronika.ba
; Date: dec 2008 - jan 2017
;**********************************
; DON'T BE A DICK PUBLIC LICENSE
; Version 1.1, December 2016
; Copyright (C) 2017 Muris Pucic Trax
;
; Everyone is permitted to copy and distribute verbatim or modified copies of this license document.
;
; DON'T BE A DICK PUBLIC LICENSE TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION
;
; Do whatever you like with the original work, just don't be a dick.
; Being a dick includes - but is not limited to - the following instances:
;  1a. Outright copyright infringement - Don't just copy this and change the name.
;  1b. Selling the unmodified original with no work done what-so-ever, that's REALLY being a dick.
;  1c. Modifying the original work to contain hidden harmful content. That would make you a PROPER dick.
; If you become rich through modifications, related works/services, or supporting the original work, share the love. Only a dick would make loads off this work and not buy the original work's creator(s) a pint.
; Code is provided with no warranty. Using somebody else's code and bitching when it goes wrong makes you a DONKEY dick. Fix the problem yourself. A non-dick would submit the fix back.
;**********************************
; Notice regarding the comments: These are mostly written in Bosnian/Croatian/Serbian language. I will translate them when I find the time. In the meantime, use Google translator.
;**********************************

;***** Declaration and PIC config *****

        PROCESSOR 12f629
        #include "p12f629.inc"

        ERRORLEVEL	0,	-302	;suppress bank selection messages

        __CONFIG _CPD_OFF & _CP_ON & _BODEN_ON & _MCLRE_OFF & _PWRTE_ON & _WDT_OFF & _INTRC_OSC_NOCLKOUT

;***** Hardware *****

#define		triac		GPIO, 0	; triac gate trigger
#define		switch		GPIO, 1	; switch input
#define		NULL		GPIO, 2	; zero-crossing detection
#define		gpInput3	GPIO, 3	; -
#define		irdata		GPIO, 4	; IR data
#define		LED			GPIO, 5	; alive LED

;***** Constants and variables *****

LEVEL		equ	0x20			; mod 0-10 (0=svjetla off, 10=svjetla maximum on)
TR_DLY		equ	0x21			; za delay u ISR_1
PAUZA1		equ	0x22			; za pauzu
PAUZA2		equ	0x23			; ..
PAUZA3		equ	0x24			; ..
TMP			equ	0x25			; za neke brojace

BOOL		equ	0x26			; boolean varz

IRDA1		equ	0x27			; IRdata niz
IRDA2		equ	0x28			; IRdata niz

CKERR1		equ	0x29			; za interrupt TMR1
CKERR2		equ	0x2A			; za interrupt TMR1

IEXPOINT	equ	0x2B			; interrupt exit point flags

KEY1a		equ	0x2C			; irda memory buttonA address from eeprom
KEY2a		equ	0x2D			; irda memory buttonA command from eeprom
KEY1b		equ	0x2E			; irda memory buttonB address from eeprom
KEY2b		equ	0x2F			; irda memory buttonB command from eeprom
KEY1c		equ	0x30			; irda memory buttonC address from eeprom
KEY2c		equ	0x31			; irda memory buttonC command from eeprom
KEY1d		equ	0x32			; irda memory buttonD address from eeprom
KEY2d		equ	0x33			; irda memory buttonD command from eeprom

KEYID		equ	0x34			; rezultat uporedjivanja primljene tipke sa ramom

SLEEPL		equ	0x35			; za sleepmode counter
SLEEPH		equ	0x36			; za sleepmode counter

BLEVEL		equ	0x37			; backup levela, ako smo ga negdje promjenili - da ga vratimo
TMP2		equ	0x38			; treba mi za progmode
BFSR		equ	0x39			; jebiga, treba u progmode

;* ISR temp vars
temp_status	equ	0x5E			; za interrupt rutinu
temp_w		equ 0x5F			; za interrupt rutinu
; 0x5F - zadnja RAM lokacija

;***** Neke konstante

KolikoTipki	equ	d'8'			; koliko tipki je dostupno u sistemu ALI *2! Znaci broj bajta, jer je svaka tipka 2 bajta (addr+command) a oboje poredim
EEaddrTipki	equ	d'2'			; pocetak EEproma gdje cuvam tipke

;***** Izvedene bool bit-varijable ******

#define		bRX		BOOL, 0		; IR uspjesno primljen flag
#define		bKEY	BOOL, 1		; da li je pronadjena tipka na indeksu KEYID
#define		bRC5	BOOL, 2		; ako je pronadjen tip rc5, onda je ovo=1
#define		bSLEEP	BOOL, 3		; MORA BITI NA 0,3 !!! radi invertovanja u "HndlKey3". Sleep mode je u pitanju :) odbrojavanje u main-u
#define		bDOSLEP	BOOL, 4		; kad odbroji sleep timer ovo se setuje

#define		biEx1	IEXPOINT, 0	; interrupt exit point 1 (Idle)
#define		biEx2	IEXPOINT, 1	; interrupt exit point 2 ()
#define		biEx3	IEXPOINT, 2	; interrupt exit point 3 ()
#define		biEx4	IEXPOINT, 3	; interrupt exit point 4 ()

;***** Mains frequency in Hz *****
; if defined it uses lookup table for 50Hz, else it uses 60Hz lookup table
#define		FREQ50 1

#define		SLEEP_LEVEL		d'6'

;***** Makroi *****

BANK0	MACRO
		bcf		STATUS, RP0
		ENDM

BANK1	MACRO
		bsf		STATUS, RP0
		ENDM

IEX1	MACRO
		clrf	IEXPOINT
		bsf		biEx1
		ENDM		
IEX2	MACRO
		clrf	IEXPOINT
		bsf		biEx2
		ENDM		
IEX3	MACRO
		clrf	IEXPOINT
		bsf		biEx3
		ENDM		
IEX4	MACRO
		clrf	IEXPOINT
		bsf		biEx4
		ENDM		

;***** Struktura programske memorije *****

		ORG		0x00		; Reset vektor
		GOTO	Init		; 0
		GOTO	Init		; 1
		GOTO	Init		; 2
		GOTO	Init		; 3

;***** Interupt rutina *****

ISR		ORG		0x04		; Interapt vektor
		bcf		INTCON, GIE	; onemoguci sve interupte za sad
		; sacuvaj registre
		movwf	temp_w		; sacuvaj W reg
		movfw	STATUS		; W=STATUS reg
		movwf	temp_status	; sacuvaj ga
		BANK0				; prebacimo se i u defaultnu banku0, vratice se prethodno stanje banki (STATUS) dole na kraju

		; sta je bilo, koji interrupt?

		; ZERO crossing detection? - mora ovaj prvi da se provjerava!
ISR_1	btfss	INTCON, INTF; external interrupt?
		GOTO	ISR_2		; nije ovaj, vidi jel neki naredni poslije ovog
							; jeste GP2/INT - zero crossing detected
		CALL	TblFreq		; w=value based on LEVEL in 64us steps until overflow!
		movwf	TMR0		; TMR0=start counting now!
		bcf		INTCON, INTF; ocisti ovaj flag jer je setovan!
							; ako je LEVEL==0 onda ne pali tmr0 int
		tstf	LEVEL		; (movf LEVEL, 1) testiraj registar na zero value
		btfsc	STATUS, Z	; do if set, ako su isti
		GOTO	ISR_End		; izadji iz ovog interrupta, nece on svjetla da mu gore :)
							; else:
		btfss	bSLEEP		; preskoci timer-countdown ako ga ne treba odbrojavati
		GOTO	ISR_1a
		incf	SLEEPL, 1	; za sleep timer
		btfsc	STATUS, Z
		incf	SLEEPH, 1
		btfsc	STATUS, Z
		bsf		bDOSLEP		; sleep timer je prekoracio, javi to u Main
ISR_1a
		bsf		INTCON, T0IE; pali na TMR0 overflow interrupt
        BANK1				; za option_reg
		movlw   b'01000000' ; INTEDG=bit6
        xorwf   OPTION_REG, 1; izvrni OPTION, INTEDG
		BANK0				; vracamo se u bank0 da nastavimo
		bcf		INTCON, T0IF; ocisti flag, da odma ne uletimo po izlazu iz interrupta jer se GARANT? setovao
		GOTO	ISR_End		; zavrsi sa interruptima u ovom prolazu

		; Triac gate trigger time?
ISR_2	btfss	INTCON, T0IF; timer0 interrupt?
		GOTO	ISR_3		; nije ovaj, vidi jel naredni
		btfss	INTCON, T0IE; ali jesmol ga uopste trebali? - MORA! jer se mozda desio TMR1 interrupt
		GOTO	ISR_3		; ma nismo, ovo je onaj TMR1 trazio interrupt, ali kako ovaj tmr0 brojac pici li pici
							; jeste TMR0 overflow
		bsf		triac		; okini triac + pauza 20-30us, cekaj 20-tak us da triac provede (tako pise u njegovoj dokumentaciji)
							; u medjuvremenu da odradimo sta pametno :)
		movlw	b'11011011'	; T0IE=0 T0IF=0
		andwf	INTCON, 1	; ocisti ih
		GOTO	$+1			; pauzica
		GOTO	$+1			; pauzica
		GOTO	$+1			; pauzica
		GOTO	$+1			; pauzica
		GOTO	$+1			; pauzica
		GOTO	$+1			; pauzica
							;
		bcf		triac		; pusti triac
		GOTO	ISR_End		; bugija iz interupt rutine u ovom prolazu

		; TMR1 overflow? ako se desi + CKERR1 overflow, vjerovatno smo najebali :) idemo u odgovarajuci interrupt exit point
ISR_3	btfss	PIR1, TMR1IF; timer1 interrupt?
		GOTO	ISR_End		; nije ovaj, vidi jel naredni, ako ga ima uopste
							; jeste TMR1 overflow
		bcf		PIR1, TMR1IF; ocisti me

		incf	CKERR1, 1	; CKERR1++
		btfsc	STATUS, Z	; ako je ovo overflow, vidi CKERR2
		incf	CKERR2, 1	; CKERR2++
		btfss	STATUS, Z	; ako NIje ovo overflow
		GOTO	ISR_End		; izadji
							; a posto jeste, onda vidi koji je nas exit point!
		BANK1
		bcf		PIE1, TMR1IE; za svaki slucaj gasi tmr1 interrupt da ne bude belaja
		BANK0
		btfsc	biEx1		; exit point 1 (Idle)
		GOTO	Idle		; ovo ce da nas "resetuje" a TO mora da upali GIE jer je trenutno ugasen !!!
		btfsc	biEx2		; exit point 2 (HndlSwitch_iexp)
		GOTO	ProgMode	; ajmo u prog-mode, jer covjek drzi dugme pravo uporno i dugo :)
		btfsc	biEx3		; exit point 3 ()
		GOTO	ProgModeIRe	; greska primljena prilikom prijema IR impulsa ali u programskom modu
		btfsc	biEx4		; exit point 4 ()
		GOTO	ProgModeCan	; zavrsavamo programiranje neuspjesno, nije stisnuo daljinski na vrijeme

		; else	
		; ako nije definisan interrupt exit point, samo se vracamo fino

		; Nema ih vise, izlazimo...
ISR_End	; vrati varijable i vrati se sa retfie
		movfw	temp_status	; w=temp status
		movwf	STATUS		; STATUS=W
		swapf	temp_w, 1	; i konacno W, u 2 koraka
		swapf	temp_w, 0	;
		RETFIE				; vrati se...

;***** TMR1 time counter *****
; Greska okidanja triaca je u ~26usec, znaci kasnimo toliko od detekcije NULL-e
; posto mi je prescaler 1:64 (64us) ne mogu to dotjerati! :( ali neka...

TblFreq	movfw	LEVEL
		addwf	PCL, 1				; hop
		retlw	0					; mora biti ova invalidna lokacija na pocetku, radi optimizovane interrupt rutine (LEVEL moze biti 0-10) !
		#ifdef FREQ50
		; 50 Hz - OK
			retlw	d'140'	; 255-137*64us = 7.552ms
			retlw	d'150'	; 255- *64us = ms
			retlw	d'160'	; 255- *64us = ms
			retlw	d'170'	; 255- *64us = ms
			retlw	d'180'	; 255- *64us = ms
			retlw	d'190'	; 255- *64us = ms
			retlw	d'200'	; 255- *64us = ms
			retlw	d'210'	; 255- *64us = ms
			retlw	d'220'	; 255- *64us = ms
			retlw	d'237'	; 255-242*64us = >0.832ms
		#else
		; 60 Hz - OK
			retlw	d'150'	; 255-150*64us = 6.720ms
			retlw	d'160'	; 255- *64us = ms
			retlw	d'170'	; 255- *64us = ms
			retlw	d'180'	; 255- *64us = ms
			retlw	d'190'	; 255- *64us = ms
			retlw	d'201'	; 255- *64us = ms
			retlw	d'212'	; 255- *64us = ms
			retlw	d'223'	; 255- *64us = ms
			retlw	d'234'	; 255- *64us = ms
			retlw	d'241'	; 255-245*64us = >0.640ms
		#endif

;***** Inicijalizacija uc-a *****

Init	BANK1				; bank 1
	    CALL	0x3FF		;get the calibration value
    	movwf	OSCCAL		;calibrate
		movlw	b'01000101'	;
		movwf	OPTION_REG	; gp2/int interrupt on rising edge, TMR0 prescaler=1:64 dodijeljen na TMR0
		movlw	b'00000010'	; 
		movwf	WPU			; pullup only on switch pin
		movlw	b'00011110'	;
		movwf	TRISIO		; inputs-outputs
        #IFDEF ANSEL
			clrf	ANSEL	; all digital - PIC12F675
        #ENDIF
		clrf	PIE1		; gasi interrupt za TMR1 (bit 0), njega mi palimo/gasimo rucno
		BANK0

		clrf	GPIO		; sve sto mi je output - pogasi
        #IFDEF ADCON0
			clrf	ADCON0	; ad converter off - PIC12F675
        #ENDIF
		movlw	h'07'		;
		movwf	CMCON		; no capture and compare
        ; PIR1
        clrf	PIR1		; no periferal interrupt occured initially
		; T1CON (za TMR1)
        movlw	b'00110000'	; disable timer1 za sada!, prescaler 1:8  
        movwf   T1CON        
		; INTCON
        movlw	b'01010000'	; enable GP2/INT external interrupt, periferal interrupts enable, i clear flagove
        movwf	INTCON

		clrf	BOOL		; ocisti nam ovo
		clrf	IEXPOINT	; i ovo
		clrf	LEVEL		; inicijalno =0 "svjetlo off"

		movlw	d'10'
		movwf	BLEVEL		; za svaki slucaj, ovo moram inicijalizovati za slucaj nestanka struje i povratka a programiranje daljinskog je vec davno obavljeno!

		CALL	GetSetup	; ucitaj tipke iz EEprom-a u RAM
		GOTO	Idle		; idi na startup

;***** Idle *****
; IR-Reset & Startap rutina. Ovdje dolazimo i nakon greske u prijemu IR signala.
Idle	bsf		INTCON, GIE	; enable interrupts (trenutno samo external GP2/INT sto je cool)
		movlw	d'2'		; led blink 3 puta
		CALL	LEDblink
		; u protivnom idemo u main:
Main
		btfsc	bDOSLEP		; uradi ako je vrijeme za spavanje
		GOTO	MainSleep	; gasi svjetla
		btfss	switch		; ako je stisnut
		GOTO	HndlSwitch	; obradi dugme
Main_	btfss	irdata		; ako ima ulazni IR umpuls
		GOTO	HndlIRdata	; odradi IR prijem
		; u protivnom, vrtimo se ko budala
		GOTO	Main

;***** Sleep mode counter *****
; Spava se za nekih 2min
MainSleep
		;bcf		bSLEEP; ovo ce da odradi SoftDown, pa da mi dzabe ne radimo to isto
		bcf		bDOSLEP
		movlw	d'4'		; blinkamo 4x da javimo sleep-period-end
		CALL	LEDblink
		GOTO	SoftDown	; fino ih ugasi do 0, pa picis u Main odatle

;***** Pritisnuto dugme ******
; Ovdje ce trebati mjeriti trajanje tastera i slicne gluposti
HndlSwitch
		IEX2				; ako se desi interrupt sad u mjerenju dugmeta, idi u HndlSwitch_iexp
		movlw	h'eb'		; ovi ckerr-ovi odgovaraju za ~10sec, ako sam dobro izracunao :)
		movwf	CKERR1
		movlw	h'ff'
		movwf	CKERR2
		CALL	TMR1en		; turn on counter&interrupt on just defined ckerr1&2

		CALL	Delay_10ms	; prvo cekamo 10ms
		btfsc	switch		; ako sad nema signala, ovo je bila samo greska, vratimo se u Main!
		GOTO	Main_		; samo se fino vrati, ali poslije poziva na dugme radi raje... ovo su neke fluktuacije na mrezi
		CALL	Delay_10ms	; JOS cekamo 10ms
		btfsc	switch		; ako sad nema signala, ovo je bila samo greska, vratimo se u Main!
		GOTO	Main_		; samo se fino vrati, ali poslije poziva na dugme radi raje... ovo su neke fluktuacije na mrezi

		bsf		LED			; pali led
HndlSwitch_a
		CALL	Delay_01s	; button debounce delay ! neophodan radi 220V ~ fluktuacija
		btfss	switch		; button still pressed?
		GOTO	HndlSwitch_a; jos ga nije pustio
		bcf		LED			; gasi led

		CALL	TMR1di		; turn off counter&interrupt
		; let's see how long this impulse lasted with CKERR2:1
		; ako smo ovdje dosli, znaci da je drzao manje od 20sec, jer interrupt preuzima nakon 20s
		; a to znaci da uporedjujemo samo ckerr1 :) i samo gledamo jel drzao vise od 1,5sec!

		; IF CKERR>=3*0.52sec => sleep mode!
		movlw	h'eb'+d'3'	; X=3 inkrementa od osnove odozgo na pocetku HndlSwitch-a
		subwf	CKERR1, 0	; Y, subtract Y-X
		; if X=Y then now Z=1.
		; if Y<X then now C=0.
		; if X<=Y then now C=1.
		btfsc	STATUS, C	; do if Y>=X
							;GOTO	SetSleepMode; ajde u sleep mode
		GOTO	HndlKey3	; ovo ce da ishendla sleep-mode
							; else: normal click
		tstf	LEVEL
		btfsc	STATUS, Z	; do if zero - ako je ugaseno
		GOTO	SoftStart	; pali svjetlo polako do maksimuma a ne do BLEVEL-a na pritisak saltera
							; else: gasi
		GOTO	SoftDown	; gasi svjetlo polako

;***** SleepMode ******
; Ovdje dolazimo duzim pritiskom tastera, ili dugmetom na daljinskom!
SetSleepMode
		movlw	h'1F'
		movwf	SLEEPL
		movlw	h'D1'
		movwf	SLEEPH		; 2min pauza

		movlw	d'1'
		CALL	LEDblink

		tstf	LEVEL		; sta su svjetla?
		btfsc	STATUS, Z	; uradi ako su ugasena
		GOTO	SetSleepMode_1
							; else upaljena su vec, samo ih nastimaj na 6 ako su >6
		; IF LEVEL>6 => down-it to 6
		movlw	SLEEP_LEVEL		; X=6
		subwf	LEVEL, 0	; Y, subtract Y-X
		; if X(6)=Y then now Z=1.
		; if Y<X(6) then now C=0.
		; if X(6)<=Y then now C=1.
		btfss	STATUS, C	; do if NOT X<=Y
		GOTO	Main		; ne treba smanjivati nivo svjetla, vec je <=6
							; veci je level ili je jednak (takav mi IF), mozemo ga smanjiti na 6 (ponekad bespotrebno ako je vec bilo level==6)
		movlw	SLEEP_LEVEL
		movwf	LEVEL		; level svjetla je sad 6
		GOTO	Main		; bjezi u main da pocne odbrojavanje
SetSleepMode_1
		; softStart na level 6!
		movlw	SLEEP_LEVEL
		CALL	SoftStart_0	; softaj ih na level 6!
		GOTO	Main		; bjezi u main da pocne odbrojavanje

;***** SoftStart ******
; treba biti oprezan da ne prekoracimo vrijednost 10
SoftStart_b
		movfw	BLEVEL
		GOTO	SoftStart_0
SoftStart
		movlw	d'10'
SoftStart_0
		movwf	TMP
SoftStart_1
		incf	LEVEL, 1	; ++
		CALL	Delay_50ms
		decfsz	TMP, 1		; --, skip if zero
		GOTO	SoftStart_1	; jos!
		;bcf		bSLEEP		; ako smo bili na sleep, pa utisali do 0, pa upalili prije isteka sleep-a ovo ce prekinuti timer da nam se ne ugasi svjetlo
		GOTO	Main

;***** SoftDown ******
; Malo smo naopaki ali je to cool i mora tako.
SoftDown_1
		CALL	Delay_50ms
SoftDown
		tstf	LEVEL
		btfsc	STATUS, Z
		GOTO	SoftDown_2	; dosta je!
		decf	LEVEL, 1	; --
		GOTO	SoftDown_1	; jos, do nule!
SoftDown_2
		bcf		bSLEEP		; gasi sleep mode ako je bio slucajno upaljen
		GOTO	Main		; ajde u main, ugasili smo ih

;***** LED Blink, W puta *****
LEDblink
		movwf	TMP
LEDblink_1
		CALL	Delay_01s	; cek malko
		bsf		LED
		CALL	Delay_01s	; cek malko
		bcf		LED
		decfsz	TMP, 1		; --, skip if zero
		GOTO	LEDblink_1	; jos
		RETURN				; vrati se...

;***** ProgMode - programiranje kodova iz daljinskog *****
;* Interrupt exit point za dugme
ProgMode
		bsf		INTCON, GIE	; moramo ih upaliti, ovdje smo dosli sa ugasenim GIE

		movfw	LEVEL
		movwf	BLEVEL		; backupiraj trenutni level
		movlw	d'2'
		movwf	LEVEL		; smanji svjetlo na 30% tokom programiranja

		movlw	d'10'		; blink LED 10 puta
		CALL	LEDblink
		; sad primamo KolikoTipki/2 u RAM po dva bajta!
							; podesi FSR na prvu tipku
		movlw	LOW(KEY1a)
		movwf	FSR			; FSR=prva tipka addr
		movlw	KolikoTipki	; koliko ima tipki ali *2
		movwf	TMP2
		bcf		STATUS, C	; brisi jebeni C koji me je izjebao
		rrf		TMP2, 1		; /2, tako mi vise godi u petlji
ProgMode_1					; primanje tipke?
		movfw	FSR
		movwf	BFSR		; backupiramo ga jer ga FetchIRdata pokvari	
		IEX4				; ako se sad desi interrupt idemo u ProgModeCan
		movlw	h'eb'		; nekih 10tak sekundi za ocekivanje IR tipke? ako ne dodje, prekidamo programiranje
		movwf	CKERR1
		movlw	h'ff'
		movwf	CKERR2
		CALL	TMR1en

		btfsc	irdata		; ako jos nema ulaznog IR umpulsa
		GOTO	$-1

		CALL	TMR1di		; gasi interrupt
		IEX3				; ako se odsad desi interrupt idemo u ProgModeIRe
		CALL	FetchIRdata	; pokusaj pokupit taj dolazeci podatak, unutra se interrupt sam prebacio na 0,5s!
		btfss	bRX			; if this was an error we received
		GOTO	ProgMode_1	; probaj opet
							; interrupt je ugasen unutar FetchIRdata funkcije, gore cemo ga opet upalit
		movfw	BFSR
		movwf	FSR			; vrati ovo na nase
		; ako smo ovdje - imamo podatak u IRDA1 i IRDA2
		movfw	IRDA1
		movwf	INDF
		incf	FSR, 1		; fsr++ naredni bajt iste tipke
		movfw	IRDA2
		movwf	INDF
		incf	FSR, 1		; fsr++ naredna tipka
							; javi da je primljena ova tipka na LED i sijalicu
		movlw	d'1'		; blink LED 1 puta - primljena tipka
		CALL	LEDblink
							; fadiraj svjetlo na 6 pa vrati na level 2
		movlw	d'6'
		movwf	LEVEL
		CALL	Delay_01s
		CALL	Delay_01s
		movlw	d'2'
		movwf	LEVEL		; vrati na level 2
							; primanje naredne tipke, ako ima
		decfsz	TMP2, 1		; --, skip if zero
		GOTO	ProgMode_1	; jos!
							; programiranje je gotovo:
		CALL	SnimiSetup	; snimi primljene tipke u EEPROM
		movlw	d'10'		; blink LED 8 puta - programiranje zavrseno
		CALL	LEDblink
		movfw	BLEVEL
		movwf	LEVEL		; vrati level na prethodni prije programiranja
		; ako je slucajno programiranje vrseno sa ugasenim svjetlom, BLEVEL nam je=0 a to nam ne godi.
		tstf	BLEVEL
		btfss	STATUS, Z	; ako nije nula
		GOTO	Main		; vrati se u main
							; u protivnom nastimaj ga na maximum, pa se onda vrati u main
		movlw	d'10'
		movwf	BLEVEL
		GOTO	Main		; sad se vrati u main...

;*interrupt exit point 3
ProgModeIRe
		bsf		INTCON, GIE	; obavezno, jer je ugasen
							; javi IR error na LED
		movlw	d'2'		; led blink 2 puta
		CALL	LEDblink
		GOTO	ProgMode_1	; nastavi programiranje tipki gdje si stao

;*interrupt exit point 4
ProgModeCan
		bsf		INTCON, GIE	; nije obavezno, upalice ga Idle kad sad skocimo na njega
		movfw	BLEVEL
		movwf	LEVEL		; vrati level na prethodni prije programiranja
		GOTO	Idle		; vratimo se samo na Idle jer je ocigledno odustao od programiranja

;***** IR signal present ******
; Let's see what IR protocol is this, we support only RC5 and NEC
HndlIRdata
		IEX1				; ako se desi interrupt u funkcijama za primanje IR-a, kao i u mjerenju impulsa u FetchIRdata, da se ide u Idle!
		CALL	FetchIRdata	; uhvati infrared code
		btfss	bRX			; if this was an error we received
		GOTO	Main		; go back to main...
		; Ovdje IRDA1 i IRDA2 kod, treba uraditi nesto pametno :)
		CALL	NadjiTipku	; jel to neka nasa tipka?
		btfss	bKEY		; if this is not our key
		GOTO	Main		; vala nije :)

		; jel keyIndex=1
		movlw	d'1'
		subwf	KEYID, 0
		btfsc	STATUS, Z 	; ako jest
		GOTO	HndlKey1

		; jel keyIndex=2
		movlw	d'2'
		subwf	KEYID, 0
		btfsc	STATUS, Z 	; ako jest
		GOTO	HndlKey2

		; jel keyIndex=3
		movlw	d'3'
		subwf	KEYID, 0
		btfsc	STATUS, Z 	; ako jest
		GOTO	HndlKey3

		; jel keyIndex=4
		movlw	d'4'
		subwf	KEYID, 0
		btfsc	STATUS, Z 	; ako jest
		GOTO	HndlKey4

		; else: ovdje se nebi trebalo NIKAD doci!
		GOTO	Main		; go back to main...

;***** Tipka 1 na daljinskom stisnuta *****
; UP/ON key
HndlKey1
		; ako je nula, odma idi na nivo BLEVEL (zadnji upamceni na pritisak OFF dugmeta)!
		tstf	LEVEL
		btfsc	STATUS, Z	; do if zero
		GOTO	SoftStart_b	; was: (GOTO SoftStart) upali softstart na vrijednost BLEVEL-a
		; else:
		movlw	d'10'
		subwf	LEVEL, 0
		btfss	STATUS, Z	; ako nije na najvecem nivou, mozemo mu povecati
		incf	LEVEL, 1	; povecaj nivo svjetla
		GOTO	Main		; vrati se u main

;***** Tipka 2 na daljinskom stisnuta *****
; DOWN key
HndlKey2
		tstf	LEVEL
		btfss	STATUS, Z	; ako nije na najmenjem nivou, mozemo mu ga smanjiti pa cak i na 0 da ih ugasi skroz
		decf	LEVEL, 1	; smanji nivo svjetla
		; ako smo ga smanjili na 0, onda da ukinemo i sleep mode posto je MOZDA upaljen :)
		tstf	LEVEL
		btfsc	STATUS, Z	; ako jeste 0
		bcf		bSLEEP		; gasi jer je mozda upaljen. ovo inace ne smeta, ali LED migne nakon 2min dzabe		
		GOTO	Main

;***** Tipka 3 na daljinskom stisnuta *****
; SLEEP key
HndlKey3
		movlw	b'00001000'	; bSLEEP bit pozicija
		xorwf	BOOL, 1		; invert SLEEP bool var
		btfsc	bSLEEP		; ako treba sleep mode, idi podesi ga
		GOTO	SetSleepMode; ajde
		movlw	d'4'		; blinkamo 4x da javimo sleep-period-cancel
		CALL	LEDblink
		GOTO	Main		; u protivnom, haj samo u main

;***** Tipka 4 na daljinskom stisnuta *****
; OFF key
HndlKey4
		; ako je svjetlo vec OFF, nemoj snimati bLevel!
		tstf	LEVEL
		btfsc	STATUS, Z	; do if iz zero
		GOTO	HndlKey4_	; preskoci podesenje bLevela na nulu! ne smijemo to dozvoliti nikad!
		movfw	LEVEL
		movwf	BLEVEL		; da upamtimo LEVEL za naredno paljenje svjetala :)
HndlKey4_
		GOTO	SoftDown	; ovo se brine o gasenju svjetla

;***** FetchIRdata *****
; Skonta o kojem je IR protokolu rijec i prima IR kod pozivom odgovarajuce funkcije za prijem
FetchIRdata
		CALL	TMR1en_		; turn on counter&interrupt on 0,52428 secs
		bcf		bRX			; assume this was an error
		bcf		bRC5		; assume it's not rc5 we'r about to get
		btfss	irdata		; signal still here?
		GOTO	$-1
		CALL	TMR1di		; turn off counter&interrupt
		; let's see how long this impulse lasted with TMR1 H:L counter (>3ms=NEC, <3ms=RC5) 3ms equals to 0x0177 TMR1H=1h, TMR1L=77h

		;***** PIClist.com 16bit UNSIGNED compare by David Cary 2001-03-30 *****
		; to indicate the X=Y, Y<X, or X<Y.
		movlw	h'1'		; Xhi, for 3ms
		subwf	TMR1H, 0 	; Yhi, subtract Y-X
		skpz				; Are they equal ?
	    goto	FetchIRdata_1
		; yes, they are equal -- compare lo
		movlw	h'77'		; Xlo, for 3ms
		subwf	TMR1L, 0	; Ylo, subtract Y-X
FetchIRdata_1
		; if X=Y then now Z=1.
		; if Y<X then now C=0.
		; if X<=Y then now C=1.
		;btfss	STATUS, C	; if TMR1H:L < 3ms
		;CALL	GetRC5		; this one is more time-critical
		;btfsc	STATUS, C	; else
		;CALL	GetNEC		; we have more time with this one
		btfss	STATUS, C	; if TMR1H:L < 3ms
		bsf		bRC5
		btfsc	bRC5		; if TMR1H:L was < 3ms
		CALL	GetRC5		; this one is more time-critical
		btfss	bRC5		; else
		CALL	GetNEC		; we have more time with this one
		RETURN				; return...

;***** GetRC5 rutina ******
; Addr = IRDA1
; Comm = IRDA2
GetRC5	CALL	TMR1en_		; pali interrupt na 0,52428 sec
		;--ovo se nece implementirati ako je prethodno izmjerena duzina pulsa, jer ga nece biti
		;btfss	irdata		; 1) do if clear - ako je signal jos prisutan
		;GOTO	$-1			; cekaj da nestane signal
		;--

		CALL	Delay_420us	; 444us 2) - smanjeno jer je vec nesto vremena proteklo!
		btfss	irdata		; 3) do if clear - ako je signal prisutan
		GOTO	GetRC5e		; izadji sa greskom

		CALL	Delay_444us	; 4)
		CALL	Delay_444us
		btfsc	irdata		; 5) do if set - ako signala nema
		GOTO	GetRC5e		; izadji sa greskom

		bsf		LED			; pali led
		; petlja za prijem podataka
		clrf	IRDA2
		clrf	IRDA1
		movlw	LOW(IRDA1)
		movwf	FSR
		movlw	d'12'		; 12 bita primamo
		movwf	TMP
GetRC5a	CALL	Delay_444us	; 6)
		CALL	Delay_444us	;
							; sample and save:
		btfss	irdata		; do if clear - uradi ako signala ima
		GOTO	GetRC5b
		; signala nema, cekamo da dodje
		bsf		INDF, 0
		btfsc	irdata
		GOTO	$-1			; sad cekaj da signal dodje da se sinhronizujemo na prelazu
		GOTO	GetRC5c		; preskoci ovo
GetRC5b	; signala ima, cekamo da nestane
		btfss	irdata
		GOTO	$-1			; sad cekamo da signal nestane da se sinhronizujemo na prelazu
GetRC5c	bcf		STATUS, C
		rlf		INDF, 1
		CALL	Delay_444us	; 7,8)

		; ako je TMP == 8, povecaj FSR za 1 (INDF=>IRDATA2)
		; ako je TMP == 0, izlazimo sa setovanim bRX
		movlw	d'8'
		subwf	TMP, 0		; oduzmi
		btfsc	STATUS, Z	; do if set
		incf	FSR, 1		; nadalje radimo sa IRDA2

		decfsz	TMP, 1		; --, skip if zero
		GOTO	GetRC5a		; jos!

		CALL	Delay_01s	; sacekaj malo ako je daljinski prebrz a garant svi RC5 jesu
		CALL	Delay_01s	; sacekaj malo ako je daljinski prebrz a garant svi RC5 jesu
		bsf		bRX
GetRC5e	btfss	irdata		; ako sta ima na kraju
		GOTO	$-1			; sacekaj da nestane
		CALL	TMR1di		; gasi interrupt
		bcf		LED			; gasi led
		bcf		IRDA1, 5	; iskljuci toggle bit definitivno
		RETURN				; vrati se

;***** GetNEC rutina ******
; Addr = IRDA1
; Comm = IRDA2
GetNEC	CALL	TMR1en_		; pali interrupt na 0,52428 sec
		;--ovo se nece implementirati ako je prethodno izmjerena duzina pulsa, jer ga nece biti
		;btfss	irdata
		;goto	$-1			; cekaj da start-impuls nestane
		;--

		CALL	Delay_256us	; 1) - smanjeno jer je vec nesto vremena proteklo!
		movlw	d'6'		; bilo 7, ali je sad 6 jer sam gore izdvojio jednu pauzu radi smanjenja
		movwf	TMP
GetNECa	CALL	Delay_280us
		decfsz	TMP, 1
		GOTO	GetNECa
		btfss	irdata		; 2)
		GOTO	GetNECe

		CALL	Delay_280us	; 3)
		btfss	irdata		; 4)
		GOTO	GetNECe

		bsf		LED			; pali led

		clrf	IRDA1
		clrf	IRDA2
		movlw	LOW(IRDA1)
		movwf	FSR
		movlw	d'32'		; 32 bita hvatamo
		movwf	TMP
GetNECb	btfsc	irdata		; 5)
		GOTO	$-1
		btfss	irdata		; 6)
		GOTO	$-1

		CALL	Delay_280us	; 7)
		CALL	Delay_280us
		CALL	Delay_280us

		; jesmol primili 2 bajta koja dolaze
		movlw	d'16'
		subwf	TMP, 0
		btfsc	STATUS, Z
		incf	FSR, 1		; INDF(FSR) = IRDA2

		bcf		INDF, 0 ; moram radi overwritanja
		btfss	irdata		; 7-a)
		bsf		INDF, 0

		decf	TMP, 1
		btfsc	STATUS, Z	; 8) ako je kraj
		GOTO	GetNECc

		bcf		STATUS, C	; 9)
		rlf		INDF, 1

		GOTO	GetNECb		; 10) jos!

		; validan izlaz
GetNECc	bsf		bRX			; nesto smo primili
		btfsc	irdata		; cekaj pojavu impulsa (stop bit)
		GOTO	$-1
GetNECe	btfss	irdata		; cekaj nestanak impulsa
		GOTO	$-1
		CALL	TMR1di		; gasi interrupt
		CALL	Delay_01s	; sacekaj malo jer je NEC isto tako brz a salje onaj repeat-command koji ja ne tretiram
		CALL	Delay_01s	; sacekaj malo jer je NEC isto tako brz a salje onaj repeat-command koji ja ne tretiram
		bcf		LED			; gasi led
		RETURN				; vrati se...

;***** TMR1 int enabler and disabler *****
; Ako se desi interrupt, idemo na interrupt exit point definisan sa makroima IEX1...n
TMR1en_	movlw	h'FF'		; FFh za minimalni interrupt na 0,52428 sec
		movwf	CKERR1
		movlw	h'FF'		; FFh & FFh = minimalni interrupt na 0,52428 sec
		movwf	CKERR2
TMR1en	clrf	TMR1L
		clrf	TMR1H
		bsf		T1CON, TMR1ON; pali brojac
		bcf		PIR1, TMR1IF; moramo!
		BANK1
		bsf		PIE1, TMR1IE; pali tmr1 interrupt
		BANK0
		RETURN				; vrati se..
TMR1di	BANK1
		bcf		PIE1, TMR1IE; gasi tmr1 interrupt
		BANK0
		bcf		PIR1, TMR1IF; reci da nije ni bio za svaki slucaj
		bcf		T1CON, TMR1ON; gasi brojac
		RETURN				; vrati se...

;***** Provjerava tipku koja je pritisnuta iz IRDA1 i IRDA2 sa dostupnim iz RAM-a *****
; izlaz: varijabla bKEY = [0,1] i KEYID = (1,2,...KolikoTipki/2)
NadjiTipku
		clrf	KEYID		; =0
		bcf		bKEY		; nismo nasli nijednu za sad
		movlw	LOW(KEY1a)
		movwf	FSR			; FSR=prva tipka addr
		movlw	KolikoTipki	; koliko ima tipki ali *2
		movwf	TMP
		bcf		STATUS, C	; brisi jebeni status C bit
		rrf		TMP, 1		; /2
NadjiTipku_1
		incf	KEYID, 1	; keyid ++
		movfw	IRDA1		; poredimo prvi
		subwf	INDF, 0
		btfsc	STATUS, Z
		GOTO	NadjiTipku_2; isti su, prvi bajt
		incf	FSR, 1
		incf	FSR, 1
		GOTO	NadjiTipku_4
NadjiTipku_2
		incf	FSR, 1		; poredimo drugi
		movfw	IRDA2
		subwf	INDF, 0
		btfsc	STATUS, Z
		GOTO	NadjiTipku_3; isti su, i drugi bajt!
		incf	FSR, 1
		GOTO	NadjiTipku_4
NadjiTipku_3
		bsf		bKEY		; reci da si pronaso
		RETURN				; vrati se uspjesno, da se ne ganjamo vise u petlji
NadjiTipku_4
		decfsz	TMP, 1		; --, skip if zero
		GOTO	NadjiTipku_1; jos!
		RETURN				; vrati se neuspjesno...

;***** SnimiSetup - Snima tipke iz RAM-a u EEPROM *****
SnimiSetup
		movlw	EEaddrTipki	; EEPROM pocetak za tipke!
		CALL	EEaddr
		movlw	LOW(KEY1a)
		movwf	FSR			; FSR=prva tipka addr
		movlw	KolikoTipki	; koliko ima tipki ali *2
		movwf	TMP
SnimiSetup_1
		movfw	INDF		; RAM data je u W
		CALL	EEwrite		; kopiraj u EEPROM
		CALL	EEinc		; eeadr++
		incf	FSR, 1		; fsr++
		decfsz	TMP, 1		; --, skip if zero
		GOTO	SnimiSetup_1; jos!
		RETURN				; vrati se...

;***** GetSetup - Ucitava tipke iz eeproma u RAM *****
GetSetup
		movlw	EEaddrTipki	; EEPROM pocetak za tipke!
		CALL	EEaddr
		movlw	LOW(KEY1a)
		movwf	FSR			; FSR=prva tipka addr
		movlw	KolikoTipki	; koliko ima tipki ali *2
		movwf	TMP
GetSetup_1
		CALL	EEread
		movwf	INDF		; kopiraj u RAM
		CALL	EEinc		; eeadr++
		incf	FSR, 1		; fsr++
		decfsz	TMP, 1		; --, skip if zero
		GOTO	GetSetup_1	; jos!
		RETURN				; vrati se...

;***** Puni EEADR iz W *****
EEaddr	BANK1
		movwf	EEADR
		BANK0
		RETURN				; vrati se...

;***** Inkrementira EE adresu *****
EEinc	BANK1
		incf	EEADR, 1
		BANK0
		RETURN				; vrati se...

;***** Cita iz EEPROM-a sa lokacije EEADR i vraca rez u W reg *****
; preduslov: podesen EEADR
EEread	BANK1
		bsf		EECON1, RD 	; EE Read
		movfw	EEDATA		; Move data to W
		BANK0
		RETURN				; vrati se...

;***** Snima u EEPROM na lokaciju EEADR podatak W *****
; preduslov: podesen EEADR
EEwrite	BANK1
		movwf	EEDATA		; EEDATA=W
		bsf		EECON1, WREN; Enable write
		;bcf		INTCON, GIE	; kako pise u dokumentaciji :)
		movlw	h'55' 		; Unlock write
		movwf	EECON2 		;
		movlw	h'AA' 		;
		movwf	EECON2 		;
		bsf		EECON1, WR 	; Start the write
EEwrit	btfsc	EECON1, WR	; ako se nije zavrsio upis
		GOTO	EEwrit		; vrati se i cekaj da se zavrsi vala
		;bsf		INTCON, GIE	; vrati interrupte jer meni tako godi u ovoj aplikaciji
		bcf		EECON1, WREN; onemoguci upise u eeprom na dalje
		BANK0
		RETURN				; vrati se...

;***** Pauze ****

Delay_256us				;250 cycles (+2 because of additinal GOTO Delay_1b_loop)
		movlw	h'53'
		movwf	PAUZA1
		GOTO	Delay_1b_loop
Delay_280us				;274 cycles (+2 because of additinal GOTO Delay_1b_loop)
		movlw	h'5B'
		movwf	PAUZA1
		GOTO	Delay_1b_loop
Delay_420us				;415 cycles (+2 because of additinal GOTO Delay_1b_loop)
		movlw	h'8A'
		movwf	PAUZA1
		GOTO	Delay_1b_loop
Delay_444us				;439 cycles
		movlw	h'92'
		movwf	PAUZA1
		;GOTO	Delay_1b_loop ; OPTIMIZATION NOTE: removed becaues we are just bove the Delay_1b_loop
Delay_1b_loop
		decfsz	PAUZA1, 1
		goto	Delay_1b_loop
		goto	$+1		;2 cycles
		return			;4 cycles (including call)

Delay_10ms				;9993 cycles (+2 because of additinal GOTO Delay_1b_loop)
		movlw	h'CE'
		movwf	PAUZA1
		movlw	h'08'
		movwf	PAUZA2
		GOTO	Delay_2b_loop
Delay_50ms				;49998 cycles (+2 because of additinal GOTO Delay_1b_loop)
		movlw	h'0F'
		movwf	PAUZA1
		movlw	h'28'
		movwf	PAUZA2
		GOTO	Delay_2b_loop
Delay_01s				;99993 cycles
		movlw	h'1E'
		movwf	PAUZA1
		movlw	h'4F'
		movwf	PAUZA2
		;GOTO	Delay_2b_loop ; OPTIMIZATION NOTE: removed becaues we are just bove the Delay_1b_loop
Delay_2b_loop
		decfsz	PAUZA1, 1
		goto	$+2
		decfsz	PAUZA2, 1
		goto	Delay_2b_loop
		goto	$+1		;3 cycles
		nop
		return			;4 cycles (including call)

;***** Kraj *****

		END

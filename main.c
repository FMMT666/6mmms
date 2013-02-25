//
//  6mmm/s   V0.5
// ===============
//
//  (c)2012 ASkr
//  www.askrprojects.net
//
//
//  NOTICE:
//    Code uses SourceBoost C Compiler notation.
//
//
//
//  TODO:
//   - no RX code (no space left)
//   - math is not perfect (rounding)
//   - ...
//
//  CHANGES V0.5, 26.09.2012 (D/M/Y):
//   - wait for push button being released after start-up screen to
//     avoid "set mass" dialog
//   - changed display init, decreased brightness/contrast to match
//     +5V operatin voltage (init was copied from 3.3V project ;-)
//   - first, few memory saving actions taken
//     BEFORE:  18 RAM, 159 ROM
//     AFTER:   18 RAM, 159 ROM
//
//  CHANGES V0.4, 21.08.2012 (D/M/Y):
//   - set default mass to 1.4g (Nerf whistler dart)
//
//  CHANGES V0.3, 28.07.2012 (D/M/Y):
//   - added rounded values (aka.: snipped remainder)
//   - changed comparator reference from 0.6V to 1.2V
//   - added potentiometer-button routines
//   - added "set new mass" feature via pot-button
//
//  CHANGES V0.2:
//   - added display
// 
//  CHANGES V0.1:
//   - initial release
//


#include <system.h>
#include <string.h>
#include <stdlib.h>
#include <memory.h>

//                  PIC16HV616
//              +-------\/-------+
//        VDD ->| Vdd        GND |<- GND
//     PINDBG <-| RA5        RA0 |-> DISD/PGD
//         TX <-| RA4        RA1 |-> DISC/PGC
//     RX/VPP ->| RA3        AN2 |<- but/pot
//             -| RC5        RC0 |-> DISCS
//             -| C2OUT  C12IN1- |<- C1<-MEAS1
//  MEAS2->C2 ->| C12IN3-    RC2 |-> DISRS
//              +----------------+
//


// CHIP CONFIG
#pragma DATA _CONFIG,0x3DC4		// MCLR as PIN
//#pragma DATA _CONFIG,0x3DE4  // MCLR as MCLR
#pragma CLOCK_FREQ 8000000


// STATES (MEASUREMENT)
#define STATE_OFF       0
#define STATE_WAIT_1    1
#define STATE_WAIT_2    2
#define STATE_DONE      3

// DISPLAY
#define DISPLAY_CMD			0			// command to write
#define DISPLAY_DAT			1			// data to send

// POTENTIOMETER
#define POT_BUTTRIG    1000   // any value above this will trigger a "button event"
#define POT_MASS_SCALE    4   // mass = AD-value * POT_MASS_SCALE

// OTHER
#define MAX_MASS       3000		// maximum value of mass
#define MIN_MASS          1   // minimum value of mass (must not be 0)
#define MAX_TIME      62500		// maximum time (measured)
#define MIN_TIME        286   // minimum time (measured)
#define DEFAULT_MASS   1400   // default mass in in mg (1E-6 kg) max: 3000!

// MACROS
#define PINDCLK(a)	(porta.RA1 = (a))				// display clock
#define PINDDAT(a)	(porta.RA0 = (a))				// display data
#define PINDCS(a)		(portc.RC0 = (a))				// display cs
#define PINDRS(a)		(portc.RC2 = (a))				// display rs
#define PINDBG(a)		(porta.RA5 = (a))				// DEBUG timing pin; high during measurement
#define PINTX(a)    (porta.RA4 = (a))				// soft-UART TX pin
#define TIM1(a)     (t1con.TMR1ON = (a))		// timer 1 run control
#define INTS_OFF		(pie1 = 0)
#define INTS_WAIT_1 (pie1 = 0b00001000)			// C1 enabled
#define INTS_WAIT_2 (pie1 = 0b00010001)			// C2 and T1 (timeout) enabled


// GLOBALS
volatile unsigned char gState = STATE_OFF;
volatile unsigned int  gTime  = 0;
unsigned int           gWeight = DEFAULT_MASS;




//************************************************************************************************
//***
//***
//***
//************************************************************************************************
void PicInit()
{
	// pulls
	option_reg = 0b01111111;	// enable pulls (PORTA)
	
	// pins and ports
	porta = 0b11111111;
	portc = 0b11111111;
	wpua  = 0b00000100;	// RA2 pull
	trisa = 0b00000100;	// RA2 but/pot
	trisc = 0b00001010;	// C12IN1- (AN5) and C12IN3- (AN7)
	ansel = 0b10100100;	// C12IN1- (AN5), C12IN3- (AN7), AN2

	// comparators and references, part of timer1
//	vrcon   = 0b00110000; // use 0.6V ref on C1 and C2  *** changed in V0.3 ***
	vrcon   = 0b11110110; // use CVREF ref on C1 and C2, low range, 1.25V
	
	cm2con1 = 0b00001100; // C1/C2 hyst on; timer1 fosc/4
	cm1con0 = 0b10000101; // ON, NINV, VREF, no OUT, IN1-
	cm2con0 = 0b10000111; // ON, NINV, VREF, no OUT, IN3-

	// timer2 (soft UART)
	pr2   = 208;
	t2con = 0b00000100;		// ->104us overflow -> 9600bits/s
	
	// timer1 (measurement)
	t1con = 0b00000000;		// pre 1:1, internal, OFF (fosc/4 see above); 500ns/tick

	// a/d converter
	adcon0 = 0b10001001;	// A/D converter on, ref Vdd, AN2
	adcon1 = 0b00100000;	// A/D clock 1/32	
	
	// interrupts
	pir1   = 0;
	pie1   = 0;						// just in case (brown out, reset)
	intcon = 0b11000000;	// GIE, PEIE	
	
}






//************************************************************************************************
//***
//***
//***
//************************************************************************************************
void interrupt()
{
	
	// DEBUG ONLY
	PINDBG( 1 );
	
	switch( gState )
	{
		//-----------------------------------------------------------------------------------------
		// wait for COMP1
		case STATE_WAIT_1:
		{
			TIM1( 1 );
			gState++;			// advance to next state
			INTS_WAIT_2;	// next interrupt state
		}break;
		
		//-----------------------------------------------------------------------------------------
		// wait for COMP2 of TIMER1 (timeout)
		case STATE_WAIT_2:
		{
			TIM1( 0 );
			// check if this was a valid object detection (COMP2 int)
			if( pir1 & 0b00010000 )
			{
				gTime = ( (unsigned)tmr1h << 8 ) | tmr1l;
				// limit to 1.6 - 350.0 m/s
				if( ( gTime > MAX_TIME ) || (gTime < MIN_TIME ) )
				{
					// back to state 1
					gState = STATE_WAIT_1;
					INTS_WAIT_1;
				}
				else
				{
					// measurement valid
					gState = STATE_DONE;
					INTS_OFF;
				}
			}
			else
			// no, this was TIMER1 interrupt (timeout)
			{
				// back to state 1
				gState = STATE_WAIT_1;
				INTS_WAIT_1;
			}
			
			tmr1l = 0;
			tmr1h = 0;
			
		}break;

		//-----------------------------------------------------------------------------------------
		// something went wrong...
		default:
		{
			gState = STATE_WAIT_1;
			INTS_WAIT_1;
		}


	}

	pir1 &= 0b00000010; // clear all except TMR2IF
	
	
	// DEBUG ONLY
	PINDBG( 0 );
	
	
}




//**************************************************************************************
//***
//***
//***
//**************************************************************************************
/*
char *lltoa(char *buffer, long i)
{
	unsigned long n;
	unsigned int negate = 0;
	int c = 24;

	if (i < 0)
	{
		negate=1;
		n = -i;
	}
	else
		if (i == 0)
		{
			buffer[0] = '0';
			buffer[1] = 0;
			return buffer;
		}
		else
		{
			n = i;
		}
	
	buffer[c--] = 0;
	do
	{
		buffer[c--] = (n % 10) + '0';
		n = n / 10;
	} while (n);
	
	if (negate)
	{
		buffer[c--] = '-';
	}

	return &buffer[c+1];
}
*/



//************************************************************************************************
//***
//***
//***
//************************************************************************************************
void SerialSendChar(unsigned char ch)
{
	unsigned char i;
	
	pir1.TMR2IF = 0;
	while( pir1.TMR2IF == 0)
	{;}
	pir1.TMR2IF = 0;
	
	for( i=0; i<10; i++ )
	{
		switch( i )
		{
			case 0: PINTX( 0 ); break; // startbit
			case 9: PINTX( 1 ); break; // stopbit
			default:
				PINTX( ch & 0x01 );
				ch >>= 1;
		}
		while( pir1.TMR2IF == 0)
		{;}
		pir1.TMR2IF = 0;
	}
}


//************************************************************************************************
//***
//***
//***
//************************************************************************************************
void SerialSendString(unsigned char *ch)
{
	if( ch == NULL )
		return;
	while( *ch != 0 )
		SerialSendChar( *ch++ );
}



//************************************************************************************************
//***
//***
//***
//************************************************************************************************
int AdjustString(unsigned char *out, unsigned char *in, int digits)
{
	int i, j;
	
	if( ( in == NULL ) || ( out == NULL) )
		return 0;
		
	i = strlen( in );
	if( i == 0 )
		return 0;
		
	if( i <= digits )
	{
		*out++ = '0';
		*out++ = '.';
		while( i < digits-- )
			*out++ = '0';
		digits = 0;
	}

	for( j=0; j<i; j++ )
	{
		if( i-j == digits )
			*out++ = '.';
		else
			*out++ = *in++;
	}
	*out = 0;

	return 1;	
}



//************************************************************************************************
//***
//***
//***
//************************************************************************************************
int AdjustRemainder( unsigned char *out, unsigned char *in, int commaDigits )
{
	unsigned char trig = 0;
	unsigned char i;
	
	if( ( in == NULL ) || ( out == NULL) || ( commaDigits < 0 ) )
		return 0;
		
	i = strlen( in );
	if( i == 0 )
		return 0;

	commaDigits++;

	while( *in )
	{
		if( *in == '.' )
			trig++; // always 1

		if( trig )
		{
			if( commaDigits == 0 )
			{
				*out = 0;
				break;
			}
			commaDigits--;
		}
		
		*out++ = *in++;
		
	}
	
	return 1;
	
}



//************************************************************************************************
//***
//***
//*** ~70us
//************************************************************************************************
void DisplayWrite(unsigned char rs, unsigned char dat)
{
	unsigned char i;
	
	PINDCS( 0 );
	PINDRS( rs );
	
	for( i=0; i<8; i++ )
	{
		if( dat & 0x80 )
			PINDDAT( 1 );
		else
		{
			PINDDAT( 0 );
			PINDDAT( 0 );
		}
			
		PINDCLK( 1 );
		dat <<= 1;
		PINDCLK( 0 );
	}
	PINDCS( 1 );
	
//	delay_us( 30 );
}



//************************************************************************************************
//***
//***
//***
//************************************************************************************************
void DisplayClear()
{
	DisplayWrite( DISPLAY_CMD, 0x01 );
	delay_ms( 2 );
}



//************************************************************************************************
//***
//***
//***
//************************************************************************************************
void DisplayHome()
{
	DisplayWrite( DISPLAY_CMD, 0x02 );
	delay_ms( 2 );
}



//************************************************************************************************
//***
//***
//***
//************************************************************************************************
void DisplayWriteString( unsigned char *dat )
{
	if( dat == NULL )
		return;
	
	while( *dat != NULL )
		DisplayWrite( DISPLAY_DAT, *dat++ );
}



//************************************************************************************************
//***
//***
//***
//************************************************************************************************
void DisplaySetCursor( unsigned char x, unsigned char y )
{
	if( x > 15 )
		x = 15;
	if( y > 2 )
		y = 2;
	DisplayWrite( DISPLAY_CMD, 0x80 + y*16 + x );
}



//************************************************************************************************
//***
//***
//***
//************************************************************************************************
void DisplayClearLine( unsigned char line, unsigned char column )
{
	if( line > 2 )
		return;
	DisplayWriteString("                ");
	if( column < 16 )
		DisplaySetCursor( column, line );
}



//************************************************************************************************
//***
//***
//***
//************************************************************************************************
void DisplayInit()
{
	DisplayWrite( DISPLAY_CMD, 0x39 );
	DisplayWrite( DISPLAY_CMD, 0x1D );
	DisplayWrite( DISPLAY_CMD, 0x50 );
	DisplayWrite( DISPLAY_CMD, 0x6C );
	DisplayWrite( DISPLAY_CMD, 0x76 );				// *NEW* V0.5: 0x7C -> 0x76
	DisplayWrite( DISPLAY_CMD, 0x38 );
	DisplayWrite( DISPLAY_CMD, 0b00001100 );
	DisplayWrite( DISPLAY_CMD, 0x01 );
	delay_ms( 2 );
	DisplayWrite( DISPLAY_CMD, 0x06 );
}



//************************************************************************************************
//***
//***
//***
//************************************************************************************************
void CalcResults()
{
	unsigned long time;
	unsigned long speed;
	unsigned long energy;
	unsigned char str1[26];						// TODO: this is not true anymore (str1 and str2 are swapped)
	unsigned char str2[27];
	unsigned int  i,j;
	
	//-----------------------------------------------------------------------------------
	// calc
	// TODO: adaptive precision improvement (no fixed divisions, etc...)
	if( gTime > 0 )
	{
		time = gTime;										// 1 digit = 10us (per 1m)
		speed = (1000000000L / time);		// 1 digit = 0.1mm/s (1E-4 m/s) (speed  max: 3.5E6)
		energy = speed/100;							// 1 digit = 0.1 m/s (1E-1 m/s) (energy max: 34965)
		energy *= energy;								// max 1.2E9
		energy /= 1000;									// 1 digit = 0.1 (m/s)^2 (max 1.2E6)
		energy >>= 1;                   // divide by 2 (max 611k)
		energy *= gWeight;							// 1E-7 J (max 1.8E9)
	}
	else
	{
		speed = 0;
		energy = 0;
	}


	//-----------------------------------------------------------------------------------
	// output

	DisplayClear();
	DisplayHome();

	// speed
	SerialSendString("\r\n---\r\n");
	ltoa( speed, &str1, 10 );
	AdjustString( &str2, &str1, 4 );
	AdjustRemainder( &str1, &str2, 2 );
	SerialSendString(" v = ");
	SerialSendString( &str1 );
	SerialSendString(" m/s\r\n");

	DisplayWriteString(" v = ");
	DisplayWriteString( &str1 );
	DisplayWriteString(" m/s");

	// mass
	ltoa( gWeight, &str1, 10);
	AdjustString( &str2, &str1, 3);
	SerialSendString(" m = ");
	SerialSendString( &str2 );
	SerialSendString(" g\r\n");

	DisplaySetCursor( 0, 1 );
	DisplayWriteString(" m = ");
	DisplayWriteString( &str2 );
	DisplayWriteString(" g");

	// energy
	ltoa( energy, &str1, 10 );
	AdjustString( &str2, &str1, 7 );
	AdjustRemainder( &str1, &str2, 3 );
	
	SerialSendString(" E = ");
	SerialSendString( &str1 );
	SerialSendString(" J\r\n");

	DisplaySetCursor( 0, 2 );
	DisplayWriteString(" E = ");
	DisplayWriteString( &str1 );
	DisplayWriteString(" J");
	
}



//************************************************************************************************
//***
//***
//***
//************************************************************************************************
unsigned int MeasurePot()
{
	adcon0.GO = 1;
	while( adcon0.GO )
		;
	return ( adresh * 256 ) + adresl;
}



//************************************************************************************************
//***
//***
//***
//************************************************************************************************
void InputNewMass()
{
	unsigned int i, lastValue = DEFAULT_MASS;
	unsigned char str1[6];
	unsigned char str2[6];


	DisplayClear();
	DisplayHome();
	DisplayWriteString("  SET NEW MASS  ");
	
	do
	{
		delay_ms( 100 );
	}while( MeasurePot() > POT_BUTTRIG );
	
	delay_ms( 50 );
	
	for(;;)
	{
		// measure potentiometer
		i = MeasurePot();
		if( i <= POT_BUTTRIG )
			lastValue = i * POT_MASS_SCALE;
		else
			break;

		if( lastValue > MAX_MASS )
			lastValue = MAX_MASS;
		else
		{
			if( lastValue < MIN_MASS ) // TODO: dangerous (MIN_MASS == 0)
				lastValue = MIN_MASS;
		}

		delay_ms( 50 );
			
		// show new value (display and serial)
		ltoa( lastValue, &str1, 10 );
		AdjustString( &str2, &str1, 3 );
		
		SerialSendString(" m = ");
		SerialSendString( &str2 );
		SerialSendString(" g\r\n");
		
		DisplaySetCursor( 0, 2 );
		DisplayWriteString("  m = ");
		DisplayWriteString( &str2 );
		DisplayWriteString(" g  ");
	}

	gWeight = lastValue;

	DisplayClear();
	DisplaySetCursor( 0, 1 );
	DisplayWriteString("      DONE      ");


	do
	{
		delay_ms( 100 );
	}while( MeasurePot() > POT_BUTTRIG );
	
}



//************************************************************************************************
//***
//***
//***
//************************************************************************************************
void main( void )
{
	unsigned char buttonEntersMassMenu = 0;
	
	
	// PIC init
	PicInit();
	
	PINTX( 1 );
	PINDCLK( 0 );
	
	
	// display init
	delay_ms( 100 );
	DisplayInit();
	
	DisplayClear();
	DisplayHome();

	DisplayWriteString("  6mmm/s  V0.5  ");
	DisplayWriteString(">--------------<");
	DisplayWriteString("(C)2012     ASkr");
	
	// we start in measurement mode
	gState = STATE_WAIT_1;
	gTime  = 0;
	INTS_WAIT_1;


	// main loop
	for(;;)
	{

		//-----------------------------------------------------------------------------------
		// wait for measurement (or button)
		while( gState != STATE_DONE )
		{
			// TODO: disable interrupt
			if( MeasurePot() > POT_BUTTRIG )
			{
				// *NEW* V0.5; button pressed the first time?
				if( buttonEntersMassMenu )
				{
					InputNewMass();
					
					// There's something wrong here (Sourceboost 7.05).
					// After Calling "CalcResults()", the code runs through
					// the line marked with **********:
					//  - check interrupts
					//  - check memory (PCLATH)
					CalcResults();
				}
				else
				{
					// If we're still in the init screen, the button won't
					// trigger the set mass menu, but simply bring us out
					// of there...
					buttonEntersMassMenu = 1;
					
					DisplayClear();
					DisplayHome();
				
					DisplayWriteString("                ");
					DisplayWriteString("     READY?     ");
					
					while( MeasurePot() > POT_BUTTRIG )
					{;}
					
					// show measurement screen
					CalcResults();
				}
				
			}// END Pot > TRIG
		}// END while loop

		CalcResults();

		//-----------------------------------------------------------------------------------
		// re-arm measurement mode
		gState = STATE_WAIT_1;
//		gTime  = 0;  // ********** disabled, see above...
		INTS_WAIT_1;

		// In case the measurement was don during the init screen phase,
		// make the button work on the first press afterwards...
		buttonEntersMassMenu = 1;
		
	}// END main loop

	
}





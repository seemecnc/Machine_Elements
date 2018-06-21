/*
 * StrainEffector.cpp
 *
 * Created: 18/03/2017 15:13:49
 * Author : David Crocker, Escher Technologies Limited
 * License: GNU GPL version 3
 *
 * Version 1.3, 2017-07-31
 * 
 * adopted by SeeMeCNC 2018 full credit to Escher Technologies
 * GNU GPL version 3
 */ 

#include "ecv.h"

#ifdef __ECV__
#define __attribute__(_x)
#define __volatile__
#pragma ECV noverifyincludefiles
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <avr/fuse.h>

#ifdef __ECV__
#pragma ECV verifyincludefiles
#undef cli
#undef sei
extern void cli();
extern void sei();
#endif

#define ISR_DEBUG	(0)

#define BITVAL(_x) static_cast<uint8_t>(1u << (_x))

// Pin assignments:
// PB0/MOSI/OC0A/~OC1A	debug output, unused (except for programming) if debug disabled
// PB1/MISO/OC1A		output, high indicates triggered and turns on LED
// PB2/ADC1/SCK			input from Duet, used for programming the sensitivity
// PB3/ADC3				unused
// PB4/ADC2				input from strain amplifier
// PB5/ADC0/RESET		not available, used for programming

const unsigned int PortBDuetOutputBitNum = 1;
const unsigned int PortBDuetInputBitNum = 2;

#if ISR_DEBUG
const unsigned int PortBDebugBitNum = 0;					// use PB0/MOSI for debugging
const uint8_t PortBUnusedBitMask = BITVAL(3);				// bit mask for unused port B pins so we can enable pullup resistors on them
#else
const uint8_t PortBUnusedBitMask = BITVAL(0) | BITVAL(3);	// bit mask for unused port B pins so we can enable pullup resistors on them
#endif

const unsigned int AdcStrainInputChan = 2;					// ADC channel for the strain amplifier output
const unsigned int DidrStrainBitNum = 4;

// Approximate MPU frequency (8MHz internal RC oscillator)
const uint32_t F_CPU = 8000000uL;

const uint32_t fastTickFrequency = F_CPU/8;					// nominal TC0 clock rate 1MHz using prescaler = 8
const uint16_t slowTickFrequency = fastTickFrequency/256;	// nominal timer 0 interrupt frequency, about 3906.25Hz (rounds down to 3906)

const uint16_t strainReadingsAveraged = 16;					// 16 readings @ 2 bytes each = 32 bytes

const uint16_t kickFrequency = 10;							// how often we kick the watchdog
const uint16_t kickIntervalTicks = slowTickFrequency/kickFrequency;

const unsigned int avShift = 12;							// a shift of 13 allows a 50% decay in 4096 cycles which is about 0.21 seconds
const unsigned int defaultThresholdMilliVolts = 51;			// trigger threshold in mV (51 here corresponds to programming a value of 50)
const uint16_t ledOnTicks = slowTickFrequency/50;			// leave LED on for 100ms - partdaddy original was 10
const uint16_t ledFlashTicks = slowTickFrequency/5;			// flash LED time - partdaddy original was 4 for 0.25s

const unsigned int EdgeCaptureBufferEntries = 20;			// the size of our edge capture buffer, must be an even number
const uint16_t BitsPerSecond = 1000;						// the nominal bit rate that the data is transmitted at
const uint32_t NominalBitLength = fastTickFrequency/BitsPerSecond;
const uint16_t MinBitLength = (NominalBitLength * 10)/13;	// allow 30% clock speed tolerance
const uint16_t MaxBitLength = (NominalBitLength * 13)/10;	// allow 30% clock speed tolerance
const uint16_t ErrorRecoveryDelayBits = 25;					// after an error we wait for the line to be low for this long
static_assert(NominalBitLength * ErrorRecoveryDelayBits < (1ul << 16), "Potential overflow in bit timing calculations");
const uint16_t ErrorRecoveryTime = NominalBitLength * ErrorRecoveryDelayBits;

const unsigned int MaxBytesReceived = 3;					// maximum number of received bytes in a programming message
const uint8_t ProgByte = 105;								// magic byte to start programming
const uint8_t EraseByte = 131;								// magic byte to clear EEPROM

struct NvData
{
	uint16_t sensitivity;
	uint16_t checksum;
	static const  uint16_t magic = 0x52A6;
} nvData;

#ifndef __ECV__
FUSES = {0xE2u, 0xDFu, 0xFFu};		// 8MHz RC clock
#endif

// Filter class. This maintains the sum of the last 'strainReadingsAveraged' values passed to it.
class Filter
{
public:
	uint16_t readings[strainReadingsAveraged];
	volatile uint16_t sum;
	uint8_t index;

	// Initialise the filter
	void Init(uint16_t initialValue)
	post(FilterInvariant());

	// Add a reading to the filter
	void AddReading(uint16_t adcVal)
	pre(FilterInvariant())
	post(FilterInvariant())
	{
		//partdaddy
		//sum = sum - readings[index] + adcVal;
		sum = sum - readings[index] + adcVal;
		readings[index] = adcVal;
		index = static_cast<uint8_t>((index + 1u) % strainReadingsAveraged);
	}

	ghost(
		bool FilterInvariant() const
		returns((forall r in readings :- r <= 1023) && sum == + over readings);
	)
};

void Filter::Init(uint16_t initialValue)
{
	for (uint8_t i = 0; i < strainReadingsAveraged; ++i)
	writes(i; readings; volatile)
	keep(i <= strainReadingsAveraged)
	keep(forall j in 0..(i-1) :- readings[j] == 0)
	decrease(strainReadingsAveraged - i)
	{
		readings[i] = initialValue;
	}
	index = 0;
	sum = initialValue * strainReadingsAveraged;
}

Filter strainFilter;
volatile uint16_t tickCounter = 0;
uint16_t lastKickTicks = 0;									// when we last kicked the watchdog
uint16_t timeOn = 0;										// when we last turned the LED on

volatile uint32_t rollingAverage = 0;
volatile uint16_t shiftedRollingAverage = 0;				// this is always equal to (rollingAverage >> avShift)

uint16_t threshold;											// this sets the sensitivity

volatile uint16_t edgeCaptures[EdgeCaptureBufferEntries];
typedef uint8_t invariant(value <= EdgeCaptureBufferEntries) edgeCaptureIndex_t;
volatile edgeCaptureIndex_t numberOfEdgesCaptured = 0;

uint8_t bytesReceived[MaxBytesReceived];
typedef uint8_t invariant(value <= MaxBytesReceived) numBytesReceived_t;
numBytesReceived_t numBytesReceived = 0;											

ghost(
	bool AverageInvariant() const
		returns(shiftedRollingAverage == rollingAverage >> avShift);
)

template<class X> inline X min(X _a, X _b)
{
	return (_a < _b) ? _a : _b;
}

template<class X> inline X max(X _a, X _b)
{
	return (_a > _b) ? _a : _b;
}

// EEPROM access functions
void readEEPROM(uint8_t ucAddress, uint8_t * array p, uint8_t len)
pre(len != 0)
{
	do
	writes(p.all; volatile)
	{
		while ((EECR & (1u << EEPE)) != 0) { }	// Wait for completion of previous write
		EEAR = ucAddress++;						// Set up address register
		EECR |= (1u << EERE);					// Start eeprom read by writing EERE
		*p++ = EEDR;							// Return data from data register
	} while (--len != 0);
}

void writeEEPROM(uint8_t ucAddress, const uint8_t * array p, uint8_t len)
pre(len != 0)
{
	do
	writes(volatile)
	{
		while ((EECR & (1u << EEPE)) != 0) { }	// Wait for completion of previous write
		EECR = (0u << EEPM1) | (0u << EEPM0);	// Set Programming mode
		EEAR = ucAddress++;						// Set up address and data registers
		EEDR = *p++;
		EECR |= (1u << EEMPE);					// Write logical one to EEMPE
		EECR |= (1u << EEPE);					// Start eeprom write by setting EEPE
	} while (--len != 0);
}

// ISR for the timer 0 compare match interrupt
#ifdef __ECV__
void ADC_vect()
#else
ISR(ADC_vect)
#endif
writes(strainFilter; volatile)
pre(strainFilter.FilterInvariant(); AverageInvariant())
post(strainFilter.FilterInvariant(); AverageInvariant())
{
#if ISR_DEBUG
	PORTB |= BITVAL(PortBDebugBitNum);
#endif

	const uint16_t adcVal = ADC & 1023u;						// get the ADC reading from the previous conversion
	strainFilter.AddReading(adcVal);
	rollingAverage = rollingAverage + strainFilter.sum - shiftedRollingAverage;
	shiftedRollingAverage = (uint16_t)(rollingAverage >> avShift);

#if ISR_DEBUG
	PORTB &= ~BITVAL(PortBDebugBitNum);
#endif
}

// Get the time now - must be called with interrupts disabled
inline uint16_t IGetTimeNow()
{
	uint8_t tim = TCNT0;
	const bool ovf = (TIFR & BITVAL(TOV0)) != 0;
	if (ovf)										// check for consistent overflow flag
	{
		tim = TCNT0;								// counter overflowed around the time we read it, so read it again
	}
	return (((ovf) ? tickCounter + 1 : tickCounter) << 8) | tim;
}

// Get the time now
uint16_t GetTimeNow()
{
	cli();
	const uint16_t ret = IGetTimeNow();
	sei();
	return ret;
}

// ISR for handling changes to the state of the input pin
// We store the times of transitions of the input pin in a buffer, with low to high transitions at even indices
#ifdef __ECV__
void INT0_vect()
#else
ISR(INT0_vect)
#endif
{
	const uint8_t numEdgesCaptured = numberOfEdgesCaptured;			// capture volatile variable
	if (numEdgesCaptured < EdgeCaptureBufferEntries && ((numEdgesCaptured ^ (PINB >> PortBDuetInputBitNum)) & 1u) != 0)		// low-to-high transitions must be stored at even indices
	{
		edgeCaptures[numEdgesCaptured] = IGetTimeNow();			// record the time at which this edge was detected
		numberOfEdgesCaptured = numEdgesCaptured + 1;
	}
}

// ISR for handling timer 0 overflow interrupt
#ifdef __ECV__
void TIMER0_OVF_vect()
#else
ISR(TIMER0_OVF_vect)
#endif
{
	++tickCounter;
}

// Get a 16-bit volatile value from outside the ISR. As it's more than 8 bits long, we need to disable interrupts while fetching it.
inline uint16_t GetVolatileWord(volatile uint16_t& val)
writes(volatile)
{
	cli();
	const uint16_t locVal = val;
	sei();
	return locVal;
}

// Check whether we need to kick the watchdog
void CheckWatchdog()
writes(lastKickTicks; volatile)
{
	if (GetVolatileWord(tickCounter) - lastKickTicks >= kickIntervalTicks)
	{
#ifndef __ECV__
		wdt_reset();											// kick the watchdog
#endif
		lastKickTicks += kickIntervalTicks;
	}
}

// Delay for a specified number of ticks, kicking the watchdog as needed
void DelayTicks(uint16_t ticks)
writes(lastKickTicks; volatile)
{
	const uint16_t startTicks = GetVolatileWord(tickCounter);
	for (;;)
	{
		CheckWatchdog();
		if (GetVolatileWord(tickCounter) - startTicks >= ticks)
		{
			break;
		}
	}
}

inline void SetOutputOn()
writes(volatile)
{
	PORTB |= BITVAL(PortBDuetOutputBitNum);
}

inline void SetOutputOff()
writes(volatile)
{
	PORTB &= ~(BITVAL(PortBDuetOutputBitNum));
}

void UpdateThreshold(uint16_t mv)
{
	threshold = ((uint32_t)mv * strainReadingsAveraged * 1024u)/1100u;		// we use the 1.1V internal voltage reference for the ADC
}

// Flash the LED the specified number of times
void FlashLed(uint8_t numFlashes)
{
	SetOutputOff();
	DelayTicks(ledFlashTicks);
	while (numFlashes != 0)
	{
		SetOutputOn();
		DelayTicks(ledFlashTicks);
		SetOutputOff();
		DelayTicks(ledFlashTicks);
		--numFlashes;
	}
}

// Check whether we have received a valid message.
void CheckReceivedMessage()
pre(numBytesReceived != 0; numBytesReceived <= MaxBytesReceived)
post(numBytesReceived < MaxBytesReceived)
{
	switch (bytesReceived[0])
	{
	case EraseByte:
		if (numBytesReceived >= 2)
		{
			if (bytesReceived[1] == EraseByte)
			{
				nvData.sensitivity = nvData.checksum = 0xFFFFu;
				UpdateThreshold(defaultThresholdMilliVolts);
				writeEEPROM(0, reinterpret_cast<const uint8_t* array>(&nvData), sizeof(nvData));
				FlashLed(5);
			}
			numBytesReceived = 0;
		}
		break;

	case ProgByte:
		if (numBytesReceived >= 3)
		{
			if ((bytesReceived[2] ^ bytesReceived[1]) == 0x00FF)
			{
				// Valid programming message received
				nvData.sensitivity = (uint16_t)bytesReceived[1] + 1u;		// received values of 0-255 give sensitivity of 1-256mV
				nvData.checksum = NvData::magic ^ nvData.sensitivity;
				UpdateThreshold(nvData.sensitivity);
				writeEEPROM(0, reinterpret_cast<const uint8_t* array>(&nvData), sizeof(nvData));
				FlashLed(4);
			}
			numBytesReceived = 0;
		}
		break;

	default:
		numBytesReceived = 0;			// invalid first byte
		break;
	}
}

// Check for programming commands. The protocol for each byte sent is:
//  0 0 1 0 b7 b6 b5 b4 /b4 b3 b2 b1 b0 /b0
// with a return to 0 after the last data byte. So the pattern with the minimum number of edges (4) is:
//  0 0 1 0 0 0 0 0 1 1 1 1 1 0
//      ^ ^         ^         ^
// A pattern with the maximum number of edges (12) is:
//  0 0 1 0 1 0 1 0 1 0 1 0 1 0
// The final transition to zero may occur after the end of the data, 12 bit times after the initial transition, e.g.:
//  0 0 1 0 1 1 1 1 0 0 0 0 0 1 0
// The maximum interval between edges in a single transmitted byte is 5 bit times.
void CheckForReceivedData()
pre(numBytesReceived < MaxBytesReceived)
post(numBytesReceived < MaxBytesReceived)
{
	enum class RxdState : uint8_t
	{
		waitingForStartBit,
		waitingForHighNibble,
		waitingForLowNibble,
		errorRecovery
	};

	static RxdState state = RxdState::waitingForStartBit;
	static uint16_t startBitLength;
	static uint16_t stuffingBitStart;
	static uint16_t errorRecoveryStartTime;
	static uint8_t highNibble;
	static uint8_t bitChangeIndex;

	const uint8_t numEdgesCaptured = numberOfEdgesCaptured;			// capture volatile variable
	const uint16_t now = GetTimeNow();
	switch (state)
	{
	case RxdState::waitingForStartBit:
		if (numEdgesCaptured >= 2)
		{
			// Check for a valid start bit
			startBitLength = edgeCaptures[1] - edgeCaptures[0];
			state = (startBitLength >= MinBitLength && startBitLength <= MaxBitLength) ? RxdState::waitingForHighNibble : RxdState::errorRecovery;
		}
		break;

	case RxdState::waitingForHighNibble:
		if (now - edgeCaptures[1] > (13 * startBitLength)/2)
		{
			// 6.5 bit times have passed since the end of the start bit, so we should have the high nibble and the following stuffing bit
			uint16_t samplePoint = (startBitLength * 3)/2;		// sampling time after the end of the start bit for bit 7 (MSB)
			bitChangeIndex = 2;
			highNibble = 0;
			for (uint8_t numBits = 0; numBits < 5; ++numBits)
			{
				if (bitChangeIndex < numEdgesCaptured && edgeCaptures[bitChangeIndex] - edgeCaptures[1] < samplePoint)
				{
					++bitChangeIndex;
					if (bitChangeIndex < numEdgesCaptured && edgeCaptures[bitChangeIndex] - edgeCaptures[1] < samplePoint)
					{
						state = RxdState::errorRecovery;		// there should be at most 1 transition per bit
						return;
					}
				}
				highNibble <<= 1;
				if ((bitChangeIndex & 1u) != 0)
				{
					highNibble |= 1u;
				}
				samplePoint += startBitLength;
			}

			// The 5th bit we received should be the inverse of the 4th bit
			if ((((highNibble >> 1u) ^ highNibble) & 0x01u) == 0)
			{
				state = RxdState::errorRecovery;
				return;
			}

			stuffingBitStart = edgeCaptures[bitChangeIndex - 1];
			state = RxdState::waitingForLowNibble;
		}
		break;

	case RxdState::waitingForLowNibble:
		if (now - stuffingBitStart > (13 * startBitLength)/2)
		{
			// 6.5 bit times have passed since the start of the first stuffing bit, so we should have the high nibble and the second stuffing bit
			uint16_t samplePoint = (3 * startBitLength)/2;			// resync on the start of the stuffing bit
			uint8_t lowNibble = 0;
			for (uint8_t numBits = 0; numBits < 5; ++numBits)
			{
				if (bitChangeIndex < numEdgesCaptured && edgeCaptures[bitChangeIndex] - stuffingBitStart < samplePoint)
				{
					++bitChangeIndex;
					if (bitChangeIndex < numEdgesCaptured && edgeCaptures[bitChangeIndex] - stuffingBitStart < samplePoint)
					{
						state = RxdState::errorRecovery;
						return;
					}
				}
				lowNibble <<= 1;
				if ((bitChangeIndex & 1u) != 0)
				{
					lowNibble |= 1u;
				}
				samplePoint += startBitLength;
			}

			// The 10th bit we received should be the inverse of the 9th bit
			if ((((lowNibble >> 1u) ^ lowNibble) & 0x01u) == 0)
			{
				state = RxdState::errorRecovery;
				return;
			}

			numberOfEdgesCaptured = 0;				// ready for a new byte
			bytesReceived[numBytesReceived++] = ((highNibble << 3u) & 0xF0u) | (lowNibble >> 1u);
			CheckReceivedMessage();
			state = RxdState::waitingForStartBit;
		}
		break;

	case RxdState::errorRecovery:
		if ((PINB & BITVAL(PortBDuetInputBitNum)) != 0 || numEdgesCaptured != 0)	// when we first enter this state, numEdgesCaptured is always nonzero
		{
			errorRecoveryStartTime = now;
			numberOfEdgesCaptured = 0;
			numBytesReceived = 0;
		}
		else if (now - errorRecoveryStartTime >= ErrorRecoveryTime)
		{
			state = RxdState::waitingForStartBit;
		}
		break;
	}
}

int main(void)
writes(strainFilter; volatile)
{
	cli();

	strainFilter.Init(512);

	// Initialise the I/O ports
	DIDR0 = BITVAL(DidrStrainBitNum);							// disable digital input buffers on ADC pins
	PORTB = PortBUnusedBitMask | BITVAL(PortBDuetInputBitNum);	// enable pullup on inputs and on unused I/O pins, set the output off
#if ISR_DEBUG
	DDRB = BITVAL(PortBDuetOutputBitNum) | BITVAL(PortBDebugBitNum);
#else
	DDRB = BITVAL(PortBDuetOutputBitNum);
#endif

#ifndef __ECV__												// eCv++ doesn't understand gcc assembler syntax
	wdt_enable(WDTO_500MS);									// enable the watchdog
#endif
	
	// Set up the ADC in free-running mode
	ADMUX = (uint8_t)(AdcStrainInputChan | BITVAL(REFS1));	// select the input from the strain amplifier, 1.2V reference
	ADCSRB = 0;												// free running, unipolar input mode
	ADCSRA = BITVAL(ADEN) | BITVAL(ADATE) | BITVAL(ADIF) | BITVAL(ADIE) | BITVAL(ADPS2) | BITVAL(ADPS0);
															// enable ADC, auto trigger enable, clear interrupt, enable interrupt, prescaler = 32 (ADC clock ~= 250kHz)
	// Set up timer 0 to generate the tick interrupt
	TCCR0A = 0;												// Mode 0, no outputs enabled
	TCCR0B = BITVAL(CS01);									// Prescaler = 8, it clocks at 1MHz
	TIMSK |= BITVAL(TOIE0);									// Enable timer 0 overflow interrupt

	sei();
	ADCSRA |= BITVAL(ADSC);									// start first conversion

	DelayTicks(3 * slowTickFrequency);						// wait 3 seconds to allow the supply and the sense amplifier to stabilise

	readEEPROM(0u, reinterpret_cast<uint8_t* array>(&nvData), sizeof(nvData));
	const bool calibrated = ((nvData.sensitivity ^ nvData.checksum) == NvData::magic);
	const uint16_t thresholdMillivolts = (calibrated) ? nvData.sensitivity : defaultThresholdMilliVolts;
	UpdateThreshold(thresholdMillivolts);

	// Set up INT0 for detecting programming signals
	cli();

	MCUCR = BITVAL(ISC00);									// INT0 interrupt on both edges
	GIFR = BITVAL(INTF0);									// clear any existing interrupt
	GIMSK = BITVAL(INT0);									// enable INT0

	// Set up the rolling average
	shiftedRollingAverage = strainFilter.sum;
	//Original - partdaddy   
	//rollingAverage = ((uint32_t)shiftedRollingAverage) << avShift;
	rollingAverage = ((uint32_t)shiftedRollingAverage) << avShift;
	sei();

	// Flash the LED twice if the sensitivity has not been set, three times if it has
	FlashLed((calibrated) ? 3u : 2u);

	// Start main loop
	for (;;)
	{
		// Set the output on if we are triggered
		// Fetch values of volatile variables more than 1 byte long with interrupts disabled to prevent getting inconsistent values
		cli();
		const uint16_t locShiftedRollingAverage = shiftedRollingAverage;
		const uint16_t locSum = strainFilter.sum;
		const uint16_t now = tickCounter;
		sei();

		if (locSum >= locShiftedRollingAverage + threshold)
		{
			SetOutputOn();
			timeOn = now;
		}
		else if (now - timeOn > ledOnTicks)
		{
			SetOutputOff();
		}

		CheckForReceivedData();
		CheckWatchdog();
	}

#ifdef __ECV__
	return 0;				// to keep eCv happy
#endif
}

// End

// eLua DAC - digital analog converter

//***********************
// Module code (to become module_dac.c)

#include "lauxlib.h"
#include "lrotable.h"
#include "platform_conf.h"
//#include "module_dac.h"

// Define interface to: 
//  set timing
//  load series of values
//  get interrupt when done
//  loop values(?)
//  what else?
//Set up buffer
//ISR to feed FIFO from buffer
//Regular interrupts to transfer to FIFO
//Handle both channels


//#define DACC_CHANNEL        0 // CH0 (PB15) labelled A12
//#define DACC_CHANNEL        1 // CH1 (PB16) labelled A13

void platform_dac_init(void);
void platform_dac_setup(unsigned id);
u32 platform_dac_write(u32 id, u32 value);

//***********************
// Platform code (platform_dac.c)

#include "platform_conf.h"
#include <asf.h>
//#include "module_dac.h"

//DACC_MAX_DATA
//#ifdef BUILD_DAC

// Interrupt handler
//void DACC_Handler( void )

//void dacc_enable_interrupt(DACC, uint32_t ul_interrupt_mask)
//void dacc_disable_interrupt(DACC, uint32_t ul_interrupt_mask)
//uint32_t dacc_get_interrupt_mask(DACC)


/*
Clock is MCLK/2
25clocks to provide output after conversion starts
EOC bit in Int status reg set when conversion finishes (reading DAC_ISR clears EOC)

Trigger - free run - channel enabled and data written, alternative is external trigger

FIFO - 4 half-words
TXRDY - can accept data (if full then no TXRDY)

Half word - use lower 16 bits of data, full word - write 2 data items at once (lower half, upper half)
Can tag values (using 2 high bits of 16 bit halfword) to tell which channel to use
// So - could use full word mode to write left and right audio channel data simultaneously


*/

// Convert DAC id to a channel
static unsigned dac_channel(unsigned id)
{
  return id == 0 ? 0 : 1;
}

static u8 dac_inited = 0;

void platform_dac_init(void)
{
  pmc_enable_periph_clk(ID_DACC);
	dacc_reset(DACC);
	/* Half word transfer mode (use lower 16 bits of output data) */
  dacc_set_transfer_mode(DACC, 0);

	/* Power save:
	 * sleep mode  - 0 (disabled) [lower power when not using]
	 * fast wakeup - 0 (disabled) [come out of sleep 4 times faster, but less power savings]
	 */
	dacc_set_power_save(DACC, 0, 0);

  dac_inited = 1;
}

/*! Convert wave data to DACC value
 *  Put the sinewave to an offset of max_amplitude/2.
 *  \param wave          Waveform data
 *  \param amplitude     Amplitude value
 *  \param max_digital   Maximal digital value of input data (no sign)
 *  \param max_amplitude Maximal amplitude value
 */
#define wave_to_dacc(wave, amplitude, max_digital, max_amplitude) \
	(((int)(wave) * (amplitude) / (max_digital)) + (max_amplitude / 2))
// Scale result:wave:max_digital amplitude:max_amplitude, offset by max_amplitude/2
// Max_digital - maximum of input data
// maxamplitude - DACC_MAX_DATA
  
// Need to map or enable I/O pin(s)
void platform_dac_setup(unsigned id)
{

/* Example - (arduino) using DAC to write pin value */
	if (dacc_get_channel_status(DACC) == 0)
	{			
		/* Timing:
		 * refresh period - 0x08 (1024*8 dacc clocks) [Analog signal drops after 20us unless refresh]
		 * max speed mode -    0 (disabled)
		 * startup time   - 0x10 (1024 dacc clocks)
		 */
		dacc_set_timing(DACC, 0x08, 0, 0x10);

		/* Set up analog current */
		dacc_set_analog_control(DACC, DACC_ACR_IBCTLCH0(0x02) | DACC_ACR_IBCTLCH1(0x02) |
											DACC_ACR_IBCTLDACCORE(0x01));
		/* Disable TAG and select output channel dac_channel(id) */
    dacc_set_channel_selection(DACC, dac_channel(id));

    // enable channel if not enabled
		if ((dacc_get_channel_status(DACC) & (1 << dac_channel(id))) == 0) {
			dacc_enable_channel(DACC, dac_channel(id));
		}
  }
}


u32 platform_dac_write(u32 id, u32 value)
{
  if (value > DACC_MAX_DATA)
    return PLATFORM_ERR;    // Might be better to move error check to module
  dacc_set_channel_selection(DACC, dac_channel(id)); // Maybe cache id, or maybe use TAGs

  // Wait until DAC FIFO ready to accept data
  WAIT_WHILE((dacc_get_interrupt_status(DACC) & DACC_IMR_TXRDY) == 0);
  // Could offer non-blocking version (return error if FIFO full)
  // TODO: Should have time limit on wait
  
  /* borrowed from arduino, see what mapResolution does, that part would be in module_dac */
//	rangevalue = mapResolution(ulValue, _writeResolution, DAC_BIT_RESOLUTION);

	dacc_write_conversion_data(DACC, value);
// Wait after was from arduino - seems like might be better to run ahead and only wait if FIFO full
//	WAIT_WHILE((dacc_get_interrupt_status(DACC) & DACC_ISR_EOC) == 0);  
  return PLATFORM_OK;
}

//#endif // BUILD_DAC

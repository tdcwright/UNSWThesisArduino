#ifndef ENCODER_CMD_DEFS_H
#define ENCODER_CMD_DEFS_H

// With reference to https://github.com/Ben0it/ABIEncoder

// Operations on the LS7366R
#define OP_CLEAR                	0x0
#define OP_READ                 	0x40
#define OP_WRITE                	0x80
#define OP_LOAD                 	0xc0

// LS7366R registers
#define REG_MDR0                	0x8
#define REG_MDR1                	0x10
#define REG_DTR                 	0x18
#define REG_CNTR                	0x20
#define REG_OTR                 	0x28
#define REG_STR                 	0x30

// MDR0 settings
// Count resolution
#define	MDR0_B1B0_CLOCK_DIR			  0x0		// A = clock, B = direction
#define	MDR0_B1B0_X1				      0x1		// A single counting channel
#define	MDR0_B1B0_X2				      0x2		// Two counter channels
#define	MDR0_B1B0_X4				      0x3		// All edges of both channels
// Count mode
#define	MDR0_B3B2_FREE_RUNNING	  0x0		// Free counting
#define	MDR0_B3B2_SINGLE_CYCLE	  0x4		// Single cycle counting
#define	MDR0_B3B2_RANGE_LIMIT		  0x8		// Count with limit
#define	MDR0_B3B2_MODULU_COUNT	  0xc		// Modulo DTR counting
// Handling the index signal
#define	MDR0_B5B4_NO_INDEX			  0x0		// Index ignored
#define	MDR0_B5B4_LOAD_CNTR			  0x10	// The arrival of the index loads DTR into CNTR
#define	MDR0_B5B4_RESET_CNTR		  0x20	// The arrival of the index resets CNTR to zero
#define	MDR0_B5B4_LOAD_OTR			  0x30	// The arrival of the CNTR load index in OTR
// Index Type
#define	MDR0_B6_ASYNC_INDEX			  0x0		// Index asynchrone
#define	MDR0_B6_SYNC_INDEX			  0x40	// Index synchrone
// Filtering
#define	MDR0_B7_FILTER_CLOCK_F1	  0x0		// Clock division factor = 1
#define	MDR0_B7_FILTER_CLOCK_F2	  0x80	// Clock division factor = 2

// MDR1 settings
// meter length
#define	MDR1_B1B0_COUNT_32_BITS	  0x0		// 32-bit counting
#define	MDR1_B1B0_COUNT_24_BITS	  0x1		// 24-bit counting
#define	MDR1_B1B0_COUNT_16_BITS	  0x2		// 16-bit count
#define	MDR1_B1B0_COUNT_8_BITS	  0x3		// 8-bit counting
// Enable or disable counting
#define	MDR1_B2_ENABLE_COUNTING	  0x0		// Enable counting
#define	MDR1_B2_DISABLE_COUNTING  0x4		// Disables counting
// Output event handling
#define	MDR1_B4_NO_FLAG_INDEX		  0x0		// Flag on IDX inactive
#define	MDR1_B4_FLAG_INDEX			  0x10	// Flag on IDX (INDEX detected)
#define	MDR1_B5_NO_FLAG_COMPARE		0x0	  // Flag on CMP inactive
#define	MDR1_B5_FLAG_COMPARE		  0x20	// Flag on CMP (CNTR equals DTR)
#define	MDR1_B6_NO_FLAG_UNDERFLOW	0x0	  // Flag on BW inactive
#define	MDR1_B6_FLAG_UNDERFLOW		0x40	// Flag on BW (underflow on CNTR)
#define	MDR1_B7_NO_FLAG_OVERFLOW	0x0	  // Flag on CY inactive
#define	MDR1_B7_FLAG_OVERFLOW		  0x80	// Flag on CY (capacity overflow on CNTR)

// LS7366R default configuration (MDR0 + MDR1)
#define MDR0_INITIAL_VALUE      	(MDR0_B7_FILTER_CLOCK_F1 | MDR0_B6_ASYNC_INDEX | MDR0_B5B4_NO_INDEX | MDR0_B3B2_FREE_RUNNING | MDR0_B1B0_X4)
#define MDR1_INITIAL_VALUE      	(MDR1_B7_NO_FLAG_OVERFLOW | MDR1_B6_NO_FLAG_UNDERFLOW | MDR1_B5_NO_FLAG_COMPARE | MDR1_B4_NO_FLAG_INDEX | MDR1_B2_ENABLE_COUNTING | MDR1_B1B0_COUNT_32_BITS)

#endif
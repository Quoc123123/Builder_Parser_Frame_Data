#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>



#define MAX_FRAME_SIZE                      100

#define PROTOCOL_START_FLAG_SIZE            2
#define PROTOCOL_FRAME_TYPE_SIZE            1
#define PROTOCOL_PAYLOAD_SIZE               2
#define PROTOCOL_CRC16_SIZE                 2


#define PROTOCOL_HEADER1_VALUE              0xA5
#define PROTOCOL_HEADER2_VALUE              0x5A



#define PROTOCOL_CMD_START_STREAMING        0xF1
#define PROTOCOL_CMD_STOP_STREAMING         0xF2
#define PROTOCOL_CMD_REBOOT_STREAMING       0xF3
#define PROTOCOL_CMD_SLEEP_STREAMING        0xF4




/*
 * #define CRC_POLY_xxxx
 *
 * The constants of the form CRC_POLY_xxxx define the polynomials for some well
 * known CRC calculations.
 */

#define		CRC_POLY_16		0xA001
#define		CRC_POLY_32		0xEDB88320ul
#define		CRC_POLY_64		0x42F0E1EBA9EA3693ull
#define		CRC_POLY_CCITT		0x1021
#define		CRC_POLY_DNP		0xA6BC
#define		CRC_POLY_KERMIT		0x8408
#define		CRC_POLY_SICK		0x8005

/*
 * #define CRC_START_xxxx
 *
 * The constants of the form CRC_START_xxxx define the values that are used for
 * initialization of a CRC value for common used calculation methods.
 */

#define		CRC_START_8		0x00
#define		CRC_START_16		0x0000
#define		CRC_START_MODBUS	0xFFFF
#define		CRC_START_XMODEM	0x0000
#define		CRC_START_CCITT_1D0F	0x1D0F
#define		CRC_START_CCITT_FFFF	0xFFFF
#define		CRC_START_KERMIT	0x0000
#define		CRC_START_SICK		0x0000
#define		CRC_START_DNP		0x0000
#define		CRC_START_32		0xFFFFFFFFul
#define		CRC_START_64_ECMA	0x0000000000000000ull
#define		CRC_START_64_WE		0xFFFFFFFFFFFFFFFFull

/*
 * Prototype list of global functions
 */

unsigned char *		checksum_NMEA(      const unsigned char *input_str, unsigned char *result  );
uint8_t			crc_8(              const unsigned char *input_str, size_t num_bytes       );
uint16_t		crc_16(             const unsigned char *input_str, size_t num_bytes       );
uint32_t		crc_32(             const unsigned char *input_str, size_t num_bytes       );
uint64_t		crc_64_ecma(        const unsigned char *input_str, size_t num_bytes       );
uint64_t		crc_64_we(          const unsigned char *input_str, size_t num_bytes       );
uint16_t		crc_ccitt_1d0f(     const unsigned char *input_str, size_t num_bytes       );
uint16_t		crc_ccitt_ffff(     const unsigned char *input_str, size_t num_bytes       );
uint16_t		crc_dnp(            const unsigned char *input_str, size_t num_bytes       );
uint16_t		crc_kermit(         const unsigned char *input_str, size_t num_bytes       );
uint16_t		crc_modbus(         const unsigned char *input_str, size_t num_bytes       );
uint16_t		crc_sick(           const unsigned char *input_str, size_t num_bytes       );
uint16_t		crc_xmodem(         const unsigned char *input_str, size_t num_bytes       );
uint8_t			update_crc_8(       uint8_t  crc, unsigned char c                          );
uint16_t		update_crc_16(      uint16_t crc, unsigned char c                          );
uint32_t		update_crc_32(      uint32_t crc, unsigned char c                          );
uint64_t		update_crc_64_ecma( uint64_t crc, unsigned char c                          );
uint16_t		update_crc_ccitt(   uint16_t crc, unsigned char c                          );
uint16_t		update_crc_dnp(     uint16_t crc, unsigned char c                          );
uint16_t		update_crc_kermit(  uint16_t crc, unsigned char c                          );
uint16_t		update_crc_sick(    uint16_t crc, unsigned char c, unsigned char prev_byte );


typedef enum
{
    PROTOCOL_CONTROL_FRAME = 0xA1,
    PROTOCOL_DATA_FRAME = 0xA2,
} protocol_frame_type_t;

static uint8_t tx_msg_buf        [MAX_FRAME_SIZE];
static uint8_t rx_msg_buf        [MAX_FRAME_SIZE];

static void             init_crc16_tab( void );

static bool             crc_tab16_init          = false;
static uint16_t         crc_tab16[256];


bool builder_function(protocol_frame_type_t frame_type, uint16_t payload_size, uint8_t *payload, uint8_t *output);
bool parser_function(uint8_t *input, uint8_t* fram_type, uint8_t* payload_size, uint8_t *payload);

bool test_builder_function(void);
bool test_parser_function(void);

int main(void)
{
  	test_builder_function();
  	

    return 0;
}


bool builder_function(protocol_frame_type_t frame_type, uint16_t payload_size, uint8_t *payload, uint8_t *output)
{
    output[0] = PROTOCOL_HEADER1_VALUE;
    output[1] = PROTOCOL_HEADER2_VALUE;
    output[2] = (uint8_t)frame_type;
    
    output[PROTOCOL_START_FLAG_SIZE + PROTOCOL_FRAME_TYPE_SIZE] = (uint8_t)(payload_size >> 8);
    output[PROTOCOL_START_FLAG_SIZE + PROTOCOL_FRAME_TYPE_SIZE + 1] = (uint8_t)(payload_size);

    memcpy(&output[PROTOCOL_START_FLAG_SIZE + PROTOCOL_FRAME_TYPE_SIZE + PROTOCOL_PAYLOAD_SIZE], \
                                                    payload, payload_size);
    
    
    uint16_t crc_value = crc_16(output, PROTOCOL_START_FLAG_SIZE + PROTOCOL_FRAME_TYPE_SIZE + payload_size);
    
    memcpy(&output[PROTOCOL_START_FLAG_SIZE + PROTOCOL_FRAME_TYPE_SIZE + PROTOCOL_FRAME_TYPE_SIZE + payload_size], &crc_value, PROTOCOL_CRC16_SIZE);
    return true;
}

bool test_builder_function(void)
{
	uint8_t payload[MAX_FRAME_SIZE];
    uint16_t payload_size;

    payload_size = 0x0001;
    payload[0] = PROTOCOL_CMD_START_STREAMING;
    builder_function(PROTOCOL_CONTROL_FRAME, payload_size,  payload, tx_msg_buf);

    // payload_size = 0x0005;
    // payload[0] = 0x68;
    // payload[1] = 0x65;
    // payload[2] = 0x6C;
    // payload[3] = 0x6C;
    // payload[4] = 0x6F;
    // builder_function(PROTOCOL_DATA_FRAME, payload_size,  payload, tx_msg_buf);
    
    int i;
    for(i = 0; i < 11; i++)
    {
        printf("%x\n", tx_msg_buf[i]);
    }
    
    return true;
}


bool parser_function(uint8_t *input, uint8_t* fram_type, uint8_t* payload_size, uint8_t *payload)
{
	// Check crc value
	*fram_type = input[PROTOCOL_START_FLAG_SIZE];
	*payload_size = ((uint16_t)(input[PROTOCOL_START_FLAG_SIZE + PROTOCOL_FRAME_TYPE_SIZE]) << 8) | (input[PROTOCOL_START_FLAG_SIZE + PROTOCOL_FRAME_TYPE_SIZE + 1]);
	memcpy(&input[PROTOCOL_START_FLAG_SIZE + PROTOCOL_FRAME_TYPE_SIZE + PROTOCOL_PAYLOAD_SIZE], payload, *payload_size);
}

bool test_parser_function(void)
{
//	uint8_t payload[MAX_FRAME_SIZE];
//    uint16_t payload_size;
//
//    payload_size = 0x0001;
//    payload[0] = PROTOCOL_CMD_START_STREAMING;
//    builder_function(PROTOCOL_CONTROL_FRAME, payload_size,  payload, tx_msg_buf);
}

uint16_t crc_16( const unsigned char *input_str, size_t num_bytes ) {

	uint16_t crc;
	const unsigned char *ptr;
	size_t a;

	if ( ! crc_tab16_init ) init_crc16_tab();

	crc = CRC_START_16;
	ptr = input_str;

	if ( ptr != NULL ) for (a=0; a<num_bytes; a++) {

		crc = (crc >> 8) ^ crc_tab16[ (crc ^ (uint16_t) *ptr++) & 0x00FF ];
	}

	return crc;

}  /* crc_16 */

/*
 * uint16_t crc_modbus( const unsigned char *input_str, size_t num_bytes );
 *
 * The function crc_modbus() calculates the 16 bits Modbus CRC in one pass for
 * a byte string of which the beginning has been passed to the function. The
 * number of bytes to check is also a parameter.
 */

uint16_t crc_modbus( const unsigned char *input_str, size_t num_bytes ) {

	uint16_t crc;
	const unsigned char *ptr;
	size_t a;

	if ( ! crc_tab16_init ) init_crc16_tab();

	crc = CRC_START_MODBUS;
	ptr = input_str;

	if ( ptr != NULL ) for (a=0; a<num_bytes; a++) {

		crc = (crc >> 8) ^ crc_tab16[ (crc ^ (uint16_t) *ptr++) & 0x00FF ];
	}

	return crc;

}  /* crc_modbus */

/*
 * uint16_t update_crc_16( uint16_t crc, unsigned char c );
 *
 * The function update_crc_16() calculates a new CRC-16 value based on the
 * previous value of the CRC and the next byte of data to be checked.
 */

uint16_t update_crc_16( uint16_t crc, unsigned char c ) {

	if ( ! crc_tab16_init ) init_crc16_tab();

	return (crc >> 8) ^ crc_tab16[ (crc ^ (uint16_t) c) & 0x00FF ];

}  /* update_crc_16 */

/*
 * static void init_crc16_tab( void );
 *
 * For optimal performance uses the CRC16 routine a lookup table with values
 * that can be used directly in the XOR arithmetic in the algorithm. This
 * lookup table is calculated by the init_crc16_tab() routine, the first time
 * the CRC function is called.
 */

static void init_crc16_tab( void ) {

	uint16_t i;
	uint16_t j;
	uint16_t crc;
	uint16_t c;

	for (i=0; i<256; i++) {

		crc = 0;
		c   = i;

		for (j=0; j<8; j++) {

			if ( (crc ^ c) & 0x0001 ) crc = ( crc >> 1 ) ^ CRC_POLY_16;
			else                      crc =   crc >> 1;

			c = c >> 1;
		}

		crc_tab16[i] = crc;
	}

	crc_tab16_init = true;

}  /* init_crc16_tab */

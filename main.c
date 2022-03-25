#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>



/*
 * #define CRC_POLY_xxxx
 *
 */
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
 * typedef list of global variables
 */
typedef enum
{
    PROTOCOL_CONTROL_FRAME = 0xA1,
    PROTOCOL_DATA_FRAME = 0xA2,
} protocol_frame_type_t;

/*
 * Prototype list of global functions
 */
uint16_t		crc_ccitt_ffff(     const unsigned char *input_str, size_t num_bytes       );
uint16_t		update_crc_ccitt(   uint16_t crc, unsigned char c                          );
bool 			builder_function(protocol_frame_type_t frame_type, uint16_t payload_size, uint8_t *payload, uint8_t *output);
bool 			parser_function(uint8_t *input, uint8_t* fram_type, uint16_t* payload_size, uint8_t *payload);

bool 			test_control_frame(void);
bool 			test_data_frame(void);


/*
 * Prototype list of static functions
 */
static uint16_t		crc_ccitt_generic( const unsigned char *input_str, size_t num_bytes, uint16_t start_value );
static void         init_crcccitt_tab( void );

/*
 * static variable list
 */
static uint8_t tx_msg_buf [MAX_FRAME_SIZE];
static bool             crc_tabccitt_init       = false;
static uint16_t         crc_tabccitt[256];


int main(void)
{
  	if(test_control_frame())
	{
		printf("Control frame test passed\n");
	}
	else
	{
		printf("Control frame test failed\n");
	}

	if(test_data_frame())
	{
		printf("Data frame test passed\n");
	}
	else
	{
		printf("Data frame test failed\n");
	}

    return 0;
}

/*
 * builder_function(protocol_frame_type_t frame_type, uint16_t payload_size, uint8_t *payload, uint8_t *output)
 *
 * The function to build a protocol frame.
 *  Input:
 * 		+ Frame type: 
 * 			PROTOCOL_CONTROL_FRAME
    		PROTOCOL_DATA_FRAME
		+ payload_size: size of the payload
		+ output: pointer to the output buffer
	Return:
		+ true: if the frame is built successfully
		+ false: if the frame is not built successfully
 */
bool builder_function(protocol_frame_type_t frame_type, uint16_t payload_size, uint8_t *payload, uint8_t *output)
{
    //|-- Header 1 --|-- Header 2 --|-- Frame type--|-- payload length --|------------ payload ------------|--  CRC16 ---|
    //|--- 1 byte ---|--- 1 byte ---|--- 1 byte --  |---- 2 bytes -------|------------ n bytes ------------|-- 2 bytes ---|
    //|------------------------------------------------ frame size -------------------------------------------------------|
	printf("\n\n\n");
	printf("=======================================================================================\n");
	printf("================================== START BUILDER ======================================\n");
	printf("=======================================================================================\n");	

	/* TODO: Validate inpout param */
    output[0] = PROTOCOL_HEADER1_VALUE;
    output[1] = PROTOCOL_HEADER2_VALUE;
    output[2] = (uint8_t)frame_type;
    
    output[PROTOCOL_START_FLAG_SIZE + PROTOCOL_FRAME_TYPE_SIZE] = (uint8_t)(payload_size >> 8);
    output[PROTOCOL_START_FLAG_SIZE + PROTOCOL_FRAME_TYPE_SIZE + 1] = (uint8_t)(payload_size);

    memcpy(&output[PROTOCOL_START_FLAG_SIZE + PROTOCOL_FRAME_TYPE_SIZE + PROTOCOL_PAYLOAD_SIZE], \
                                                    payload, payload_size);
    
    
    uint16_t crc_value = crc_ccitt_ffff(output, PROTOCOL_START_FLAG_SIZE + PROTOCOL_FRAME_TYPE_SIZE + PROTOCOL_PAYLOAD_SIZE + payload_size);
    
    output[PROTOCOL_START_FLAG_SIZE + PROTOCOL_FRAME_TYPE_SIZE + PROTOCOL_PAYLOAD_SIZE + payload_size] = (uint8_t)(crc_value >> 8);
	output[PROTOCOL_START_FLAG_SIZE + PROTOCOL_FRAME_TYPE_SIZE + PROTOCOL_PAYLOAD_SIZE + payload_size + 1] = (uint8_t)(crc_value);

	printf("=======================================================================================\n");
	printf("================================== END BUILDER ========================================\n");
	printf("=======================================================================================\n");	
	printf("\n\n\n");

    return true;
}

/*
 * bool test_control_frame(void)
 *
 * 	The function to test the control frame.
 *  Input: None
	Return: True or false
 */
bool test_control_frame(void)
{
	uint8_t payload_builder[MAX_FRAME_SIZE];
    uint16_t payload_size_builder;

	uint8_t payload_parser[MAX_FRAME_SIZE];
    uint16_t payload_size_parser;
	uint8_t frame_type_parser;

    payload_size_builder = 0x0001;
    payload_builder[0] = PROTOCOL_CMD_START_STREAMING;
    
	builder_function(PROTOCOL_CONTROL_FRAME, payload_size_builder,  payload_builder, tx_msg_buf);
	return parser_function(tx_msg_buf, &frame_type_parser,  &payload_size_parser, payload_parser);
}

/*
 * bool test_data_frame(void)
 *
 * 	The function to test the data frame.
 *  Input: None
	Return: True or false
 */
bool test_data_frame(void)
{
	uint8_t payload_builder[MAX_FRAME_SIZE];
    uint16_t payload_size_builder;

	uint8_t payload_parser[MAX_FRAME_SIZE];
    uint16_t payload_size_parser;
	uint8_t frame_type_parser;

	payload_size_builder = 0x0005;
    payload_builder[0] = 0x68;
    payload_builder[1] = 0x65;
    payload_builder[2] = 0x6C;
    payload_builder[3] = 0x6C;
    payload_builder[4] = 0x6F;

	builder_function(PROTOCOL_DATA_FRAME, payload_size_builder,  payload_builder, tx_msg_buf);
	return parser_function(tx_msg_buf, &frame_type_parser,  &payload_size_parser, payload_parser);
}

/*
 * bool parser_function(uint8_t *input, uint8_t* frame_type, uint16_t* payload_size, uint8_t *payload)
 *
 * The function to build a protocol frame.
 *  Input:
 * 		+ input: pointer to the input buffer
		+ frame_type: pointer to the frame type
		+ payload_size: pointer to the payload size
		+ payload: pointer to the payload
	Return:
		+ true: if the frame is parse successfully
		+ false: if the frame is not parse successfully
 */
bool parser_function(uint8_t *input, uint8_t* frame_type, uint16_t* payload_size, uint8_t *payload)
{
	printf("\n\n\n");
	printf("=======================================================================================\n");
	printf("================================== START PARSER =======================================\n");
	printf("=======================================================================================\n");

	bool status = true;
	/* TODO: Validate intput parameter */
	*frame_type = input[PROTOCOL_START_FLAG_SIZE];
	*payload_size = ((uint16_t)(input[PROTOCOL_START_FLAG_SIZE + PROTOCOL_FRAME_TYPE_SIZE]) << 8) | (input[PROTOCOL_START_FLAG_SIZE + PROTOCOL_FRAME_TYPE_SIZE + 1]);
	memcpy(payload, &input[PROTOCOL_START_FLAG_SIZE + PROTOCOL_FRAME_TYPE_SIZE + PROTOCOL_PAYLOAD_SIZE], *payload_size);

	uint16_t old_crc_val = ((uint16_t)input[PROTOCOL_START_FLAG_SIZE + PROTOCOL_FRAME_TYPE_SIZE + PROTOCOL_PAYLOAD_SIZE  + *payload_size] << 8) | (input[PROTOCOL_START_FLAG_SIZE + PROTOCOL_FRAME_TYPE_SIZE + PROTOCOL_PAYLOAD_SIZE  + *payload_size + 1]);
	uint16_t new_crc_val = crc_ccitt_ffff(input, PROTOCOL_START_FLAG_SIZE + PROTOCOL_FRAME_TYPE_SIZE + PROTOCOL_PAYLOAD_SIZE  + *payload_size);

	if(old_crc_val != new_crc_val)
	{
		status = false;
		printf("Failed to validate CRC value\n");
	}
	else
	{
		printf("Validate CRC successfully\n");
	}

	printf("=======================================================================================\n");
	printf("================================== END PARSER =========================================\n");
	printf("=======================================================================================\n");
	printf("\n\n\n");

	return status;
}

/*
 * uint16_t crc_xmodem( const unsigned char *input_str, size_t num_bytes );
 *
 * The function crc_xmodem() performs a one-pass calculation of an X-Modem CRC
 * for a byte string that has been passed as a parameter.
 */

uint16_t crc_xmodem( const unsigned char *input_str, size_t num_bytes ) {

	return crc_ccitt_generic( input_str, num_bytes, CRC_START_XMODEM );

}  /* crc_xmodem */

/*
 * uint16_t crc_ccitt_1d0f( const unsigned char *input_str, size_t num_bytes );
 *
 * The function crc_ccitt_1d0f() performs a one-pass calculation of the CCITT
 * CRC for a byte string that has been passed as a parameter. The initial value
 * 0x1d0f is used for the CRC.
 */

uint16_t crc_ccitt_1d0f( const unsigned char *input_str, size_t num_bytes ) {

	return crc_ccitt_generic( input_str, num_bytes, CRC_START_CCITT_1D0F );

}  /* crc_ccitt_1d0f */

/*
 * uint16_t crc_ccitt_ffff( const unsigned char *input_str, size_t num_bytes );
 *
 * The function crc_ccitt_ffff() performs a one-pass calculation of the CCITT
 * CRC for a byte string that has been passed as a parameter. The initial value
 * 0xffff is used for the CRC.
 */

uint16_t crc_ccitt_ffff( const unsigned char *input_str, size_t num_bytes ) {

	return crc_ccitt_generic( input_str, num_bytes, CRC_START_CCITT_FFFF );

}  /* crc_ccitt_ffff */

/*
 * static uint16_t crc_ccitt_generic( const unsigned char *input_str, size_t num_bytes, uint16_t start_value );
 *
 * The function crc_ccitt_generic() is a generic implementation of the CCITT
 * algorithm for a one-pass calculation of the CRC for a byte string. The
 * function accepts an initial start value for the crc.
 */

static uint16_t crc_ccitt_generic( const unsigned char *input_str, size_t num_bytes, uint16_t start_value ) {

	uint16_t crc;
	const unsigned char *ptr;
	size_t a;

	if ( ! crc_tabccitt_init ) init_crcccitt_tab();

	crc = start_value;
	ptr = input_str;

	if ( ptr != NULL ) for (a=0; a<num_bytes; a++) {

		crc = (crc << 8) ^ crc_tabccitt[ ((crc >> 8) ^ (uint16_t) *ptr++) & 0x00FF ];
	}

	return crc;

}  /* crc_ccitt_generic */

/*
 * uint16_t update_crc_ccitt( uint16_t crc, unsigned char c );
 *
 * The function update_crc_ccitt() calculates a new CRC-CCITT value based on
 * the previous value of the CRC and the next byte of the data to be checked.
 */

uint16_t update_crc_ccitt( uint16_t crc, unsigned char c ) {

	if ( ! crc_tabccitt_init ) init_crcccitt_tab();

	return (crc << 8) ^ crc_tabccitt[ ((crc >> 8) ^ (uint16_t) c) & 0x00FF ];

}  /* update_crc_ccitt */

/*
 * static void init_crcccitt_tab( void );
 *
 * For optimal performance, the routine to calculate the CRC-CCITT uses a
 * lookup table with pre-compiled values that can be directly applied in the
 * XOR action. This table is created at the first call of the function by the
 * init_crcccitt_tab() routine.
 */

static void init_crcccitt_tab( void ) {

	uint16_t i;
	uint16_t j;
	uint16_t crc;
	uint16_t c;

	for (i=0; i<256; i++) {

		crc = 0;
		c   = i << 8;

		for (j=0; j<8; j++) {

			if ( (crc ^ c) & 0x8000 ) crc = ( crc << 1 ) ^ CRC_POLY_CCITT;
			else                      crc =   crc << 1;

			c = c << 1;
		}

		crc_tabccitt[i] = crc;
	}

	crc_tabccitt_init = true;

}  /* init_crcccitt_tab */

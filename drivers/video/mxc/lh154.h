#ifndef __LH154_H__
#define __LH154_H__

/*
 * LH154 supported DCS commands
 */

#define LH154_CMD_NOP		0x00	/* No Operation 			Command */
#define LH154_CMD_SWRESET	0x01	/* Software Reset 			Command */
#define LH154_CMD_RDNUMED	0x05	/* Read Number of Error on DSI 		Read 1 */
#define LH154_CMD_RDDPM		0x0A	/* Read Display Power Mode 		Read 1 */
#define LH154_CMD_RDDMADCTL	0x0B	/* Read Display MADCTL 			Read 1 */
#define LH154_CMD_RDDCOLMOD	0x0C	/* Read Display Pixel Format 		Read 1 */
#define LH154_CMD_RDDIM		0x0D	/* Read Display Image Mode	 	Read 1 */
#define LH154_CMD_RDDSM		0x0E	/* Read Display Signal Mode	 	Read 1 */
#define LH154_CMD_RDDSDR	0x0F	/* Read Display Self Diagnostic Result 	Read 1 */
#define LH154_CMD_SLPIN		0x10	/* Sleep In			 	Command */
#define LH154_CMD_SLPOUT	0x11	/* Sleep Out			 	Command */
#define LH154_CMD_NORON		0x13	/* Normal Display Mode On 		Command */
#define LH154_CMD_INVOFF	0x20	/* Display Inversion Off 		Command */
#define LH154_CMD_INVON		0x21	/* Display Inversion On 		Command */
#define LH154_CMD_ALLPOFF	0x22	/* All Pixels Off		 	Command */
#define LH154_CMD_ALLPON	0x23	/* All Pixels On 			Command */
#define LH154_CMD_DISPOFF	0x28	/* Display Off 				Command */
#define LH154_CMD_DISPON	0x29	/* Display On 				Command */
#define LH154_CMD_CASET		0x2A	/* (Note 4) Column Address Set 		Write 4 */
#define LH154_CMD_PASET		0x2B	/* (Note 4) Page Address Set 		Write 4 */
#define LH154_CMD_RAMWR		0x2C	/* Memory Write 			Write Any Length */
#define LH154_CMD_RAMRD		0x2E	/* Memory Read 				Read Any length */
#define LH154_CMD_TEOFF		0x34	/* Tearing Effect Line Off	 	Command */
#define LH154_CMD_TEON		0x35	/* Tearing Effect Line On 		Write 1 */
#define LH154_CMD_MADCTL	0x36	/* Memory Data Access Control 		Write 1 */
#define LH154_CMD_IDMOFF	0x38	/* Idle Mode Off 			Command */
#define LH154_CMD_IDMON		0x39	/* Idle Mode On 			Command */
#define LH154_CMD_COLMOD	0x3A	/* Interface Pixel Format	 	Write 1 */
#define LH154_CMD_RAMWRC	0x3C	/* Write Memory Continue 		Write Any length */
#define LH154_CMD_RAMRDC	0x3E	/* Read Memory Continue 		Read Any length */
#define LH154_CMD_RDDDBS	0xA1	/* Read DDB Start		 	Read 1~5 */
#define LH154_CMD_RDDDBC	0xA8	/* Read DDB Continue		 	Read 1~5 */
#define LH154_CMD_KEYNOTE	0xB3	/* KEYNOTE 				Write 1 */
#define LH154_CMD_HIFA		0xB4	/* HIFA 				Write 1 */
#define LH154_CMD_RDID1		0xDA	/* Read ID1 				Read (1) */
#define LH154_CMD_RDID2		0xDB	/* Read ID2 				Read (1) */
#define LH154_CMD_RDID3		0xDC	/* Read ID3 				Read (1) */

#endif /* __LH154_H__ */

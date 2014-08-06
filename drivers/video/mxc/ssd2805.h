#ifndef __SSD2805_H__
#define __SSD2805_H__

#define ssd2805_bit(b)			(1 << b)
#define ssd2805_bits(x,a,b)		(((x) & ((1 << (a-b+2))-1)) << b)

#define SSD2805_REG_DIR		0xB0	/* Device Identification Register */
#define SSD2805_DIR_VALUE	0x2805
#define SSD2805_REG_VICR1	0xB1	/* RGB Interface Control Register 1 */
#define SSD2805_VICR1_VSA(x)	ssd2805_bits(x,15, 8)
#define SSD2805_VICR1_HSA(x)	ssd2805_bits(x, 7, 0)
#define SSD2805_REG_VICR2	0xB2	/* RGB Interface Control Register 2 */
#define SSD2805_VICR2_VBP(x)	ssd2805_bits(x,15, 8)
#define SSD2805_VICR2_HBP(x)	ssd2805_bits(x, 7, 0)
#define SSD2805_REG_VICR3	0xB3	/* RGB Interface Control Register 3 */
#define SSD2805_VICR3_VFP(x)	ssd2805_bits(x,15, 8)
#define SSD2805_VICR3_HFP(x)	ssd2805_bits(x, 7, 0)
#define SSD2805_REG_VICR4	0xB4	/* RGB Interface Control Register 4 */
#define SSD2805_VICR4_HACT(x)	ssd2805_bits(x,15, 0)
#define SSD2805_REG_VICR5	0xB5	/* RGB Interface Control Register 5 */
#define SSD2805_VICR5_VACT(x)	ssd2805_bits(x,15, 0)
#define SSD2805_REG_VICR6	0xB6	/* RGB Interface Control Register 6 */
#define SSD2805_VICR6_NVD	ssd2805_bit( 6)
#define SSD2805_VICR6_BLLP	ssd2805_bit( 5)
#define SSD2805_VICR6_VCS	ssd2805_bit( 4)
#define SSD2805_VICR6_VM(x)	ssd2805_bits(x, 3, 2)
#define SSD2805_VICR6_VPF(x)	ssd2805_bits(x, 1, 0)
#define SSD2805_REG_CFGR	0xB7	/* Configuration Register */
#define SSD2805_CFGR_EOT	ssd2805_bit( 9)
#define SSD2805_CFGR_ECD	ssd2805_bit( 8)
#define SSD2805_CFGR_REN	ssd2805_bit( 7)
#define SSD2805_CFGR_DCS	ssd2805_bit( 6)
#define SSD2805_CFGR_CSS	ssd2805_bit( 5)
#define SSD2805_CFGR_HCLK	ssd2805_bit( 4)
#define SSD2805_CFGR_VEN	ssd2805_bit( 3)
#define SSD2805_CFGR_SLP	ssd2805_bit( 2)
#define SSD2805_CFGR_CKE	ssd2805_bit( 1)
#define SSD2805_CFGR_HS		ssd2805_bit( 0)
#define SSD2805_REG_VCR		0xB8	/* VC Control Register */
#define SSD2805_VCR_VCM(x)	ssd2805_bits(x, 7, 6)
#define SSD2805_VCR_VCE(x)	ssd2805_bits(x, 5, 4)
#define SSD2805_VCR_VC2(x)	ssd2805_bits(x, 3, 2)
#define SSD2805_VCR_VC1(x)	ssd2805_bits(x, 1, 0)
#define SSD2805_REG_PCR		0xB9	/* PLL Control Register */
#define SSD2805_PCR_PEN		ssd2805_bit( 0)
#define SSD2805_REG_PLCR	0xBA	/* PLL Configuration Register */
#define SSD2805_PLCR_PDIV(x)	ssd2805_bits(x,15,12)
#define SSD2805_PLCR_DIV(x)	ssd2805_bits(x,11, 8)
#define SSD2805_PLCR_MUL(x)	ssd2805_bits(x, 7, 0)
#define SSD2805_REG_CCR		0xBB	/* Clock Control Register */
#define SSD2805_CCR_SYSD(x)	ssd2805_bits(x, 7, 6)
#define SSD2805_CCR_LPD(x)	ssd2805_bits(x, 5, 0)
#define SSD2805_REG_PSCR1	0xBC	/* Packet Size Control Register 1 */
#define SSD2805_PSCR1_TDCL(x)	ssd2805_bits(x,15, 0)
#define SSD2805_REG_PSCR2	0xBD	/* Packet Size Control Register 2 */
#define SSD2805_PSCR2_TDCH(x)	ssd2805_bits(x,15, 0)
#define SSD2805_REG_PSCR3	0xBE	/* Packet Size Control Register 3 */
#define SSD2805_PSCR3_PDC(x)	ssd2805_bits(x,11, 0)
#define SSD2805_REG_GPDR	0xBF	/* Generic Packet Drop Register */
#define SSD2805_GPDR_GPD(x)	ssd2805_bits(x,15, 0)
#define SSD2805_REG_OCR		0xC0	/* Operation Control Register */
#define SSD2805_0CR_COP		ssd2805_bit( 0)
#define SSD2805_REG_MRSR	0xC1	/* Maximum Return Size Register */
#define SSD2805_MRSR_MRS(x)	ssd2805_bits(x,15, 0)
#define SSD2805_REG_RDCR	0xC2	/* Return Data Count Register */
#define SSD2805_RDCR_RDC(x)	ssd2805_bits(x,15, 0)
#define SSD2805_REG_ARSR	0xC3	/* ACK Response Register */
#define SSD2805_ARSR_AR(x)	ssd2805_bits(x,15, 0)
#define SSD2805_REG_LCR		0xC4	/* Line Control Register */
#define SSD2805_LCR_FBC		ssd2805_bit( 2)
#define SSD2805_LCR_FBT		ssd2805_bit( 1)
#define SSD2805_LCR_FBW		ssd2805_bit( 0)
#define SSD2805_REG_ICR		0xC5	/* Interrupt Control Register */
#define SSD2805_ICR_LSEE	ssd2805_bit(15)
#define SSD2805_ICR_LSAE	ssd2805_bit(14)
#define SSD2805_ICR_LLEE	ssd2805_bit(13)
#define SSD2805_ICR_LLAE	ssd2805_bit(12)
#define SSD2805_ICR_MSEE	ssd2805_bit(11)
#define SSD2805_ICR_MSAE	ssd2805_bit(10)
#define SSD2805_ICR_MLEE	ssd2805_bit( 9)
#define SSD2805_ICR_MLAE	ssd2805_bit( 8)
#define SSD2805_ICR_PLSE	ssd2805_bit( 7)
#define SSD2805_ICR_LPTOE	ssd2805_bit( 6)
#define SSD2805_ICR_HSTOE	ssd2805_bit( 5)
#define SSD2805_ICR_ARRE	ssd2805_bit( 3)
#define SSD2805_ICR_BTARE	ssd2805_bit( 2)
#define SSD2805_ICR_POE		ssd2805_bit( 1)
#define SSD2805_ICR_RDRE	ssd2805_bit( 0)
#define SSD2805_REG_ISR		0xC6	/* Interrupt Status Register */
#define SSD2805_ISR_LSE		ssd2805_bit(15)
#define SSD2805_ISR_LSA		ssd2805_bit(14)
#define SSD2805_ISR_LLE		ssd2805_bit(13)
#define SSD2805_ISR_LLA		ssd2805_bit(12)
#define SSD2805_ISR_MSE		ssd2805_bit(11)
#define SSD2805_ISR_MSA		ssd2805_bit(10)
#define SSD2805_ISR_MLE		ssd2805_bit( 9)
#define SSD2805_ISR_MLA		ssd2805_bit( 8)
#define SSD2805_ISR_PLS		ssd2805_bit( 7)
#define SSD2805_ISR_LPTO	ssd2805_bit( 6)
#define SSD2805_ISR_HSTO	ssd2805_bit( 5)
#define SSD2805_ISR_ATR		ssd2805_bit( 4)
#define SSD2805_ISR_ARR		ssd2805_bit( 3)
#define SSD2805_ISR_BTAR	ssd2805_bit( 2)
#define SSD2805_ISR_PO		ssd2805_bit( 1)
#define SSD2805_ISR_RDR		ssd2805_bit( 0)
#define SSD2805_REG_ESR		0xC7	/* Error Status Register */
#define SSD2805_ESR_CRCE	ssd2805_bit(10)
#define SSD2805_ESR_ECCE2	ssd2805_bit( 9)
#define SSD2805_ESR_ECCE1	ssd2805_bit( 8)
#define SSD2805_ESR_LSO		ssd2805_bit( 7)
#define SSD2805_ESR_LLO		ssd2805_bit( 6)
#define SSD2805_ESR_MSO		ssd2805_bit( 5)
#define SSD2805_ESR_MLO		ssd2805_bit( 4)
#define SSD2805_ESR_VIE		ssd2805_bit( 3)
#define SSD2805_ESR_CONT	ssd2805_bit( 2)
#define SSD2805_ESR_VMM		ssd2805_bit( 0)
#define SSD2805_REG_DAR1	0xC9	/* Delay Adjustment Register 1 */
#define SSD2805_REG_DAR2	0xCA	/* Delay Adjustment Register 2 */
#define SSD2805_REG_DAR3	0xCB	/* Delay Adjustment Register 3 */
#define SSD2805_REG_DAR4	0xCC	/* Delay Adjustment Register 4 */
#define SSD2805_REG_DAR5	0xCD	/* Delay Adjustment Register 5 */
#define SSD2805_REG_DAR6	0xCE	/* Delay Adjustment Register 6 */
#define SSD2805_REG_HTTR1	0xCF	/* HS TX Timer Register 1 */
#define SSD2805_REG_HTTR2	0xD0	/* HS TX Timer Register 2 */
#define SSD2805_REG_LRTR1	0xD1	/* LP RX Timer Register 1 */
#define SSD2805_REG_LRTR2	0xD2	/* LP RX Timer Register 2 */
#define SSD2805_REG_TSR		0xD3	/* TE Status Register */
#define SSD2805_TSR_TE		ssd2805_bit( 0)
#define SSD2805_REG_LRR		0xD4	/* SPI Read Register */
#define SSD2805_LRR_RRA(x)	ssd2805_bits(x,15, 0)
#define SSD2805_REG_TR		0xD6	/* Test Register */
#define SSD2805_REG_RR		0xD7	/* Read Register */
#define SSD2805_RR_RD(x)	ssd2805_bits(x,15, 0)

#define SSD2805_CYCLE_REG	(0x70 | 0x00)
#define SSD2805_CYCLE_READ	(0x70 | 0x01)
#define SSD2805_CYCLE_WRITE	(0x70 | 0x02)

#endif /* __SSD2805_H__ */

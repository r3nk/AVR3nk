/*!
*******************************************************************************
*******************************************************************************
** \brief   Symbols and macros for the MCP2515 device.
**
** This file is part of AVR3nk, available at https://github.com/r3nk/AVR3nk
**
*******************************************************************************
*******************************************************************************
*/

#ifndef MCP2515_PRIV_H
#define MCP2515_PRIV_H

//*****************************************************************************
//********************************** SPI COMMANDS *****************************
//*****************************************************************************
//(MCP 2515-I-P.pdf pages 63-68)
#define MCP2515_SPI_RESET               0xC0
#define MCP2515_SPI_READ                0x03
#define MCP2515_SPI_READ_RXB0SIDH       0x90
#define MCP2515_SPI_READ_RXB0D0         0x92
#define MCP2515_SPI_READ_RXB1SIDH       0x94
#define MCP2515_SPI_READ_RXB1D0         0x96
#define MCP2515_SPI_WRITE               0x02
#define MCP2515_SPI_WRITE_TXB0SIDH      0x40
#define MCP2515_SPI_WRITE_TXB0D0        0x41
#define MCP2515_SPI_WRITE_TXB1SIDH      0x42
#define MCP2515_SPI_WRITE_TXB1D0        0x43
#define MCP2515_SPI_WRITE_TXB2SIDH      0x44
#define MCP2515_SPI_WRITE_TXB2D0        0x45
#define MCP2515_SPI_RTS_BASE            0x80
#define MCP2515_SPI_RTS_TXB0            0x81
#define MCP2515_SPI_RTS_TXB1            0x82
#define MCP2515_SPI_RTS_TXB2            0x84
#define MCP2515_SPI_READ_STATUS         0xA0
#define MCP2515_SPI_RX_STATUS           0xB0
#define MCP2515_SPI_BIT_MODIFY          0x05

//*****************************************************************************
//****************************** REGISTER ADDRESSES ***************************
//*****************************************************************************
//(MCP 2515-I-P.pdf page 61)
#define MCP2515_RXF0SIDH    0x00
#define MCP2515_RXF0SIDL    0x01
#define MCP2515_RXF0EID8    0x02
#define MCP2515_RXF0EID0    0x03
#define MCP2515_RXF1SIDH    0x04
#define MCP2515_RXF1SIDL    0x05
#define MCP2515_RXF1EID8    0x06
#define MCP2515_RXF1EID0    0x07
#define MCP2515_RXF2SIDH    0x08
#define MCP2515_RXF2SIDL    0x09
#define MCP2515_RXF2EID8    0x0A
#define MCP2515_RXF2EID0    0x0B
#define MCP2515_BFPCTRL     0x0C
#define MCP2515_TXRTSCTRL   0x0D
#define MCP2515_CANSTAT     0x0E
#define MCP2515_CANCTRL     0x0F
#define MCP2515_RXF3SIDH    0x10
#define MCP2515_RXF3SIDL    0x11
#define MCP2515_RXF3EID8    0x12
#define MCP2515_RXF3EID0    0x13
#define MCP2515_RXF4SIDH    0x14
#define MCP2515_RXF4SIDL    0x15
#define MCP2515_RXF4EID8    0x16
#define MCP2515_RXF4EID0    0x17
#define MCP2515_RXF5SIDH    0x18
#define MCP2515_RXF5SIDL    0x19
#define MCP2515_RXF5EID8    0x1A
#define MCP2515_RXF5EID0    0x1B
#define MCP2515_TEC         0x1C
#define MCP2515_REC         0x1D
#define MCP2515_RXM0SIDH    0x20
#define MCP2515_RXM0SIDL    0x21
#define MCP2515_RXM0EID8    0x22
#define MCP2515_RXM0EID0    0x23
#define MCP2515_RXM1SIDH    0x24
#define MCP2515_RXM1SIDL    0x25
#define MCP2515_RXM1EID8    0x26
#define MCP2515_RXM1EID0    0x27
#define MCP2515_CNF3        0x28
#define MCP2515_CNF2        0x29
#define MCP2515_CNF1        0x2A
#define MCP2515_CANINTE     0x2B
#define MCP2515_CANINTF     0x2C
#define MCP2515_EFLG        0x2D
#define MCP2515_TXB0CTRL    0x30
#define MCP2515_TXB0SIDH    0x31
#define MCP2515_TXB0SIDL    0x32
#define MCP2515_TXB0EID8    0x33
#define MCP2515_TXB0EID0    0x34
#define MCP2515_TXB0DLC     0x35
#define MCP2515_TXB0D0      0x36
#define MCP2515_TXB0D1      0x37
#define MCP2515_TXB0D2      0x38
#define MCP2515_TXB0D3      0x39
#define MCP2515_TXB0D4      0x3A
#define MCP2515_TXB0D5      0x3B
#define MCP2515_TXB0D6      0x3C
#define MCP2515_TXB0D7      0x3D
#define MCP2515_TXB1CTRL    0x40
#define MCP2515_TXB1SIDH    0x41
#define MCP2515_TXB1SIDL    0x42
#define MCP2515_TXB1EID8    0x43
#define MCP2515_TXB1EID0    0x44
#define MCP2515_TXB1DLC     0x45
#define MCP2515_TXB1D0      0x46
#define MCP2515_TXB1D1      0x47
#define MCP2515_TXB1D2      0x48
#define MCP2515_TXB1D3      0x49
#define MCP2515_TXB1D4      0x4A
#define MCP2515_TXB1D5      0x4B
#define MCP2515_TXB1D6      0x4C
#define MCP2515_TXB1D7      0x4D
#define MCP2515_TXB2CTRL    0x50
#define MCP2515_TXB2SIDH    0x51
#define MCP2515_TXB2SIDL    0x52
#define MCP2515_TXB2EID8    0x53
#define MCP2515_TXB2EID0    0x54
#define MCP2515_TXB2DLC     0x55
#define MCP2515_TXB2D0      0x56
#define MCP2515_TXB2D1      0x57
#define MCP2515_TXB2D2      0x58
#define MCP2515_TXB2D3      0x59
#define MCP2515_TXB2D4      0x5A
#define MCP2515_TXB2D5      0x5B
#define MCP2515_TXB2D6      0x5C
#define MCP2515_TXB2D7      0x5D
#define MCP2515_RXB0CTRL    0x60
#define MCP2515_RXB0SIDH    0x61
#define MCP2515_RXB0SIDL    0x62
#define MCP2515_RXB0EID8    0x63
#define MCP2515_RXB0EID0    0x64
#define MCP2515_RXB0DLC     0x65
#define MCP2515_RXB0D0      0x66
#define MCP2515_RXB0D1      0x67
#define MCP2515_RXB0D2      0x68
#define MCP2515_RXB0D3      0x69
#define MCP2515_RXB0D4      0x6A
#define MCP2515_RXB0D5      0x6B
#define MCP2515_RXB0D6      0x6C
#define MCP2515_RXB0D7      0x6D
#define MCP2515_RXB1CTRL    0x70
#define MCP2515_RXB1SIDH    0x71
#define MCP2515_RXB1SIDL    0x72
#define MCP2515_RXB1EID8    0x73
#define MCP2515_RXB1EID0    0x74
#define MCP2515_RXB1DLC     0x75
#define MCP2515_RXB1D0      0x76
#define MCP2515_RXB1D1      0x77
#define MCP2515_RXB1D2      0x78
#define MCP2515_RXB1D3      0x79
#define MCP2515_RXB1D4      0x7A
#define MCP2515_RXB1D5      0x7B
#define MCP2515_RXB1D6      0x7C
#define MCP2515_RXB1D7      0x7D

//*****************************************************************************
//******************************* BIT POSITIONS *******************************
//*****************************************************************************

// BFPCTRL
//(MCP 2515-I-P.pdf page 61 table 11.2)
#define MCP2515_B1BFS       5
#define MCP2515_B0BFS       4
#define MCP2515_B1BFE       3
#define MCP2515_B0BFE       2
#define MCP2515_B1BFM       1
#define MCP2515_B0BFM       0

// TXRTSCTRL
//(MCP 2515-I-P.pdf page 61 table 11.2)
#define MCP2515_B2RTS       5
#define MCP2515_B1RTS       4
#define MCP2515_B0RTS       3
#define MCP2515_B2RTSM      2
#define MCP2515_B1RTSM      1
#define MCP2515_B0RTSM      0

// CANSTAT
//(MCP 2515-I-P.pdf page 61 table 11.2)
#define MCP2515_OPMOD2      7
#define MCP2515_OPMOD1      6
#define MCP2515_OPMOD0      5
#define MCP2515_ICOD2       3
#define MCP2515_ICOD1       2
#define MCP2515_ICOD0       1

// CANCTRL
//(MCP 2515-I-P.pdf page 61 table 11.2)
#define MCP2515_REQOP2      7
#define MCP2515_REQOP1      6
#define MCP2515_REQOP0      5
#define MCP2515_ABAT        4
#define MCP2515_OSM         3
#define MCP2515_CLKEN       2
#define MCP2515_CLKPRE1     1
#define MCP2515_CLKPRE0     0

// CNF3
//(MCP 2515-I-P.pdf page 61 table 11.2)
#define MCP2515_WAKFIL      6
#define MCP2515_PHSEG22     2
#define MCP2515_PHSEG21     1
#define MCP2515_PHSEG20     0

// CNF2
//(MCP 2515-I-P.pdf page 61 table 11.2)
#define MCP2515_BTLMODE     7
#define MCP2515_SAM         6
#define MCP2515_PHSEG12     5
#define MCP2515_PHSEG11     4
#define MCP2515_PHSEG10     3
#define MCP2515_PHSEG2      2
#define MCP2515_PHSEG1      1
#define MCP2515_PHSEG0      0

// CNF1
//(MCP 2515-I-P.pdf page 61 table 11.2)
#define MCP2515_SJW1        7
#define MCP2515_SJW0        6
#define MCP2515_BRP5        5
#define MCP2515_BRP4        4
#define MCP2515_BRP3        3
#define MCP2515_BRP2        2
#define MCP2515_BRP1        1
#define MCP2515_BRP0        0

// CANINTE
//(MCP 2515-I-P.pdf page 61 table 11.2)
#define MCP2515_MERRE       7
#define MCP2515_WAKIE       6
#define MCP2515_ERRIE       5
#define MCP2515_TX2IE       4
#define MCP2515_TX1IE       3
#define MCP2515_TX0IE       2
#define MCP2515_RX1IE       1
#define MCP2515_RX0IE       0

// CANINTF
//(MCP 2515-I-P.pdf page 61 table 11.2)
#define MCP2515_MERRF       7
#define MCP2515_WAKIF       6
#define MCP2515_ERRIF       5
#define MCP2515_TX2IF       4
#define MCP2515_TX1IF       3
#define MCP2515_TX0IF       2
#define MCP2515_RX1IF       1
#define MCP2515_RX0IF       0

// EFLG
//(MCP 2515-I-P.pdf page 61 table 11.2)
#define MCP2515_RX1OVR      7
#define MCP2515_RX0OVR      6
#define MCP2515_TXB0        5
#define MCP2515_TXEP        4
#define MCP2515_RXEP        3
#define MCP2515_TXWAR       2
#define MCP2515_RXWAR       1
#define MCP2515_EWARN       0

// TXB0CTRL, TXB1CTRL, TXB2CTRL
//(MCP 2515-I-P.pdf page 61 table 11.2)
#define MCP2515_ABTF        6
#define MCP2515_MLOA        5
#define MCP2515_TXERR       4
#define MCP2515_TXREQ       3
#define MCP2515_TXP1        1
#define MCP2515_TXP0        0

// RXB0CTRL
//(MCP 2515-I-P.pdf page 61 table 11.2)
#define MCP2515_RXM1        6
#define MCP2515_RXM0        5
#define MCP2515_RXRTR       3
#define MCP2515_BUKT        2
#define MCP2515_BUKT1       1
#define MCP2515_FILHIT0     0

// RXB1CTRL
//(MCP 2515-I-P.pdf page 61 table 11.2)
//(attention: RXM1, RXM0, RXRTR, FILHIT0 already defined by register RXB0CTRL)
#define MCP2515_FILHIT2     2
#define MCP2515_FILHIT1     1

// TXB0SIDL, TXB1SIDL
//(MCP 2515-I-P.pdf page 20 register 3-4)
#define MCP2515_EXIDE       3

// RXB0SIDL, RXB1SIDL
//(MCP 2515-I-P.pdf page 30 register 4-5)
#define MCP2515_SRR         4
#define MCP2515_IDE         3

// RXB0DLC, RXB1DLC, TXB0DLC, TXB1DLC
//(MCP 2515-I-P.pdf page 31 register 4-8)
#define MCP2515_RTR         6
#define MCP2515_DLC3        3
#define MCP2515_DLC2        2
#define MCP2515_DLC1        1
#define MCP2515_DLC0        0

// READ_STATUS instruction
//(MCP 2515-I-P.pdf page 67 figure 12-8)
#define MCP2515_RS_TX2IF    7
#define MCP2515_RS_TX2REQ   6
#define MCP2515_RS_TX1IF    5
#define MCP2515_RS_TX1REQ   4
#define MCP2515_RS_TX0IF    3
#define MCP2515_RS_TX0REQ   2
#define MCP2515_RS_RX1IF    1
#define MCP2515_RS_RX0IF    0

#endif // MCP2515_PRIV_H

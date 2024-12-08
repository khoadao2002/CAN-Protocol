#include "CAN_SPI.h"
#include "MCP2515.h"

/** Local Function Prototypes */
static uint32_t convertReg2ExtendedCANid(uint8_t tempRXBn_EIDH, uint8_t tempRXBn_EIDL, uint8_t tempRXBn_SIDH, uint8_t tempRXBn_SIDL);
static uint32_t convertReg2StandardCANid(uint8_t tempRXBn_SIDH, uint8_t tempRXBn_SIDL) ;
static void convertCANid2Reg(uint32_t tempPassedInID, uint8_t canIdType, id_reg_t *passedIdReg);

/** Global Variables */
ctrl_status_t ctrlStatus;
ctrl_error_status_t errorStatus;
id_reg_t idReg;

/** CAN SPI APIs */

/* Configuration for the MCP2515 in sleep mode */
void CANSPI_Sleep(void)
{
  /* Clear CAN bus wakeup interrupt */
  MCP2515_BitModify(MCP2515_CANINTF, 0x40, 0x00);

  /* Enable CAN bus activity wakeup */
  MCP2515_BitModify(MCP2515_CANINTE, 0x40, 0x40);

  MCP2515_SetSleepMode();
}

/* Initialization for the CAN protocol */
int CANSPI_Initialize(void)
{

	/* The Rx buffer of MCP2515 includes the 2 Mask & 5 the Filter */
	RXF0 RXF0reg;
	RXF1 RXF1reg;
	RXF2 RXF2reg;
	RXF3 RXF3reg;
	RXF4 RXF4reg;
	RXF5 RXF5reg;
	RXM0 RXM0reg;
	RXM1 RXM1reg;

	/* Set the Rx Mask values */
	RXM0reg.RXM0SIDH = 0xFF;
	RXM0reg.RXM0SIDL = 0xE0;
	RXM0reg.RXM0EID8 = 0x00;
	RXM0reg.RXM0EID0 = 0x00;

	RXM1reg.RXM1SIDH = 0xFF;
	RXM1reg.RXM1SIDL = 0xEB;
	RXM1reg.RXM1EID8 = 0xFF;
	RXM1reg.RXM1EID0 = 0xFF;

	/* Set the Rx Filter values */
	RXF0reg.RXF0SIDH = 0x00;
	RXF0reg.RXF0SIDL = 0x00;	//Standard Filter
	RXF0reg.RXF0EID8 = 0x00;
	RXF0reg.RXF0EID0 = 0x00;

	RXF1reg.RXF1SIDH = 0x00;
	RXF1reg.RXF1SIDL = 0x00;	//Standard Filter
	RXF1reg.RXF1EID8 = 0x00;
	RXF1reg.RXF1EID0 = 0x00;

	RXF2reg.RXF2SIDH = 0x00;
	RXF2reg.RXF2SIDL = 0x00;	//Standard Filter
	RXF2reg.RXF2EID8 = 0x00;
	RXF2reg.RXF2EID0 = 0x00;

	RXF3reg.RXF3SIDH = 0x00;
	RXF3reg.RXF3SIDL = 0x00;	//Standard Filter
	RXF3reg.RXF3EID8 = 0x00;
	RXF3reg.RXF3EID0 = 0x00;

	RXF4reg.RXF4SIDH = 0x00;
	RXF4reg.RXF4SIDL = 0x00;	//Standard Filter
	RXF4reg.RXF4EID8 = 0x00;
	RXF4reg.RXF4EID0 = 0x00;

	RXF5reg.RXF5SIDH = 0x00;
	RXF5reg.RXF5SIDL = 0x08;	//Extended Filter
	RXF5reg.RXF5EID8 = 0x00;
	RXF5reg.RXF5EID0 = 0x00;

	/* Check the status of MCP2515 (ready or not ready) */
	if(!MCP2515_Initialize())
	return -1;

	/* Set the MCP2515 to configuration Mode */
	if(!MCP2515_SetConfigMode())
	return -2;

	/* Configuration for the Filter & Mask */
	MCP2515_WriteByteSequence(MCP2515_RXM0SIDH, MCP2515_RXM0EID0, &(RXM0reg.RXM0SIDH));
	MCP2515_WriteByteSequence(MCP2515_RXM1SIDH, MCP2515_RXM1EID0, &(RXM1reg.RXM1SIDH));
	MCP2515_WriteByteSequence(MCP2515_RXF0SIDH, MCP2515_RXF0EID0, &(RXF0reg.RXF0SIDH));
	MCP2515_WriteByteSequence(MCP2515_RXF1SIDH, MCP2515_RXF1EID0, &(RXF1reg.RXF1SIDH));
	MCP2515_WriteByteSequence(MCP2515_RXF2SIDH, MCP2515_RXF2EID0, &(RXF2reg.RXF2SIDH));
	MCP2515_WriteByteSequence(MCP2515_RXF3SIDH, MCP2515_RXF3EID0, &(RXF3reg.RXF3SIDH));
	MCP2515_WriteByteSequence(MCP2515_RXF4SIDH, MCP2515_RXF4EID0, &(RXF4reg.RXF4SIDH));
	MCP2515_WriteByteSequence(MCP2515_RXF5SIDH, MCP2515_RXF5EID0, &(RXF5reg.RXF5SIDH));

	/* Accept All (Standard + Extended) */
	MCP2515_WriteByte(MCP2515_RXB0CTRL, 0x04);    //Enable BUKT allowed the data shift to Filter 1 when Filter 0 is full, Accept Filter 0
	MCP2515_WriteByte(MCP2515_RXB1CTRL, 0x01);    //Accept Filter 1


	/* Configuration for the bit timing */
	/* TQ (Time Quantum) = 1 / F_sys = ..A..(s)
	 * bit timing = 1 / bit rate () = ..B..(s) => bit timing = B/A = ..C..(TQ)
	 * the bit timing includes 4 segments: Synchronization Segment, Propagation Segment, Phase1_Segment, Phase2_Segment
	 * T_SyncSeg = 1 TQ (Default).
	 * T_PropSeg = 1..8 TQ.		(option)
	 * T_Phase1Seg = 1..8 TQ.	(option)
	 * T_Phase2Seg = 1..8 TQ.	(option)
	 * bit timing = T_SyncSeg + T_PropSeg + T_Phase1Seg + T_Phase2Seg.
	 * */

	/* Set bit rate equals 250Kbps*/
	MCP2515_WriteByte(MCP2515_CNF1, 0x00);

	MCP2515_WriteByte(MCP2515_CNF2, 0xAC);

	MCP2515_WriteByte(MCP2515_CNF3, 0x83);

	/* Shift MCP2515 to normal status */
	if(!MCP2515_SetNormalMode())
	return -3;

	return 1;
}

/* Configuration for the CAN transmission */
uint8_t CANSPI_Transmit(uCAN_MSG *tempCanMsg)
{
  uint8_t returnValue = 0;

  idReg.tempSIDH = 0;
  idReg.tempSIDL = 0;
  idReg.tempEID8 = 0;
  idReg.tempEID0 = 0;

  /* Check the one of 3 Transmitter status (TX0, TX1, TX2), if any Transmitter is ready (TXBnREQ bit is pulled high)   */
  ctrlStatus.ctrl_status = MCP2515_ReadStatus();

  /* Check bit TXBnREQ of Transmitters */
  if (ctrlStatus.TXB0REQ != 1)
  {
    /* Write ID value into idReg (SIDH, SIDL, EID8, EID0) */
    convertCANid2Reg(tempCanMsg->frame.id, tempCanMsg->frame.idType, &idReg);

    /* Loading ID, DLC (data length code), Data values into TX0 */
    MCP2515_LoadTxSequence(MCP2515_LOAD_TXB0SIDH, &(idReg.tempSIDH), tempCanMsg->frame.dlc, &(tempCanMsg->frame.data0));

    /* Request Tx buffer transmit message */
    MCP2515_RequestToSend(MCP2515_RTS_TX0);

    returnValue = 1;
  }
  else if (ctrlStatus.TXB1REQ != 1)
  {
    convertCANid2Reg(tempCanMsg->frame.id, tempCanMsg->frame.idType, &idReg);

    MCP2515_LoadTxSequence(MCP2515_LOAD_TXB1SIDH, &(idReg.tempSIDH), tempCanMsg->frame.dlc, &(tempCanMsg->frame.data0));
    MCP2515_RequestToSend(MCP2515_RTS_TX1);

    returnValue = 1;
  }
  else if (ctrlStatus.TXB2REQ != 1)
  {
    convertCANid2Reg(tempCanMsg->frame.id, tempCanMsg->frame.idType, &idReg);

    MCP2515_LoadTxSequence(MCP2515_LOAD_TXB2SIDH, &(idReg.tempSIDH), tempCanMsg->frame.dlc, &(tempCanMsg->frame.data0));
    MCP2515_RequestToSend(MCP2515_RTS_TX2);

    returnValue = 1;
  }

  return (returnValue);
}

/* Configuration for the CAN Reception */
uint8_t CANSPI_Receive(uCAN_MSG *tempCanMsg)
{
  uint8_t returnValue = 0;
  rx_reg_t rxReg;
  ctrl_rx_status_t rxStatus;

  /* Check for the Receiver status */
  rxStatus.ctrl_rx_status = MCP2515_GetRxStatus();

  /* Check rx buffer value, if the rx buffer value equals 0 (not message given), else message given */
  if (rxStatus.rxBuffer != 0)
  {
    /* Receive data from RX */
    if ((rxStatus.rxBuffer == MSG_IN_RXB0)|(rxStatus.rxBuffer == MSG_IN_BOTH_BUFFERS))
    {
      MCP2515_ReadRxSequence(MCP2515_READ_RXB0SIDH, rxReg.rx_reg_array, sizeof(rxReg.rx_reg_array));
    }
    else if (rxStatus.rxBuffer == MSG_IN_RXB1)
    {
      MCP2515_ReadRxSequence(MCP2515_READ_RXB1SIDH, rxReg.rx_reg_array, sizeof(rxReg.rx_reg_array));
    }

    /* Check ID Type (Standard or Extended) */
    if (rxStatus.msgType == dEXTENDED_CAN_MSG_ID_2_0B)
    {
      tempCanMsg->frame.idType = (uint8_t) dEXTENDED_CAN_MSG_ID_2_0B;
      tempCanMsg->frame.id = convertReg2ExtendedCANid(rxReg.RXBnEID8, rxReg.RXBnEID0, rxReg.RXBnSIDH, rxReg.RXBnSIDL);
    }
    else
    {
      tempCanMsg->frame.idType = (uint8_t) dSTANDARD_CAN_MSG_ID_2_0B;
      tempCanMsg->frame.id = convertReg2StandardCANid(rxReg.RXBnSIDH, rxReg.RXBnSIDL);
    }

    tempCanMsg->frame.dlc   = rxReg.RXBnDLC;
    tempCanMsg->frame.data0 = rxReg.RXBnD0;
    tempCanMsg->frame.data1 = rxReg.RXBnD1;
    tempCanMsg->frame.data2 = rxReg.RXBnD2;
    tempCanMsg->frame.data3 = rxReg.RXBnD3;
    tempCanMsg->frame.data4 = rxReg.RXBnD4;
    tempCanMsg->frame.data5 = rxReg.RXBnD5;
    tempCanMsg->frame.data6 = rxReg.RXBnD6;
    tempCanMsg->frame.data7 = rxReg.RXBnD7;

    returnValue = 1;
  }

  return (returnValue);
}

/* Check Rx buffer value */
uint8_t CANSPI_messagesInBuffer(void)
{
  uint8_t messageCount = 0;

  ctrlStatus.ctrl_status = MCP2515_ReadStatus();

  if(ctrlStatus.RX0IF != 0)
  {
    messageCount++;
  }

  if(ctrlStatus.RX1IF != 0)
  {
    messageCount++;
  }

  return (messageCount);
}

/* Check Buss_Off condition of CAN bus */
uint8_t CANSPI_isBussOff(void)
{
  uint8_t returnValue = 0;

  errorStatus.error_flag_reg = MCP2515_ReadByte(MCP2515_EFLG);

  if(errorStatus.TXBO == 1)
  {
    returnValue = 1;
  }

  return (returnValue);
}

/* Check Error Passive condition of the Receiver */
uint8_t CANSPI_isRxErrorPassive(void)
{
  uint8_t returnValue = 0;

  errorStatus.error_flag_reg = MCP2515_ReadByte(MCP2515_EFLG);

  if(errorStatus.RXEP == 1)
  {
    returnValue = 1;
  }

  return (returnValue);
}

/* Check Error Passive of Transmitter */
uint8_t CANSPI_isTxErrorPassive(void)
{
  uint8_t returnValue = 0;

  errorStatus.error_flag_reg = MCP2515_ReadByte(MCP2515_EFLG);

  if(errorStatus.TXEP == 1)
  {
    returnValue = 1;
  }

  return (returnValue);
}

/* Convert the SIDH, SIDL, EID8, EDI0 values to 29-bits ID (Extended Frame) */
static uint32_t convertReg2ExtendedCANid(uint8_t tempRXBn_EIDH, uint8_t tempRXBn_EIDL, uint8_t tempRXBn_SIDH, uint8_t tempRXBn_SIDL)
{
  uint32_t returnValue = 0;
  uint32_t ConvertedID = 0;
  uint8_t CAN_standardLo_ID_lo2bits;
  uint8_t CAN_standardLo_ID_hi3bits;

  CAN_standardLo_ID_lo2bits = (tempRXBn_SIDL & 0x03);
  CAN_standardLo_ID_hi3bits = (tempRXBn_SIDL >> 5);
  ConvertedID = (tempRXBn_SIDH << 3);
  ConvertedID = ConvertedID + CAN_standardLo_ID_hi3bits;
  ConvertedID = (ConvertedID << 2);
  ConvertedID = ConvertedID + CAN_standardLo_ID_lo2bits;
  ConvertedID = (ConvertedID << 8);
  ConvertedID = ConvertedID + tempRXBn_EIDH;
  ConvertedID = (ConvertedID << 8);
  ConvertedID = ConvertedID + tempRXBn_EIDL;
  returnValue = ConvertedID;
  return (returnValue);
}

/* Convert SIDL, SIDH, EID8, EDI0 values to 11-bits ID (Standard Frame) */
static uint32_t convertReg2StandardCANid(uint8_t tempRXBn_SIDH, uint8_t tempRXBn_SIDL)
{
  uint32_t returnValue = 0;
  uint32_t ConvertedID;

  ConvertedID = (tempRXBn_SIDH << 3);
  ConvertedID = ConvertedID + (tempRXBn_SIDL >> 5);
  returnValue = ConvertedID;

  return (returnValue);
}

/* Convert the 29-bits ID (Extended Frame) or the 11-bits ID (Standard Frame) into SIDL, SIDH, EID8, EID0 */
static void convertCANid2Reg(uint32_t tempPassedInID, uint8_t canIdType, id_reg_t *passedIdReg)
{
  uint8_t wipSIDL = 0;

  if (canIdType == dEXTENDED_CAN_MSG_ID_2_0B)
  {
    //EID0
    passedIdReg->tempEID0 = 0xFF & tempPassedInID;
    tempPassedInID = tempPassedInID >> 8;

    //EID8
    passedIdReg->tempEID8 = 0xFF & tempPassedInID;
    tempPassedInID = tempPassedInID >> 8;

    //SIDL
    wipSIDL = 0x03 & tempPassedInID;
    tempPassedInID = tempPassedInID << 3;
    wipSIDL = (0xE0 & tempPassedInID) + wipSIDL;
    wipSIDL = wipSIDL + 0x08;
    passedIdReg->tempSIDL = 0xEB & wipSIDL;

    //SIDH
    tempPassedInID = tempPassedInID >> 8;
    passedIdReg->tempSIDH = 0xFF & tempPassedInID;
  }
  else
  {
    passedIdReg->tempEID8 = 0;
    passedIdReg->tempEID0 = 0;
    tempPassedInID = tempPassedInID << 5;
    passedIdReg->tempSIDL = 0xFF & tempPassedInID;
    tempPassedInID = tempPassedInID >> 8;
    passedIdReg->tempSIDH = 0xFF & tempPassedInID;
  }
}

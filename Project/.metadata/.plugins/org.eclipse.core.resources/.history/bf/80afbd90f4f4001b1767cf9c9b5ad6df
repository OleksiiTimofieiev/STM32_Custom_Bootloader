/*
 * boot.c
 *
 *  Created on: Aug 4, 2021
 *      Author: otimofieiev
 */

#include "boot.h"

void printMsg(UART uart, char *format,...)
{
#ifdef BL_DEBUG_MSG_EN
	char str[80];

	va_list args;
	va_start(args, format);
	vsprintf(str, format, args);
	if (uart == Console)
	{
		HAL_UART_Transmit(C_UART, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
	}
	else
	{
		HAL_UART_Transmit(D_UART, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
	}
	va_end(args);

#endif
}

void bootloader_uart_read_data(void)
{
    uint8_t rcv_len=0;

	while(1)
	{
		memset(bl_rx_buffer,0,200);
		//here we will read and decode the commands coming from host
		//first read only one byte from the host , which is the "length" field of the command packet
		HAL_UART_Receive(C_UART,bl_rx_buffer,1,HAL_MAX_DELAY);
		rcv_len= bl_rx_buffer[0];
		HAL_UART_Receive(C_UART,&bl_rx_buffer[1],rcv_len,HAL_MAX_DELAY);
		switch(bl_rx_buffer[1])
		{
              case BL_GET_VER:
                  bootloader_handle_getver_cmd(bl_rx_buffer);
                  break;
              // case BL_GET_HELP:
              //     bootloader_handle_gethelp_cmd(bl_rx_buffer);
              //     break;
              // case BL_GET_CID:
              //     bootloader_handle_getcid_cmd(bl_rx_buffer);
              //     break;
              // case BL_GET_RDP_STATUS:
              //     bootloader_handle_getrdp_cmd(bl_rx_buffer);
              //     break;
              // case BL_GO_TO_ADDR:
              //     bootloader_handle_go_cmd(bl_rx_buffer);
              //     break;
              // case BL_FLASH_ERASE:
              //     bootloader_handle_flash_erase_cmd(bl_rx_buffer);
              //     break;
              // case BL_MEM_WRITE:
              //     bootloader_handle_mem_write_cmd(bl_rx_buffer);
              //     break;
              // case BL_EN_RW_PROTECT:
              //     bootloader_handle_en_rw_protect(bl_rx_buffer);
              //     break;
              // case BL_MEM_READ:
              //     bootloader_handle_mem_read(bl_rx_buffer);
              //     break;
              // case BL_READ_SECTOR_P_STATUS:
              //     bootloader_handle_read_sector_protection_status(bl_rx_buffer);
              //     break;
              // case BL_OTP_READ:
              //     bootloader_handle_read_otp(bl_rx_buffer);
              //     break;
              // case BL_DIS_R_W_PROTECT:
              //     bootloader_handle_dis_rw_protect(bl_rx_buffer);
              //     break;
               default:
            	  printMsg(Debug, "BL_DEBUG_MSG:Invalid command code received from host \n");
                  break;
		}

	}

}

void bootloader_jump_to_user_app(void)
{
  void (*app_reset_handler)(void);

  printMsg(Debug, "booloader jump to user app\r\n");

  // configure MSP
  uint32_t msp_value = *(volatile uint32_t *)FLASH_SECTOR2_BASE_ADDRESS;

  //  cmsis
  __set_MSP(msp_value);

  uint32_t resethandler_address = *(volatile uint32_t *) (FLASH_SECTOR2_BASE_ADDRESS + 4);

  app_reset_handler = (void*) resethandler_address;

  printMsg(Debug, "BL_DEBUG_MSG: app reset handler addr : %#x\n",app_reset_handler);

  //3. jump to reset handler of the user application
  app_reset_handler();
}

/*This function sends ACK if CRC matches along with "len to follow"*/
void bootloader_send_ack(uint8_t command_code, uint8_t follow_len)
{
	 //here we send 2 byte.. first byte is ack and the second byte is len value
	uint8_t ack_buf[2];
	ack_buf[0] = BL_ACK;
	ack_buf[1] = follow_len;
	HAL_UART_Transmit(C_UART,ack_buf,2,HAL_MAX_DELAY);
}

/*This function sends NACK */
void bootloader_send_nack(void)
{
	uint8_t nack = BL_NACK;
	HAL_UART_Transmit(C_UART,&nack,1,HAL_MAX_DELAY);
}

//This verifies the CRC of the given buffer in pData .
uint8_t bootloader_verify_crc (uint8_t *pData, uint32_t len, uint32_t crc_host)
{
    uint32_t uwCRCValue=0xff;

    for (uint32_t i=0 ; i < len ; i++)
	{
        uint32_t i_data = pData[i];
        uwCRCValue = HAL_CRC_Accumulate(&hcrc, &i_data, 1);
	}

	 /* Reset CRC Calculation Unit */
    __HAL_CRC_DR_RESET(&hcrc);

	if( uwCRCValue == crc_host)
	{
		return VERIFY_CRC_SUCCESS;
	}

	return VERIFY_CRC_FAIL;
}

//Just returns the macro value .
uint8_t get_bootloader_version(void)
{
  return (uint8_t)BL_VERSION;
}

/* This function writes data in to C_UART */
void bootloader_uart_write_data(uint8_t *pBuffer,uint32_t len)
{
    /*you can replace the below ST's USART driver API call with your MCUs driver API call */
	HAL_UART_Transmit(C_UART,pBuffer,len,HAL_MAX_DELAY);

}

void bootloader_handle_getver_cmd(uint8_t *bl_rx_buffer)
{
    uint8_t bl_version;

    // 1) verify the checksum
    printMsg(Debug, "BL_DEBUG_MSG:bootloader_handle_getver_cmd\n");

	//Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

    if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
    {
    	printMsg(Debug,"BL_DEBUG_MSG:checksum success !!\n");
        // checksum is correct..
        bootloader_send_ack(bl_rx_buffer[0],1);
        bl_version=get_bootloader_version();
        printMsg(Debug,"BL_DEBUG_MSG:BL_VER : %d %#x\n",bl_version,bl_version);
        bootloader_uart_write_data(&bl_version,1);

    }else
    {
    	printMsg(Debug,"BL_DEBUG_MSG:checksum fail !!\n");
        //checksum is wrong send nack
        bootloader_send_nack();
    }
}

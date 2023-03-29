uint8_t BLECommandsReceiver(uint8_t *buff, uint8_t count)
{    
    uint8_t _string[20]={'0', '2'};
    uint8_t command;
    uint8_t num_of_packet;
    uint8_t check_sum = 0;
    uint8_t index = 0;
    uint8_t result_index = 0;
    uint8_t index_of_start_data = 4;
    uint8_t data_size = 15;
    const uint8_t top = 20;
    const uint8_t left = 20;
    const uint8_t step = 20;
    uint8_t index_in_packet = 0;

    
    uint8_t j = 0;
    while (j < 200) //Внимание!! j инкрементируется не в одном месте
    {
                switch (byte_num)
                {
                    case 0:
                        if (buff[j] == '0') byte_num = 1;
                        index_in_packet = 0;
                        break;
                    case 1:
                        if (buff[j] == '2') byte_num = 2;
                        break;
                    case 2:
                        command = buff[j];
                        byte_num = 3;
                        break;
                    case 3:
                        num_of_packet = buff[j];
                        indexUrl = num_of_packet * data_size;
                        current_packet_num++;
                        byte_num = 4;
                        break;
                    case 4:
                        byte_num = 5;
                        break;
                }
                if (byte_num == 5)
                {
                    switch (command)
                    {
                    case BLE_CMD_DATETIME:
                        for (int a = 0; a < 10; a++)
                        {
                            check_sum += buff[j + a];
                        }                                
                        if (buff[j + 10] == check_sum)
                        {
                            cur_day = cur_month = cur_month = cur_year = cur_tss = cur_tmm = cur_thh = 0;
                            cur_day = (uint16_t)buff[j + 3];
                            cur_month = (uint16_t)buff[j + 4];
                            cur_year = (uint16_t)(2000 + buff[j + 5]);
                            cur_tss = (uint32_t)buff[j + 6];
                            cur_tmm = (uint32_t)buff[j + 7];
                            cur_thh = (uint32_t)buff[j + 8];    
                        
                            TimeSet((uint32_t)cur_thh, (uint32_t)cur_tmm, (uint32_t)cur_tss);
                            WriteBackupRegister((uint16_t)cur_day, (uint16_t)cur_month, (uint16_t)cur_year);
                        
                            buff[j] = 0xFF;
                            return 2;                    
                        }
                        break;
                    case BLE_CMD_SERIAL:
                        for (int a = 0; a < 10; a++)
                        {
                            check_sum += buff[j + a];
                        }                                
                        if (buff[j+10] == check_sum)
                        {
                            SERIAL[2] = buff[j + 3];
                            SERIAL[3] = buff[j + 4];
                            SERIAL[4] = buff[j + 5];
                            SERIAL[5] = buff[j + 6];
                            SERIAL[6] = buff[j + 7];
                            SERIAL[7] = buff[j + 8];
                            SERIAL[8] = buff[j + 9];
                            
                            FmcProgramSerial();                        
                        
                            delay_1ms(200);                                    
                            DeviceOff();
                        
                            return 1;
                        }
                        break;
                    case BLE_CMD_SETURL:
                        send_buff[indexUrl + result_index] = buff[j];
                        result_index++;
                        check_sum += buff[j];
                        if (buff[j] == 0)
                        {
                            last_packet_num = num_of_packet;
                            byte_num = 0;
                            result_index = 0;
                            j++;
                            if (current_packet_num == last_packet_num + 1)
                            {
                                last_packet_num = 0;
                                current_packet_num = 0;
                                ILI9341_WriteString(left, top, send_buff, Font_Arial, ILI9341_RED, ILI9341_WHITE);                          
                                return BLE_CMD_SETURL;
                            }
                        }
                        break;
                    case BLE_CMD_GETURL:
                        break;
                    case BLE_CMD_SETPORT:
                        if (FillBuff(buff, j)) ILI9341_WriteString(left, top + step * 2, send_buff, Font_Arial, ILI9341_RED, ILI9341_WHITE);  
                        return BLE_CMD_SETPORT;
                        break;
                    case BLE_CMD_GETPORT:
                        break;
                    case BLE_CMD_SETLOGIN:
                        if (FillBuff(buff, j)) ILI9341_WriteString(left, top + step * 3, send_buff, Font_Arial, ILI9341_RED, ILI9341_WHITE);  
                        return BLE_CMD_SETLOGIN;
                        break;
                    case BLE_CMD_GETLOGIN:
                        sprintf(send_buff,"Login2");
                        send_buf_UART_0(send_buff, 7);
                        break;
                    case BLE_CMD_SETPASSWORD:
                        if (FillBuff(buff, j)) ILI9341_WriteString(left, top + step * 4, send_buff, Font_Arial, ILI9341_RED, ILI9341_WHITE);  
                        return BLE_CMD_SETPASSWORD;
                        break;
                    case BLE_CMD_GETPASSWORD:
                        break;
                    case BLE_CMD_SETPOINT:
                        if (FillBuff(buff, j)) ILI9341_WriteString(left, top + step * 5, send_buff, Font_Arial, ILI9341_RED, ILI9341_WHITE);  
                        return BLE_CMD_SETPOINT;
                        break;
                    case BLE_CMD_GETPOINT:
                        break;
                    case BLE_CMD_SETID:
                        if (FillBuff(buff, j)) ILI9341_WriteString(left, top + step * 6, send_buff, Font_Arial, ILI9341_RED, ILI9341_WHITE);  
                        return BLE_CMD_SETID;
                        break;
                    case BLE_CMD_GETID:
                        break;
                    }
                }
            j++;
            switch (command)
            {
                case BLE_CMD_SETURL:
                    index_in_packet++;
                    if (index_in_packet == BLE_PACKET_SIZE - 1)
                    {
                        if (last_packet_num > 0)
                        {
                            if (current_packet_num == last_packet_num + 1)
                            {
                                last_packet_num = 0;
                                current_packet_num = 0;
                                ILI9341_WriteString(left, top, send_buff, Font_Arial, ILI9341_RED, ILI9341_WHITE);                          
                                indexUrl = 0;
                                return BLE_CMD_SETURL;
                            }
                        }
                        j++;
                        byte_num = 0;
                        result_index = 0;
                    }
                    break;
            }
        }
    return 0;
}

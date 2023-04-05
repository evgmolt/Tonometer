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

uint8_t BLECommandsReceiver(uint8_t *buff)
{
    const uint8_t top = 20;
    const uint8_t left = 20;
    const uint8_t step = 20;
    bool print_allow = false; 
    for (int i = 0; i < UART0_count; i++)
    {
        checksum += buff[i];
        switch (byte_num)
        {
            case 0:
                if (buff[i] == '0') byte_num = 1;
                checksum = buff[i];
                index_in_packet = 0;
                break;
            case 1:
                if (buff[i] == '2') byte_num = 2;
                break;
            case 2:
                command = buff[i];
                byte_num = 3;
                break;
            case 3:
                num_of_packet = buff[i];
                current_packet_num++;
                byte_num = 4;
                break;
            case 4:
                byte_num = 5;
                break;
        }
        if (byte_num == 5)
        {
            send_buff[result_index] = buff[i];
            result_index++;
            switch (command)
            {
                case BLE_CMD_SERIAL:
                    if (result_index == 6) //длина номера -1 без '0' и '2'
                    {
                        if (checksum == buff[i + 1])
                        {
                            for (uint8_t i = 0; i < 7; i++)
                            {
                                SERIAL[i + 2] = send_buff[i];
                            }
                        }
                    }
                    FmcProgramSerial();                        
                    delay_1ms(200);                                    
                    DeviceOff();
                    return 1;
                case BLE_CMD_SETURL:
                    if (index_in_packet == BLE_PACKET_SIZE - 2)
                    {
                        index_in_packet = 0;
                        byte_num = 0;
                    }
                    print_allow = true;
                    break;
                case BLE_CMD_GETURL:
                    if (buff[i + 1] == checksum) 
                    {
                        sprintf(send_buff,"URL222");
                        send_buf_UART_0(send_buff, 7);
                    }
                    break;
                case BLE_CMD_SETPORT:
                    print_allow = true;
                    break;
                case BLE_CMD_GETPORT:
                    if (buff[i + 1] == checksum) 
                    {
                        sprintf(send_buff,"8080");
                        send_buf_UART_0(send_buff, 5);
                    }
                    break;
                case BLE_CMD_SETLOGIN:
                    if (index_in_packet == BLE_PACKET_SIZE - 2)
                    {
                        index_in_packet = 0;
                        byte_num = 0;
                    }
                    print_allow = true;
                    break;
                case BLE_CMD_GETLOGIN:
                    sprintf(send_buff,"Login222");
                    send_buf_UART_0(send_buff, 9);
                    break;
                case BLE_CMD_SETPASSWORD:
                    if (index_in_packet == BLE_PACKET_SIZE - 2)
                    {
                        index_in_packet = 0;
                        byte_num = 0;
                    }
                    print_allow = true;
                    break;
                case BLE_CMD_GETPASSWORD:
                    break;
                case BLE_CMD_SETPOINT:
                    print_allow = true;
                    break;
                case BLE_CMD_GETPOINT:
                    sprintf(send_buff,"Point222");
                    send_buf_UART_0(send_buff, 9);
                    break;
                case BLE_CMD_SETID:
                    print_allow = true;
                    break;
                case BLE_CMD_GETID:
                    sprintf(send_buff,"ID222");
                    send_buf_UART_0(send_buff, 6);
                    break;
            }
            if (buff[i] == 0)
            {
                if (checksum == buff[i + 1])
                {
                    if (print_allow) ILI9341_WriteString(left, step + (command - 6) * step, send_buff, Font_Arial, ILI9341_RED, ILI9341_WHITE);  
                    checksum = 0;
                    byte_num = 0;
                    index_in_packet = 0;
                    current_packet_num = 0;
                    result_index = 0;
                }
            }
        }
        index_in_packet++;
    }
    UART0_count = 0;
}

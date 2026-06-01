// 3.1) spi_bus_init
esp_err_t spi_bus_init(spi_host_device_t spi_host,
                       gpio_num_t pin_mosi,
                       gpio_num_t pin_miso,
                       gpio_num_t pin_sclk,
                       gpio_num_t pin_cs,
                       spi_device_handle_t *mcp4132_handle)
{
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = pin_mosi,
        .miso_io_num = pin_miso,
        .sclk_io_num = pin_sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 2
    };

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 1000000,
        .mode = 0,
        .spics_io_num = pin_cs,
        .queue_size = 1,
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0
    };

    esp_err_t ret = spi_bus_initialize(spi_host, &bus_cfg, SPI_DMA_DISABLED);

    if (ret != ESP_OK) {
        return ret;
    }

    ret = spi_bus_add_device(spi_host, &dev_cfg, mcp4132_handle);

    if (ret != ESP_OK) {
        spi_bus_free(spi_host);
        return ret;
    }

    return ESP_OK;
}


// 3.2) mcp4132_write_register
esp_err_t mcp4132_write_register(spi_device_handle_t mcp4132_handle,
                                 uint8_t address,
                                 uint16_t value)
{
    uint8_t tx_data[2];

    address = address & 0x0F;
    value = value & 0x01FF;

    tx_data[0] = (address << 4) | ((value >> 8) & 0x01);
    tx_data[1] = value & 0xFF;

    spi_transaction_t trans = {
        .length = 16,
        .tx_buffer = tx_data,
        .rx_buffer = NULL
    };

    return spi_device_transmit(mcp4132_handle, &trans);
}


// 3.3) mcp4132_read_register
uint16_t mcp4132_read_register(spi_device_handle_t mcp4132_handle,
                               uint8_t address)
{
    uint8_t tx_data[2];
    uint8_t rx_data[2];
    uint16_t value;

    address = address & 0x0F;

    tx_data[0] = (address << 4) | 0x0C;
    tx_data[1] = 0x00;

    spi_transaction_t trans = {
        .length = 16,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data
    };

    if (spi_device_transmit(mcp4132_handle, &trans) != ESP_OK) {
        return 0xFFFF;
    }

    value = (((uint16_t)(rx_data[0] & 0x01)) << 8) | rx_data[1];

    return value & 0x01FF;
}


// 4.1) mcp4132_set_wiper
esp_err_t mcp4132_set_wiper(spi_device_handle_t mcp4132_handle,
                            uint8_t n)
{
    if (n > 128) {
        return ESP_ERR_INVALID_ARG;
    }

    return mcp4132_write_register(mcp4132_handle, 0x00, (uint16_t)n);
}


// 4.2) mcp4132_set_cutoff_frequency
esp_err_t mcp4132_set_cutoff_frequency(spi_device_handle_t mcp4132_handle,
                                       float fc_hz)
{
    const float rab_ohm = 10000.0f;
    const float rw_ohm = 75.0f;
    const float c_farads = 10.0e-9f;

    float rwb_target;
    float n_float;
    uint8_t n;

    if (fc_hz <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    rwb_target = 1.0f / (2.0f * 3.14159265f * fc_hz * c_farads);

    n_float = ((rwb_target - rw_ohm) * 128.0f) / rab_ohm;

    if (n_float < 0.0f) {
        n = 0;
    }
    else if (n_float > 128.0f) {
        n = 128;
    }
    else {
        n = (uint8_t)(n_float + 0.5f);
    }

    return mcp4132_set_wiper(mcp4132_handle, n);
}


// sample_timer_callback
void sample_timer_callback(void *arg)
{
    TaskHandle_t task_handle = (TaskHandle_t)arg;

    xTaskNotifyGive(task_handle);
}


// 5) app_main
void app_main(void)
{
    spi_device_handle_t mcp4132_handle;

    adc1_channel_t geophone_adc_channel = ADC1_CHANNEL_6;

    float vref = 3.3f;
    float adc_max = 4095.0f;
    float voltage;

    int adc_raw;
    int len;

    uint8_t current_wiper = 0xFF;

    char uart_msg[64];

    spi_bus_init(SPI2_HOST,
                 GPIO_NUM_23,
                 GPIO_NUM_19,
                 GPIO_NUM_18,
                 GPIO_NUM_5,
                 &mcp4132_handle);

    adc1_config_width(ADC_WIDTH_BIT_12);

    adc1_config_channel_atten(geophone_adc_channel, ADC_ATTEN_DB_11);

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    uart_param_config(UART_NUM_0, &uart_config);

    uart_set_pin(UART_NUM_0,
                 UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE);

    uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);

    esp_timer_handle_t sample_timer;

    esp_timer_create_args_t timer_args = {
        .callback = sample_timer_callback,
        .arg = xTaskGetCurrentTaskHandle(),
        .name = "sample_timer_1khz"
    };

    esp_timer_create(&timer_args, &sample_timer);

    esp_timer_start_periodic(sample_timer, 1000);

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        adc_raw = adc1_get_raw(geophone_adc_channel);

        voltage = ((float)adc_raw * vref) / adc_max;

        if (voltage > 1.4f) {
            if (current_wiper != 95) {
                mcp4132_set_wiper(mcp4132_handle, 95);

                current_wiper = 95;

                len = snprintf(uart_msg,
                               sizeof(uart_msg),
                               "Senal mayor a 1.4 V. Wiper actual N = %u\r\n",
                               current_wiper);

                uart_write_bytes(UART_NUM_0, uart_msg, len);
            }
        }
        else if (voltage < 0.9f) {
            if (current_wiper != 42) {
                mcp4132_set_wiper(mcp4132_handle, 42);

                current_wiper = 42;

                len = snprintf(uart_msg,
                               sizeof(uart_msg),
                               "Senal menor a 0.9 V. Wiper actual N = %u\r\n",
                               current_wiper);

                uart_write_bytes(UART_NUM_0, uart_msg, len);
            }
        }
    }
}
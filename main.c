#include <stdio.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <string.h>
#include <esp_log.h>
#include <led_strip.h>
#define BLINK_GPIO 8
static const char* TAG = "RS485_TX";
uint8_t tx_buffer[50];
uint8_t rx_buffer[50];
bool gpio_state= 1;
led_strip_handle_t led_strip;
QueueHandle_t uart_event_queue;

   void RS485_SetTX(void);
    void RS485_SetRX(void);
void rgb_led_init(void)
{
    led_strip_config_t led_strip_config = {
        .strip_gpio_num = GPIO_NUM_8,
        .max_leds = 1,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = 0
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = 0
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&led_strip_config, &rmt_config, &led_strip));
    //led_strip_clear(led_strip);
     led_strip_refresh(led_strip);
}
void gpio_isr(void* arg)
{
    ESP_DRAM_LOGI(TAG,"BUTTON PRESSED");
    gpio_state = 0;

}
void RS485_Send(uart_port_t uart_port,uint8_t* buf,uint16_t size)
{
    RS485_SetTX();
    uart_write_bytes(uart_port,buf,size);
    uart_wait_tx_done(uart_port,portMAX_DELAY);
    RS485_SetRX();
}
void gpio_isr_init(void)
{
    gpio_config_t io_conf;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;

    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pin_bit_mask = 1ULL << GPIO_NUM_9;
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_NUM_9, gpio_isr, NULL);
    gpio_intr_enable(GPIO_NUM_9);

}
void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
        };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1,5,4,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, 1024 * 2, 1024 * 2, 30, &uart_event_queue, 0);
    
}
void RS485_Init(void)
{
    uart_init();
    gpio_reset_pin(14);
    gpio_set_direction(14,GPIO_MODE_OUTPUT);

}
void RS485_SetTX()
{
    gpio_set_level(14,1);
}
void RS485_SetRX()
{
    gpio_set_level(14,0);
}
void uart_event_task(void *pvParameter)
{

    uart_event_t event;
    uint8_t cnt = 0;
    size_t len = 0;
    RS485_SetRX();
    while (1)
    {
        if (xQueueReceive(uart_event_queue, (void*)&event,portMAX_DELAY) == pdTRUE)
        {
            switch (event.type)
            {
                
                case UART_DATA:
                    uart_read_bytes(UART_NUM_1, rx_buffer, event.size, portMAX_DELAY);
                    
                    if(strncmp((char*)rx_buffer,"{gpio:1}",8) == 0)
                    {
                       
                        switch((++cnt) % 3)

                        {
                            case 0:
                                led_strip_set_pixel(led_strip, 0 , 100, 20 ,0);
                                led_strip_refresh(led_strip);
                                break;
                            case 1:
                                led_strip_set_pixel(led_strip, 0 , 0, 100 ,20);
                                led_strip_refresh(led_strip);
                                break;
                            case 2:
                                led_strip_set_pixel(led_strip, 0 , 20, 0 ,100);
                                led_strip_refresh(led_strip);
                                break;
                            default: break;
                        }
                        
                    }
                    ESP_LOGI(TAG,"Received : %.*s",event.size,rx_buffer);
                    memset(rx_buffer,0,sizeof(rx_buffer));
                    break;
                case UART_FRAME_ERR:
                    ESP_LOGE(TAG,"UART_FRAME_ERR");
                    break;
                    default:break;
            }     
        }
    }

}

void app_main(void)
{
    RS485_Init();
    gpio_isr_init();
    rgb_led_init();
    xTaskCreate(uart_event_task, "uart_event_task", 2048 * 4, NULL, 5, NULL);
    while(1)
    
    {
        if(gpio_state == 0)
        {
            gpio_state = 1;
            RS485_Send(UART_NUM_1,(uint8_t*)"{gpio:1}",9);

        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
  
}
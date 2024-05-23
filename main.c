#include <stdio.h>
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "ov2640.h"
#include "pico/time.h"

// modified for ikafly
const int PIN_LED = 29;

const int PIN_CAM_SIOC = 15; // I2C0 SCL
const int PIN_CAM_SIOD = 14; // I2C0 SDA
const int PIN_CAM_RESETB = 2;
const int PIN_CAM_XCLK = 0;
const int PIN_CAM_Y2_PIO_BASE = 1;

//const uint8_t CMD_REG_WRITE = 'w';
//const uint8_t CMD_REG_READ = 'r';
//const uint8_t CMD_CAPTURE = 'c';

//uint8_t image_buf[352*288*2];
uint8_t image_buf[1024*5]; // enough for 160x120 jpeg, 2kByte avg.

int main() {
	stdio_usb_init();
	sleep_ms(1000);
	//	uart_set_baudrate(uart0, 1000000);

//	printf("\n\nBooted!\n");

	gpio_init(PIN_LED);
	gpio_set_dir(PIN_LED, GPIO_OUT);

	struct ov2640_config config;
	config.sccb = i2c1;
	config.pin_sioc = PIN_CAM_SIOC;
	config.pin_siod = PIN_CAM_SIOD;

	config.pin_resetb = PIN_CAM_RESETB;
	config.pin_xclk = PIN_CAM_XCLK;
	config.pin_y2_pio_base = PIN_CAM_Y2_PIO_BASE;

	config.pio = pio0;
	config.pio_sm = 0;

	config.dma_channel = 0;
	config.image_buf = image_buf;
	config.image_buf_size = sizeof(image_buf);

	ov2640_init(&config);
	ov2640_jpeg_init(&config);

//	ov2640_reg_write(&config, 0xff, 0x01);
//	uint8_t midh = ov2640_reg_read(&config, 0x1C);
//	uint8_t midl = ov2640_reg_read(&config, 0x1D);
//	printf("MIDH = 0x%02x, MIDL = 0x%02x\n", midh, midl);

//	uint8_t ch = 0;
	while (true) {
		printf("\n\n");
//		ch = getchar_timeout_us(10*1000);
//		if (ch == 'c') {
//			gpio_put(PIN_LED, !gpio_get(PIN_LED));
			ov2640_capture_frame(&config);
//			fwrite(config.image_buf, 1, config.image_buf_size, stdout);
			int32_t size = ov2640_get_jpeg_size(&config);
//			printf("%d\n", size);
			for (int32_t i = 0; i < size; i++)
				printf("%02x", config.image_buf[i]);
			// separator between image(64 running '0')
			for (int16_t i = 0; i < 64; i++) 
				printf("00");

//			busy_wait_ms(50);
//		} else {
//			busy_wait_ms(10);
//			continue;
//		}
		sleep_ms(1500);

	}

	return 0;
}

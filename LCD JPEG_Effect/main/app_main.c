
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <esp_log.h>

#include "pretty_effect.h"
const static char *TAG = "LCD JPEG Effect Demo";

#define LCD_HOST		VSPI_HOST
#define DMA_CHAN		1

#define PIN_NUM_MISO	12
#define PIN_NUM_MOSI	13
#define PIN_NUM_CLK		14
#define PIN_NUM_CS		15

#define PIN_NUM_DC		2
#define PIN_NUM_RST		-1
#define PIN_NUM_BCKL	-1

//为了加快SPI传输速度，每次SPI会通过DMA传输很多行一起显示，这里定义一次传输多少行，数字越大，占用内存越多，但传输越快，
//值要可以被240整除，16行，传15次一屏
#define PARALLEL_LINES 16

// LCD需要初始化一堆命令/参数值，存储在这个结构，看lcd_init函数中while段就明白了
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //数据中没有数据；最高位置1即 bit7 最高位为1会延时后再发送下一组，0xFF是结尾用于循环退出
} lcd_init_cmd_t;

// 液晶初始化参数查PDF文档修改
DRAM_ATTR static const lcd_init_cmd_t ili_init_cmds[]={
    /* Power contorl B, power control = 0, DC_ENA = 1 */
    {0xCF, {0x00, 0x83, 0X30}, 3},
    /* Power on sequence control,
     * cp1 keeps 1 frame, 1st frame enable
     * vcl = 0, ddvdh=3, vgh=1, vgl=2
     * DDVDH_ENH=1
     */
    {0xED, {0x64, 0x03, 0X12, 0X81}, 4},
    /* Driver timing control A,
     * non-overlap=default +1
     * EQ=default - 1, CR=default
     * pre-charge=default - 1
     */
    {0xE8, {0x85, 0x01, 0x79}, 3},
    /* Power control A, Vcore=1.6V, DDVDH=5.6V */
    {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
    /* Pump ratio control, DDVDH=2xVCl */
    {0xF7, {0x20}, 1},
    /* Driver timing control, all=0 unit */
    {0xEA, {0x00, 0x00}, 2},
    /* Power control 1, GVDD=4.75V */
    {0xC0, {0x26}, 1},
    /* Power control 2, DDVDH=VCl*2, VGH=VCl*7, VGL=-VCl*3 */
    {0xC1, {0x11}, 1},
    /* VCOM control 1, VCOMH=4.025V, VCOML=-0.950V */
    {0xC5, {0x35, 0x3E}, 2},
    /* VCOM control 2, VCOMH=VMH-2, VCOML=VML-2 */
    {0xC7, {0xBE}, 1},
    /* Memory access contorl, MX=MY=0, MV=1, ML=0, BGR=1, MH=0 */
    {0x36, {0x28}, 1},
    /* Pixel format, 16bits/pixel for RGB/MCU interface */
    {0x3A, {0x55}, 1},
    /* Frame rate control, f=fosc, 70Hz fps */
    {0xB1, {0x00, 0x1B}, 2},
    /* Enable 3G, disabled */
    {0xF2, {0x08}, 1},
    /* Gamma set, curve 1 */
    {0x26, {0x01}, 1},
    /* Positive gamma correction */
    {0xE0, {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00}, 15},
    /* Negative gamma correction */
    {0XE1, {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F}, 15},
    /* Column address set, SC=0, EC=0xEF */
    {0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
    /* Page address set, SP=0, EP=0x013F */
    {0x2B, {0x00, 0x00, 0x01, 0x3f}, 4},
    /* Memory write */
    {0x2C, {0}, 0},
    /* Entry mode set, Low vol detect disabled, normal display */
    {0xB7, {0x07}, 1},
    /* Display function control */
    {0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
    /* Sleep out */
    {0x11, {0}, 0x80},
    /* Display on */
    {0x29, {0}, 0x80},
    {0, {0}, 0xff},
};

// 发送命令到LCD，使用轮询方式阻塞等待传输完成。
// 由于数据传输量很少，因此在轮询方式处理可提高速度。使用中断方式的开销要超过轮询方式。
void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd)
{
	esp_err_t ret;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));		// 清空结构体
	t.length=8;						// 要传输的位数 一个字节 8位
	t.tx_buffer=&cmd;				// 将命令填充进去
	t.user=(void*)0;				// 设置D/C 线，在SPI传输前回调中根据此值处理DC信号线
	ret=spi_device_polling_transmit(spi, &t);		// 开始传输
	assert(ret==ESP_OK);			// 一般不会有问题
}

// 发送数据到LCD，使用轮询方式阻塞等待传输完成。
// 由于数据传输量很少，因此在轮询方式处理可提高速度。使用中断方式的开销要超过轮询方式。
void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
	esp_err_t ret;
	spi_transaction_t t;
	if (len==0) return;				// 长度为0 没有数据要传输
	memset(&t, 0, sizeof(t));		// 清空结构体
	t.length=len*8;					// 要写入的数据长度 Len 是字节数，len, transaction length is in bits.
	t.tx_buffer=data;				// 数据指针
	t.user=(void*)1;				// 设置D/C 线，在SPI传输前回调中根据此值处理DC信号线
	ret=spi_device_polling_transmit(spi, &t);		// 开始传输
	assert(ret==ESP_OK);			// 一般不会有问题
}

// 此函数在SPI传输开始之前被调用（在irq上下文中！），通过用户字段的值来设置D/C信号线
void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
	int dc = (int)t->user;
	gpio_set_level(PIN_NUM_DC, dc);
}

// 未实现
uint32_t lcd_get_id(spi_device_handle_t spi)
{
	// 获取屏幕驱动芯片ID指令 0x04
	lcd_cmd(spi, 0x04);
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.length=8*3;
	t.flags = SPI_TRANS_USE_RXDATA;
	t.user = (void*)1;
	esp_err_t ret = spi_device_polling_transmit(spi, &t);
	assert( ret == ESP_OK );
	return *(uint32_t*)t.rx_data;
}

// 初始化SPI液晶屏
// SPI 3.2 240*320 ST7789V / ILI9341
void lcd_init(spi_device_handle_t spi)
{
	int cmd=0;
	const lcd_init_cmd_t* lcd_init_cmds;

	// 初始化其它控制引脚
	gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);		// 设置D/C 线(cmd/data)
	//gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);	// 开发板与主控rst接在一起，不用控制
	//gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);	// 与VCC接在一起，不用控制

	// Reset the display
	//gpio_set_level(PIN_NUM_RST, 0);
	//vTaskDelay(100 / portTICK_RATE_MS);
	//gpio_set_level(PIN_NUM_RST, 1);
	//vTaskDelay(100 / portTICK_RATE_MS);

	uint32_t lcd_id = lcd_get_id(spi);

	printf("LCD ID: %08X\n", lcd_id);

	printf("LCD ILI9341 initialization.\n");
	lcd_init_cmds = ili_init_cmds;

	// 循环发送设置所有寄存器
	while (lcd_init_cmds[cmd].databytes!=0xff) {
		lcd_cmd(spi, lcd_init_cmds[cmd].cmd);
		lcd_data(spi, lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].databytes&0x1F);
		if (lcd_init_cmds[cmd].databytes&0x80) {
			vTaskDelay(100 / portTICK_RATE_MS);
		}
		cmd++;
	}

	// gpio_set_level(PIN_NUM_BCKL, 0);	// 点亮LCD屏
}


// 发送一行数据到液晶
static void send_lines(spi_device_handle_t spi, int ypos, uint16_t *linedata)
{
	esp_err_t ret;
	int x;
	// 传输描述符，声明为静态，因此它不是分配在堆栈上；因为在计算下一行的时候，SPI驱动程序还会访问它
	static spi_transaction_t trans[6];
	// 初始化传输
	for (x=0; x<6; x++) {
		memset(&trans[x], 0, sizeof(spi_transaction_t));
		if ((x&1)==0) {
			//Even transfers are commands
			trans[x].length=8;
			trans[x].user=(void*)0;
		} else {
			//Odd transfers are data
			trans[x].length=8*4;
			trans[x].user=(void*)1;
		}
		trans[x].flags=SPI_TRANS_USE_TXDATA;
	}
	trans[0].tx_data[0]=0x2A;           //Column Address Set
	trans[1].tx_data[0]=0;              //Start Col High
	trans[1].tx_data[1]=0;              //Start Col Low
	trans[1].tx_data[2]=(320)>>8;       //End Col High
	trans[1].tx_data[3]=(320)&0xff;     //End Col Low
	trans[2].tx_data[0]=0x2B;           //Page address set
	trans[3].tx_data[0]=ypos>>8;        //Start page high
	trans[3].tx_data[1]=ypos&0xff;      //start page low
	trans[3].tx_data[2]=(ypos+PARALLEL_LINES)>>8;    //end page high
	trans[3].tx_data[3]=(ypos+PARALLEL_LINES)&0xff;  //end page low
	trans[4].tx_data[0]=0x2C;           //memory write
	trans[5].tx_buffer=linedata;        //finally send the line data
	trans[5].length=320*2*8*PARALLEL_LINES;          //Data length, in bits
	trans[5].flags=0; //undo SPI_TRANS_USE_TXDATA flag

	// 队列传输所有
	for (x=0; x<6; x++) {
		ret = spi_device_queue_trans(spi, &trans[x], portMAX_DELAY);
		assert(ret==ESP_OK);
	}
}

// 等待传输完成
static void send_line_finish(spi_device_handle_t spi)
{
	spi_transaction_t *rtrans;
	esp_err_t ret;
	//Wait for all 6 transactions to be done and get back the results.
	for (int x=0; x<6; x++) {
		ret=spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
		assert(ret==ESP_OK);
		//We could inspect rtrans now if we received any info back. The LCD is treated as write-only, though.
	}
}


// 计算特效后发送到液晶（死循环）
static void CalcShowEffect(spi_device_handle_t spi)
{
	uint16_t *lines[2];
	// 为像素缓冲区分配内存
	for (int i=0; i<2; i++) {
		lines[i] = heap_caps_malloc(320*PARALLEL_LINES*sizeof(uint16_t), MALLOC_CAP_DMA);
		assert(lines[i]!=NULL);
	}
	int frame = 0;			// 计算特效行的帧号
	int sending_line=-1;	// 当前发送到LCD的行
	int calc_line=0;		// 正在计算的行的索引

	while(1) {
		frame++;// 每次发到屏幕的是16*320个像素，发送15次为一帧
		for (int y=0; y<240; y+=PARALLEL_LINES) {//16
			// 计算一行特效
			CalcLineEffect(lines[calc_line], y, frame, PARALLEL_LINES);
			// 等待上一次传输完成，
			if (sending_line!=-1){
				send_line_finish(spi);
			}
			// 交换计算的行和正在发送的行在缓存中的数组下标
			sending_line = calc_line;
			calc_line = (calc_line==1)?0:1;
			// 发送计算好特效的行
			// 注意：此行将加入队列马上发送，发送是后台运行的，可以去计算下一行的特效，期间不要修改/处理line[sending_line]; 因为SPI发送处理要一直读取它
			send_lines(spi, y, lines[sending_line]);
		}
	}
}

void app_main(void)
{
	esp_err_t ret;
	spi_device_handle_t spi;
	// SPI总线配置
	spi_bus_config_t buscfg = {
		.miso_io_num = PIN_NUM_MISO,
		.mosi_io_num = PIN_NUM_MOSI,
		.sclk_io_num = PIN_NUM_CLK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz=PARALLEL_LINES*320*2+8
	};
	// SPI驱动接口配置
	spi_device_interface_config_t devcfg = {
		.clock_speed_hz = 30*1000*1000,				// SPI时钟 30 MHz
		.mode = 0,									// SPI模式0
		.spics_io_num = PIN_NUM_CS,					// CS片选信号引脚
		.queue_size = 7,							// 事务队列大小 7个
		.pre_cb = lcd_spi_pre_transfer_callback,	// 数据传输前回调，用作D/C（数据命令）线分别处理
	};
	// 初始化SPI总线
	ret = spi_bus_initialize(LCD_HOST, &buscfg, DMA_CHAN);
	ESP_ERROR_CHECK(ret);
	// 添加SPI总线驱动
	ret=spi_bus_add_device(LCD_HOST, &devcfg, &spi);
	ESP_ERROR_CHECK(ret);
	// 初始化LCD
	lcd_init(spi);

	// 解码JPG成16位RGB到内存中
	ret = JPEG_Effect_init();
	ESP_ERROR_CHECK(ret);
	// 开始将解码到内存中的图片数据计算特效后显示在LCD上
	CalcShowEffect(spi);
}

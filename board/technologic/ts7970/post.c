/*
 * Copyright (C) 2016 Technologic Systems
 *
 * Author: Mark Featherston <mark@embeddedarm.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>

#include <asm/arch-mx6/sys_proto.h>
#include <asm/arch-imx/cpu.h>
#include <asm/arch/mx6-pins.h>
#include <asm/gpio.h>
#include <asm/imx-common/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <cli.h>
#include <command.h>
#include <i2c.h>
#include <status_led.h>
#include <usb.h>

#include <miiphy.h>

#include "post.h"

#define LOOP_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

iomux_v3_cfg_t const custom1_led_pads[] = {
	MX6_PAD_GPIO_2__GPIO1_IO02 | MUX_PAD_CTRL(NO_PAD_CTRL), // YEL_LED#
	MX6_PAD_GPIO_9__GPIO1_IO09 | MUX_PAD_CTRL(NO_PAD_CTRL), // EN_LED2#
	MX6_PAD_EIM_D27__GPIO3_IO27 | MUX_PAD_CTRL(NO_PAD_CTRL), // EN_LED3#
	MX6_PAD_DISP0_DAT7__GPIO4_IO28 | MUX_PAD_CTRL(NO_PAD_CTRL), // EN_LED4#
	MX6_PAD_EIM_D23__GPIO3_IO23 | MUX_PAD_CTRL(NO_PAD_CTRL), // EN_LED5#
	MX6_PAD_DISP0_DAT10__GPIO4_IO31 | MUX_PAD_CTRL(NO_PAD_CTRL), // EN_LED6#
};

iomux_v3_cfg_t const posttest_pads[] = {
	MX6_PAD_SD4_DAT6__GPIO2_IO14 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // UART2_CTS
	MX6_PAD_SD4_DAT5__GPIO2_IO13 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // UART2_RTS
};

int do_mmcops(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]);
int do_mem_mtest(cmd_tbl_t *cmdtp, int flag, int argc,
			char * const argv[]);
void do_usb_start(void);

int fpga_test(void)
{
	int ret = 0;
	int i;
	uint8_t val;

	/* FPGA should be loaded before running this test.  
	 * The TS-7970 has no model register, so we can only verify
	 * some sane values in existing registers
	 */
	ret |= i2c_read(0x28, 51, 2, &val, 1);
	if(val == 0) {
		printf("Read back 0 for addr 51.  FPGA not sane?\n");
		ret = 1;
	}

	/* DIO header should always default to 0x7c for GPIO 
	 * on the crossbar
	 */
	for(i = 19; i < 25; i++) {
		ret |= i2c_read(0x28, i, 2, &val, 1);
		if((val & 0x7c) != 0x7c) {
			printf("Read back 0x%X from addr %d.  Should be 0x7c\n", (val & 0x7c), i);
			ret = 1;
		}
	}

	if (ret == 0) printf("FPGA test passed\n");
	else printf("FPGA test failed\n");
	return ret;
}

int marvell_phy_test(void)
{
	int ret = 0;
	unsigned int oui;
	unsigned char model;
	unsigned char rev;

	if (miiphy_info ("FEC", 0x1, &oui, &model, &rev) != 0) {
		printf("Failed to find PHY\n");
		return 1;
	}

	if(oui != 0x5043) {
		printf("Wrong PHY?  Bad OUI 0x%X 0x5043\n", oui);
		ret |= 1;
	}

	if(model != 0x1d) {
		printf("Wrong PHY?  Bad model 0x%X not 0x1d\n", oui);
		ret |= 1;
	}

	if (ret == 0) printf("PHY test passed\n");
	else printf("PHY test failed\n");
	return ret;
}

int micrel_phy_test(void)
{
	int ret = 0;
	unsigned int oui;
	unsigned char model;
	unsigned char rev;

	if (miiphy_info ("FEC", 0x7, &oui, &model, &rev) != 0) {
		printf("Failed to find PHY\n");
		return 1;
	}

	if(oui != 0x0885) {
		printf("Wrong PHY?  Bad OUI 0x%X 0x0885\n", oui);
		ret |= 1;
	}

	if(model != 0x22) {
		printf("Wrong PHY?  Bad model 0x%X not 0x22\n", oui);
		ret |= 1;
	}

	if (ret == 0) printf("PHY test passed\n");
	else printf("PHY test failed\n");
	return ret;
}

int emmc_test(void)
{
	int ret = 0, i;
	uint32_t *loadaddr = (uint32_t *)0x12000000;

	char *query_argv[3] = { "mmc", "dev", "1" };
	char *write_argv[5] = { "mmc", "write", "0x12000000", "0x0", "0x800" };
	char *read_argv[5] = { "mmc", "read", "0x12000000", "0x0", "0x800" };

	/* This tests simple enumeration */
	ret |= do_mmcops(0, 0, 3, query_argv);

	if(!getenv("post_nowrite")) {
		memset(loadaddr, 0xAAAAAAAA, 1024*1024*4);
		ret |= do_mmcops(0, 0, 5, write_argv);
		memset(loadaddr, 0x00000000, 1024*1024*4);
		ret |= do_mmcops(0, 0, 5, read_argv);

		for (i = 0; i < (1024*1024)/4; i++)
		{
			if (loadaddr[i] != 0xAAAAAAAA)
			{
				printf("\tWrong value at %d\n", i);
				printf("got 0x%X, expected 0xAAAAAAAA\n", loadaddr[i]);
				ret = 1;
			}
		}

		memset(loadaddr, 0x55555555, 1024*1024*4);
		ret |= do_mmcops(0, 0, 5, write_argv);
		memset(loadaddr, 0x00000000, 1024*1024*4);
		ret |= do_mmcops(0, 0, 5, read_argv);

		for (i = 0; i < (1024*1024)/4; i++)
		{
			if (loadaddr[i] != 0x55555555)
			{
				printf("\tWrong value at %d\n", i);
				printf("got 0x%X, expected 0x55555555\n", loadaddr[i]);
				ret = 1;
			}
		}
	}

	if (ret == 0) printf("eMMC test passed\n");
	else printf("eMMC test failed\n");
	return ret;
}

int wifi_test(void)
{
	static int ret = 0;
	static bool initialized = 0;

	/* This test only works once per POR, so cache the result */
	if(!initialized) {
		uint8_t val;
		initialized = 1;

		/* Wifi is very complex and implementing a real test in u-boot is probably 
		 * not wise but the bluetooth on the same chip shows sign of life by going low
		 * after the chip is enabled. */
		gpio_direction_input(IMX_GPIO_NR(2, 13)); // RTS
		if(gpio_get_value(IMX_GPIO_NR(2, 13)) != 1)
			ret = 1;

		val = 3;
		ret |= i2c_write(0x28, 13, 2, &val, 1);
		mdelay(500);
		if(gpio_get_value(IMX_GPIO_NR(2, 13)) != 0)
			ret = 1;
	}

	if (ret == 0) printf("WIFI test passed\n");
	else printf("WIFI test failed\n");
	return ret;
}

/* Check for m41t000 rtc */
int rtc_test(void)
{
	int ret;

	ret = i2c_probe(0x68);

	if (ret == 0) printf("RTC test passed\n");
	else printf("RTC test failed\n");
	return ret;
}

int mem_test(void)
{
	int ret = 0;
	int argc = 5;
	/* Arguments are the memory addresses, pattern (not relevant in alt test) and
	 * 20 which is the number of iterations (in hex) which takes about 1 second
	 * on the i.MX6 quad */
	char *argv[5] = { "mtest", "0x10000000", "0x10010000", "1", "20" };

	ret |= do_mem_mtest(0, 0, argc, argv);

	if (ret == 0) printf("RAM test passed\n");
	else printf("RAM test failed\n");
	return ret;
}

int usbhub_test(void)
{
	int i;
	struct usb_device *dev = NULL;

	do_usb_start();

	for (i = 0; i < USB_MAX_DEVICE; i++) {
		dev = usb_get_dev_index(i);
		if (dev == NULL)
			break;

		if(dev->descriptor.idVendor == 0x424 &&
		   dev->descriptor.idProduct == 0x2514) {
		   	printf("USB test passed\n");
			return 0;
		}
	}

	printf("Did not find SMSC USB hub!\n");
	return 1;
}

int is_quad(void)
{
#ifdef CONFIG_MX6Q
	return 1;
#else
	return 0;
#endif
}

// Scale voltage to silabs 0-2.5V
uint16_t sscale(uint16_t data){
	return data * (2.5/1023) * 1000;
}

// Scale voltage for resistor dividers
uint16_t rscale(uint16_t data, uint16_t r1, uint16_t r2)
{
	uint16_t ret = (data * (r1 + r2)/r2);
	return sscale(ret);
}

int silabs_test(void)
{
	uint8_t tmp[32];
	int ret;

	ret = i2c_read(0x10, 0, 0, tmp, 32);

	if (ret == 0) printf("Silabs test passed\n");
	else printf("Silabs test failed\n");
	return ret;
}

void leds_test(void)
{
	int i;
	red_led_on();
	green_led_on();
	blue_led_on();
	yellow_led_on();

	for(i = 0; i < 24; i++){
		if(i % 4 == 0) red_led_on();
		else red_led_off();
		if(i % 4 == 1) yellow_led_on();
		else yellow_led_off();
		if(i % 4 == 2) green_led_on();
		else green_led_off();
		if(i % 4 == 3) blue_led_on();
		else blue_led_off();
		mdelay(100);
	}

	red_led_on();
	green_led_on();
	blue_led_on();
	yellow_led_on();
}

#define EIM_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_SPEED_MED  | PAD_CTL_DSE_240ohm)

iomux_v3_cfg_t const eim_pads[] = {
	MX6_PAD_CSI0_DAT12__EIM_DATA08 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_CSI0_DAT13__EIM_DATA09 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_CSI0_DAT14__EIM_DATA10 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_CSI0_DAT15__EIM_DATA11 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_CSI0_DAT16__EIM_DATA12 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_CSI0_DAT17__EIM_DATA13 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_CSI0_DAT18__EIM_DATA14 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_CSI0_DAT19__EIM_DATA15 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_EIM_DA0__EIM_AD00 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_EIM_DA1__EIM_AD01 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_EIM_DA2__EIM_AD02 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_EIM_DA3__EIM_AD03 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_EIM_DA4__EIM_AD04 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_EIM_DA5__EIM_AD05 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_EIM_DA6__EIM_AD06 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_EIM_DA7__EIM_AD07 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_EIM_DA8__EIM_AD08 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_EIM_DA9__EIM_AD09 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_EIM_DA10__EIM_AD10 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_EIM_DA11__EIM_AD11 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_EIM_DA12__EIM_AD12 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_EIM_DA13__EIM_AD13 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_EIM_DA14__EIM_AD14 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_EIM_DA15__EIM_AD15 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_EIM_A16__EIM_ADDR16 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_EIM_OE__EIM_OE_B | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_EIM_RW__EIM_RW | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_EIM_CS0__EIM_CS0_B | MUX_PAD_CTRL(EIM_PAD_CTRL),
};

#define EIM_CS0 0x08000000
#define MRAM_SIZE (1 << 17)

static int mram_test(void)
{
	int ret = 0;
	uint8_t wbuf[MRAM_SIZE];
	uint8_t rbuf;
	uint8_t *base = (uint8_t *)EIM_CS0;
	int i;
	struct weim *weim_regs = (struct weim *)0x21B8000;

	/* ungate clock */
	writel(readl(0x020c4080) | 0xC00, 0x020c4080);

	imx_iomux_v3_setup_multiple_pads(eim_pads, ARRAY_SIZE(eim_pads));

	writel(0x00150031, &weim_regs->cs0gcr1);
	writel(0x00000000, &weim_regs->cs0gcr2);
	writel(0x08000000, &weim_regs->cs0rcr1);
	writel(0x00000000, &weim_regs->cs0rcr2);
	writel(0x08000000, &weim_regs->cs0wcr1);
	writel(0x00000000, &weim_regs->cs0wcr2);
	
	set_chipselect_size(CS0_128);

	srand(0);
	for (i = 0; i < MRAM_SIZE; i++) {
		wbuf[i] = (uint8_t)rand();
		base[i] = wbuf[i];
	}
 
	for (i = 0; i < MRAM_SIZE; i++) {
		rbuf = base[i];
		if(wbuf[i] != rbuf){
			fprintf(stderr, "At offset %d, expected 0x%X, got 0x%X\n", i, wbuf[i], rbuf);
			ret = 1;
			break;
		}
	}

	if (ret == 0) printf("MRAM test passed\n");
	else printf("MRAM test failed\n");

	return ret;
}
/* peekpoke 32 0x020C401C 0x900800
*/

static int do_post_test(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int ret = 0;

	leds_test();

	ret |= mram_test();
	ret |= marvell_phy_test();
	ret |= emmc_test();
	ret |= mem_test();
	ret |= silabs_test();
	ret |= rtc_test();
	ret |= usbhub_test();

	if (ret == 0) printf("All POST tests passed\n");
	else printf("One or more POST tests failed\n");
	return ret;
}

U_BOOT_CMD(post, 1, 1,	do_post_test,
	"Runs a POST test",
	""
);

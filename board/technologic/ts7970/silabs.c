/*
 * Copyright (C) 2016 Technologic Systems
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <stdlib.h>
#include <asm/io.h>
#include <asm/arch/gpio.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <status_led.h>
#include <i2c.h>


#define TS7970_POWER_FAIL	IMX_GPIO_NR(1, 0)

// Scale voltage to silabs 0-2.5V
uint16_t inline sscale(uint16_t data){
	return data * (2.5/1023) * 1000;
}

// Scale voltage for resistor dividers
uint16_t inline rscale(uint16_t data, uint16_t r1, uint16_t r2)
{
	uint16_t ret = (data * (r1 + r2)/r2);
	return sscale(ret);
}

void enable_supercaps(void)
{
	uint8_t val = 0x1;

	gpio_direction_input(TS7970_POWER_FAIL);

	/* Tell supercaps to charge */
	i2c_write(0x10, 0, 0, &val, 1);
}

void disable_supercaps(void)
{
	uint8_t val = 0x0;
	i2c_write(0x10, 0, 0, &val, 1);
}

/* Not optional on this board */
int tssilo_is_detected(void)
{
	return true;
}

void read_adcs(uint16_t *data)
{
	uint8_t tmp[32];
	memset(tmp, 0, 32);
	int i, ret;

	ret = i2c_read(0x10, 0, 0, tmp, 32);

	if(ret){
		printf("I2C Read failed with %d\n", ret);
		return;
	}
    for (i = 0; i < 15; i++){
    	data[i] = (tmp[i*2] << 8) | tmp[(i*2)+1];
    }
}

void board_sleep(int deciseconds)
{
	uint8_t dat[4] = {0};
	uint8_t opt_resetswitchwkup = 1;
	uint8_t opt_sleepmode = 1;

	dat[0]=(0x1 | (opt_resetswitchwkup << 1) |
	  ((opt_sleepmode-1) << 4) | 1 << 6);
	dat[3] = (deciseconds & 0xff);
	dat[2] = ((deciseconds >> 8) & 0xff);
	dat[1] = ((deciseconds >> 16) & 0xff);
	i2c_write(0x10, 0, 0, dat, 4);
}

void block_charge(int blkpct)
{
	int i = 0;
	int chgpct = 0;
	int pfailed = 0;

	enable_supercaps();

	while(chgpct < blkpct) {
		int chgmv;
		uint16_t data[16];
		read_adcs(data);

		if(ctrlc())
			return;

		if(gpio_get_value(TS7970_POWER_FAIL) != 0) {
			if(pfailed == 0) {
				pfailed = 1;
				puts("Cannot boot with POWER_FAIL asserted\n");
			}
			if(i % 250) {
				printf("VIN: %dmV, Supercaps: %dmV\r",
					rscale(data[4], 2870, 147), 
					rscale(data[7], 200, 147));
			}
		} else {
			pfailed = 0;
			chgmv = rscale(data[7], 200, 147);
			chgpct = chgmv*100/5000;
			if(i % 250)
				printf("Charging to %d%%... %03d/100%% (%dmV)\r",
					blkpct,
					chgpct, 
					chgmv);
		}

		i++;
		udelay(1000);
	}
	puts("\n");
	puts("Fully charged caps\n");
}

static int do_microctl(cmd_tbl_t *cmdtp, int flag, 
	int argc, char * const argv[])
{
	int i;

	for (i = 1; i < argc; i++)
	{
		int micros;
		int pct;
		char *p;
		uint16_t data[16];
		if(argv[i][0] == '-')
			p = &argv[i][1];
		else
			p = &argv[i][0];

		switch(p[0]) {
			case 'o':
				disable_supercaps();
				break;
			case 's':
				if(i+1 == argc) {
					printf("Missing option for microseconds to sleep\n");
					return 1;
				}
				micros = simple_strtoul(argv[++i], NULL, 10);
				printf("Sleep for %d seconds\n", micros);
				board_sleep(micros);
				break;
			case 'i':
				read_adcs(data);

				printf("VDD_HIGH_CAP=%d\n", sscale(data[0]));
				printf("SILAB_P10=0x%X\n", data[1]);
				printf("SILAB_P11=0x%X\n", data[2]);
				printf("SILAB_P12=0x%X\n", data[3]);
				printf("VIN=%d\n", rscale(data[4], 2870, 147));
				printf("V5P3_A=%d\n", rscale(data[5], 200, 147));
				printf("V3P1=%d\n", rscale(data[6], 499, 499));
				printf("AN_SUP_CHRG=%d\n", rscale(data[7], 200, 147));
				printf("V1P8=%d\n", sscale(data[8]));
				printf("AN_SUP_CAP_2=%d\n", sscale(data[9]));
				printf("RAM_VREF=%d\n", sscale(data[10]));
				printf("AN_SUP_CAP_1=%d\n", sscale(data[11]));
				break;
			case 'b':
				if(i+1 == argc) {
					printf("Missing option for percentage of charge to wait for\n");
					return 1;
				}
				pct = simple_strtoul(argv[++i], NULL, 10);
				block_charge(pct);
				break;
			case 'e':
				enable_supercaps();
				break;
			default:
				printf("Unknown option '%s'\n", argv[i]);
				return 1;
		}
	}

	return 0;
}

U_BOOT_CMD(tsmicroctl, 3, 1, do_microctl,
	"TS supervisory microcontroller access",
	"  Usage: tsmicroctl [seconds]\n"
	"  If the seconds argument is supplied the board will sleep.\n"
	"  if not specified, it will print out the ADC values\n"
);

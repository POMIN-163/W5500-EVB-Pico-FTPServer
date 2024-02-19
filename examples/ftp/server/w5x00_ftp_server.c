/**
 * Copyright (c) 2021 WIZnet Co.,Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * ----------------------------------------------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------------------------------------------
 */
#include <stdio.h>

#include "port_common.h"

#include "wizchip_conf.h"
#include "w5x00_spi.h"

#include "hardware/uart.h"
#include "hardware/pwm.h"

#include "tf_card.h"
#include "fatfs/ff.h"
#include "ftpd.h"
#include <string.h>
#include "lcd.h"
#include "pic.h"
/**
 * ----------------------------------------------------------------------------------------------------
 * Macros
 * ----------------------------------------------------------------------------------------------------
 */
/* Clock */
#define PLL_SYS_KHZ (133 * 1000)

/* Buffer */
#define ETHERNET_BUF_MAX_SIZE (1024 * 5)

/**
 * ----------------------------------------------------------------------------------------------------
 * Variables
 * ----------------------------------------------------------------------------------------------------
 */
/* Network */
static wiz_NetInfo g_net_info =
    {
        .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56}, // MAC address
        .ip = {192, 168, 0, 188},                     // IP address
        .sn = {255, 255, 255, 0},                    // Subnet Mask
        .gw = {192, 168, 0, 1},                     // Gateway
        .dns = {114, 114, 114, 114},                         // DNS server
        .dhcp = NETINFO_STATIC                       // DHCP enable/disable
};

/* FTP */
static uint8_t g_ftp_buf[ETHERNET_BUF_MAX_SIZE] = {
    0,
};

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */
/* Clock */
static void set_clock_khz(void);
FATFS fs;
FIL fil;
FRESULT fr;     /* FatFs return code */
UINT br;
UINT bw;

float s;
uint32_t t;
uint32_t maxLatency;
uint32_t minLatency;
uint32_t totalLatency;
bool skipLatency;
FRESULT scan_file (
    char *path,        /* Start node to be scanned (***also used as work area***) */
    char *files,
    int *num
)
{
    FRESULT res;
    DIR dir;
    UINT i;
    static FILINFO fno;
    *num = 0;
    res = f_opendir(&dir, path);                       /* Open the directory */
    if (res == FR_OK) {
        for (;;) {
            res = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            i = strlen(path);
            char size[32];
            char ftime[32];
            sprintf(size, "%ld", (fno.fattrib & AM_DIR) ? 0 : fno.fsize);
			files += sprintf(files, "%crwxr-xr-x 1 pomin pomin %s Jan 19 2024 %s\r\n", (fno.fattrib & AM_DIR) ? 'd' : '-', size, fno.fname);
            *num = (*num) + 1;
        }
        f_closedir(&dir);
    }

    return res;
}

void fatfs_init(void)
{
    fr = f_mount(&fs, "/", 1);
    if (fr != FR_OK) {
        printf("mount error %d\n", fr);
    }
    printf("mount ok\n");

    switch (fs.fs_type) {
        case FS_FAT12:
            printf("Type is FAT12\n");
            break;
        case FS_FAT16:
            printf("Type is FAT16\n");
            break;
        case FS_FAT32:
            printf("Type is FAT32\n");
            break;
        case FS_EXFAT:
            printf("Type is EXFAT\n");
            break;
        default:
            printf("Type is unknown\n");
            break;
    }
    printf("Card size: %7.2f GB (GB = 1E9 bytes)\n\n", fs.csize * fs.n_fatent * 512E-9);
    char buf[1024] = "/";
    int num;
    scan_file("/", buf, &num);
    printf("%s", buf);
}


/**
 * ----------------------------------------------------------------------------------------------------
 * Main
 * ----------------------------------------------------------------------------------------------------
 */
int main()
{
    /* Initialize */
    uint8_t retval = 0;
    u8 rotation = 3;

    set_clock_khz();

    stdio_init_all();
    // Initialise UART 0
    uart_init(uart0, 115200);
    // Set the GPIO pin mux to the UART - 0 is TX, 1 is RX
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);

    // LED
    const uint32_t LED_PIN = 25;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // BackLight PWM (125MHz / 65536 / 4 = 476.84 Hz)
    gpio_set_function(PIN_LCD_BLK, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(PICO_DEFAULT_LED_PIN);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.f);
    pwm_init(slice_num, &config, true);
    int bl_val = 196;
    // Square bl_val to make brightness appear more linear
    pwm_set_gpio_level(PIN_LCD_BLK, bl_val * bl_val);

    wizchip_spi_initialize();
    wizchip_cris_initialize();

    wizchip_reset();
    wizchip_initialize();
    wizchip_check();

    network_initialize(g_net_info);
    LCD_Init();
    LCD_SetRotation(rotation);
    LCD_ShowPicture(0, 0, 320, 240, (u8 *)gImage_pic);
    fatfs_init();

    ftpd_init(g_net_info.ip);

    /* Get network information */
    print_network_information(g_net_info);

    /* Infinite loop */
    while (1)
    {
        /* Run FTP server */
        if ((retval = ftpd_run(g_ftp_buf)) < 0)
        {
            printf(" FTP server error : %d\n", retval);

            while (1)
                ;
        }
    }
}

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */
/* Clock */
static void set_clock_khz(void)
{
    // set a system clock frequency in khz
    set_sys_clock_khz(PLL_SYS_KHZ, true);

    // configure the specified clock
    clock_configure(
        clk_peri,
        0,                                                // No glitchless mux
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, // System PLL on AUX mux
        PLL_SYS_KHZ * 1000,                               // Input frequency
        PLL_SYS_KHZ * 1000                                // Output (must be same as no divider)
    );
}

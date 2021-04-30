#ifndef __LINUX_SPI_GPT_SPI_H
#define __LINUX_SPI_GPT_SPI_H

/**
 * struct gspi_platform_data - Platform data of the GPT SPI driver
 * @num_chipselect:	Number of chip select by the IP.
 * @little_endian:	If registers should be accessed little endian or not.
 * @bits_per_word:	Number of bits per word.
 * @devices:		Devices to add when the driver is probed.
 * @num_devices:	Number of devices in the devices array.
 */
struct gspi_platform_data {
	u16 num_chipselect;
	u8 bits_per_word;
	struct spi_board_info *devices;
	u8 num_devices;
};

extern void gpt_spi_set_cs(struct spi_device *spi,bool enable);
extern int gpt_spi_setup(struct spi_device*spi);
#endif /* __LINUX_SPI_GPT_SPI_H */

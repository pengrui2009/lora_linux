#ifndef __SX1278_HAL_H__
#define __SX1278_HAL_H__
#include <stdint.h>
#include <linux/mutex.h>

struct spidev_data 
{
	spinlock_t			spi_lock;
	struct spi_device	*spi;
	/* the point of the data */
	uint8_t				*buffer_ptr;
	/* the length of the data */
	size_t 				buffer_len;
};

#endif /*__SX1278_HAL_H__*/
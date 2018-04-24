#include <linux/completion.h>

#include "sx1278_hal.h"

static struct spidev_data spidev;
/*
 * We can't use the standard synchronous wrappers for file I/O; we
 * need to protect against async removal of the underlying spi_device.
 */
static void spidev_complete(void *arg)
{
	complete(arg);
}

static ssize_t
spidev_sync(struct spidev_data *spidev, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	message->complete = spidev_complete;
	message->context = &done;

	spin_lock_irq(&spidev->spi_lock);
	if (spidev->spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_async(spidev->spi, message);
	spin_unlock_irq(&spidev->spi_lock);

	if (status == 0) {
		wait_for_completion(&done);
		status = message->status;
		if (status == 0)
			status = message->actual_length;
	}
	return status;
}

static inline ssize_t
spidev_sync_write(struct spidev_data *spidev)
{
	struct spi_transfer	t = {
			.tx_buf		= spidev->buffer_ptr,
			.len		= spidev->buffer_len,
		};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spidev_sync(spidev, &m);
}

static inline ssize_t
spidev_sync_read(struct spidev_data *spidev)
{
	struct spi_transfer	t = {
			.rx_buf		= spidev->buffer_ptr,
			.len		= spidev->buffer_len,
		};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spidev_sync(spidev, &m);
}

void sx1276_write(uint16_t addr, uint8_t* data_ptr, uint8_t data_len)
{
	spidev.buffer_ptr[0] = addr | 0x80;
	memcpy(&spidev.buffer_ptr[1], data_ptr, data_len);
	spidev.buffer_len = data_len + 1;
	spidev_sync_write(spidev);
}

uint8_t sx1276_read(uint16_t addr, uint8_t* data_ptr, uint8_t data_len)
{
	uint8_t result = 0;
	spidev.buffer_ptr[0] = addr | 0x7F;
	spidev.buffer_len = data_len + 1;
	spidev_sync_read(spidev);
    return data;
}

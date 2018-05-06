#ifndef SX127X_H_
#define SX127X_H_

#include <linux/types.h>

#include "sx127xlib.h"
#if 0
enum sx127x_ioctl_cmd {
	SX127X_IOCTL_CMD_GETMODULATION,
	SX127X_IOCTL_CMD_SETMODULATION,
	SX127X_IOCTL_CMD_GETCARRIERFREQUENCY,
	SX127X_IOCTL_CMD_SETCARRIERFREQUENCY,
	SX127X_IOCTL_CMD_GETSF,
	SX127X_IOCTL_CMD_SETSF,
	SX127X_IOCTL_CMD_GETOPMODE,
	SX127X_IOCTL_CMD_SETOPMODE,
	SX127X_IOCTL_CMD_GETPAOUTPUT,
	SX127X_IOCTL_CMD_SETPAOUTPUT,
	SX127X_IOCTL_CMD_GETOUTPUTPOWER,
	SX127X_IOCTL_CMD_SETOUTPUTPOWER,
	SX127X_IOCTL_CMD_GETBANDWIDTH,
	SX127X_IOCTL_CMD_SETBANDWIDTH,
	SX127X_IOCTL_CMD_GETSYNCWORD,
	SX127X_IOCTL_CMD_SETSYNCWORD,
	SX127X_IOCTL_CMD_GETCRC,
	SX127X_IOCTL_CMD_SETCRC,
	SX127X_IOCTL_CMD_GETINVERTIQ,
	SX127X_IOCTL_CMD_SETINVERTIQ,
};

enum sx127x_modulation {
	SX127X_MODULATION_FSK,
	SX127X_MODULATION_OOK,
	SX127X_MODULATION_LORA,
	SX127X_MODULATION_INVALID
};

/* the last 3 modes are only valid in lora mode */
enum sx127x_opmode {
	SX127X_OPMODE_SLEEP,
	SX127X_OPMODE_STANDBY,
	SX127X_OPMODE_FSTX,
	SX127X_OPMODE_TX,
	SX127X_OPMODE_FSRX,
	SX127X_OPMODE_RX,
	SX127X_OPMODE_RXCONTINUOS,
	SX127X_OPMODE_RXSINGLE,
	SX127X_OPMODE_CAD
};

enum sx127x_pa {
	SX127X_PA_RFO,
	SX127X_PA_PABOOST
};
#endif
struct sx127x_pkt {
	size_t len;
	size_t hdrlen;
	size_t payloadlen;

	__s16 snr;
	__s16 rssi;
	__u32 fei;
	__u8 crcfail;
} __attribute__ ((packed));


struct sx127x {
    char *name;
	struct device *chardevice;
	struct work_struct irq_work;
	struct spi_device* spidevice;
	struct gpio_desc *gpio_reset, *gpio_txen, *gpio_rxen;
    struct gpio_desc *gpio_dio0;
	u32 fosc;
	//enum sx127x_pa pa;
    struct sx127x_cfg cfg;
	struct mutex mutex;

	struct list_head device_entry;
	dev_t devt;
	bool open;

	/* device state */
	bool loraregmap;
	enum sx127x_opmode opmode;
	/* tx */
	wait_queue_head_t writewq;
	int transmitted;
	/* rx */
	wait_queue_head_t readwq;
	struct kfifo out;
};

#endif /* SX127X_H_ */

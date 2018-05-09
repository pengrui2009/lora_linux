#ifndef SX127X_H_
#define SX127X_H_

#include <linux/types.h>
#include <asm/atomic.h> //atomic
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

struct sx127x_platform_data {
    struct gpio_desc *gpio_reset;
    struct gpio_desc *gpio_dio0;
    struct gpio_desc *gpio_dio1;
    struct gpio_desc *gpio_dio2;
    struct gpio_desc *gpio_dio3;
    struct gpio_desc *gpio_dio4;

    int irq_gpio_dio0;
    int irq_gpio_dio1;
    int irq_gpio_dio2;
    int irq_gpio_dio3;
    int irq_gpio_dio4;

};

/*!
 * Radio driver supported modems
 */
typedef enum
{
    MODEM_FSK = 0,
    MODEM_LORA,
}RadioModems_t;

/*!
 * Radio driver internal state machine states definition
 */
typedef enum
{
    RF_IDLE = 0,   //!< The radio is idle
    RF_RX_RUNNING, //!< The radio is in reception state
    RF_TX_RUNNING, //!< The radio is in transmission state
    RF_CAD,        //!< The radio is doing channel activity detection
}RadioState_t;

/*!
 * Radio FSK modem parameters
 */
typedef struct
{
    uint32_t Channel;
    int8_t   Power;
    uint32_t Fdev;
    uint32_t Bandwidth;
    uint32_t BandwidthAfc;
    uint32_t Datarate;
    uint16_t PreambleLen;
    bool     FixLen;
    uint8_t  PayloadLen;
    bool     CrcOn;
    bool     IqInverted;
    bool     RxContinuous;
    uint32_t TxTimeout;
    uint32_t RxSingleTimeout;
}RadioFskSettings_t;

/*!
 * Radio FSK packet handler state
 */
typedef struct
{
    uint8_t  PreambleDetected;
    uint8_t  SyncWordDetected;
    int8_t   RssiValue;
    int32_t  AfcValue;
    uint8_t  RxGain;
    uint16_t Size;
    uint16_t NbBytes;
    uint8_t  FifoThresh;
    uint8_t  ChunkSize;
}RadioFskPacketHandler_t;

/*!
 * Radio LoRa modem parameters
 */
typedef struct
{
    //tx
    uint32_t tChannel;
    uint32_t tBandwidth;
    uint32_t tDatarate;
    uint8_t  tCoderate;
    int8_t   Power;
    //rx
    uint32_t rChannel;
    uint32_t rBandwidth;
    uint32_t rDatarate;
    uint8_t  rCoderate;
    
    //common
    bool     LowDatarateOptimize;
    uint16_t PreambleLen;
    bool     FixLen;
    uint8_t  PayloadLen;
    bool     CrcOn;
    bool     FreqHopOn;
    uint8_t  HopPeriod;
    bool     IqInverted;
    bool     RxContinuous;
    uint32_t TxTimeout;
    bool     PublicNetwork;
}RadioLoRaSettings_t;

/*!
 * Radio LoRa packet handler state
 */
 #if 0
typedef struct
{
    int8_t SnrValue;
    int16_t RssiValue;
    uint8_t Size;
}RadioLoRaPacketHandler_t;
#endif
/*!
 * Radio Settings
 */
struct sx127x_cfg 
{
    
    RadioState_t             State;
    RadioModems_t            Modem;
    enum sx127x_pa           pa;
    RadioFskSettings_t       Fsk;
    //RadioFskPacketHandler_t  FskPacketHandler;
    RadioLoRaSettings_t      LoRa;
    //RadioLoRaPacketHandler_t LoRaPacketHandler;
}RadioSettings_t;

#if 0
struct sx127x_cfg {
    enum sx127x_modulation modulation;
    //the freq of tx
    u32 tx_freq;
    // LORA [0: 7.8 kHz, 1: 10.4 kHz, 2: 15.6 kHz, 3: 20.8 kHz, 4: 31.2 kHz,
    // 5: 41.6 kHz, 6: 62.5 kHz, 7: 125 kHz, 8: 250 kHz, 9: 500 kHz, other: Reserved]  
    u8 tx_bw;
    // LORA [6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]
    u8 tx_sf;
    // LORA [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
    u8 tx_codingrate;
    bool tx_crc;
    bool tx_invertiq;
    
    u32 rx_freq;
    u8 rx_bw;
    u8 tx_sf;
    u8 tx_codingrate;
    bool rx_crc;
    bool rx_invertiq;
    //the power of send data
    s8 txpower;
    enum sx127x_pa pa;
    u8 syncword;
  
};
#endif

struct sx127x {
    char *name;
    struct sx127x_platform_data plat_data;
    struct sx127x_cfg cfg;

    
    struct device *dev;
    struct device *chardev;
    struct work_struct irq_work;
    //struct spi_device* spidevice;
    
    u32 fosc;
    
    
    struct mutex mutex;
    struct list_head device_entry;
    dev_t devt;
    bool open;
    atomic_t opened;
    /* device state */
    //bool loraregmap;
    //enum sx127x_opmode opmode;
    /* tx */
    wait_queue_head_t writewq;
    int transmitted;
    /* rx */
    wait_queue_head_t readwq;
    struct kfifo out;
	/* cad */
    wait_queue_head_t cadwq;
    int caddone;
};

#endif /* SX127X_H_ */

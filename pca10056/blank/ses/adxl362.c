#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"

#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "adxl362.h"

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */
static bool adxl362_inited = false;

#define ADXL_MAX_CMD_LEN  32

static uint8_t       m_tx_buf[ADXL_MAX_CMD_LEN];    /**< TX buffer. */
static uint8_t       m_rx_buf[ADXL_MAX_CMD_LEN];    /**< RX buffer. */
static uint8_t       tx_len;

static void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
    NRF_LOG_INFO("Transfer completed.");
    if (m_rx_buf[0] != 0)
    {
        NRF_LOG_INFO(" Received:");
        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
    }
}

static void adxl362_get_reg(uint8_t addr, uint8_t *val, uint8_t size)
{
    memset(m_tx_buf, 0, ADXL_MAX_CMD_LEN);
    m_tx_buf[0] = ADXL362_READ_REG;
    m_tx_buf[1] = addr;
    tx_len = 2;

    if (size > ADXL_MAX_CMD_LEN) {
      NRF_LOG_INFO("AXDL362 get reg size should not exceed %d.", ADXL_MAX_CMD_LEN);
      return;
    }

    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, tx_len, m_rx_buf, tx_len + size));
    while (!spi_xfer_done)
    {
      __WFE();
    }
    NRF_LOG_FLUSH();

    memcpy(val, m_rx_buf + tx_len, size);
}

static void adxl362_set_reg(uint8_t addr, uint16_t val, uint8_t size)
{
    memset(m_tx_buf, 0, ADXL_MAX_CMD_LEN);
    m_tx_buf[0] = ADXL362_WRITE_REG;
    m_tx_buf[1] = addr;
    m_tx_buf[2] = val & 0x00ff;
    m_tx_buf[3] = val >> 8;
    tx_len = 4;

    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, tx_len, m_rx_buf, 0));
    while (!spi_xfer_done)
    {
      __WFE();
    }
    NRF_LOG_FLUSH();
}

int adxl362_init(void)
{
    uint8_t partid, devid_ad, devid_mst = 0;
    uint8_t regval; 

    partid = devid_ad = devid_mst = 0;

    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = SPI_SS_PIN;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;

    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));

    NRF_LOG_INFO("SPI init done.");

    // Write to SOFT RESET, "R"
    unsigned short reset = ADXL362_RESET_KEY;
    adxl362_set_reg(ADXL362_REG_SOFT_RESET, reset, 1);
    nrf_delay_ms(10);

    adxl362_get_reg(ADXL362_REG_PARTID, &partid, 1);
    adxl362_get_reg(ADXL362_REG_DEVID_AD, &devid_ad, 1);
    adxl362_get_reg(ADXL362_REG_DEVID_MST, &devid_mst, 1);

    if((partid == ADXL362_PART_ID) && (devid_ad == ADXL362_DEVICE_AD)
	&& (devid_mst == ADXL362_DEVICE_MST))
    {
      adxl362_inited = true;    
      NRF_LOG_RAW_INFO("AXDL362 reset done.");
      nrf_delay_ms(10);

      adxl362_get_reg(ADXL362_REG_POWER_CTL, &regval, 1);
      NRF_LOG_RAW_INFO("AXDL362 power mode 0x%x.", regval);

      adxl362_get_reg(ADXL362_REG_FILTER_CTL, &regval, 1);
      NRF_LOG_RAW_INFO("AXDL362 range and output rate 0x%x.", regval);

      // Set Measure On
      adxl362_set_reg(ADXL362_REG_POWER_CTL,
		regval | ADXL362_POWER_CTL_MEASURE(ADXL362_MEASURE_ON), 1);
      adxl362_get_reg(ADXL362_REG_POWER_CTL, &regval, 1);
      
      return 0;
    }
    else {
      NRF_LOG_RAW_INFO("AXDL362 reset fail.");
      return -1;
    }
}

void adxl_get_xyz(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t xyzValues[6] = {0, 0, 0, 0, 0, 0};
    volatile uint16_t u16 = 0;
    
    adxl362_get_reg(ADXL362_REG_XDATA_L, xyzValues, 6);
    u16 = (xyzValues[1] << 8) + xyzValues[0];
    *x = (int16_t) u16;
    u16 = (xyzValues[3] << 8) + xyzValues[2];
    *y = (int16_t) u16;
    u16 = (xyzValues[5] << 8) + xyzValues[4];
    *z = (int16_t) u16;
}

void adxl_get_floag_xyz(float *x, float *y, float *z)
{
    uint8_t xyzValues[6] = {0, 0, 0, 0, 0, 0};
    volatile uint16_t u16 = 0;
    volatile int16_t s16 = 0;
    adxl362_get_reg(ADXL362_REG_XDATA_L, xyzValues, 6);

    char selectedRange = 2;

    u16 = (xyzValues[1] << 8) + xyzValues[0];
    s16 = (int16_t)u16;
    *x = ((float)s16) / (1000 / (selectedRange / 2));
    u16 = (xyzValues[3] << 8) + xyzValues[2];
    s16 = (int16_t)u16;
    *y = ((float)s16) /(1000 / (selectedRange / 2));
    u16 = (xyzValues[5] << 8) + xyzValues[4];
    s16 = (int16_t)u16;
    *z = ((float)s16) /(1000 / (selectedRange / 2));
}

bool is_adxl_inited()
{
    return adxl362_inited;
}

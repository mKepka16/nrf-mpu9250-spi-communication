#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

/* HW selection (matches your DTS) */
#define MPU9250_SPI_LABEL DT_NODELABEL(spi1)
#define SPI_GPIO DT_NODELABEL(gpio0)
#define SPI_GPIO_CS 25
#define WHO_AM_I_REG 0x75
#define WHO_AM_I_EXPECT 0x71
#define GYRO_XOUT_H 0x43

static const struct device *mpu9250_dev = DEVICE_DT_GET(MPU9250_SPI_LABEL);
static const struct device *gpio_dev = DEVICE_DT_GET(SPI_GPIO);

static struct spi_config spi_cfg = {
    .frequency = 1000000U, /* 1 MHz is fine; you can start lower */
    .operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8),
    .slave = 0, /* nRF SPIM ignores this when you do manual CS */
    .cs = NULL, /* we toggle CS via GPIO */
};

/* Multi-byte read: keeps CS low across address + data */
static int mpu9250_read_regs(uint8_t start_reg, uint8_t *dst, size_t len)
{
  if (!len)
    return -EINVAL;

  int err;
  uint8_t addr = start_reg | 0x80; /* set READ bit (MSB) */

  struct spi_buf tx_bufs[] = {{.buf = &addr, .len = 1}};
  struct spi_buf_set tx = {.buffers = tx_bufs, .count = 1};

  struct spi_buf rx_bufs[] = {{.buf = dst, .len = len}};
  struct spi_buf_set rx = {.buffers = rx_bufs, .count = 1};

  /* Assert CS for the whole transaction (address then data) */
  gpio_pin_set(gpio_dev, SPI_GPIO_CS, 0);
  err = spi_write(mpu9250_dev, &spi_cfg, &tx);
  if (err == 0)
  {
    err = spi_read(mpu9250_dev, &spi_cfg, &rx);
  }
  gpio_pin_set(gpio_dev, SPI_GPIO_CS, 1);

  return err;
}

int main(void)
{
  LOG_INF("Starting MPU-9250 manual SPI test (burst-capable)");

  if (!device_is_ready(mpu9250_dev) || !device_is_ready(gpio_dev))
  {
    LOG_ERR("Device not ready");
    return -ENODEV;
  }

  /* Configure CS pin and deassert (high) */
  gpio_pin_configure(gpio_dev, SPI_GPIO_CS, GPIO_OUTPUT);
  gpio_pin_set(gpio_dev, SPI_GPIO_CS, 1);

  /* 1-byte example: WHO_AM_I */
  uint8_t who = 0;
  int err = mpu9250_read_regs(WHO_AM_I_REG, &who, 1);
  if (err)
  {
    LOG_ERR("WHO_AM_I read failed: %d", err);
  }
  else
  {
    LOG_INF("WHO_AM_I = 0x%02X (expect 0x%02X)", who, WHO_AM_I_EXPECT);
  }

  /* Multi-byte example (uncomment later): read 14 bytes from ACCEL_XOUT_H */
  uint8_t sens[6];

  while (1)
  {
    err = mpu9250_read_regs(GYRO_XOUT_H, sens, sizeof(sens)); // auto-increments
    uint16_t gyro_x = (sens[0] << 8) | sens[1];
    uint16_t gyro_y = (sens[2] << 8) | sens[3];
    uint16_t gyro_z = (sens[4] << 8) | sens[5];
    // LOG_HEXDUMP_INF(sens, sizeof(sens), "Sensor data: ");
    LOG_INF("Gyro (x,y,z) = (%d, %d, %d)", gyro_x, gyro_y, gyro_z);
    k_msleep(1000);
  }
}

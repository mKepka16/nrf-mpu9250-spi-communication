#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

/* HW selection (matches your DTS) */
#define WHO_AM_I_REG 0x75
#define WHO_AM_I_EXPECT 0x71
#define GYRO_XOUT_H 0x43

#define SPI_OPERATION SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8)
static const struct spi_dt_spec mpu9250_dev = SPI_DT_SPEC_GET(DT_NODELABEL(mpu9250), SPI_OPERATION, 0);

/* Multi-byte read: keeps CS low across address + data */
static int mpu9250_read_regs(uint8_t start_reg, uint8_t *dst, size_t len)
{
  if (!len)
    return -EINVAL;

  uint8_t tx[1 + len];
  uint8_t rx[1 + len];

  tx[0] = start_reg | 0x80;  // READ bit
  memset(&tx[1], 0xFF, len); // dummy bytes to clock data out

  struct spi_buf txb = {.buf = tx, .len = sizeof(tx)};
  struct spi_buf rxb = {.buf = rx, .len = sizeof(rx)};
  struct spi_buf_set txset = {.buffers = &txb, .count = 1};
  struct spi_buf_set rxset = {.buffers = &rxb, .count = 1};

  int err = spi_transceive_dt(&mpu9250_dev, &txset, &rxset);
  if (err)
    return err;

  memcpy(dst, &rx[1], len); // skip the first (garbage) byte
  return 0;
}

int main(void)
{
  LOG_INF("Starting MPU-9250 SPI test");

  if (!spi_is_ready_dt(&mpu9250_dev))
  {
    LOG_ERR("MPU-9250 not ready");
    return -ENODEV;
  }

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

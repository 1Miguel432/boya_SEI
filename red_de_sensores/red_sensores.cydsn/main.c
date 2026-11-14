#include "project.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "bme68x.h"

#define SCD41_ADDR   0x62
#define BME680_ADDR  0x77     
#define PERIODO_MS   5000
#define MOSTRAR_DBG  1        

/* ------------------------- I2C bajo nivel ------------------------- */
static int i2c_wait(uint8 flag)
{
    uint32 t = 100000;
    while (!(I2C_MasterStatus() & flag)) {
        if (--t == 0) return -1;
        CyDelayUs(1);
    }
    return (I2C_MasterStatus() & I2C_MSTAT_ERR_XFER) ? -1 : 0;
}

static int i2c_write(uint8 addr, uint8 *buf, uint8 len, uint8 mode)
{
    I2C_MasterClearStatus();
    if (I2C_MasterWriteBuf(addr, buf, len, mode) != I2C_MSTR_NO_ERROR) return -1;
    return i2c_wait(I2C_MSTAT_WR_CMPLT);
}

static int i2c_read(uint8 addr, uint8 *buf, uint8 len, uint8 mode)
{
    I2C_MasterClearStatus();
    if (I2C_MasterReadBuf(addr, buf, len, mode) != I2C_MSTR_NO_ERROR) return -1;
    return i2c_wait(I2C_MSTAT_RD_CMPLT);
}

/* --------------------------- SCD41 --------------------------- */
/* Comandos de 16 bits; cada palabra de respuesta trae CRC-8 (poly 0x31, init 0xFF) */
static uint8 scd_crc(const uint8 *d)
{
    uint8 crc = 0xFF;
    for (int i = 0; i < 2; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8)((crc << 1) ^ 0x31) : (uint8)(crc << 1);
    }
    return crc;
}

static int scd_cmd(uint16 cmd)
{
    uint8 b[2] = { (uint8)(cmd >> 8), (uint8)(cmd & 0xFF) };
    return i2c_write(SCD41_ADDR, b, 2, I2C_MODE_COMPLETE_XFER);
}

/* Envia un comando y lee n palabras de respuesta.
   0 = ok, -1 = error de I2C, -2 = error de CRC */
static int scd_read_words(uint16 cmd, uint16 *w, uint8 n)
{
    uint8 raw[9];
    if (scd_cmd(cmd)) return -1;
    CyDelay(1);                                     /* tiempo de ejecucion del comando */
    if (i2c_read(SCD41_ADDR, raw, (uint8)(3 * n), I2C_MODE_COMPLETE_XFER)) return -1;
    for (uint8 i = 0; i < n; i++) {
        if (scd_crc(&raw[3 * i]) != raw[3 * i + 2]) return -2;
        w[i] = (uint16)((raw[3 * i] << 8) | raw[3 * i + 1]);
    }
    return 0;
}

static int scd_init(void)
{
    scd_cmd(0x3F86); CyDelay(500);     /* stop_periodic_measurement */
    scd_cmd(0x3646); CyDelay(30);      /* reinit */
    return scd_cmd(0x21B1);            /* start_periodic_measurement (dato cada 5 s) */
}

/* 1 = hay dato nuevo, 0 = todavia no, negativo = error (-1 I2C, -2 CRC) */
static int scd_estado(void)
{
    uint16 w;
    int r = scd_read_words(0xE4B8, &w, 1);    /* get_data_ready_status */
    if (r) return r;
    return (w & 0x07FF) ? 1 : 0;
}

static int scd_read(uint16 *co2, float *t, float *h)
{
    uint16 w[3];
    int r = scd_read_words(0xEC05, w, 3);      /* read_measurement */
    if (r) return r;
    *co2 = w[0];
    *t = -45.0f + 175.0f * w[1] / 65535.0f;
    *h = 100.0f * w[2] / 65535.0f;
    return 0;
}

/* --------------------- BME680 (API de Bosch) --------------------- */
static uint8 bme_addr = BME680_ADDR;
static struct bme68x_dev bme;
static struct bme68x_conf bme_conf;
static struct bme68x_heatr_conf bme_heatr;

static BME68X_INTF_RET_TYPE bme_i2c_read(uint8_t reg, uint8_t *data, uint32_t len, void *intf)
{
    uint8 a = *(uint8 *)intf;
    if (i2c_write(a, &reg, 1, I2C_MODE_NO_STOP)) return -1;
    if (i2c_read(a, data, (uint8)len, I2C_MODE_REPEAT_START)) return -1;
    return 0;
}

static BME68X_INTF_RET_TYPE bme_i2c_write(uint8_t reg, const uint8_t *data, uint32_t len, void *intf)
{
    uint8 a = *(uint8 *)intf;
    uint8 buf[64];
    if (len + 1 > sizeof(buf)) return -1;
    buf[0] = reg;
    memcpy(&buf[1], data, len);
    return i2c_write(a, buf, (uint8)(len + 1), I2C_MODE_COMPLETE_XFER) ? -1 : 0;
}

static void bme_delay_us(uint32_t us, void *intf)
{
    (void)intf;
    if (us >= 1000) CyDelay(us / 1000);
    CyDelayUs((uint16)(us % 1000));
}

static int bme_init(void)
{
    bme.read = bme_i2c_read;
    bme.write = bme_i2c_write;
    bme.delay_us = bme_delay_us;
    bme.intf = BME68X_I2C_INTF;
    bme.intf_ptr = &bme_addr;
    bme.amb_temp = 25;
    if (bme68x_init(&bme) != BME68X_OK) return -1;

    bme_conf.filter  = BME68X_FILTER_SIZE_3;
    bme_conf.odr     = BME68X_ODR_NONE;
    bme_conf.os_hum  = BME68X_OS_2X;
    bme_conf.os_pres = BME68X_OS_4X;
    bme_conf.os_temp = BME68X_OS_8X;
    if (bme68x_set_conf(&bme_conf, &bme) != BME68X_OK) return -1;

    bme_heatr.enable     = BME68X_ENABLE;
    bme_heatr.heatr_temp = 320;   /* grados C */
    bme_heatr.heatr_dur  = 150;   /* ms */
    return bme68x_set_heatr_conf(BME68X_FORCED_MODE, &bme_heatr, &bme) == BME68X_OK ? 0 : -1;
}

static int bme_read(struct bme68x_data *d)
{
    uint8_t n = 0;
    bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
    uint32_t espera = bme68x_get_meas_dur(BME68X_FORCED_MODE, &bme_conf, &bme)
                      + (uint32_t)bme_heatr.heatr_dur * 1000;
    bme.delay_us(espera, bme.intf_ptr);
    if (bme68x_get_data(BME68X_FORCED_MODE, d, &n, &bme) != BME68X_OK || n == 0) return -1;
    return 0;
}

/* ------------------------- Utilidades ------------------------- */
/* Float -> "123.45" sin depender del soporte de %f en newlib-nano */
static void f2s(char *out, float v)
{
    long x = (long)(v * 100.0f + (v >= 0 ? 0.5f : -0.5f));
    sprintf(out, "%s%ld.%02ld", x < 0 ? "-" : "", labs(x) / 100, labs(x) % 100);
}

/* ---------------------------- main ---------------------------- */
int main(void)
{
    char linea[256], t1[12], h1[12], t2[12], h2[12], p[12], g[12];

    CyGlobalIntEnable;
    UART_Start();
    I2C_Start();
    CyDelay(1000);

    int scdOk = (scd_init() == 0);
    int bmeOk = (bme_init() == 0);
    if (!scdOk) UART_PutString("{\"error\":\"SCD41 no responde\"}\r\n");
    if (!bmeOk) UART_PutString("{\"error\":\"BME680 no responde\"}\r\n");
    UART_PutString("{\"info\":\"nodo listo\"}\r\n");

    uint32 t_ms = 0;
    uint16 co2 = 0;
    float  tScd = 0, hScd = 0;
    uint8  sin_dato = 0;

    for (;;) {
        CyDelay(PERIODO_MS);
        t_ms += PERIODO_MS;

        /* --- SCD41 --- */
        int listo = -9, lectura = -9;
        if (scdOk) {
            listo = scd_estado();
            if (listo == 1) {
                lectura = scd_read(&co2, &tScd, &hScd);
                sin_dato = 0;
            } else if (++sin_dato >= 3) {
                /* 15 s sin dato: el sensor pudo reiniciarse y quedar en reposo */
                UART_PutString("{\"aviso\":\"SCD41 sin datos, reiniciando medicion\"}\r\n");
                scd_cmd(0x21B1);
                sin_dato = 0;
            }
        }

#if MOSTRAR_DBG
        sprintf(linea, "{\"scd41_dbg\":{\"listo\":%d,\"lectura\":%d}}\r\n", listo, lectura);
        UART_PutString(linea);
#endif

        /* --- BME680 --- */
        struct bme68x_data d;
        memset(&d, 0, sizeof(d));
        if (bmeOk) bme_read(&d);

        f2s(t1, tScd);           f2s(h1, hScd);
        f2s(t2, d.temperature);  f2s(h2, d.humidity);
        f2s(p, d.pressure / 100.0f);        /* Pa -> hPa */
        f2s(g, d.gas_resistance / 1000.0f); /* ohm -> kohm */

        /* --- Paquete JSON: una linea por lectura --- */
        sprintf(linea,
            "{\"nodo\":\"nodo_01\",\"t_ms\":%lu,"
            "\"scd41\":{\"co2_ppm\":%u,\"temp_c\":%s,\"hum_pct\":%s},"
            "\"bme680\":{\"temp_c\":%s,\"hum_pct\":%s,\"pres_hpa\":%s,\"gas_kohm\":%s}}\r\n",
            (unsigned long)t_ms, co2, t1, h1, t2, h2, p, g);
        UART_PutString(linea);
    }
}
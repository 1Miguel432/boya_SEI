# Red de sensores

Nodo ambiental del proyecto de la boya: un PSoC 5LP (CY8CKIT-059) lee dos
sensores por I2C y envía una línea JSON cada 5 s por UART.

## Sensores

| Sensor | Mide | Dirección I2C |
|---|---|---|
| SCD41 (Sensirion) | CO₂, temperatura, humedad | 0x62 |
| BME680 (Bosch) | Temperatura, humedad, presión, gas | 0x77 |

## Conexiones

| Señal | PSoC |
|---|---|
| SCL | P12[0] |
| SDA | P12[1] |
| UART Tx (a la PC) | P12[6] |
| UART Rx | P12[7] |

Los dos sensores comparten VCC, GND, SDA y SCL.

## Proyecto en PSoC Creator

- Componente **I2C Master** llamado `I2C`, 100 kHz, Fixed Function.
- Componente **UART** llamado `UART`, 115200 8N1.
- Heap Size en `0x400` (Design Wide Resources → System), necesario para `sprintf`.
- Usa la API de Bosch [BME68x_SensorAPI](https://github.com/boschsensortec/BME68x_SensorAPI)
  (`bme68x.c`, `bme68x.h`, `bme68x_defs.h`), ya incluida en el proyecto.

## Archivos

- `red_sensores.cydsn/`: proyecto completo. Abrir `red_sensores.cyprj` con
  File → Open → Project/Workspace y compilar (Build). La carpeta
  Generated_Source se vuelve a crear sola al compilar.


## Salida

```json
{"nodo":"nodo_01","t_ms":5000,
 "scd41":{"co2_ppm":612,"temp_c":27.72,"hum_pct":75.46},
 "bme680":{"temp_c":30.18,"hum_pct":67.64,"pres_hpa":1003.47,"gas_kohm":16.53}}
```

`MOSTRAR_DBG` en `main.c` imprime además el estado del SCD41 en cada ciclo
(1 = activado, 0 = desactivado).
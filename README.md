# SHT30 Temperature/Humidity sensor I2C driver

This is a basic driver for the SHT30 temperature and humidity sensor for the ESP32 set of chips.

## Example code

This should help with using the component:

```
#include "sht30.h"

sht30_t   sht30;

sht30.i2c_port = I2C_NUM_0;
sht30.addr = 0x44;
if (sht30_init(&sht30) != ESP_OK)
{
  // error handling...
}
  
// display temperature and humidity every 10 seconds
while (1)
{
  printf("SHT30: %5.2f C %5.2f %%\n",
    sht30_get_temperature(&sht30, true),
    sht30_get_humidity(&sht30, false)
  );
  vTaskDelay(pdMS_TO_TICKS(10000));
}
```

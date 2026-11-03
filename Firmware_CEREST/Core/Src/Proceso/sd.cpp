#include "Proceso/sd.hpp"

static bool isMounted = false;

// Compartidos entre Guardar() y Cargar() — nunca se ejecutan al mismo
// tiempo (proyecto de un solo hilo), así que una sola copia de cada
// uno es segura y elimina la duplicación de RAM.
static FIL archivoCompartido;
static Header headerCompartido;

bool SDcard::Inicio()
{
    if (BSP_SD_IsDetected() != SD_PRESENT)
    {
        isMounted = false;
        return false;
    }

    if (isMounted) return true;

    if (f_mount(&SDFatFS, (TCHAR const*)SDPath, 1) == FR_OK)
    {
        isMounted = true;
        return true;
    }

    return false;
}

uint32_t SDcard::ComputeCrc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    for(uint32_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for(int j = 0; j < 8; j++)
        {
            if(crc & 1) crc = (crc >> 1) ^ 0xEDB88320;
            else crc >>= 1;
        }
    }
    return ~crc;
}

bool SDcard::Guardar(const char *nombreArchivo, uint32_t magic, const void *data, uint32_t len)
{
    if (!Inicio()) return false;

    if (f_open(&archivoCompartido, nombreArchivo, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK)
        return false;

    headerCompartido.magic = magic;
    headerCompartido.dataLen = len;
    headerCompartido.crc32 = ComputeCrc32(static_cast<const uint8_t*>(data), len);

    UINT bytesWritten = 0;
    FRESULT res = f_write(&archivoCompartido, &headerCompartido, sizeof(Header), &bytesWritten);

    if (res == FR_OK && bytesWritten == sizeof(Header))
    {
        res = f_write(&archivoCompartido, data, len, &bytesWritten);
    }

    f_close(&archivoCompartido);

    return (res == FR_OK && bytesWritten == len);
}

bool SDcard::Cargar(const char *nombreArchivo, uint32_t magic, void *outData, uint32_t len)
{
    if (!Inicio()) return false;

    if (f_open(&archivoCompartido, nombreArchivo, FA_READ) != FR_OK)
        return false;

    UINT bytesRead = 0;

    if (f_read(&archivoCompartido, &headerCompartido, sizeof(Header), &bytesRead) != FR_OK || bytesRead != sizeof(Header))
    {
        f_close(&archivoCompartido);
        return false;
    }

    if (headerCompartido.magic != magic || headerCompartido.dataLen != len)
    {
        f_close(&archivoCompartido);
        return false;
    }

    if (f_read(&archivoCompartido, outData, len, &bytesRead) != FR_OK || bytesRead != len)
    {
        f_close(&archivoCompartido);
        return false;
    }

    f_close(&archivoCompartido);

    uint32_t crcReal = ComputeCrc32(static_cast<const uint8_t*>(outData), len);
    return (crcReal == headerCompartido.crc32);
}

extern "C" uint8_t BSP_SD_IsDetected(void)
{
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET)
    {
        return SD_PRESENT;
    }
    return SD_NOT_PRESENT;
}
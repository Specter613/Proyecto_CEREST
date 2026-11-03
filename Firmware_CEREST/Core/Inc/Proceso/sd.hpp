
#ifndef INC_PROCESO_SD_HPP_
#define INC_PROCESO_SD_HPP_

#include <cstdint>
#include <cstring>
#include "fatfs.h"

struct Header
{
    uint32_t magic;
    uint32_t dataLen;
    uint32_t crc32;
};

namespace SDLayout
{
    constexpr const char* kMagFilename = "mag_cal.bin";
    constexpr uint32_t kMagMagic       = 0xCAFEBABE;

    constexpr const char* kImuFilename = "imu_cal.bin";
    constexpr uint32_t kImuMagic       = 0xDEADBEEF;

    constexpr const char* kPidFilename = "pid_cal.bin";
    constexpr uint32_t kPidMagic       = 0x50494431;
}

class SDcard
{
public:
    // Monta la tarjeta SD
    static bool Inicio();

    // Guarda el bloque con Magic + CRC32 en la SD
    static bool Guardar(const char *nombreArchivo, uint32_t magic, const void *data, uint32_t len);

    // Carga y valida los datos desde el archivo
    static bool Cargar(const char *nombreArchivo, uint32_t magic, void *outData, uint32_t len);

private:
    static uint32_t ComputeCrc32(const uint8_t *data, uint32_t len);

    struct Header
    {
        uint32_t magic;
        uint32_t crc32;
        uint32_t dataLen;
    };
};


#endif /* INC_PROCESO_SD_HPP_ */

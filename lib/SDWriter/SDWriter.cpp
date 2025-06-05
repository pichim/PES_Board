#include "SDWriter.h"

SDWriter::SDWriter(PinName mosi, PinName miso, PinName sck, PinName cs) :
    m_SDBlockDevice(mosi, miso, sck, cs), m_FATFileSystem("sd")
{
}

SDWriter::~SDWriter()
{
    // automatically attempt to close & unmount
    closeFile();
    unmount();
}

bool SDWriter::mount()
{
    if (m_mounted) {
        return true;
    }
    if (m_SDBlockDevice.init() != 0) {
        printf("SDWriter: block device init failed\n");
        return false;
    }
    if (m_SDBlockDevice.frequency(10000000) != 0) {
        printf("SDWriter: set frequency failed (not fatal)\n");
    }
    if (m_FATFileSystem.mount(&m_SDBlockDevice) != 0) {
        printf("SDWriter: FS mount failed\n");
        m_SDBlockDevice.deinit();
        return false;
    }
    if (!ensureDirExists("/sd/data")) {
        printf("SDWriter: failed to create data directory\n");
        m_FATFileSystem.unmount();
        m_SDBlockDevice.deinit();
        return false;
    }
    m_mounted = true;
    return true;
}

bool SDWriter::unmount()
{
    if (!m_mounted) {
        return true;
    }
    closeFile();
    m_FATFileSystem.unmount();
    m_SDBlockDevice.deinit();
    m_mounted = false;
    return true;
}

bool SDWriter::openNextFile()
{
    if (!m_mounted) {
        printf("SDWriter: not mounted, can't open file\n");
        return false;
    }
    closeFile(); // in case one was open
    return openNumberedFile();
}

void SDWriter::closeFile()
{
    if (m_FilePtr) {
        fclose(m_FilePtr);
        m_FilePtr = nullptr;
        printf("SDWriter: file closed\n");
    }
}

bool SDWriter::writeByte(uint8_t b)
{
    if (!m_FilePtr) {
        return false;
    }
    size_t written = fwrite(&b, 1, 1, m_FilePtr);
    if (written != 1) {
        printf("SDWriter: writeByte failed\n");
        return false;
    }
    return true;
}

// bool SDWriter::writeFloat(float value)
// {
//     if (!m_FilePtr) {
//         return false;
//     }
//     // fwrite 1 float
//     size_t written = fwrite(&value, sizeof(float), 1, m_FilePtr);
//     if (written != 1) {
//         printf("SDWriter: writeFloat failed (wrote %u of 1)\n",
//                (unsigned)written);
//         return false;
//     }
//     return true;
// }

bool SDWriter::writeFloats(const float* data, size_t count)
{
    if (!m_FilePtr) {
        return false;
    }
    size_t written = fwrite(data, sizeof(float), count, m_FilePtr);
    if (written != count) {
        printf("SDWriter: writeFloats failed (wrote %u of %u)\n",
               (unsigned)written, (unsigned)count);
        return false;
    }
    return true;
}

bool SDWriter::flush()
{
    if (!m_FilePtr) {
        return false;
    }
    if (fflush(m_FilePtr) != 0) {
        printf("SDWriter: fflush failed\n");
        return false;
    }
    // on some systems, you could also call fsync(fileno(m_FilePtr)) if available.
    return true;
}

bool SDWriter::ensureDirExists(const char* path)
{
    // check if directory already exists
    DIR* dir = opendir(path);
    if (dir) {
        closedir(dir);
        return true;
    }
    // try to create directory
    int result = mkdir(path, 0777);
    if (result == 0) {
        printf("SDWriter: created directory %s\n", path);
        return true;
    } else if (errno == EEXIST) {
        // directory already exists, which is fine
        return true;
    } else {
        printf("SDWriter: failed to create directory %s (errno: %d)\n", path, errno);
        return false;
    }
}

bool SDWriter::openNumberedFile()
{
    for (int i = 0; i <= 999; i++) {
        sprintf(m_file_path, "/sd/data/%03d.bin", i);
        FILE* test_fp = fopen(m_file_path, "r");
        if (!test_fp) {
            // file doesn't exist yet, try to create it
            m_FilePtr = fopen(m_file_path, "wb");
            if (m_FilePtr) {
                printf("SDWriter: opened %s\n", m_file_path);
                return true;
            } else {
                printf("SDWriter: failed to create %s\n", m_file_path);
                return false;
            }
        } else {
            fclose(test_fp); // file exists, try next
        }
    }
    printf("SDWriter: no more file slots up to 999!\n");
    return false;
}

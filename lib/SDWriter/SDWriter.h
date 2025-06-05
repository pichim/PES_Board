/**
 * @file SDWriter.h
 * @brief Defines the SDWriter class for low-level SD card file operations.
 *
 * The SDWriter class provides functionality for mounting and unmounting an SD card, 
 * opening sequential files (`/sd/data/001.bin`, `/sd/data/002.bin`, etc.), and 
 * writing binary floats or single bytes. It abstracts block device initialization 
 * and FAT file system operations, offering a streamlined interface for embedded logging.
 *
 * Data is written in raw binary format for efficiency. The user can also call `flush()` 
 * to ensure data is physically committed, reducing the risk of corruption due to power loss.
 *
 * @dependencies
 * This class relies on:
 * - **SDBlockDevice**: Handles low-level SD card communication over SPI.
 * - **FATFileSystem**: Manages file creation, opening, writing, and closing.
 *
 * @usage
 * 1. Mount the SD card using `mount()`.
 * 2. Open a new sequential file using `openNextFile()`.
 * 3. Write binary float data using `writeFloats(...)` or a single byte using `writeByte(...)`.
 * 4. Optionally, call `flush()` to ensure data is physically written.
 * 5. When done, close the file and unmount the SD card.
 *
 * @example
 * ```
 * SDWriter writer(MOSI_PIN, MISO_PIN, SCK_PIN, CS_PIN);
 *
 * if (writer.mount()) {
 *     if (writer.openNextFile()) {
 *         float data[10] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f,
 *                           6.0f, 7.0f, 8.0f, 9.0f, 10.0f};
 *         writer.writeFloats(data, 10);
 *         
 *         // ensure data is saved
 *         writer.flush();
 *         
 *         writer.closeFile();
 *     }
 *     writer.unmount();
 * } else {
 *     printf("Failed to mount SD card.\n");
 * }
 * ```
 *
 * @author M. Peter / pmic / pichim
 */

#ifndef SD_WRITER_H_
#define SD_WRITER_H_

#include "mbed.h"

#include <SDBlockDevice.h>
#include <FATFileSystem.h>

class SDWriter
{
public:
    explicit SDWriter(PinName mosi,
                      PinName miso,
                      PinName sck,
                      PinName cs);
    virtual ~SDWriter();

    bool mount();
    bool unmount();

    // opens a new file like /sd/data/001.bin, /sd/data/002.bin, etc.
    bool openNextFile();
    void closeFile();

    // write a single byte (e.g. "number of floats" header).
    bool writeByte(uint8_t b);

    // // write one floats to the file in binary.
    // bool writeFloat(float value);

    // write 'count' floats to the file in binary.
    bool writeFloats(const float* data, size_t count);

    // flush data to SD so it's physically written.
    bool flush();

private:
    SDBlockDevice m_SDBlockDevice;
    FATFileSystem m_FATFileSystem;

    FILE* m_FilePtr{nullptr};
    bool  m_mounted{false};
    char  m_file_path[64];   // current file path

    bool ensureDirExists(const char* path);
    bool openNumberedFile();
};
#endif /* SD_WRITER_H_ */

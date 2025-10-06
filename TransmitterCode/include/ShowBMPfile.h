// *************************************** ShowBMPfile  .h  *****************************************

#include <Arduino.h>
#include "1Definitions.h"
#ifndef SHOWBMPFILE_H
#define SHOWBMPFILE_H

#define CHUNK_PIXELS 2048 // Process this many pixels per batch for optimal speed

bool displayBMP565_Fast(const char *filename, int x, int y)
{
    File bmpFile = SD.open(filename);
    if (!bmpFile)
    {
        Serial.println("ERROR: Failed to open file");
        return false;
    }

    KickTheDog();

    uint8_t header[54];
    if (bmpFile.read(header, 54) != 54)
    {
        Serial.println("ERROR: Failed to read header");
        bmpFile.close();
        return false;
    }

    if (header[0] != 'B' || header[1] != 'M')
    {
        Serial.println("ERROR: Not a BMP file");
        bmpFile.close();
        return false;
    }

    uint32_t dataOffset = *(uint32_t *)&header[10];
    int32_t width = *(int32_t *)&header[18];
    int32_t height = *(int32_t *)&header[22];
    uint16_t bitsPerPixel = *(uint16_t *)&header[28];

    Serial.print("BMP: ");
    Serial.print(width);
    Serial.print("x");
    Serial.print(height);
    Serial.print(" @ ");
    Serial.print(bitsPerPixel);
    Serial.println(" bpp");

    if (bitsPerPixel != 16)
    {
        Serial.println("ERROR: Not 16-bit BMP");
        bmpFile.close();
        return false;
    }

    bool topDown = false;
    if (height < 0)
    {
        topDown = true;
        height = -height;
    }

    uint32_t rowSize = ((width * 2 + 3) / 4) * 4;
    uint32_t padding = rowSize - (width * 2);

    KickTheDog();

    bmpFile.seek(dataOffset);
    while (NEXTION.available())
        NEXTION.read();

    uint8_t *rowBuffer = (uint8_t *)malloc(width * 2);
    if (!rowBuffer)
    {
        Serial.println("ERROR: Failed to allocate memory");
        bmpFile.close();
        return false;
    }

// Large buffer for batching multiple rows worth of commands
// Adjust size based on available RAM - larger = faster but more RAM
#define CMD_BUFFER_SIZE 4096
    uint8_t *cmdBuffer = (uint8_t *)malloc(CMD_BUFFER_SIZE);
    if (!cmdBuffer)
    {
        Serial.println("ERROR: Failed to allocate command buffer");
        free(rowBuffer);
        bmpFile.close();
        return false;
    }

    int cmdBufferPos = 0;

    for (int32_t row = 0; row < height; row++)
    {
        if (row % 10 == 0)
        {
            KickTheDog();
        }

        int32_t drawRow = topDown ? row : (height - 1 - row);

        size_t bytesRead = bmpFile.read(rowBuffer, width * 2);
        if (bytesRead != (size_t)(width * 2))
        {
            Serial.println("ERROR: Failed to read pixel data");
            free(rowBuffer);
            free(cmdBuffer);
            bmpFile.close();
            return false;
        }

        uint16_t currentColor = rowBuffer[0] | (rowBuffer[1] << 8);
        int spanStart = 0;

        for (int32_t col = 1; col <= width; col++)
        {
            uint16_t pixelColor;

            if (col < width)
            {
                pixelColor = rowBuffer[col * 2] | (rowBuffer[col * 2 + 1] << 8);
            }
            else
            {
                pixelColor = ~currentColor;
            }

            if (pixelColor != currentColor || col == width)
            {
                int spanWidth = col - spanStart;

                // Build command string directly in the buffer
                char tempCmd[80];
                int cmdLen = snprintf(tempCmd, sizeof(tempCmd), "fill %d,%d,%d,1,%u",
                                      x + spanStart, y + (int)drawRow, spanWidth, (unsigned int)currentColor);

                // Check if command + terminators will fit in buffer
                // Leave room for safety margin
                if (cmdBufferPos + cmdLen + 3 > CMD_BUFFER_SIZE - 100)
                {
                    // Flush buffer to Nextion as ONE write operation
                    NEXTION.write(cmdBuffer, cmdBufferPos);
                    cmdBufferPos = 0;
                    // Small delay to let Nextion process
                    delayMicroseconds(100);
                }

                // Copy command directly into buffer (not using print!)
                memcpy(cmdBuffer + cmdBufferPos, tempCmd, cmdLen);
                cmdBufferPos += cmdLen;
                // Add terminators
                cmdBuffer[cmdBufferPos++] = 0xFF;
                cmdBuffer[cmdBufferPos++] = 0xFF;
                cmdBuffer[cmdBufferPos++] = 0xFF;

                currentColor = pixelColor;
                spanStart = col;
            }
        }
        if (padding > 0)
        {
            bmpFile.seek(bmpFile.position() + padding);
        }

        if (row % 10 == 0)
        {
            Serial.print(".");
        }
    }

    // Flush any remaining commands in ONE final write
    if (cmdBufferPos > 0)
    {
        NEXTION.write(cmdBuffer, cmdBufferPos);
    }
    KickTheDog();
    Serial.println("\nDone!");
    free(rowBuffer);
    free(cmdBuffer);
    bmpFile.close();
    return true;
}

/*
 * Alternative version with full-row processing for even faster display
 * Uses more memory but can be faster for wider images
 */
bool displayBMP565_FastRow(const char *filename, int x, int y)
{
    File bmpFile = SD.open(filename);
    if (!bmpFile)
    {
        return false;
    }

    uint8_t header[54];
    if (bmpFile.read(header, 54) != 54)
    {
        bmpFile.close();
        return false;
    }

    if (header[0] != 'B' || header[1] != 'M')
    {
        bmpFile.close();
        return false;
    }

    uint32_t dataOffset = *(uint32_t *)&header[10];
    uint32_t width = *(uint32_t *)&header[18];
    uint32_t height = *(uint32_t *)&header[22];
    uint16_t bitsPerPixel = *(uint16_t *)&header[28];
    uint32_t compression = *(uint32_t *)&header[30];

    if (bitsPerPixel != 16 || (compression != 0 && compression != 3))
    {
        bmpFile.close();
        return false;
    }

    uint32_t rowSize = ((width * 2 + 3) / 4) * 4;
    uint32_t padding = rowSize - (width * 2);

    bmpFile.seek(dataOffset);

    // Allocate buffer for one full row
    uint8_t *rowBuffer = (uint8_t *)malloc(width * 2);
    if (!rowBuffer)
    {
        bmpFile.close();
        return false;
    }

    // Process row by row (bottom to top)
    for (int32_t row = height - 1; row >= 0; row--)
    {
        if (bmpFile.read(rowBuffer, width * 2) != width * 2)
        {
            free(rowBuffer);
            bmpFile.close();
            return false;
        }

        NEXTION.print("picq ");
        NEXTION.print(x);
        NEXTION.print(",");
        NEXTION.print(y + row);
        NEXTION.print(",");
        NEXTION.print(width);
        NEXTION.print(",1");
        NEXTION.write(0xFF);
        NEXTION.write(0xFF);
        NEXTION.write(0xFF);
        NEXTION.write(rowBuffer, width * 2);

        delayMicroseconds(100);

        if (padding > 0)
        {
            bmpFile.seek(bmpFile.position() + padding);
        }
    }
    free(rowBuffer);
    bmpFile.close();
    return true;
}
#endif
/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 */

#include <stdint.h>
#include <string.h>

#include <array>
#include <map>

extern "C" {
#include "platform.h"
#include "drivers/sdcard.h"
#include "io/asyncfatfs/asyncfatfs.h"
#include "io/asyncfatfs/fat_standard.h"
}

#include "gtest/gtest.h"

static constexpr uint32_t sectorSize = 512;
static constexpr uint32_t rootSector = 66;
static std::map<uint32_t, std::array<uint8_t, sectorSize>> disk;
static bool driverFlushed;
static unsigned flushCalls;
static unsigned writeCalls;
static bool asynchronousWrites;
static afatfsFilePtr_t openedFile;

struct PendingOperation {
    uint32_t sector;
    uint8_t *buffer;
    sdcard_operationCompleteCallback_c callback;
    uint32_t callbackData;
};

static PendingOperation pendingRead;
static PendingOperation pendingWrite;

static void fileOpened(afatfsFilePtr_t file)
{
    openedFile = file;
}

static void completeWrite(bool success)
{
    const auto operation = pendingWrite;
    pendingWrite = {};
    if (success) {
        memcpy(disk[operation.sector].data(), operation.buffer, sectorSize);
    }
    operation.callback(SDCARD_BLOCK_OPERATION_WRITE, operation.sector,
        success ? operation.buffer : nullptr, operation.callbackData);
}

class AsyncFatfsFlushTest : public testing::Test {
protected:
    void SetUp() override
    {
        afatfs_destroy(true);
        disk.clear();
        pendingRead = {};
        pendingWrite = {};
        driverFlushed = true;
        flushCalls = 0;
        writeCalls = 0;
        asynchronousWrites = false;
        openedFile = nullptr;
    }

    void mountFat16()
    {
        // A small valid FAT16 image with an existing freefile. Only touched
        // sectors are materialized; the test never relies on private FS state.
        auto &mbr = disk[0];
        mbr[510] = 0x55;
        mbr[511] = 0xaa;
        mbrPartitionEntry_t partition = {};
        partition.type = MBR_PARTITION_TYPE_FAT16;
        partition.lbaBegin = 1;
        partition.numSectors = 8192;
        memcpy(mbr.data() + 446, &partition, sizeof(partition));

        auto &boot = disk[1];
        fatVolumeID_t volume = {};
        volume.bytesPerSector = sectorSize;
        volume.sectorsPerCluster = 1;
        volume.reservedSectorCount = 1;
        volume.numFATs = 2;
        volume.rootEntryCount = 16;
        volume.totalSectors16 = 8192;
        volume.FATSize16 = 32;
        memcpy(boot.data(), &volume, sizeof(volume));
        boot[510] = 0x55;
        boot[511] = 0xaa;

        fatDirectoryEntry_t freefile = {};
        fat_convertFilenameToFATStyle("FREESPAC.E", reinterpret_cast<uint8_t *>(freefile.filename));
        freefile.attrib = FAT_FILE_ATTRIBUTE_READ_ONLY | FAT_FILE_ATTRIBUTE_SYSTEM;
        freefile.firstClusterLow = 256;
        freefile.fileSize = sectorSize;
        memcpy(disk[rootSector].data(), &freefile, sizeof(freefile));

        afatfs_init();
        for (unsigned i = 0; i < 100 && afatfs_getFilesystemState() != AFATFS_FILESYSTEM_STATE_READY; i++) {
            afatfs_poll();
        }
        ASSERT_EQ(AFATFS_FILESYSTEM_STATE_READY, afatfs_getFilesystemState());
        ASSERT_TRUE(afatfs_sectorCacheInSync());
        flushCalls = 0;
        writeCalls = 0;
    }

    void createFile()
    {
        ASSERT_TRUE(afatfs_fopen("LOG.BFL", "w", fileOpened));
        for (unsigned i = 0; i < 20 && !openedFile; i++) {
            afatfs_poll();
        }
        ASSERT_NE(nullptr, openedFile);
    }
};

TEST_F(AsyncFatfsFlushTest, PollDoesNotForcePartialDriverBatches)
{
    mountFat16();
    driverFlushed = false;
    for (unsigned i = 0; i < 20; i++) {
        afatfs_poll();
    }
    EXPECT_EQ(0U, flushCalls);
    EXPECT_EQ(0U, writeCalls);
}

TEST_F(AsyncFatfsFlushTest, ExplicitFlushWaitsForTheDriverCache)
{
    mountFat16();
    driverFlushed = false;
    EXPECT_FALSE(afatfs_flush());
    EXPECT_EQ(1U, flushCalls);
    driverFlushed = true;
    EXPECT_TRUE(afatfs_flush());
    EXPECT_EQ(2U, flushCalls);
}

TEST_F(AsyncFatfsFlushTest, WaitsForFilesystemWriteBeforeFlushingDriver)
{
    mountFat16();
    createFile();
    asynchronousWrites = true;
    driverFlushed = false;
    ASSERT_FALSE(afatfs_sectorCacheInSync());
    EXPECT_FALSE(afatfs_flush());
    ASSERT_NE(nullptr, pendingWrite.callback);
    EXPECT_EQ(1U, writeCalls);
    EXPECT_EQ(0U, flushCalls);

    // No extra filesystem write or driver sync while the first DMA is pending.
    EXPECT_FALSE(afatfs_flush());
    EXPECT_EQ(1U, writeCalls);
    EXPECT_EQ(0U, flushCalls);
    completeWrite(true);
    EXPECT_TRUE(afatfs_sectorCacheInSync());
    EXPECT_FALSE(afatfs_flush());
    EXPECT_EQ(1U, flushCalls);
    driverFlushed = true;
    EXPECT_TRUE(afatfs_flush());
}

TEST_F(AsyncFatfsFlushTest, FailedFilesystemWriteMustBeRetriedBeforeCompletion)
{
    mountFat16();
    createFile();
    asynchronousWrites = true;
    ASSERT_FALSE(afatfs_flush());
    ASSERT_NE(nullptr, pendingWrite.callback);
    completeWrite(false);
    EXPECT_FALSE(afatfs_sectorCacheInSync());
    EXPECT_FALSE(afatfs_flush());
    EXPECT_EQ(2U, writeCalls);
    EXPECT_EQ(0U, flushCalls);
    ASSERT_NE(nullptr, pendingWrite.callback);
    completeWrite(true);
    EXPECT_TRUE(afatfs_flush());
}

TEST_F(AsyncFatfsFlushTest, CleanUnmountWaitsForDriverWriteBackCache)
{
    mountFat16();
    createFile();
    driverFlushed = false;
    for (unsigned i = 0; i < 20; i++) {
        EXPECT_FALSE(afatfs_destroy(false));
    }
    EXPECT_GT(flushCalls, 0U);
    EXPECT_EQ(AFATFS_FILESYSTEM_STATE_READY, afatfs_getFilesystemState());

    driverFlushed = true;
    bool destroyed = false;
    for (unsigned i = 0; i < 20 && !destroyed; i++) {
        destroyed = afatfs_destroy(false);
    }
    EXPECT_TRUE(destroyed);
    EXPECT_EQ(AFATFS_FILESYSTEM_STATE_UNKNOWN, afatfs_getFilesystemState());
}

extern "C" {
bool sdcard_poll(void)
{
    if (pendingWrite.callback) {
        return false;
    }
    if (pendingRead.callback) {
        const auto operation = pendingRead;
        pendingRead = {};
        memcpy(operation.buffer, disk[operation.sector].data(), sectorSize);
        operation.callback(SDCARD_BLOCK_OPERATION_READ, operation.sector,
            operation.buffer, operation.callbackData);
    }
    return true;
}

bool sdcard_flush(void)
{
    flushCalls++;
    return driverFlushed;
}

bool sdcard_readBlock(uint32_t sector, uint8_t *buffer, sdcard_operationCompleteCallback_c callback, uint32_t callbackData)
{
    if (pendingRead.callback || pendingWrite.callback) {
        return false;
    }
    pendingRead = {sector, buffer, callback, callbackData};
    return true;
}

sdcardOperationStatus_e sdcard_writeBlock(uint32_t sector, uint8_t *buffer, sdcard_operationCompleteCallback_c callback, uint32_t callbackData)
{
    writeCalls++;
    if (asynchronousWrites) {
        pendingWrite = {sector, buffer, callback, callbackData};
        return SDCARD_OPERATION_IN_PROGRESS;
    }
    memcpy(disk[sector].data(), buffer, sectorSize);
    return SDCARD_OPERATION_SUCCESS;
}

sdcardOperationStatus_e sdcard_beginWriteBlocks(uint32_t, uint32_t)
{
    return SDCARD_OPERATION_SUCCESS;
}
}

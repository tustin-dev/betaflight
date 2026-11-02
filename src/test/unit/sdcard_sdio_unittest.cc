/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 */

#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

extern "C" {
#include "platform.h"
#include "pg/bus_spi.h"
#include "drivers/sdcard_impl.h"
#include "drivers/sdmmc_sdio.h"
#include "pg/sdio.h"

sdcard_t sdcard;
sdioConfig_t sdioConfig_System;
SD_CardInfo_t SD_CardInfo;
SD_CardType_t SD_CardType;
}

#include "gtest/gtest.h"

namespace {
timeMs_t nowMs;
bool cardReady;
SD_Error_t writeStartResult, readStartResult, writeStatus, readStatus;
unsigned initCalls, readCalls;
alignas(32) std::array<uint8_t, 512> inputBuffer, readBuffer;

struct Write {
    uint32_t block;
    uint32_t count;
    const uint8_t *buffer;
};
struct Completion {
    sdcardBlockOperation_e operation;
    uint32_t block;
    uint8_t *buffer;
    uint32_t cookie;
};
std::vector<Write> writes;
std::vector<Completion> completions;

void complete(sdcardBlockOperation_e operation, uint32_t block, uint8_t *buffer, uint32_t cookie)
{
    completions.push_back({operation, block, buffer, cookie});
}

class SdcardSdioTest : public testing::Test {
protected:
    void SetUp() override
    {
        nowMs = 0;
        cardReady = true;
        writeStartResult = readStartResult = SD_OK;
        writeStatus = readStatus = SD_OK;
        initCalls = readCalls = 0;
        writes.clear();
        completions.clear();
        inputBuffer.fill(0xa5);
        std::memset(&sdcard, 0, sizeof(sdcard));
        std::memset(&sdioConfig_System, 0, sizeof(sdioConfig_System));
        std::memset(&SD_CardInfo, 0, sizeof(SD_CardInfo));
        SD_CardInfo.CardCapacity = 1024 * 1024;
        SD_CardType = SD_HIGH_CAPACITY;
        sdcardConfig_t config = {};
        config.mode = SDCARD_MODE_SDIO;
        sdcardSdioVTable.sdcard_init(&config, nullptr);
        ASSERT_TRUE(sdcardSdioVTable.sdcard_poll());
    }

    void beginCache(uint32_t block, uint32_t count)
    {
        sdcard.useCache = 1;
        ASSERT_EQ(SDCARD_OPERATION_SUCCESS, sdcardSdioVTable.sdcard_beginWriteBlocks(block, count));
    }

    sdcardOperationStatus_e write(uint32_t block, uint8_t value = 0xa5)
    {
        inputBuffer.fill(value);
        return sdcardSdioVTable.sdcard_writeBlock(block, inputBuffer.data(), complete, 42);
    }

    void finishWrite()
    {
        writeStatus = SD_OK;
        cardReady = true;
        ASSERT_TRUE(sdcardSdioVTable.sdcard_poll());
    }
};
}

TEST_F(SdcardSdioTest, UncachedWriteCompletesInOnePollWhenCardIsReady)
{
    ASSERT_EQ(SDCARD_OPERATION_IN_PROGRESS, write(20));
    EXPECT_FALSE(sdcardSdioVTable.sdcard_flush());
    EXPECT_TRUE(completions.empty());
    finishWrite();
    ASSERT_EQ(1U, completions.size());
    EXPECT_EQ(inputBuffer.data(), completions[0].buffer);
    EXPECT_EQ(20U, completions[0].block);
    EXPECT_EQ(42U, completions[0].cookie);
    EXPECT_TRUE(sdcardSdioVTable.sdcard_flush());
    EXPECT_TRUE(sdcardSdioVTable.sdcard_poll());
    EXPECT_EQ(1U, completions.size());
    EXPECT_EQ(SDCARD_OPERATION_IN_PROGRESS, write(21));
}

TEST_F(SdcardSdioTest, ReleasedSourceBufferDoesNotMeanCardHasCommitted)
{
    ASSERT_EQ(SDCARD_OPERATION_IN_PROGRESS, write(20));
    writeStatus = SD_OK;
    cardReady = false;
    EXPECT_FALSE(sdcardSdioVTable.sdcard_poll());
    ASSERT_EQ(1U, completions.size());
    EXPECT_EQ(inputBuffer.data(), completions[0].buffer);
    EXPECT_FALSE(sdcardSdioVTable.sdcard_flush());
    EXPECT_EQ(SDCARD_OPERATION_BUSY, write(21));
    finishWrite();
    EXPECT_TRUE(sdcardSdioVTable.sdcard_flush());
    EXPECT_EQ(1U, completions.size());
}

TEST_F(SdcardSdioTest, CacheOwnsSixteenCopiesInOneAlignedTransfer)
{
    beginCache(100, 32);
    for (uint32_t i = 0; i < 16; i++) {
        ASSERT_EQ(SDCARD_OPERATION_SUCCESS, write(100 + i, i));
        if (i < 15) EXPECT_TRUE(writes.empty());
    }
    ASSERT_EQ(1U, writes.size());
    EXPECT_EQ(100U, writes[0].block);
    EXPECT_EQ(16U, writes[0].count);
    EXPECT_EQ(0U, reinterpret_cast<uintptr_t>(writes[0].buffer) % 32);
    inputBuffer.fill(0xff);
    for (uint32_t i = 0; i < 16; i++) {
        for (uint32_t j = 0; j < 512; j++) EXPECT_EQ(i, writes[0].buffer[i * 512 + j]);
    }
    EXPECT_TRUE(completions.empty());
    EXPECT_EQ(SDCARD_OPERATION_BUSY, write(116));
    finishWrite();
    EXPECT_TRUE(sdcardSdioVTable.sdcard_flush());
    EXPECT_TRUE(completions.empty());
    EXPECT_EQ(SDCARD_OPERATION_SUCCESS, write(116));
}

TEST_F(SdcardSdioTest, ExplicitFlushCommitsPartialBatchAndWaitsForCard)
{
    beginCache(100, 32);
    ASSERT_EQ(SDCARD_OPERATION_SUCCESS, write(100, 1));
    ASSERT_EQ(SDCARD_OPERATION_SUCCESS, write(101, 2));
    EXPECT_FALSE(sdcardSdioVTable.sdcard_flush());
    ASSERT_EQ(1U, writes.size());
    EXPECT_EQ(100U, writes[0].block);
    EXPECT_EQ(2U, writes[0].count);
    EXPECT_FALSE(sdcardSdioVTable.sdcard_poll()); // DMA is still active.
    writeStatus = SD_OK;
    cardReady = false;
    EXPECT_FALSE(sdcardSdioVTable.sdcard_poll());
    EXPECT_FALSE(sdcardSdioVTable.sdcard_flush());
    EXPECT_EQ(SDCARD_OPERATION_BUSY, write(102));
    finishWrite();
    EXPECT_TRUE(sdcardSdioVTable.sdcard_flush());
}

TEST_F(SdcardSdioTest, ShortAdvertisedChainFlushesOnItsFinalBlock)
{
    beginCache(100, 3);
    ASSERT_EQ(SDCARD_OPERATION_SUCCESS, write(100));
    ASSERT_EQ(SDCARD_OPERATION_SUCCESS, write(101));
    ASSERT_TRUE(writes.empty());
    ASSERT_EQ(SDCARD_OPERATION_SUCCESS, write(102));
    ASSERT_EQ(1U, writes.size());
    EXPECT_EQ(3U, writes[0].count);
    finishWrite();
    EXPECT_EQ(SDCARD_STATE_READY, sdcard.state);
    EXPECT_TRUE(sdcardSdioVTable.sdcard_flush());
}

TEST_F(SdcardSdioTest, NoncontiguousWriteDrainsTheOldBatchBeforeRetry)
{
    beginCache(100, 32);
    ASSERT_EQ(SDCARD_OPERATION_SUCCESS, write(100));
    ASSERT_EQ(SDCARD_OPERATION_SUCCESS, write(101));
    EXPECT_EQ(SDCARD_OPERATION_BUSY, write(400));
    ASSERT_EQ(1U, writes.size());
    EXPECT_EQ(100U, writes[0].block);
    EXPECT_EQ(2U, writes[0].count);
    finishWrite();
    EXPECT_EQ(SDCARD_OPERATION_IN_PROGRESS, write(400));
    ASSERT_EQ(2U, writes.size());
    EXPECT_EQ(400U, writes[1].block);
    EXPECT_EQ(1U, writes[1].count);
}

TEST_F(SdcardSdioTest, NewMultiBlockHintCannotRebasePendingCache)
{
    beginCache(100, 32);
    ASSERT_EQ(SDCARD_OPERATION_SUCCESS, write(100));
    ASSERT_EQ(SDCARD_OPERATION_SUCCESS, write(101));
    EXPECT_EQ(SDCARD_OPERATION_BUSY, sdcardSdioVTable.sdcard_beginWriteBlocks(400, 8));
    ASSERT_EQ(1U, writes.size());
    EXPECT_EQ(100U, writes[0].block);
    finishWrite();
    EXPECT_EQ(SDCARD_OPERATION_SUCCESS, sdcardSdioVTable.sdcard_beginWriteBlocks(400, 8));
    EXPECT_EQ(SDCARD_OPERATION_SUCCESS, write(400));
    EXPECT_FALSE(sdcardSdioVTable.sdcard_flush());
    ASSERT_EQ(2U, writes.size());
    EXPECT_EQ(400U, writes[1].block);
    EXPECT_EQ(1U, writes[1].count);
}

TEST_F(SdcardSdioTest, ReadWaitsForPendingCachedWrites)
{
    beginCache(100, 32);
    ASSERT_EQ(SDCARD_OPERATION_SUCCESS, write(100));
    EXPECT_FALSE(sdcardSdioVTable.sdcard_readBlock(100, readBuffer.data(), complete, 13));
    EXPECT_EQ(0U, readCalls);
    finishWrite();
    ASSERT_TRUE(sdcardSdioVTable.sdcard_readBlock(100, readBuffer.data(), complete, 13));
    readStatus = SD_OK;
    EXPECT_TRUE(sdcardSdioVTable.sdcard_poll());
    ASSERT_EQ(1U, completions.size());
    EXPECT_EQ(SDCARD_BLOCK_OPERATION_READ, completions[0].operation);
    EXPECT_EQ(readBuffer.data(), completions[0].buffer);
}

TEST_F(SdcardSdioTest, IdleFlushDeadlineWorksAcrossClockWrap)
{
    nowMs = UINT32_MAX - 5;
    beginCache(100, 32);
    ASSERT_EQ(SDCARD_OPERATION_SUCCESS, write(100));
    nowMs += 9;
    EXPECT_TRUE(sdcardSdioVTable.sdcard_poll());
    EXPECT_TRUE(writes.empty());
    nowMs++;
    EXPECT_FALSE(sdcardSdioVTable.sdcard_poll());
    ASSERT_EQ(1U, writes.size());
    EXPECT_EQ(1U, writes[0].count);
}

TEST_F(SdcardSdioTest, CacheFailureRetainsAcknowledgedBytesAndFailsClosed)
{
    beginCache(100, 2);
    ASSERT_EQ(SDCARD_OPERATION_SUCCESS, write(100, 11));
    ASSERT_EQ(SDCARD_OPERATION_SUCCESS, write(101, 22));
    ASSERT_EQ(1U, writes.size());
    writeStatus = SD_ERROR;
    EXPECT_FALSE(sdcardSdioVTable.sdcard_poll());
    EXPECT_FALSE(sdcardSdioVTable.sdcard_isFunctional());
    EXPECT_FALSE(sdcardSdioVTable.sdcard_flush());
    EXPECT_EQ(SDCARD_OPERATION_BUSY, write(102));
    EXPECT_EQ(11, writes[0].buffer[0]);
    EXPECT_EQ(22, writes[0].buffer[512]);
    EXPECT_EQ(1U, writes.size());
    EXPECT_TRUE(completions.empty());
}

TEST_F(SdcardSdioTest, RejectedCacheDmaStartRetainsTheOwnedBatch)
{
    beginCache(100, 2);
    ASSERT_EQ(SDCARD_OPERATION_SUCCESS, write(100, 11));
    writeStartResult = SD_ERROR;
    ASSERT_EQ(SDCARD_OPERATION_SUCCESS, write(101, 22));
    ASSERT_EQ(1U, writes.size());
    EXPECT_FALSE(sdcardSdioVTable.sdcard_isFunctional());
    EXPECT_FALSE(sdcardSdioVTable.sdcard_flush());
    EXPECT_EQ(11, writes[0].buffer[0]);
    EXPECT_EQ(22, writes[0].buffer[512]);
}

TEST_F(SdcardSdioTest, UncachedDmaErrorReleasesCallerExactlyOnce)
{
    ASSERT_EQ(SDCARD_OPERATION_IN_PROGRESS, write(100));
    writeStatus = SD_ERROR;
    EXPECT_FALSE(sdcardSdioVTable.sdcard_poll());
    ASSERT_EQ(1U, completions.size());
    EXPECT_EQ(nullptr, completions[0].buffer);
    EXPECT_FALSE(sdcardSdioVTable.sdcard_poll());
    EXPECT_EQ(1U, completions.size());
    EXPECT_FALSE(sdcardSdioVTable.sdcard_flush());
}

TEST_F(SdcardSdioTest, RejectedUncachedStartDoesNotAcquireCallerBuffer)
{
    writeStartResult = SD_ERROR;
    EXPECT_EQ(SDCARD_OPERATION_FAILURE, write(100));
    EXPECT_TRUE(completions.empty());
    EXPECT_FALSE(sdcardSdioVTable.sdcard_isFunctional());
}

TEST_F(SdcardSdioTest, MissingWriteInterruptTimesOutWithoutBlocking)
{
    ASSERT_EQ(SDCARD_OPERATION_IN_PROGRESS, write(100));
    nowMs += SDCARD_TIMEOUT_WRITE_MSEC + 1;
    EXPECT_FALSE(sdcardSdioVTable.sdcard_poll());
    ASSERT_EQ(1U, completions.size());
    EXPECT_EQ(nullptr, completions[0].buffer);
    EXPECT_FALSE(sdcardSdioVTable.sdcard_isFunctional());
}

TEST_F(SdcardSdioTest, ReadErrorReturnsInsteadOfLoopingForever)
{
    ASSERT_TRUE(sdcardSdioVTable.sdcard_readBlock(100, readBuffer.data(), complete, 13));
    readStatus = SD_ERROR;
    EXPECT_FALSE(sdcardSdioVTable.sdcard_poll());
    ASSERT_EQ(1U, completions.size());
    EXPECT_EQ(SDCARD_BLOCK_OPERATION_READ, completions[0].operation);
    EXPECT_EQ(nullptr, completions[0].buffer);
    EXPECT_FALSE(sdcardSdioVTable.sdcard_poll());
    EXPECT_EQ(1U, completions.size());
    EXPECT_EQ(1U, initCalls);
}

TEST_F(SdcardSdioTest, ReadTimeoutReturnsFailureOnce)
{
    ASSERT_TRUE(sdcardSdioVTable.sdcard_readBlock(100, readBuffer.data(), complete, 13));
    nowMs += SDCARD_TIMEOUT_READ_MSEC + 1;
    EXPECT_FALSE(sdcardSdioVTable.sdcard_poll());
    ASSERT_EQ(1U, completions.size());
    EXPECT_EQ(nullptr, completions[0].buffer);
    EXPECT_FALSE(sdcardSdioVTable.sdcard_poll());
    EXPECT_EQ(1U, completions.size());
}

extern "C" {
timeMs_t millis(void) { return nowMs; }
bool sdcard_isInserted(void) { return true; }
bool SD_InitialiseHardware(dmaResource_t *) { return true; }
SD_Error_t SD_Init(void) { initCalls++; return SD_OK; }
bool SD_GetState(void) { return cardReady; }
SD_Error_t SD_GetCardInfo(void) { return SD_OK; }
SD_Error_t SD_CheckWrite(void) { return writeStatus; }
SD_Error_t SD_CheckRead(void) { return readStatus; }
SD_Error_t SD_WriteBlocks_DMA(uint64_t block, uint32_t *buffer, uint32_t size, uint32_t count)
{
    EXPECT_EQ(512U, size);
    EXPECT_EQ(0U, reinterpret_cast<uintptr_t>(buffer) % 32);
    writes.push_back({static_cast<uint32_t>(block), count, reinterpret_cast<uint8_t *>(buffer)});
    writeStatus = SD_BUSY;
    return writeStartResult;
}
SD_Error_t SD_ReadBlocks_DMA(uint64_t, uint32_t *, uint32_t size, uint32_t count)
{
    EXPECT_EQ(512U, size);
    EXPECT_EQ(1U, count);
    readCalls++;
    readStatus = SD_BUSY;
    return readStartResult;
}
}

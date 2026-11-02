/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 */

#include <cstring>

extern "C" {
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

static SD_Error_t writeStatus;
static bool cardReady;
static unsigned stateCalls, callbackCalls;
static uint8_t *completedBuffer;

static void operationComplete(sdcardBlockOperation_e operation, uint32_t block, uint8_t *buffer, uint32_t cookie)
{
    EXPECT_EQ(SDCARD_BLOCK_OPERATION_WRITE, operation);
    EXPECT_EQ(7U, block);
    EXPECT_EQ(42U, cookie);
    callbackCalls++;
    completedBuffer = buffer;
}

class SdcardSdioPollTest : public testing::Test {
protected:
    uint8_t buffer[512] = {};

    void SetUp() override
    {
        std::memset(&sdcard, 0, sizeof(sdcard));
        sdcard.enabled = true;
        sdcard.state = SDCARD_STATE_SENDING_WRITE;
        sdcard.operationStartTime = 1000;
        sdcard.pendingOperation.blockIndex = 7;
        sdcard.pendingOperation.buffer = buffer;
        sdcard.pendingOperation.callback = operationComplete;
        sdcard.pendingOperation.callbackData = 42;
        writeStatus = SD_OK;
        cardReady = true;
        stateCalls = callbackCalls = 0;
        completedBuffer = nullptr;
    }
};

TEST_F(SdcardSdioPollTest, ReadyCardCompletesWriteInOnePoll)
{
    EXPECT_TRUE(sdcardSdioVTable.sdcard_poll());
    EXPECT_EQ(SDCARD_STATE_READY, sdcard.state);
    EXPECT_EQ(1U, stateCalls);
    EXPECT_EQ(1U, callbackCalls);
    EXPECT_EQ(buffer, completedBuffer);

    EXPECT_TRUE(sdcardSdioVTable.sdcard_poll());
    EXPECT_EQ(1U, callbackCalls);
}

TEST_F(SdcardSdioPollTest, BusyCardReturnsWithoutWaitingOrRepeatingCallback)
{
    cardReady = false;
    EXPECT_FALSE(sdcardSdioVTable.sdcard_poll());
    EXPECT_EQ(SDCARD_STATE_WAITING_FOR_WRITE, sdcard.state);
    EXPECT_EQ(1U, stateCalls);
    EXPECT_EQ(1U, callbackCalls);
    EXPECT_EQ(buffer, completedBuffer);

    EXPECT_FALSE(sdcardSdioVTable.sdcard_poll());
    EXPECT_EQ(2U, stateCalls);
    EXPECT_EQ(1U, callbackCalls);

    cardReady = true;
    EXPECT_TRUE(sdcardSdioVTable.sdcard_poll());
    EXPECT_EQ(SDCARD_STATE_READY, sdcard.state);
    EXPECT_EQ(3U, stateCalls);
    EXPECT_EQ(1U, callbackCalls);
}

TEST_F(SdcardSdioPollTest, PendingDmaDoesNotQueryTheCardOrReleaseTheBuffer)
{
    writeStatus = SD_BUSY;
    EXPECT_FALSE(sdcardSdioVTable.sdcard_poll());
    EXPECT_EQ(SDCARD_STATE_SENDING_WRITE, sdcard.state);
    EXPECT_EQ(0U, stateCalls);
    EXPECT_EQ(0U, callbackCalls);
}

TEST_F(SdcardSdioPollTest, MultiBlockChainAdvancesExactlyOnce)
{
    sdcard.multiWriteBlocksRemain = 3;
    sdcard.multiWriteNextBlock = 7;
    EXPECT_TRUE(sdcardSdioVTable.sdcard_poll());
    EXPECT_EQ(SDCARD_STATE_WRITING_MULTIPLE_BLOCKS, sdcard.state);
    EXPECT_EQ(2U, sdcard.multiWriteBlocksRemain);
    EXPECT_EQ(8U, sdcard.multiWriteNextBlock);
    EXPECT_EQ(1U, stateCalls);
    EXPECT_EQ(1U, callbackCalls);

    EXPECT_TRUE(sdcardSdioVTable.sdcard_poll());
    EXPECT_EQ(2U, sdcard.multiWriteBlocksRemain);
    EXPECT_EQ(8U, sdcard.multiWriteNextBlock);
    EXPECT_EQ(1U, callbackCalls);
}

extern "C" {
timeMs_t millis(void) { return 1000; }
bool sdcard_isInserted(void) { return true; }
bool SD_InitialiseHardware(dmaResource_t *) { return true; }
SD_Error_t SD_Init(void) { return SD_OK; }
bool SD_GetState(void) { stateCalls++; return cardReady; }
SD_Error_t SD_GetCardInfo(void) { return SD_OK; }
SD_Error_t SD_CheckRead(void) { return SD_OK; }
SD_Error_t SD_CheckWrite(void) { return writeStatus; }
SD_Error_t SD_ReadBlocks_DMA(uint64_t, uint32_t *, uint32_t, uint32_t) { return SD_OK; }
SD_Error_t SD_WriteBlocks_DMA(uint64_t, uint32_t *, uint32_t, uint32_t) { return SD_OK; }
}

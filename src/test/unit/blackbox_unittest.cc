/*
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <string.h>
#include <algorithm>
#include <string>
#include <vector>

extern "C" {
    #include "platform.h"

    #include "build/debug.h"

    #include "blackbox/blackbox.h"
    #include "blackbox/blackbox_io.h"
    #include "blackbox/blackbox_fielddefs.h"
    #include "common/utils.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"
    #include "pg/motor.h"

    #include "drivers/accgyro/accgyro.h"
    #include "drivers/accgyro/gyro_sync.h"
    #include "drivers/serial.h"

    #include "flight/failsafe.h"
    #include "flight/mixer.h"
    #include "flight/pid.h"

    #include "fc/rc_controls.h"
    #include "fc/rc_modes.h"
    #include "fc/runtime_config.h"

#ifdef USE_SDCARD
    #include "io/asyncfatfs/asyncfatfs.h"
#endif

    #include "io/gps.h"
    #include "io/serial.h"

    #include "rx/rx.h"

    #include "sensors/battery.h"
    #include "sensors/gyro.h"

    extern int16_t blackboxIInterval;
    extern int8_t blackboxPInterval;
    extern pidProfile_t *currentPidProfile;
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

gyroDev_t gyroDev;
static uint32_t testMillis;

#ifdef USE_SDCARD
static std::vector<uint8_t> sdWritten;
static uint32_t sdWriteLimit;
static uint32_t sdLargestRequest;
static unsigned sdWriteCalls;
static unsigned sdCloseCalls;
static bool sdFlushed;
static bool sdCacheInSync;
static bool sdFunctional;
static bool sdFull;
static bool sdDeferClose;
static afatfsCallback_t sdCloseCallback;
static const auto sdFile = reinterpret_cast<afatfsFilePtr_t>(1);
static const auto sdDirectory = reinterpret_cast<afatfsFilePtr_t>(2);

class BlackboxSdcardTest : public testing::Test {
protected:
    pidProfile_t profile = {};

    void SetUp() override
    {
        blackboxConfigMutable()->device = BLACKBOX_DEVICE_SDCARD;
        blackboxConfigMutable()->sample_rate = 0;
        blackboxConfigMutable()->fields_disabled_mask = UINT32_MAX;
        blackboxConfigMutable()->mode = BLACKBOX_MODE_NORMAL;
        targetPidLooptime = 1000;
        currentPidProfile = &profile;
        armingFlags = 0;
        testMillis = 0;
        sdWriteLimit = 0;
        sdLargestRequest = 0;
        sdWriteCalls = 0;
        sdCloseCalls = 0;
        sdFlushed = true;
        sdCacheInSync = true;
        sdFunctional = true;
        sdFull = false;
        sdDeferClose = false;
        sdCloseCallback = nullptr;
        blackboxDeviceEndLog(false);
        blackboxInit();
        ASSERT_TRUE(blackboxDeviceOpen());
        bool ready = false;
        for (int i = 0; i < 10 && !ready; i++) {
            ready = blackboxDeviceBeginLog();
        }
        ASSERT_TRUE(ready);
        sdWritten.clear();
        sdCloseCalls = 0;
    }

    void TearDown() override
    {
        blackboxDeviceEndLog(false);
        blackboxConfigMutable()->device = BLACKBOX_DEVICE_NONE;
        currentPidProfile = nullptr;
        armingFlags = 0;
    }

    void drain()
    {
        for (unsigned i = 0; i < BLACKBOX_SDCARD_BUFFER_SIZE + 1; i++) {
            if (blackboxDeviceReserveWriteBuffer(BLACKBOX_SDCARD_BUFFER_SIZE)) {
                return;
            }
            blackboxDeviceFlush();
        }
        FAIL() << "SD FIFO failed to drain";
    }

    void startRecording()
    {
        sdWriteLimit = 512;
        armingFlags = ARMED;
        for (int i = 0; i < 1000; i++) {
            testMillis++;
            blackboxUpdate(testMillis * 1000);
        }
        drain();
        sdWritten.clear();
    }
};

TEST_F(BlackboxSdcardTest, RetriesZeroAndPartialWritesWithoutLosingBytes)
{
    std::vector<uint8_t> expected;
    for (unsigned i = 0; i < 1200; i++) {
        expected.push_back(i * 17);
        blackboxWrite(expected.back());
    }
    for (int i = 0; i < 5; i++) {
        blackboxDeviceFlush();
    }
    EXPECT_TRUE(sdWritten.empty());
    EXPECT_EQ(5U, sdWriteCalls);
    sdWriteLimit = 31;
    drain();
    EXPECT_EQ(expected, sdWritten);
    EXPECT_LE(sdLargestRequest, 512U);
}

TEST_F(BlackboxSdcardTest, PreservesOrderAcrossRingWrap)
{
    std::vector<uint8_t> expected;
    for (unsigned i = 0; i < BLACKBOX_SDCARD_BUFFER_SIZE - 100; i++) {
        expected.push_back(i * 13);
        blackboxWrite(expected.back());
    }
    sdWriteLimit = 512;
    blackboxDeviceFlush();
    blackboxDeviceFlush();
    for (unsigned i = 0; i < 900; i++) {
        expected.push_back(i * 11);
        blackboxWrite(expected.back());
    }
    sdWriteLimit = 37;
    drain();
    EXPECT_EQ(expected, sdWritten);
}

TEST_F(BlackboxSdcardTest, HeaderBudgetTracksFifoAndNotFilesystemEstimate)
{
    const std::string text(BLACKBOX_SDCARD_BUFFER_SIZE - 9, 'x');
    ASSERT_EQ(static_cast<int>(text.size()), blackboxWriteString(text.c_str()));
    blackboxHeaderBudget = BLACKBOX_MAX_ACCUMULATED_HEADER_BUDGET;
    blackboxReplenishHeaderBudget();
    EXPECT_EQ(9, blackboxHeaderBudget);
    EXPECT_EQ(BLACKBOX_RESERVE_TEMPORARY_FAILURE, blackboxDeviceReserveBufferSpace(10));
    EXPECT_FALSE(blackboxDeviceReserveWriteBuffer(10));
}

TEST_F(BlackboxSdcardTest, ClosesOnlyAfterQueuedBytesAreAcceptedAndIsIdempotent)
{
    const std::string message(700, 'x');
    blackboxWriteString(message.c_str());
    EXPECT_FALSE(blackboxDeviceEndLog(true));
    EXPECT_EQ(0U, sdCloseCalls);
    sdWriteLimit = 400;
    EXPECT_FALSE(blackboxDeviceEndLog(true));
    EXPECT_EQ(0U, sdCloseCalls);
    EXPECT_TRUE(blackboxDeviceEndLog(true));
    EXPECT_EQ(1U, sdCloseCalls);
    EXPECT_EQ(std::vector<uint8_t>(message.begin(), message.end()), sdWritten);
    EXPECT_TRUE(blackboxDeviceEndLog(true));
    EXPECT_EQ(1U, sdCloseCalls);
}

TEST_F(BlackboxSdcardTest, FlushCompleteWaitsForDeviceAndFilesystem)
{
    blackboxWriteString("pending");
    EXPECT_FALSE(blackboxDeviceFlushForceComplete());
    sdWriteLimit = 512;
    sdFlushed = false;
    EXPECT_FALSE(blackboxDeviceFlushForceComplete());
    sdFlushed = true;
    sdCacheInSync = false;
    EXPECT_FALSE(blackboxDeviceFlushForceComplete());
    sdCacheInSync = true;
    EXPECT_TRUE(blackboxDeviceFlushForceComplete());
}

TEST_F(BlackboxSdcardTest, EndLogWaitsForAsynchronousCloseCallback)
{
    blackboxWriteString("tail");
    sdWriteLimit = 512;
    sdDeferClose = true;
    EXPECT_FALSE(blackboxDeviceEndLog(true));
    EXPECT_EQ(1U, sdCloseCalls);
    EXPECT_FALSE(blackboxDeviceEndLog(true));
    EXPECT_EQ(1U, sdCloseCalls);
    EXPECT_FALSE(blackboxDeviceOpen());
    ASSERT_NE(nullptr, sdCloseCallback);
    sdCloseCallback();
    sdCloseCallback = nullptr;
    EXPECT_TRUE(blackboxDeviceEndLog(true));
    EXPECT_EQ(1U, sdCloseCalls);
}

TEST_F(BlackboxSdcardTest, FullQueueSkipsWholeIterationThenResumesWithEventAndIFrame)
{
    startRecording();
    sdWriteLimit = 0;
    const std::string filler(BLACKBOX_SDCARD_BUFFER_SIZE, 'x');
    ASSERT_EQ(static_cast<int>(filler.size()), blackboxWriteString(filler.c_str()));
    for (int i = 0; i < 6000; i++) {
        blackboxLogIteration(1000000U + i * 1000U);
        blackboxAdvanceIterationTimers();
    }
    EXPECT_TRUE(sdWritten.empty());
    sdWriteLimit = 512;
    drain();
    EXPECT_EQ(std::vector<uint8_t>(filler.begin(), filler.end()), sdWritten);
    sdWritten.clear();

    blackboxLogIteration(7000000);
    drain();
    ASSERT_GE(sdWritten.size(), 10U);
    EXPECT_EQ('E', sdWritten[0]);
    EXPECT_EQ(FLIGHT_LOG_EVENT_LOGGING_RESUME, sdWritten[1]);
    size_t cursor = 2;
    const auto readVarint = [&]() {
        uint32_t value = 0;
        for (unsigned shift = 0; cursor < sdWritten.size() && shift < 35; shift += 7) {
            const uint8_t byte = sdWritten[cursor++];
            value |= uint32_t(byte & 0x7f) << shift;
            if (!(byte & 0x80)) break;
        }
        return value;
    };
    const uint32_t resumeIteration = readVarint();
    EXPECT_EQ(7000000U, readVarint());
    ASSERT_LT(cursor + 4, sdWritten.size());
    EXPECT_EQ('S', sdWritten[cursor]);
    cursor += 4; // all mocked slow-state values are zero
    ASSERT_EQ('I', sdWritten[cursor++]);
    EXPECT_EQ(resumeIteration, readVarint());
    EXPECT_EQ(7000000U, readVarint());

    blackboxAdvanceIterationTimers();
    sdWritten.clear();
    blackboxLogIteration(7001000);
    drain();
    ASSERT_FALSE(sdWritten.empty());
    EXPECT_EQ(blackboxShouldLogIFrame() ? 'I' : 'P', sdWritten[0]);
}

TEST_F(BlackboxSdcardTest, StandaloneEventsAreNotPartiallyQueued)
{
    startRecording();
    sdWriteLimit = 0;
    const std::string filler(BLACKBOX_SDCARD_BUFFER_SIZE - 1, 'x');
    blackboxWriteString(filler.c_str());
    flightLogEventData_t event = {};
    event.syncBeep.time = 123;
    blackboxLogEvent(FLIGHT_LOG_EVENT_SYNC_BEEP, &event);
    sdWriteLimit = 512;
    drain();
    EXPECT_EQ(std::vector<uint8_t>(filler.begin(), filler.end()), sdWritten);
}

TEST_F(BlackboxSdcardTest, FailedCardDoesNotHangShutdownWithQueuedBytes)
{
    startRecording();
    sdWriteLimit = 0;
    blackboxWriteString("pending");
    blackboxFinish();
    sdFunctional = false;
    sdFlushed = false;
    blackboxUpdate(2000000);
    EXPECT_TRUE(blackboxMayEditConfig());
    EXPECT_TRUE(blackboxDeviceReserveWriteBuffer(BLACKBOX_SDCARD_BUFFER_SIZE));
    EXPECT_TRUE(sdWritten.empty());
    EXPECT_FALSE(blackboxDeviceFlushForceComplete());
}

TEST_F(BlackboxSdcardTest, FullFilesystemDoesNotHangShutdownWithQueuedBytes)
{
    startRecording();
    sdWriteLimit = 0;
    blackboxWriteString("pending");
    blackboxFinish();
    sdFull = true;
    sdFlushed = false;
    blackboxUpdate(2000000);
    EXPECT_TRUE(blackboxMayEditConfig());
    EXPECT_TRUE(blackboxDeviceReserveWriteBuffer(BLACKBOX_SDCARD_BUFFER_SIZE));
    EXPECT_TRUE(sdWritten.empty());
    EXPECT_FALSE(blackboxDeviceFlushForceComplete());
}
#endif

TEST(BlackboxTest, TestInitIntervals)
{
    blackboxConfigMutable()->sample_rate = 4; // sample_rate = PID loop frequency / 16
    // 250Hz PIDloop
    targetPidLooptime = 4000;
    blackboxInit();
    EXPECT_EQ(8, blackboxIInterval);
    EXPECT_EQ(0, blackboxPInterval);
    EXPECT_EQ(2048, blackboxSInterval);

    // 500Hz PIDloop
    targetPidLooptime = 2000;
    blackboxInit();
    EXPECT_EQ(16, blackboxIInterval);
    EXPECT_EQ(16, blackboxPInterval);
    EXPECT_EQ(4096, blackboxSInterval);

    blackboxConfigMutable()->sample_rate = 1; // sample_rate = PID loop frequency / 2
    // 2kHz PIDloop
    targetPidLooptime = 500;
    blackboxInit();
    EXPECT_EQ(64, blackboxIInterval);
    EXPECT_EQ(2, blackboxPInterval);
    EXPECT_EQ(16384, blackboxSInterval);

    blackboxConfigMutable()->sample_rate = 2;
    // 4kHz PIDloop
    targetPidLooptime = 250;
    blackboxInit();
    EXPECT_EQ(128, blackboxIInterval);
    EXPECT_EQ(4, blackboxPInterval);
    EXPECT_EQ(32768, blackboxSInterval);

    blackboxConfigMutable()->sample_rate = 3;
    // 8kHz PIDloop
    targetPidLooptime = 125;
    blackboxInit();
    EXPECT_EQ(256, blackboxIInterval);
    EXPECT_EQ(8, blackboxPInterval);
    EXPECT_EQ(65536, blackboxSInterval);
}

TEST(BlackboxTest, Test_500Hz)
{
    blackboxConfigMutable()->sample_rate = 0;
    // 500Hz PIDloop
    targetPidLooptime = 2000;
    blackboxInit();
    EXPECT_TRUE(blackboxShouldLogIFrame());
    EXPECT_TRUE(blackboxShouldLogPFrame());
    for (int ii = 0; ii < 15; ++ii) {
        blackboxAdvanceIterationTimers();
        EXPECT_FALSE(blackboxShouldLogIFrame());
        EXPECT_TRUE(blackboxShouldLogPFrame());
    }
    blackboxAdvanceIterationTimers();
    EXPECT_TRUE(blackboxShouldLogIFrame());
    EXPECT_TRUE(blackboxShouldLogPFrame());
}

TEST(BlackboxTest, Test_1kHz)
{
    blackboxConfigMutable()->sample_rate = 0;
    // 1kHz PIDloop
    targetPidLooptime = 1000;
    blackboxInit();
    EXPECT_TRUE(blackboxShouldLogIFrame());
    EXPECT_TRUE(blackboxShouldLogPFrame());
    EXPECT_EQ(0, blackboxSlowFrameIterationTimer);
    EXPECT_FALSE(blackboxSlowFrameIterationTimer >= blackboxSInterval);
    blackboxSlowFrameIterationTimer = blackboxSInterval;
    EXPECT_TRUE(writeSlowFrameIfNeeded());
    EXPECT_EQ(0, blackboxSlowFrameIterationTimer);

    for (int ii = 0; ii < 31; ++ii) {
        blackboxAdvanceIterationTimers();
        EXPECT_FALSE(blackboxShouldLogIFrame());
        EXPECT_TRUE(blackboxShouldLogPFrame());
        EXPECT_EQ(ii + 1, blackboxSlowFrameIterationTimer);
        EXPECT_FALSE(writeSlowFrameIfNeeded());
    }
    blackboxAdvanceIterationTimers();
    EXPECT_TRUE(blackboxShouldLogIFrame());
    EXPECT_TRUE(blackboxShouldLogPFrame());
}

TEST(BlackboxTest, Test_2kHz)
{
    blackboxConfigMutable()->sample_rate = 1;
    // 2kHz PIDloop
    targetPidLooptime = 500;
    blackboxInit();
    EXPECT_EQ(64, blackboxIInterval);
    EXPECT_EQ(2, blackboxPInterval);
    EXPECT_TRUE(blackboxShouldLogIFrame());
    EXPECT_TRUE(blackboxShouldLogPFrame());
    blackboxAdvanceIterationTimers();
    EXPECT_FALSE(blackboxShouldLogIFrame());
    EXPECT_FALSE(blackboxShouldLogPFrame());

    for (int ii = 0; ii < 31; ++ii) {
        blackboxAdvanceIterationTimers();
        EXPECT_FALSE(blackboxShouldLogIFrame());
        EXPECT_TRUE(blackboxShouldLogPFrame());

        blackboxAdvanceIterationTimers();
        EXPECT_FALSE(blackboxShouldLogIFrame());
        EXPECT_FALSE(blackboxShouldLogPFrame());
    }

    blackboxAdvanceIterationTimers();
    EXPECT_TRUE(blackboxShouldLogIFrame());
    EXPECT_TRUE(blackboxShouldLogPFrame());

    blackboxAdvanceIterationTimers();
    EXPECT_FALSE(blackboxShouldLogIFrame());
    EXPECT_FALSE(blackboxShouldLogPFrame());
}

TEST(BlackboxTest, Test_8kHz)
{
    blackboxConfigMutable()->sample_rate = 3;
    // 8kHz PIDloop
    targetPidLooptime = 125;
    blackboxInit();
    EXPECT_TRUE(blackboxShouldLogIFrame());
    EXPECT_TRUE(blackboxShouldLogPFrame());
    EXPECT_EQ(0, blackboxSlowFrameIterationTimer);
    EXPECT_FALSE(blackboxSlowFrameIterationTimer >= blackboxSInterval);
    blackboxSlowFrameIterationTimer = blackboxSInterval;
    EXPECT_TRUE(writeSlowFrameIfNeeded());
    EXPECT_EQ(0, blackboxSlowFrameIterationTimer);

    for (int ii = 0; ii < 255; ++ii) {
        blackboxAdvanceIterationTimers();
        EXPECT_FALSE(blackboxShouldLogIFrame());
        EXPECT_EQ(ii + 1, blackboxSlowFrameIterationTimer);
        EXPECT_FALSE(writeSlowFrameIfNeeded());
    }
    blackboxAdvanceIterationTimers();
    EXPECT_TRUE(blackboxShouldLogIFrame());
    EXPECT_TRUE(blackboxShouldLogPFrame());
}

TEST(BlackboxTest, Test_zero_p_interval)
{
    blackboxConfigMutable()->sample_rate = 4;
    // 250Hz PIDloop
    targetPidLooptime = 4000;
    blackboxInit();
    EXPECT_EQ(8, blackboxIInterval);
    EXPECT_EQ(0, blackboxPInterval);
    EXPECT_TRUE(blackboxShouldLogIFrame());
    EXPECT_FALSE(blackboxShouldLogPFrame());

    for (int ii = 0; ii < 7; ++ii) {
        blackboxAdvanceIterationTimers();
        EXPECT_FALSE(blackboxShouldLogIFrame());
        EXPECT_FALSE(blackboxShouldLogPFrame());
    }
    blackboxAdvanceIterationTimers();
    EXPECT_TRUE(blackboxShouldLogIFrame());
    EXPECT_FALSE(blackboxShouldLogPFrame());
}

TEST(BlackboxTest, Test_CalculatePDenom)
{
    blackboxConfigMutable()->sample_rate = 0;
    // note I-frame is logged every 32ms regardless of PIDloop rate

    // 1kHz PIDloop
    targetPidLooptime = 1000;
    blackboxInit();
    EXPECT_EQ(32, blackboxIInterval);
    EXPECT_EQ(32, blackboxCalculatePDenom(1, 1)); // 1kHz logging
    EXPECT_EQ(16, blackboxCalculatePDenom(1, 2));
    EXPECT_EQ(8, blackboxCalculatePDenom(1, 4));

    // 2kHz PIDloop
    targetPidLooptime = 500;
    blackboxInit();
    EXPECT_EQ(64, blackboxIInterval);
    EXPECT_EQ(64, blackboxCalculatePDenom(1, 1));
    EXPECT_EQ(32, blackboxCalculatePDenom(1, 2)); // 1kHz logging
    EXPECT_EQ(16, blackboxCalculatePDenom(1, 4));

    // 4kHz PIDloop
    targetPidLooptime = 250;
    blackboxInit();
    EXPECT_EQ(128, blackboxIInterval);
    EXPECT_EQ(128, blackboxCalculatePDenom(1, 1));
    EXPECT_EQ(64, blackboxCalculatePDenom(1, 2));
    EXPECT_EQ(32, blackboxCalculatePDenom(1, 4)); // 1kHz logging
    EXPECT_EQ(16, blackboxCalculatePDenom(1, 8));

    // 8kHz PIDloop
    targetPidLooptime = 125;
    blackboxInit();
    EXPECT_EQ(256, blackboxIInterval);
    EXPECT_EQ(256, blackboxCalculatePDenom(1, 1));
    EXPECT_EQ(128, blackboxCalculatePDenom(1, 2));
    EXPECT_EQ(64, blackboxCalculatePDenom(1, 4));
    EXPECT_EQ(32, blackboxCalculatePDenom(1, 8)); // 1kHz logging
    EXPECT_EQ(16, blackboxCalculatePDenom(1, 16));
}

TEST(BlackboxTest, Test_CalculateRates)
{
    // 1kHz PIDloop
    targetPidLooptime = 1000;
    blackboxConfigMutable()->sample_rate = 0;
    blackboxInit();
    EXPECT_EQ(32, blackboxIInterval);
    EXPECT_EQ(1, blackboxPInterval);
    EXPECT_EQ(1, blackboxGetRateDenom());

    blackboxConfigMutable()->sample_rate = 1;
    blackboxInit();
    EXPECT_EQ(32, blackboxIInterval);
    EXPECT_EQ(2, blackboxPInterval);
    EXPECT_EQ(2, blackboxGetRateDenom());

    blackboxConfigMutable()->sample_rate = 2;
    blackboxInit();
    EXPECT_EQ(32, blackboxIInterval);
    EXPECT_EQ(4, blackboxPInterval);
    EXPECT_EQ(4, blackboxGetRateDenom());


    // 8kHz PIDloop
    targetPidLooptime = 125;
    blackboxConfigMutable()->sample_rate = 3; // 1kHz logging
    blackboxInit();
    EXPECT_EQ(256, blackboxIInterval);
    EXPECT_EQ(8, blackboxPInterval);
    EXPECT_EQ(8, blackboxGetRateDenom());

    blackboxConfigMutable()->sample_rate = 4; // 500Hz logging
    blackboxInit();
    EXPECT_EQ(256, blackboxIInterval);
    EXPECT_EQ(16, blackboxPInterval);
    EXPECT_EQ(16, blackboxGetRateDenom());

    blackboxConfigMutable()->sample_rate = 2; // 2kHz logging
    blackboxInit();
    EXPECT_EQ(256, blackboxIInterval);
    EXPECT_EQ(4, blackboxPInterval);
    EXPECT_EQ(4, blackboxGetRateDenom());

    blackboxConfigMutable()->sample_rate = 1; // 4kHz logging
    blackboxInit();
    EXPECT_EQ(256, blackboxIInterval);
    EXPECT_EQ(2, blackboxPInterval);
    EXPECT_EQ(2, blackboxGetRateDenom());

    blackboxConfigMutable()->sample_rate = 0; // 8kHz logging
    blackboxInit();
    EXPECT_EQ(256, blackboxIInterval);
    EXPECT_EQ(1, blackboxPInterval);
    EXPECT_EQ(1, blackboxGetRateDenom());

    // 0.126 PIDloop = 7.94kHz
    targetPidLooptime = 126;
    blackboxConfigMutable()->sample_rate = 3; // 0.992kHz logging
    blackboxInit();
    EXPECT_EQ(253, blackboxIInterval);
    EXPECT_EQ(8, blackboxPInterval);
    EXPECT_EQ(8, blackboxGetRateDenom());

}


// STUBS
extern "C" {

PG_REGISTER(flight3DConfig_t, flight3DConfig, PG_MOTOR_3D_CONFIG, 0);
PG_REGISTER(mixerConfig_t, mixerConfig, PG_MIXER_CONFIG, 0);
PG_REGISTER(motorConfig_t, motorConfig, PG_MOTOR_CONFIG, 0);
PG_REGISTER(batteryConfig_t, batteryConfig, PG_BATTERY_CONFIG, 0);
PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);
PG_REGISTER_ARRAY(modeActivationCondition_t, MAX_MODE_ACTIVATION_CONDITION_COUNT, modeActivationConditions, PG_MODE_ACTIVATION_PROFILE, 0);

uint8_t armingFlags;
uint8_t stateFlags;
const uint32_t baudRates[] = {0, 9600, 19200, 38400, 57600, 115200, 230400, 250000,
        400000, 460800, 500000, 921600, 1000000, 1500000, 2000000, 2470000}; // see baudRate_e
uint8_t debugMode = 0;
int16_t debug[DEBUG16_VALUE_COUNT];
extern int32_t blackboxHeaderBudget;
gpsSolutionData_t gpsSol;
gpsLocation_t GPS_home_llh;

uint32_t gpsDateTimeToEpoch(const gpsDateTime_t *) { return 0; }

gyro_t gyro;

float motor_disarmed[MAX_SUPPORTED_MOTORS];
struct pidProfile_s;
struct pidProfile_s *currentPidProfile;
uint32_t targetPidLooptime;

boxBitmask_t rcModeActivationMask;

void mspSerialAllocatePorts(void) {}
uint32_t getArmingBeepTimeMicros(void) {return 0;}
uint16_t getBatteryVoltageLatest(void) {return 0;}
bool hasServos(void) { return false; }
uint8_t getMotorCount(void) {return 4;}
bool areMotorsRunning(void) { return false; }
bool IS_RC_MODE_ACTIVE(boxId_e) {return false;}
bool isModeActivationConditionPresent(boxId_e) {return false;}
uint32_t millis(void) {return testMillis;}
bool sensors(uint32_t) {return false;}
void serialWrite(serialPort_t *, uint8_t) {}
uint32_t serialTxBytesFree(const serialPort_t *) {return 0;}
bool isSerialTransmitBufferEmpty(const serialPort_t *) {return false;}
bool featureIsEnabled(uint32_t) {return false;}
void mspSerialReleasePortIfAllocated(serialPort_t *) {}
const serialPortConfig_t *findSerialPortConfig(serialPortFunction_e ) {return NULL;}
serialPort_t *findSharedSerialPort(uint16_t , serialPortFunction_e ) {return NULL;}
serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e, serialReceiveCallbackPtr, void *, uint32_t, portMode_e, portOptions_e) {return NULL;}
void closeSerialPort(serialPort_t *) {}
portSharing_e determinePortSharing(const serialPortConfig_t *, serialPortFunction_e ) {return PORTSHARING_UNUSED;}
failsafePhase_e failsafePhase(void) {return FAILSAFE_IDLE;}
bool rxAreFlightChannelsValid(void) {return false;}
bool isRxReceivingSignal(void) {return false;}
bool isRssiConfigured(void) {return false;}
float getMotorOutputLow(void) {return 0.0;}
float getMotorOutputHigh(void) {return 0.0;}

#ifdef USE_SDCARD
bool afatfs_mkdir(const char *, afatfsFileCallback_t complete) { complete(sdDirectory); return true; }
void afatfs_findFirst(afatfsFilePtr_t, afatfsFinder_t *) {}
void afatfs_findLast(afatfsFilePtr_t) {}
afatfsOperationStatus_e afatfs_findNext(afatfsFilePtr_t, afatfsFinder_t *, fatDirectoryEntry_t **entry) {
    *entry = nullptr;
    return AFATFS_OPERATION_SUCCESS;
}
bool fat_isDirectoryEntryTerminator(fatDirectoryEntry_t *) { return true; }
bool afatfs_chdir(afatfsFilePtr_t) { return true; }
bool afatfs_fopen(const char *, const char *, afatfsFileCallback_t complete) { complete(sdFile); return true; }
bool afatfs_fclose(afatfsFilePtr_t file, afatfsCallback_t callback) {
    EXPECT_NE(nullptr, file);
    if (file == sdFile) sdCloseCalls++;
    if (callback) {
        if (sdDeferClose) sdCloseCallback = callback;
        else callback();
    }
    return true;
}
bool afatfs_funlink(afatfsFilePtr_t, afatfsCallback_t callback) { if (callback) callback(); return true; }
uint32_t afatfs_fwrite(afatfsFilePtr_t file, const uint8_t *buffer, uint32_t len) {
    EXPECT_EQ(sdFile, file);
    sdWriteCalls++;
    sdLargestRequest = std::max(sdLargestRequest, len);
    const uint32_t accepted = std::min(sdWriteLimit, len);
    sdWritten.insert(sdWritten.end(), buffer, buffer + accepted);
    return accepted;
}
afatfsFilesystemState_e afatfs_getFilesystemState(void) { return AFATFS_FILESYSTEM_STATE_READY; }
bool afatfs_isFull(void) { return sdFull; }
uint32_t afatfs_getFreeBufferSpace(void) { return 4096; }
bool afatfs_flush(void) { return sdFlushed; }
bool afatfs_sectorCacheInSync(void) { return sdCacheInSync; }
bool sdcard_isInserted(void) { return true; }
bool sdcard_isFunctional(void) { return sdFunctional; }
#endif
}

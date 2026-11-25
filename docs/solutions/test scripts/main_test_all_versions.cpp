// PES Board multi-version concurrent hardware smoke test (Nucleo F446RE).
// Copy to src/main.cpp temporarily. See PESBoard_Hardwaretest.md before powering.
#include "mbed.h"
#include "PESBoardPinMap.h"
#include "DCMotor.h"
#include "Servo.h"
#include <cmath>
#include <cstdio>
#include <cstdint>
#if defined(PESBOARD_V2026)
#include "LSM6DSOX.h"
#include "Stepper.h"
#elif defined(PESBOARD_V2025)
#include "MPU6500_I2C.h"
#else
#include "LSM9DS1.h"
#endif

// Opt-in ONLY with correctly wired, unloaded stepper motors and current limits.
#ifndef PES_TEST_ENABLE_STEPPER_DRIVERS
#define PES_TEST_ENABLE_STEPPER_DRIVERS 0
#endif
// PH1 on V2025: verify physical routing and active-low fault polarity first.
#ifndef PES_TEST_V25_FAULT_ACTIVE_LOW
#define PES_TEST_V25_FAULT_ACTIVE_LOW 0
#endif

namespace {
constexpr float KN = 180.0f / 12.0f;
constexpr float VMOT = 12.0f;
constexpr float CPR = 20.0f;
constexpr int PERIOD_US = 50;
#if defined(PESBOARD_V2026)
constexpr int SAMPLES = 100; // 5 s minimum: allow 12 rev reversal with ramp
#else
constexpr int SAMPLES = 60;
#endif
constexpr int DELAY_MS = 50;
constexpr uint8_t patterns[] = {0x00,0xff,0x55,0xaa,0xa5,0x5a,0x96,0x69,0x3c,0xc3};
struct Axes { float ax, ay, az, gx, gy, gz, mx, my, mz; };
struct Range {
    float lo = 1.0e30f, hi = -1.0e30f;
    void add(float x) { if (x < lo) lo = x; if (x > hi) hi = x; }
};
float norm(float x, float y, float z) { return sqrtf(x*x+y*y+z*z); }
const char* verdict(bool ok) { return ok ? "PASS" : "FAIL"; }

#if defined(PESBOARD_V2025)
// V2025 STEP pins are not supported by the current TIM10/TIM11 Stepper class.
// Deliberately test the physical GPIO with independent software square waves.
class TestPulse {
public:
    explicit TestPulse(PinName pin) : out(pin, 0) {}
    void start(int half_us) {
        stop(); edges = 0;
        ticker.attach(callback(this, &TestPulse::toggle), std::chrono::microseconds(half_us));
    }
    void stop() { ticker.detach(); out = 0; }
    uint32_t count() const { return edges; }
private:
    DigitalOut out;
    Ticker ticker;
    volatile uint32_t edges = 0;
    void toggle() { out = !out.read(); if (out.read()) ++edges; }
};
#endif
}

int main()
{
    // Keep power stages disabled during initialisation and operator arming.
    DigitalOut dc_enable(PB_ENABLE_DCMOTORS, DISABLE_DCMOTORS);
#if PB_HAS_STEPPERS
    DigitalOut step_enable(PB_ENABLE_STEPPERMOTORS, DISABLE_STEPPERMOTORS);
    DigitalOut step_reset(PB_RESET_STEPPERMOTORS, RESET_STEPPERMOTORS);
#if defined(PESBOARD_V2025)
    DigitalOut dir1(PB_DIR_STEPPERMOTOR1, 0), dir2(PB_DIR_STEPPERMOTOR2, 0);
#endif
#endif
#if defined(PESBOARD_V2026) || (defined(PESBOARD_V2025) && PES_TEST_V25_FAULT_ACTIVE_LOW)
    DigitalIn fault(PB_DC_MOT_STATE, PullUp);
    auto faultActive = [&]() { return fault.read() == 0; };
#else
    auto faultActive = []() { return false; };
#endif
#if defined(PESBOARD_V2026)
    printf("\nPES BOARD V2026\n");
#elif defined(PESBOARD_V2025)
    printf("\nPES BOARD V2025\n");
#else
    printf("\nPES BOARD V2019/V2023\n");
#endif
    printf("Verify correct board flag, unloaded motors, 12 V, SD removed, MOSI-MISO jumper.\n");
    printf("ADC A0..A3 floating. Do not press blue button on V2025 (D3).\n");
    printf("Send uppercase T via ST-LINK terminal to initialise and start.\n");
    while (getchar() != 'T') ThisThread::sleep_for(20ms);

#if PB_DC_CONTROL_PWM_DIR
    DCMotor m1(PB_PWM_M1, PB_DIR_M1, PB_ENC_A_M1, PB_ENC_B_M1, 156.0f, KN, VMOT, CPR);
    DCMotor m2(PB_PWM_M2, PB_DIR_M2, PB_ENC_A_M2, PB_ENC_B_M2, 31.0f, KN, VMOT, CPR);
    DCMotor m3(PB_PWM_M3, PB_DIR_M3, PB_ENC_A_M3, PB_ENC_B_M3, 78.125f, KN, VMOT, CPR);
#else
    DCMotor m1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, 156.0f, KN, VMOT, CPR);
    DCMotor m2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, 31.0f, KN, VMOT, CPR);
    DCMotor m3(PB_PWM_M3, PB_ENC_A_M3, PB_ENC_B_M3, 78.125f, KN, VMOT, CPR);
#endif
    m1.setFastPWMPeriod_mus(PERIOD_US); m2.setFastPWMPeriod_mus(PERIOD_US); m3.setFastPWMPeriod_mus(PERIOD_US);
    m1.disableMotionPlanner(); m1.setMaxVelocity(0.40f); m1.setVelocity(0);
    m2.enableMotionPlanner(); m2.setMaxVelocity(0.80f); m2.setMaxAcceleration(2.0f); m2.setVelocity(0);
    m3.enableMotionPlanner(); m3.setMaxVelocity(0.40f); m3.setMaxAcceleration(1.0f); m3.setRotation(m3.getRotation());
#if defined(PESBOARD_V2026)
    m1.setVelocityCntrlIntegratorLimitsPercent(80.0f);
    m2.setVelocityCntrlIntegratorLimitsPercent(80.0f);
    m3.setVelocityCntrlIntegratorLimitsPercent(80.0f);
#endif
    const float origin3 = m3.getRotation();

    // Use actual Servo library: software PWM on old boards, hardware on V2026.
    // Calibration values are fractions of the 20 ms frame.
    Servo s0(PB_D0, 0.0625f, 0.0875f), s1(PB_D1, 0.0625f, 0.0875f);
    Servo s2(PB_D2, 0.0625f, 0.0875f), s3(PB_D3, 0.0625f, 0.0875f);
    Servo* servos[] = {&s0,&s1,&s2,&s3};
    AnalogIn a0(PB_A0), a1(PB_A1), a2(PB_A2), a3(PB_A3);
    AnalogIn* adc[] = {&a0,&a1,&a2,&a3};
    Range adcRange[4], accRange, gyroRange, magRange;

    // Keep CS HIGH for MOSI-MISO jumper tests. No card or other SPI slave attached.
    DigitalOut sd_cs(PB_SD_CS, 1);
    SPI sd(PB_SD_MOSI, PB_SD_MISO, PB_SD_SCK);
    sd.format(8,0); sd.frequency(4000000);

#if defined(PESBOARD_V2026)
    SPI imu_spi(PB_IMU_MOSI, PB_IMU_MISO, PB_IMU_SCK);
    DigitalOut imu_cs(PB_IMU_CS, 1);
    LSM6DSOX imu(imu_spi, imu_cs);
    bool imuInit = imu.begin();
    printf("LSM6DSOX WHO_AM_I=0x%02X, init=%s\n", unsigned(imu.whoAmI()), verdict(imuInit));
    auto readImu = [&](Axes& a) {
        LSM6DSOX::Sample s;
        if (!imu.readSample(s)) return false;
        a = {s.accX,s.accY,s.accZ,s.gyroX,s.gyroY,s.gyroZ,0,0,0};
        return true;
    };
#elif defined(PESBOARD_V2025)
    I2C imu_i2c(PB_IMU_SDA, PB_IMU_SCL);
    MPU6500_I2C imu(imu_i2c);
    bool imuInit = imu.init() && imu.configuration() && imu.testConnection();
    auto readImu = [&](Axes& a) {
        imu.readAccAll(); imu.readGyroAll();
        a = {imu.getAccX(),imu.getAccY(),imu.getAccZ(),imu.getGyroX(),imu.getGyroY(),imu.getGyroZ(),0,0,0};
        return true;
    };
#else
    LSM9DS1 imu(PB_IMU_SDA, PB_IMU_SCL);
    const uint16_t id = imu.begin();
    bool imuInit = id == 0x683D;
    printf("LSM9DS1 WHO_AM_I=0x%04X, init=%s\n", unsigned(id), verdict(imuInit));
    auto readImu = [&](Axes& a) {
        imu.updateAcc(); imu.updateGyro(); imu.updateMag();
        a = {imu.readAccX(),imu.readAccY(),imu.readAccZ(),imu.readGyroX(),imu.readGyroY(),imu.readGyroZ(),imu.readMagX(),imu.readMagY(),imu.readMagZ()};
        return true;
    };
#endif

#if defined(PESBOARD_V2026)
    constexpr int stepsPerRev = 3200;
    Stepper step1(PB_STEP_STEPPERMOTOR1, PB_DIR_STEPPERMOTOR1, stepsPerRev);
    Stepper step2(PB_STEP_STEPPERMOTOR2, PB_DIR_STEPPERMOTOR2, stepsPerRev);
    Stepper* steppers[] = {&step1, &step2};
    for (auto* st : steppers) {
        st->setInternalRotation(0.0f);
        st->setMaxVelocity(5.0f);
        st->setMaxAcceleration(5.0f);
        st->enableMotionPlanner();
    }
    bool stepPass = true;
    auto stopSteps = [&]() {
        for (auto* st : steppers) {
            st->disableMotionPlanner();
            st->setVelocity(0.0f);
        }
    };
#elif defined(PESBOARD_V2025)
    TestPulse step1(PB_STEP_STEPPERMOTOR1), step2(PB_STEP_STEPPERMOTOR2);
    auto startSteps = [&]() { step1.start(10000); step2.start(6250); };
    auto stopSteps = [&]() { step1.stop(); step2.stop(); };
#endif

    uint32_t spiCount=0, spiErrors=0, imuCount=0, imuErrors=0, adcCount=0;
    bool motorPass[3] = {true,true,true};
    bool aborted = false;
    const float signs[] = {1,-1,0};
    const float pulses[3][4] = {{0,0.3f,0.7f,1},{1,0.7f,0.3f,0},{0.5f,0.5f,0.5f,0.5f}};
    printf("\nSend uppercase R to energise DC motors and start 3 phases.\n");
    while (getchar() != 'R') ThisThread::sleep_for(20ms);
    dc_enable = ENABLE_DCMOTORS;
    ThisThread::sleep_for(20ms);
    if (faultActive()) { dc_enable=DISABLE_DCMOTORS; printf("DC FAULT ON ENABLE\n"); aborted = true; }
    if (!aborted) {
        for (auto* s : servos) s->enable(0.5f);
#if PB_HAS_STEPPERS
        step_reset = !RESET_STEPPERMOTORS;
        ThisThread::sleep_for(5ms);
#if PES_TEST_ENABLE_STEPPER_DRIVERS
        step_enable = ENABLE_STEPPERMOTORS;
#endif
#endif
    }
    for (int p=0; p<3 && !aborted; ++p) {
        const float v1=0.20f*signs[p], v2=0.50f*signs[p], r3=origin3+0.125f*signs[p];
        const float r1start=m1.getRotation(), r2start=m2.getRotation(), r3start=m3.getRotation();
        m1.setVelocity(v1); m2.setVelocity(v2); m3.setRotation(r3);
        for (int k=0;k<4;++k) servos[k]->setPulseWidth(pulses[p][k]);
#if defined(PESBOARD_V2026)
        const int target = static_cast<int>(19200 * signs[p]);
        step1.setSteps(target, 5.0f);
        step2.setSteps(-target, 5.0f);
        printf("STEP targets: %d / %d\n", target, -target);
#elif defined(PESBOARD_V2025)
        stopSteps(); dir1=(p==1); dir2=(p!=1);
        ThisThread::sleep_for(2ms);
        startSteps();
#endif
        printf("\nPHASE %d: M1=%+.2f M2=%+.2f rev/s, M3=%+.1f deg; servo us:",p+1,v1,v2,(r3-origin3)*360);
        for (int k=0;k<4;++k) printf(" %.0f",1250+500*pulses[p][k]);
        printf("\n");
        int steadyGood[3]={0,0,0};
        for (int i=0;i<SAMPLES;++i) {
            if (faultActive()) { dc_enable=DISABLE_DCMOTORS; printf("DC FAULT\n"); aborted=true; break; }
            float v[4];
            for (int k=0;k<4;++k) { v[k]=adc[k]->read()*3.3f; adcRange[k].add(v[k]); }
            ++adcCount;
            for (int n=0;n<10;++n) for (uint8_t tx:patterns) {
                uint8_t rx=static_cast<uint8_t>(sd.write(tx));
                ++spiCount; if (rx!=tx) ++spiErrors;
            }
            Axes a{};
            bool readOk=imuInit && readImu(a);
            float an=norm(a.ax,a.ay,a.az), gn=norm(a.gx,a.gy,a.gz), mn=norm(a.mx,a.my,a.mz);
            bool sampleOk=readOk && std::isfinite(an) && std::isfinite(gn) && an>1.0f && an<35.0f && gn<40.0f;
#if defined(PESBOARD_V2019_2023)
            sampleOk=sampleOk && std::isfinite(mn) && mn>0.001f;
#endif
            ++imuCount;
            if (!sampleOk) ++imuErrors;
            if (readOk) { accRange.add(an); gyroRange.add(gn); magRange.add(mn); }
            const float x1=m1.getVelocity(), x2=m2.getVelocity(), x3=m3.getRotation();
            // Conservative abort on non-finite feedback, excessive motion or wrong direction.
            if (!std::isfinite(x1)||!std::isfinite(x2)||!std::isfinite(x3)||fabsf(x1)>0.8f||fabsf(x2)>1.2f||fabsf(x3-origin3)>0.30f ||
                (i>=20 && p<2 && (x1*signs[p]<-0.08f || x2*signs[p]<-0.12f))) {
                dc_enable=DISABLE_DCMOTORS; printf("UNEXPECTED MOTOR FEEDBACK -- ABORT\n"); aborted=true; break;
            }
            if (i==25 && p<2 &&
                ((m1.getRotation()-r1start)*signs[p]<0.025f ||
                 (m2.getRotation()-r2start)*signs[p]<0.05f ||
                 (fabsf(r3-r3start)>0.04f && fabsf(x3-r3start)<0.01f))) {
                dc_enable=DISABLE_DCMOTORS; printf("INSUFFICIENT ENCODER MOVEMENT -- ABORT\n"); aborted=true; break;
            }
            if (i>=SAMPLES-10) {
                if (fabsf(x1-v1)<0.05f) ++steadyGood[0];
                if (fabsf(x2-v2)<0.10f) ++steadyGood[1];
                if (fabsf(x3-r3)<5.0f/360.0f) ++steadyGood[2];
            }
            if (i%10==0) {
#if defined(PESBOARD_V2026)
                printf("STEP %d/%d (%+.3f rev/s) | %d/%d (%+.3f rev/s)\n",
                    step1.getSteps(), target, step1.getVelocity(),
                    step2.getSteps(), -target, step2.getVelocity());
#endif
                printf("M %.3f %.3f %.1fdeg | ADC %.3f %.3f %.3f %.3f V | A %.2f %.2f %.2f | G %.3f %.3f %.3f | SPI err=%lu\n",
                    x1,x2,(x3-origin3)*360,v[0],v[1],v[2],v[3],a.ax,a.ay,a.az,a.gx,a.gy,a.gz,(unsigned long)spiErrors);
#if defined(PESBOARD_V2019_2023)
                printf("MAG %.3f %.3f %.3f\n",a.mx,a.my,a.mz);
#endif
            }
            ThisThread::sleep_for(std::chrono::milliseconds(DELAY_MS));
        }
#if defined(PESBOARD_V2026)
        const bool phaseStepPass = !aborted && step1.getSteps()==target && step2.getSteps()==-target
            && !step1.isRunning() && !step2.isRunning();
        stepPass = stepPass && phaseStepPass;
        printf("STEP phase: %s (%d/%d, %d/%d; internal counts)\n", verdict(phaseStepPass),
            step1.getSteps(), target, step2.getSteps(), -target);
        if (!phaseStepPass) { step_enable=DISABLE_STEPPERMOTORS; dc_enable=DISABLE_DCMOTORS; aborted=true; }
#elif defined(PESBOARD_V2025)
        stopSteps();
        printf("STEP software rising-edge counts: %lu / %lu (not physical readback)\n",(unsigned long)step1.count(),(unsigned long)step2.count());
#endif
        for (int k=0;k<3;++k) motorPass[k] = motorPass[k] && !aborted && steadyGood[k]>=8;
        if (p<2) {
            motorPass[0] = motorPass[0] && (m1.getRotation()-r1start)*signs[p]>0.10f;
            motorPass[1] = motorPass[1] && (m2.getRotation()-r2start)*signs[p]>0.20f;
        }
        printf("Phase steady samples (of 10): %d %d %d\n",steadyGood[0],steadyGood[1],steadyGood[2]);
    }
    // Always disable motor stages before reporting or waiting for operator.
    dc_enable=DISABLE_DCMOTORS;
#if PB_HAS_STEPPERS
    step_enable=DISABLE_STEPPERMOTORS; stopSteps(); step_reset=RESET_STEPPERMOTORS;
#endif
    m1.setVelocity(0); m2.setVelocity(0); m3.setRotation(m3.getRotation());
    for (auto* s:servos) s->disable();
    sd_cs=1;
    constexpr uint32_t expectedSamples=3*SAMPLES;
    bool complete=!aborted && adcCount==expectedSamples;
    bool spiPass=complete && spiCount==expectedSamples*100 && spiErrors==0;
    bool imuPass=complete && imuInit && imuCount==expectedSamples && imuErrors==0;
    printf("\n========== SUMMARY =========\n");
    for(int k=0;k<3;++k) printf("DC M%d + encoder: %s\n",k+1,verdict(complete && motorPass[k]));
    printf("SPI loopback: %s (%lu bytes, %lu errors)\n",verdict(spiPass),(unsigned long)spiCount,(unsigned long)spiErrors);
    printf("IMU ID + sample plausibility: %s (%lu samples, %lu errors)\n",verdict(imuPass),(unsigned long)imuCount,(unsigned long)imuErrors);
    if (imuInit && imuCount) printf("IMU norm ranges: acc %.2f..%.2f m/s2, gyro %.3f..%.3f rad/s\n",accRange.lo,accRange.hi,gyroRange.lo,gyroRange.hi);
    for(int k=0;k<4;++k) if(adcCount) printf("ADC A%d: %.3f..%.3f V (FLOATING / OBSERVE)\n",k,adcRange[k].lo,adcRange[k].hi);
    printf("Servo D0..D3: MANUAL CHECK (50 Hz, 1250..1750 us)\n");
#if defined(PESBOARD_V2026)
    printf("STEP motion/count: %s (internal counts, driver enable=%d; physical movement requires manual check)\n", verdict(complete && stepPass), PES_TEST_ENABLE_STEPPER_DRIVERS);
#elif defined(PESBOARD_V2025)
    printf("STEP1/2: MANUAL CHECK (50/80 Hz, DIR reversal, driver enable=%d)\n",PES_TEST_ENABLE_STEPPER_DRIVERS);
#else
    printf("Stepper: N/A\n");
#endif
    bool automaticPass = complete && motorPass[0] && motorPass[1] && motorPass[2] && spiPass && imuPass;
#if defined(PESBOARD_V2026)
    automaticPass = automaticPass && stepPass;
#endif
    printf("Automatic subset: %s; full hardware acceptance requires manual checks.\n",verdict(automaticPass));
    printf("Power stages disabled. Reset to repeat.\n");
    while(true) ThisThread::sleep_for(1s);
}

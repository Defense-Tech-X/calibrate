#include "Runtime.hpp"

#include <cstdio>
#include <cstring>
#include <exception>

#include <timeapi.h>

namespace {

SafeRuntime* gRuntime = nullptr;

BOOL WINAPI onConsoleCtrl(DWORD) {
    if (gRuntime != nullptr) {
        gRuntime->parkThrottle();
        gRuntime->requestStop();
    }
    return TRUE;
}

} // namespace

uint64_t qpcNowUs() {
    static LARGE_INTEGER freq = [] {
        LARGE_INTEGER value{};
        QueryPerformanceFrequency(&value);
        return value;
    }();
    LARGE_INTEGER now{};
    QueryPerformanceCounter(&now);
    return static_cast<uint64_t>((now.QuadPart * 1000000.0) / static_cast<double>(freq.QuadPart));
}

SafeRuntime::SafeRuntime() {
    gRuntime = this;
    timeBeginPeriod(1);
    SetConsoleCtrlHandler(onConsoleCtrl, TRUE);
}

SafeRuntime::~SafeRuntime() {
    shutdown();
    SetConsoleCtrlHandler(onConsoleCtrl, FALSE);
    timeEndPeriod(1);
    if (gRuntime == this) {
        gRuntime = nullptr;
    }
}

void SafeRuntime::setupVisionAndController() {
    points_.setup();
    observation_.setup();
    controller_.setup();
}

void SafeRuntime::shutdown() {
    parkThrottle();
    controller_.shutdown();
    points_.shutdown();
    observation_.shutdown();
}

bool SafeRuntime::readStick(ControllerStick& stick) {
    std::lock_guard lock(io_);
    if (stop_.load()) {
        return false;
    }
    return controller_.read(stick);
}

bool SafeRuntime::writeStick(const ControllerStick& stick) {
    std::lock_guard lock(io_);
    if (stop_.load()) {
        return false;
    }
    return controller_.write(stick);
}

void SafeRuntime::parkThrottle() {
    std::lock_guard lock(io_);
    stop_.store(true);
    if (parked_ || !controller_.open()) {
        return;
    }
    try {
        ControllerStick stick;
        if (!controller_.read(stick)) {
            return;
        }
        stick.throttle = kAxisMin;
        if (controller_.write(stick)) {
            parked_ = true;
        }
    } catch (const std::exception& ex) {
        std::fprintf(stderr, "%s\n", ex.what());
    }
}

void SafeRuntime::runLoop(TickFn tick, SessionLog* session, const char* phase, uint64_t sessionId) {
    uint64_t lastObservationSequence = 0;
    bool keyframeNoted = false;
    while (!stop_.load() && !points_.failed() && !controller_.failed()) {
        const uint64_t nowUs = qpcNowUs();
        ControllerStick current;
        if (!readStick(current)) {
            break;
        }

        PointCloud cloud;
        const PointCloud* cloudPtr = nullptr;
        PointShift previousShift;
        const PointShift* sincePrevious = nullptr;
        if (points_.tryCopyNew(cloud)) {
            cloudPtr = &cloud;
            if (!previous_.empty()) {
                previousShift = measureShift(previous_, cloud.points, cloud.matrixWidth, cloud.matrixHeight);
                sincePrevious = &previousShift;
            }
            previous_ = cloud.points;
        }

        ObservationSample observed;
        const ObservationSample* observedPtr = nullptr;
        bool referenceReady = false;
        if (observation_.readLatest(observed)) {
            observedPtr = &observed;
            referenceReady = observed.locked != 0 &&
                observed.status == static_cast<uint32_t>(ObservationStatus::Valid) && observed.stale == 0;
            if (session != nullptr && observed.sequence != lastObservationSequence) {
                DiagnosticSample record{};
                record.sequence = observed.sequence;
                record.captureTimestampUs = observed.captureTimestampUs;
                record.commandTimestampUs = nowUs;
                record.status = observed.status;
                record.stale = observed.stale;
                record.reason = observed.reason;
                record.staleAgeUs = observed.staleAgeUs;
                for (int index = 0; index < 4; ++index) {
                    record.y[index] = observed.y[index];
                    record.dyDt[index] = observed.dyDt[index];
                }
                record.centerShiftX = observed.centerShiftX;
                record.centerShiftY = observed.centerShiftY;
                record.medianLkEigen = observed.medianLkEigen;
                record.referenceCount = observed.referenceCount;
                record.matchedCount = observed.matchedCount;
                record.inlierCount = observed.inlierCount;
                record.inlierRatio = observed.inlierRatio;
                record.medianFbPx = observed.medianFbPx;
                record.medianResidualPx = observed.medianResidualPx;
                record.p95ResidualPx = observed.p95ResidualPx;
                record.coverage = observed.coverage;
                record.throttle = current.throttle;
                record.yaw = current.yaw;
                record.pitch = current.pitch;
                record.roll = current.roll;
                std::snprintf(record.phase, sizeof(record.phase), "%s", phase);
                session->noteSample(record);
                if (!keyframeNoted && observed.locked != 0) {
                    session->noteKeyframe(observed);
                    keyframeNoted = true;
                }
                lastObservationSequence = observed.sequence;
            }
        }

        const bool cont = tick(nowUs, current, cloudPtr, sincePrevious, observedPtr, referenceReady);
        if (!cont) {
            break;
        }
        Sleep(1);
    }
    (void)sessionId;
}

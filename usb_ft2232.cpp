#include <QtWidgets>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QXYSeries>
#include <QtCharts/QLogValueAxis>
#include <QtConcurrent/QtConcurrent>
#include <QtCore/QElapsedTimer>
#include <QtGui/QMouseEvent>
#include <QtWidgets/QRubberBand>
#include <vector>
#include <array>
#include <deque>
#include <cmath>
#include <complex>
#include <atomic>
#include <thread>
#include <algorithm>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <processthreadsapi.h>
typedef LONG NTSTATUS;
typedef NTSTATUS(WINAPI* NtSetTimerResolution_t)(ULONG, BOOLEAN, PULONG);
#else
#include <unistd.h>
#include <sys/resource.h>
#include <sched.h>
#include <pthread.h>
#endif
#include "ftd2xx.h"
using namespace Qt::StringLiterals;

constexpr char SERIAL_NUM[] = "";
constexpr int EP_SIZE = 64 * 1024;
constexpr int PAYLOAD_LEN = 9;
constexpr int READ_CHUNK = 128 * 1024;
constexpr int TX_BURST_N = 512;
constexpr int WRITE_BACKLOG = 256 * 1024;
constexpr int MAX_ACC = 16384;
constexpr double FTDI_CLK_HZ = 60'000'000.0;
constexpr double XADC_FS_HZ = 60'000'000.0 / (3.0 * 26.0);
constexpr double TS_FTDI_US = 1e6 / FTDI_CLK_HZ;
constexpr double TS_XADC_US = 1e6 / XADC_FS_HZ;
constexpr size_t RING_SIZE = (1u << 21);
constexpr uint8_t FLAG1 = 0x55;
constexpr uint8_t FLAG2 = 0xAA;
constexpr uint8_t FLAG3 = 0x33;
constexpr uint8_t FLAG4 = 0xCC;
constexpr double VREF_VOLTS = 1.0;
constexpr double VOLTS_PER_COUNT = VREF_VOLTS / 255.0;
constexpr int FREQ_DECIMALS = 5;

const std::array<uint8_t, PAYLOAD_LEN> SW_ENTER_PAYLOAD{0,0,0,0,0,0,0,0xCD,0xAB};
const std::array<uint8_t, PAYLOAD_LEN> SW_EXIT_PAYLOAD{0,0,0,0,0,0,0,0xAC,0xFB};
const std::array<uint8_t, PAYLOAD_LEN> OSC_ENTER_PAYLOAD{0,0,0,0,0,0,0,0x5C,0x0A};
const std::array<uint8_t, PAYLOAD_LEN> OSC_EXIT_PAYLOAD{0,0,0,0,0,0,0,0xC0,0xA5};
const std::array<uint8_t, PAYLOAD_LEN> OSC_TRIG_PAYLOAD{0,0,0,0,0,0,0,0xF1,0x71};
const std::array<uint8_t, PAYLOAD_LEN> OSC_TRIG_ACK_PL{0,0,0,0,0,0,0,0xFA,0x71};

inline std::array<uint8_t, PAYLOAD_LEN> make_set_frames_payload(uint16_t frames) {
    std::array<uint8_t, PAYLOAD_LEN> p{};
    p[5] = uint8_t(frames & 0xFF);
    p[6] = uint8_t(frames >> 8);
    p[7] = 0xFA;
    p[8] = 0xF3;
    return p;
}

const std::array<uint8_t, PAYLOAD_LEN> SET_FRAMES_ACK_PL{0,0,0,0,0,0,0,0xFB,0xF3};
constexpr int DEFAULT_TRIG_FRAMES = 2048;

inline uint8_t crc8_update(uint8_t c, uint8_t b) {
    c ^= b;
    for (int i = 0; i < 8; ++i) {
        c = (c & 0x80) ? uint8_t((c << 1) ^ 0x07) : uint8_t(c << 1);
    }
    return c;
}

inline uint8_t crc_block(const uint8_t* p, size_t n, uint8_t init = 0) {
    while (n--) {
        init = crc8_update(init, *p++);
    }
    return init;
}

class SystemOptimizer {
public:
    static bool setHighPriority() {
#ifdef _WIN32
        if (!SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS)) {
            return false;
        }
        HMODULE nt = GetModuleHandleA("ntdll.dll");
        if (nt) {
            auto f = (NtSetTimerResolution_t)GetProcAddress(nt, "NtSetTimerResolution");
            if (f) {
                ULONG a;
                f(5000, TRUE, &a);
            }
        }
        SetProcessPriorityBoost(GetCurrentProcess(), FALSE);
        return true;
#else
        if (setpriority(PRIO_PROCESS, 0, -10) == -1) {
            return false;
        }
        struct sched_param p{50};
        if (sched_setscheduler(0, SCHED_FIFO, &p) == -1) {
            p.sched_priority = 0;
            sched_setscheduler(0, SCHED_OTHER, &p);
        }
        return true;
#endif
    }

    static bool setThreadHighPriority() {
#ifdef _WIN32
        return SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
#else
        struct sched_param p{80};
        return pthread_setschedparam(pthread_self(), SCHED_FIFO, &p) == 0;
#endif
    }
};

class RingBuffer {
public:
    explicit RingBuffer(size_t cap = RING_SIZE) : buf(cap), mask(cap - 1), capacity(cap) {
        if ((cap & (cap - 1)) != 0) {
            throw std::invalid_argument("Ring capacity must be power-of-2");
        }
    }

    void push(uint8_t v) {
        size_t h = head.load(std::memory_order_relaxed);
        size_t t = tail.load(std::memory_order_acquire);
        if (h - t >= capacity) {
            tail.fetch_add(1, std::memory_order_release);
        }
        buf[h & mask] = v;
        head.store(h + 1, std::memory_order_release);
    }

    size_t pop(std::vector<uint8_t>& dst, size_t n) {
        size_t h = head.load(std::memory_order_acquire);
        size_t t = tail.load(std::memory_order_relaxed);
        size_t avail = h - t;
        size_t take = std::min(avail, n);
        dst.resize(take);
        for (size_t i = 0; i < take; ++i) {
            dst[i] = buf[(t + i) & mask];
        }
        tail.store(t + take, std::memory_order_release);
        return take;
    }

    size_t available() const {
        return head.load(std::memory_order_acquire) - tail.load(std::memory_order_acquire);
    }

    void clear() {
        tail.store(0, std::memory_order_relaxed);
        head.store(0, std::memory_order_relaxed);
    }

private:
    std::vector<uint8_t> buf;
    const size_t mask;
    const size_t capacity;
    std::atomic<size_t> head{0};
    std::atomic<size_t> tail{0};
};

class Worker : public QThread {
    Q_OBJECT
public:
    enum class Mode {
        Sine,
        Triangle,
        Constant,
        SwEnterCmd,
        SwStream,
        SwExitCmd,
        OscEnterCmd,
        OscStream,
        OscExitCmd,
        OscTrigCmd,
        OscTrigWaitAck,
        OscTrigCapture,
        SetFramesCmd,
        SetFramesWaitAck
    };

    Worker() {
        for (int i = 0; i < 400; ++i) {
            sine[i] = uint8_t((std::sin(2 * M_PI * i / 400) * .5 + .5) * 255);
            tri[i] = uint8_t(std::abs(std::fmod(i / 200.0, 2.0) - 1.0) * 255);
        }
        txBuf.reserve(EP_SIZE * 2);
        captureBuf.reserve(4096);
        trigFramesTarget.store(DEFAULT_TRIG_FRAMES);
    }

    void setMode(Mode m) {
        QMetaObject::invokeMethod(this, [=] { reqMode = m; handleMode(); }, Qt::QueuedConnection);
    }

    void setConst(uint8_t v) {
        QMetaObject::invokeMethod(this, [=] { cst = v; }, Qt::QueuedConnection);
    }

    void setPaused(bool p) {
        QMetaObject::invokeMethod(this, [=] { pausedAll.store(p); }, Qt::QueuedConnection);
    }

    void setTrigVerbose(bool v) {
        QMetaObject::invokeMethod(this, [=] { trigVerbose.store(v); }, Qt::QueuedConnection);
    }

    void setTrigLogStride(int s) {
        QMetaObject::invokeMethod(this, [=] { trigLogStride = std::max(1, s); }, Qt::QueuedConnection);
    }

    void requestSetTrigFrames(uint16_t frames) {
        QMetaObject::invokeMethod(this, [=] {
            const uint16_t f = (frames == 0) ? 1 : frames;
            framesTargetPending = f;
            framesUpdatePending.store(true);
            if (pausedAll.load()) {
                return;
            }
            if (mode != Mode::OscTrigCapture && mode != Mode::OscTrigWaitAck && mode != Mode::SetFramesWaitAck) {
                mode = Mode::SetFramesCmd;
                setFramesSent = false;
                expectingCfgAck = true;
                phaseStartMs = QDateTime::currentMSecsSinceEpoch();
            }
        }, Qt::QueuedConnection);
    }

    size_t readSamples(std::vector<uint8_t>& dst, size_t n, int ch) {
        return (ch < 0 || ch >= PAYLOAD_LEN) ? 0 : channel_rings[ch].pop(dst, n);
    }

    size_t getTotalBufferLevel() const {
        size_t tot = 0;
        for (auto& r : channel_rings) {
            tot += r.available();
        }
        return tot;
    }

    void clearAllChannels() {
        for (auto& r : channel_rings) {
            r.clear();
        }
    }

signals:
    void newSamples();
    void rateUpdate(double mbps);
    void bufferLevel(size_t level);
    void trigAckReceived();
    void trigCaptureProgress(int framesReceived, int framesTotal);
    void trigSnapshotReady(QByteArray snapshotBytes);
    void trigFramesTargetChanged(int frames);
    void configAckReceived(int frames);

protected:
    void run() override {
        SystemOptimizer::setThreadHighPriority();
        FT_HANDLE h = nullptr;
        FT_STATUS st = strlen(SERIAL_NUM) ? FT_OpenEx((void*)SERIAL_NUM, FT_OPEN_BY_SERIAL_NUMBER, &h) : FT_Open(0, &h);
        if (st != FT_OK || !h) {
            qDebug() << "FTDI open fail" << st;
            return;
        }
        FT_SetBitMode(h, 0xFF, 0x40);
        FT_SetLatencyTimer(h, 1);
        FT_SetFlowControl(h, FT_FLOW_NONE, 0, 0);
        FT_Purge(h, FT_PURGE_RX | FT_PURGE_TX);
        FT_SetTimeouts(h, 0, 0);
        FT_SetUSBParameters(h, 65536, 65536);
        std::vector<uint8_t> rx(READ_CHUNK);
        qint64 t0 = QDateTime::currentMSecsSinceEpoch();
        size_t bits = 0;
        bool wasPaused = false;
        bool pausedWasTrig = false;
        QTimer tim;
        tim.setInterval(0);
        tim.setTimerType(Qt::PreciseTimer);
        connect(&tim, &QTimer::timeout, [&] {
            if (isInterruptionRequested()) {
                return;
            }
            if (pausedAll.load(std::memory_order_acquire)) {
                if (!wasPaused) {
                    FT_Purge(h, FT_PURGE_RX | FT_PURGE_TX);
                    pausedWasTrig = (mode == Mode::OscTrigCmd || mode == Mode::OscTrigWaitAck || mode == Mode::OscTrigCapture || mode == Mode::SetFramesCmd || mode == Mode::SetFramesWaitAck);
                    expectingAck = false;
                    trigSent = false;
                    trigFramesReceived = 0;
                    captureBuf.clear();
                }
                wasPaused = true;
                return;
            } else if (wasPaused) {
                wasPaused = false;
                if (pausedWasTrig) {
                    if (framesUpdatePending.load()) {
                        mode = Mode::SetFramesCmd;
                        setFramesSent = false;
                        expectingCfgAck = true;
                    } else {
                        mode = Mode::OscTrigCmd;
                        expectingAck = true;
                        trigSent = false;
                    }
                    phaseStartMs = QDateTime::currentMSecsSinceEpoch();
                }
            }
            DWORD q = 0;
            while (FT_GetQueueStatus(h, &q) == FT_OK && q >= 2) {
                q = std::min<DWORD>(q, READ_CHUNK);
                DWORD got = 0;
                if (FT_Read(h, rx.data(), q, &got) == FT_OK && got >= 2) {
                    parseRx(rx.data() + 2, got - 2);
                    bits += (got - 2) * 8;
                } else {
                    break;
                }
            }
            if (mode == Mode::Sine || mode == Mode::Triangle || mode == Mode::Constant) {
                flushTx(h, false);
            } else if (mode == Mode::SwEnterCmd || mode == Mode::SwExitCmd || mode == Mode::OscEnterCmd || mode == Mode::OscExitCmd || mode == Mode::OscTrigCmd || mode == Mode::SetFramesCmd) {
                flushTx(h, true);
            }
            const qint64 now = QDateTime::currentMSecsSinceEpoch();
            if (mode == Mode::SetFramesWaitAck && expectingCfgAck) {
                if (now - phaseStartMs > 400) {
                    qDebug() << "SET-FRAMES ACK timeout → re-send";
                    setFramesSent = false;
                    mode = Mode::SetFramesCmd;
                    phaseStartMs = now;
                }
            } else if (mode == Mode::OscTrigWaitAck && expectingAck) {
                if (now - phaseStartMs > 400) {
                    qDebug() << "TRIG ACK timeout ? re-TRIG";
                    trigSent = false;
                    mode = Mode::OscTrigCmd;
                    phaseStartMs = now;
                }
            } else if (mode == Mode::OscTrigCapture) {
                if (now - phaseStartMs > 600) {
                    const int minOk = framesMinAcceptable();
                    if (trigFramesReceived >= minOk) {
                        qDebug() << "Capture watchdog: accepting short snapshot for TRIG #" << trigCycleNo << "(" << trigFramesReceived << "/" << framesTarget() << " frames)";
                        finalizeSnapshotWithPadding(true);
                    } else {
                        qDebug() << "Capture timeout (" << trigFramesReceived << "/" << framesTarget() << " frames) for TRIG #" << trigCycleNo << " → re-TRIG";
                        captureBuf.clear();
                        trigFramesReceived = 0;
                        expectingAck = true;
                        trigSent = false;
                        mode = Mode::OscTrigCmd;
                        phaseStartMs = now;
                    }
                }
            }
            if (now - t0 >= 1000) {
                emit rateUpdate(bits / 1e6);
                emit bufferLevel(getTotalBufferLevel());
                bits = 0;
                t0 = now;
            }
        });
        tim.start();
        flushTx(h, true);
        exec();
        tim.stop();
        FT_Close(h);
    }

private:
    static void addFrame(std::vector<uint8_t>& o, const std::array<uint8_t, PAYLOAD_LEN>& p) {
        o.push_back(FLAG1);
        o.push_back(FLAG2);
        o.push_back(PAYLOAD_LEN);
        o.insert(o.end(), p.begin(), p.end());
        o.push_back(crc_block(p.data(), PAYLOAD_LEN));
        o.push_back(FLAG3);
        o.push_back(FLAG4);
    }

    static QString hex9(const std::array<uint8_t, PAYLOAD_LEN>& p) {
        QString s;
        s.reserve(3 * PAYLOAD_LEN);
        for (int i = 0; i < PAYLOAD_LEN; ++i) {
            s += QString("%1 ").arg(p[i], 2, 16, QLatin1Char('0')).toUpper();
        }
        if (!s.isEmpty()) {
            s.chop(1);
        }
        return s;
    }

    int framesTarget() const {
        return std::max(1, trigFramesTarget.load());
    }

    int samplesTarget() const {
        return framesTarget() * PAYLOAD_LEN;
    }

    int framesMinAcceptable() const {
        int t = framesTarget();
        int slack = std::min(16, std::max(4, t / 10));
        return std::max(1, t - slack);
    }

    void flushTx(FT_HANDLE h, bool force = false) {
        if (!force) {
            DWORD txq;
            DWORD rx;
            DWORD ev;
            if (FT_GetStatus(h, &rx, &txq, &ev) != FT_OK) {
                return;
            }
            if (txq >= WRITE_BACKLOG) {
                return;
            }
        }
        txBuf.clear();
        switch (mode) {
        case Mode::Sine: {
            for (int i = 0; i < TX_BURST_N; ++i) {
                for (int k = 0; k < PAYLOAD_LEN; ++k) {
                    tmp[k] = sine[(sptr + k) % 400];
                }
                sptr = (sptr + PAYLOAD_LEN) % 400;
                addFrame(txBuf, tmp);
            }
            break;
        }
        case Mode::Triangle: {
            for (int i = 0; i < TX_BURST_N; ++i) {
                for (int k = 0; k < PAYLOAD_LEN; ++k) {
                    tri[k] = tri[k];
                }
                for (int k = 0; k < PAYLOAD_LEN; ++k) {
                    tmp[k] = tri[(tptr + k) % 400];
                }
                tptr = (tptr + PAYLOAD_LEN) % 400;
                addFrame(txBuf, tmp);
            }
            break;
        }
        case Mode::Constant: {
            for (int i = 0; i < TX_BURST_N; ++i) {
                tmp.fill(cst);
                addFrame(txBuf, tmp);
            }
            break;
        }
        case Mode::SwEnterCmd: {
            addFrame(txBuf, SW_ENTER_PAYLOAD);
            break;
        }
        case Mode::SwExitCmd: {
            addFrame(txBuf, SW_EXIT_PAYLOAD);
            break;
        }
        case Mode::OscEnterCmd: {
            addFrame(txBuf, OSC_ENTER_PAYLOAD);
            break;
        }
        case Mode::OscExitCmd: {
            addFrame(txBuf, OSC_EXIT_PAYLOAD);
            break;
        }
        case Mode::SetFramesCmd: {
            if (!setFramesSent && framesUpdatePending.load()) {
                addFrame(txBuf, make_set_frames_payload(framesTargetPending));
                QString desc = QString("SET-FRAMES sent (%1 %2 FA F3)").arg(framesTargetPending & 0xFF, 2, 16, QLatin1Char('0')).arg(framesTargetPending >> 8, 2, 16, QLatin1Char('0'));
                qDebug() << desc.toUpper();
                setFramesSent = true;
                expectingCfgAck = true;
                mode = Mode::SetFramesWaitAck;
                phaseStartMs = QDateTime::currentMSecsSinceEpoch();
            }
            break;
        }
        case Mode::OscTrigCmd: {
            if (framesUpdatePending.load()) {
                mode = Mode::SetFramesCmd;
                setFramesSent = false;
                expectingCfgAck = true;
                phaseStartMs = QDateTime::currentMSecsSinceEpoch();
                break;
            }
            if (!trigSent) {
                addFrame(txBuf, OSC_TRIG_PAYLOAD);
                qDebug() << "TRIG sent once (F1 71)";
                trigSent = true;
                expectingAck = true;
                ++trigCycleNo;
                qDebug() << "TRIG #" << trigCycleNo << "armed";
                mode = Mode::OscTrigWaitAck;
                phaseStartMs = QDateTime::currentMSecsSinceEpoch();
            }
            break;
        }
        default: {
            break;
        }
        }
        if (!txBuf.empty()) {
            DWORD w;
            FT_Write(h, txBuf.data(), DWORD(txBuf.size()), &w);
        }
    }

    static bool isControlPayload(const std::array<uint8_t, PAYLOAD_LEN>& p) {
        return (p == SW_ENTER_PAYLOAD) || (p == SW_EXIT_PAYLOAD) || (p == OSC_ENTER_PAYLOAD) || (p == OSC_EXIT_PAYLOAD) || (p == OSC_TRIG_PAYLOAD) || (p == OSC_TRIG_ACK_PL) || (p == SET_FRAMES_ACK_PL);
    }

    void parseRx(const uint8_t* d, size_t n) {
        for (size_t i = 0; i < n; ++i) {
            uint8_t b = d[i];
            switch (rs) {
            case 0: {
                rs = (b == FLAG1) ? 1 : 0;
                break;
            }
            case 1: {
                rs = (b == FLAG2) ? 2 : ((b == FLAG1) ? 1 : 0);
                break;
            }
            case 2: {
                rlen = b;
                ridx = 0;
                rcrc = 0;
                rs = (b ? 3 : 4);
                break;
            }
            case 3: {
                rcrc = crc8_update(rcrc, b);
                if (ridx < PAYLOAD_LEN) {
                    payload[ridx] = b;
                }
                ++ridx;
                --rlen;
                rs = (rlen ? 3 : 4);
                break;
            }
            case 4: {
                rs = (b == rcrc) ? 5 : 0;
                break;
            }
            case 5: {
                rs = (b == FLAG3) ? 6 : 0;
                break;
            }
            case 6: {
                if (b == FLAG4) {
                    handleFrame();
                }
                rs = 0;
                break;
            }
            }
        }
    }

    void handleFrame() {
        if (payload == SW_ENTER_PAYLOAD) {
            inSw = true;
            inOsc = false;
            mode = Mode::SwStream;
            return;
        }
        if (payload == SW_EXIT_PAYLOAD) {
            inSw = false;
            mode = nextMode;
            nextMode = Mode::Sine;
            return;
        }
        if (payload == OSC_ENTER_PAYLOAD) {
            inOsc = true;
            inSw = false;
            mode = Mode::OscStream;
            return;
        }
        if (payload == OSC_EXIT_PAYLOAD) {
            inOsc = false;
            mode = nextMode;
            nextMode = Mode::Sine;
            return;
        }
        if (payload == SET_FRAMES_ACK_PL) {
            if (expectingCfgAck) {
                expectingCfgAck = false;
                if (framesUpdatePending.load()) {
                    trigFramesTarget.store(std::max(1, int(framesTargetPending)));
                    framesUpdatePending.store(false);
                    emit trigFramesTargetChanged(framesTarget());
                    qDebug() << "SET-FRAMES ACK (FB F3). New target =" << framesTarget() << "frames";
                    emit configAckReceived(framesTarget());
                }
                setFramesSent = false;
                trigSent = false;
                expectingAck = true;
                mode = Mode::OscTrigCmd;
                phaseStartMs = QDateTime::currentMSecsSinceEpoch();
            }
            return;
        }
        if (payload == OSC_TRIG_ACK_PL) {
            if (mode == Mode::OscTrigWaitAck && expectingAck) {
                qDebug() << "ACK received (FA 71)";
                qDebug() << "TRIG #" << trigCycleNo << "capture start (target" << framesTarget() << "frames)";
                captureBuf.clear();
                trigFramesReceived = 0;
                expectingAck = false;
                mode = Mode::OscTrigCapture;
                phaseStartMs = QDateTime::currentMSecsSinceEpoch();
                emit trigAckReceived();
            }
            return;
        }
        if (!isControlPayload(payload)) {
            if (mode == Mode::OscTrigCapture) {
                for (int i = 0; i < PAYLOAD_LEN; ++i) {
                    captureBuf.push_back(payload[i]);
                }
                ++trigFramesReceived;
                if (trigVerbose.load() && ((trigFramesReceived % trigLogStride) == 0)) {
                    qDebug() << "TRIG #" << trigCycleNo << "frame" << trigFramesReceived << "/" << framesTarget() << "bytes:" << hex9(payload);
                }
                emit trigCaptureProgress(trigFramesReceived, framesTarget());
                phaseStartMs = QDateTime::currentMSecsSinceEpoch();
                if (trigFramesReceived >= framesTarget()) {
                    finalizeSnapshotWithPadding(false);
                }
                return;
            }
            for (int i = 0; i < PAYLOAD_LEN; ++i) {
                channel_rings[i].push(payload[i]);
            }
            if (++contFramesSinceLastUI >= 128) {
                contFramesSinceLastUI = 0;
                emit newSamples();
            }
        }
    }

    void finalizeSnapshotWithPadding(bool fromTimeout) {
        const int frames = trigFramesReceived;
        const int targetF = framesTarget();
        const int minOk = framesMinAcceptable();
        if (frames < minOk) {
            qDebug() << "Snapshot too short (" << frames << "/" << targetF << " frames)" << "for TRIG #" << trigCycleNo << " → re-TRIG";
            captureBuf.clear();
            trigFramesReceived = 0;
            expectingAck = true;
            trigSent = false;
            mode = Mode::OscTrigCmd;
            phaseStartMs = QDateTime::currentMSecsSinceEpoch();
            return;
        }
        const int haveBytes = frames * PAYLOAD_LEN;
        const int wantBytes = targetF * PAYLOAD_LEN;
        QByteArray snap;
        snap.resize(wantBytes);
        const int copyLen = std::min(haveBytes, wantBytes);
        if (copyLen > 0) {
            memcpy(snap.data(), captureBuf.data(), copyLen);
        }
        uint8_t pad = (copyLen > 0) ? uint8_t(captureBuf[size_t(copyLen - 1)]) : 0;
        for (int i = copyLen; i < wantBytes; ++i) {
            snap[i] = char(pad);
        }
        if (frames < targetF) {
            qDebug() << "TRIG capture accepted short (" << frames << "/" << targetF << " frames) padded to" << wantBytes << "bytes";
        } else {
            qDebug() << "TRIG capture complete (" << frames << "/" << targetF << " frames)";
        }
        emit trigSnapshotReady(snap);
        captureBuf.clear();
        trigFramesReceived = 0;
        trigSent = false;
        if (framesUpdatePending.load()) {
            mode = Mode::SetFramesCmd;
            setFramesSent = false;
            expectingCfgAck = true;
        } else {
            expectingAck = true;
            mode = Mode::OscTrigCmd;
        }
        phaseStartMs = QDateTime::currentMSecsSinceEpoch();
    }

    void handleMode() {
        if (reqMode == Mode::OscTrigCmd) {
            if (inSw) {
                nextMode = Mode::OscTrigCmd;
                mode = Mode::SwExitCmd;
                return;
            }
            if (inOsc) {
                nextMode = Mode::OscTrigCmd;
                mode = Mode::OscExitCmd;
                return;
            }
            clearAllChannels();
            captureBuf.clear();
            trigFramesReceived = 0;
            contFramesSinceLastUI = 0;
            if (framesUpdatePending.load()) {
                mode = Mode::SetFramesCmd;
                setFramesSent = false;
                expectingCfgAck = true;
            } else {
                expectingAck = true;
                trigSent = false;
                mode = Mode::OscTrigCmd;
            }
            phaseStartMs = QDateTime::currentMSecsSinceEpoch();
            return;
        }
        if (reqMode == Mode::SwEnterCmd) {
            if (inSw) {
                return;
            }
            if (inOsc) {
                nextMode = Mode::SwEnterCmd;
                mode = Mode::OscExitCmd;
            } else {
                mode = Mode::SwEnterCmd;
            }
            return;
        }
        if (reqMode == Mode::OscEnterCmd) {
            if (inOsc) {
                return;
            }
            if (inSw) {
                nextMode = Mode::OscEnterCmd;
                mode = Mode::SwExitCmd;
            } else {
                mode = Mode::OscEnterCmd;
            }
            return;
        }
        if (inSw) {
            nextMode = reqMode;
            mode = Mode::SwExitCmd;
        } else if (inOsc) {
            nextMode = reqMode;
            mode = Mode::OscExitCmd;
        } else {
            mode = reqMode;
        }
    }

    Mode mode = Mode::Sine;
    Mode reqMode = Mode::Sine;
    Mode nextMode = Mode::Sine;
    bool inSw = false;
    bool inOsc = false;
    bool expectingAck = false;
    bool trigSent = false;
    int trigFramesReceived = 0;
    std::vector<uint8_t> captureBuf;
    qint64 phaseStartMs = 0;
    std::atomic<bool> framesUpdatePending{false};
    uint16_t framesTargetPending = DEFAULT_TRIG_FRAMES;
    std::atomic<int> trigFramesTarget;
    bool setFramesSent = false;
    bool expectingCfgAck = false;
    std::atomic<bool> trigVerbose{true};
    int trigLogStride = 1;
    int trigCycleNo = 0;
    int contFramesSinceLastUI = 0;
    std::array<uint8_t, 400> sine{};
    std::array<uint8_t, 400> tri{};
    int sptr = 0;
    int tptr = 0;
    uint8_t cst = 0;
    std::array<uint8_t, PAYLOAD_LEN> tmp{};
    std::array<uint8_t, PAYLOAD_LEN> payload{};
    uint8_t rs = 0;
    uint8_t rlen = 0;
    uint8_t ridx = 0;
    uint8_t rcrc = 0;
    std::array<RingBuffer, PAYLOAD_LEN> channel_rings;
    std::vector<uint8_t> txBuf;
    std::atomic<bool> pausedAll{false};
};

class EnhancedChartView : public QChartView {
    Q_OBJECT
public:
    EnhancedChartView(QChart* chart, QWidget* parent = nullptr) : QChartView(chart, parent), m_rubberBand(nullptr) {
        setRenderHint(QPainter::Antialiasing);
        setRubberBand(QChartView::NoRubberBand);
        setMouseTracking(true);
    }

    void setZoomEnabled(bool enabled) {
        m_zoomEnabled = enabled;
    }

    void setMarkersEnabled(bool enabled) {
        m_markersEnabled = enabled;
    }

signals:
    void rectangleZoom(const QRectF& rect);
    void markerAdded(double x, double y);
    void mousePosition(double x, double y);

protected:
    void mousePressEvent(QMouseEvent* e) override {
        if (e->button() == Qt::LeftButton && m_zoomEnabled) {
            m_origin = e->pos();
            if (!m_rubberBand) {
                m_rubberBand = new QRubberBand(QRubberBand::Rectangle, this);
            }
            m_rubberBand->setGeometry(QRect(m_origin, QSize()));
            m_rubberBand->show();
        } else if (e->button() == Qt::RightButton && m_markersEnabled) {
            QPointF chartPos = chart()->mapToValue(e->pos());
            emit markerAdded(chartPos.x(), chartPos.y());
        }
        QChartView::mousePressEvent(e);
    }

    void mouseMoveEvent(QMouseEvent* e) override {
        if (m_rubberBand && m_rubberBand->isVisible()) {
            m_rubberBand->setGeometry(QRect(m_origin, e->pos()).normalized());
        }
        QPointF chartPos = chart()->mapToValue(e->pos());
        emit mousePosition(chartPos.x(), chartPos.y());
        QChartView::mouseMoveEvent(e);
    }

    void mouseReleaseEvent(QMouseEvent* e) override {
        if (m_rubberBand && m_rubberBand->isVisible() && e->button() == Qt::LeftButton) {
            QRect rect = m_rubberBand->geometry();
            m_rubberBand->hide();
            if (rect.width() > 10 && rect.height() > 10) {
                QPointF tl = chart()->mapToValue(rect.topLeft());
                QPointF br = chart()->mapToValue(rect.bottomRight());
                emit rectangleZoom(QRectF(tl, br).normalized());
            }
        }
        QChartView::mouseReleaseEvent(e);
    }

    void keyPressEvent(QKeyEvent* e) override {
        if (e->key() == Qt::Key_Escape && m_rubberBand && m_rubberBand->isVisible()) {
            m_rubberBand->hide();
        }
        QChartView::keyPressEvent(e);
    }

private:
    QPoint m_origin;
    QRubberBand* m_rubberBand;
    bool m_zoomEnabled = false;
    bool m_markersEnabled = false;
};

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    MainWindow() {
        SystemOptimizer::setHighPriority();
        setupUI();
        setupWorker();
        connectSignals();
        updateScale();
        statusBar()->showMessage("High-priority mode active - Right-click for markers, drag for zoom");
        limiter.start();
        fftWatcher = new QFutureWatcher<FFTResult>(this);
        connect(fftWatcher, &QFutureWatcher<FFTResult>::finished, this, [=] {
            auto res = fftWatcher->result();
            if (!rbOSCT->isChecked() || paused) {
                fftBusy.store(false);
                return;
            }
            seriesFFT->replace(res.psd);
            if (!res.psd.isEmpty()) {
                double minX = std::max(1e-9, double(res.psd.front().x()));
                double maxX = double(res.psd.back().x());
                axisXFFT->setRange(minX, maxX);
                double mn = res.psd[0].y();
                double mx = res.psd[0].y();
                for (const auto& p : res.psd) {
                    if (p.y() < mn) {
                        mn = p.y();
                    }
                    if (p.y() > mx) {
                        mx = p.y();
                    }
                }
                setRangeIfChanged(axisYFFT, mn, mx);
            }
            freqValueLbl->setText(humanFreq(res.peakHz));
            fftBusy.store(false);
        });
    }

    ~MainWindow() override {
        worker->requestInterruption();
        worker->wait();
        delete worker;
    }

private:
    bool isOscSelected() const {
        return rbOSC->isChecked() || rbOSCM->isChecked() || rbOSCT->isChecked();
    }

    double yScaled(uint8_t s) const {
        return isOscSelected() ? (double(s) * VOLTS_PER_COUNT) : double(s);
    }

    void clearAllPlots() {
        accSingle.clear();
        seriesSingle->clear();
        for (auto& mp : multiPlots) {
            mp.acc.clear();
            mp.series->clear();
        }
        seriesFFT->clear();
        auto removeScatterSeries = [](QChart* c) {
            QList<QAbstractSeries*> toRemove;
            for (auto* s : c->series()) {
                if (qobject_cast<QScatterSeries*>(s)) {
                    toRemove.push_back(s);
                }
            }
            for (auto* s : toRemove) {
                c->removeSeries(s);
                delete s;
            }
        };
        removeScatterSeries(chartSingle);
        removeScatterSeries(chartFFT);
    }

private slots:
    void applyConst() {
        bool ok = false;
        QString t = constEdit->text().trimmed();
        int v = t.startsWith("0x", Qt::CaseInsensitive) ? t.toInt(&ok, 16) : t.toInt(&ok, 10);
        if (ok && v >= 0 && v <= 255) {
            worker->setConst(uint8_t(v));
        } else {
            QMessageBox::warning(this, "Invalid", "Enter 0–255 or 0xXX");
        }
    }

    void applyFrames() {
        bool ok = false;
        int v = framesEdit->text().trimmed().toInt(&ok, 10);
        if (!ok || v <= 0 || v > 65535) {
            QMessageBox::warning(this, "Invalid", "Frames must be 1..65535");
            return;
        }
        worker->requestSetTrigFrames(uint16_t(v));
        statusBar()->showMessage(QString("Applying TRIG frames = %1 …").arg(v));
    }

    void changeChannel(int idx) {
        currentChannel = idx;
        accSingle.clear();
        seriesSingle->clear();
        clearMarkers();
    }

    void updateScale() {
        if (!originalRangesStored) {
            originalXMin = xMinSpin->value();
            originalXMax = xMaxSpin->value();
            originalYMin = yMinSpin->value();
            originalYMax = yMaxSpin->value();
            originalRangesStored = true;
        }
        double xmin = xMinSpin->value();
        double xmax = xMaxSpin->value();
        if (xmin >= xmax) {
            xmax = xmin + 1.0;
        }
        axisXSingle->setRange(xmin, xmax);
        for (auto& mp : multiPlots) {
            mp.axisX->setRange(xmin, xmax);
        }
        axisYSingle->setTitleText(isOscSelected() ? "Voltage (V)" : "Level");
        for (auto& mp : multiPlots) {
            mp.axisY->setTitleText(isOscSelected() ? "Voltage (V)" : "Level");
        }
        auto setY = [&](QValueAxis* ax) {
            if (autoYCheck->isChecked()) {
                return;
            }
            double ymin = yMinSpin->value();
            double ymax = yMaxSpin->value();
            if (isOscSelected()) {
                ymin *= VOLTS_PER_COUNT;
                ymax *= VOLTS_PER_COUNT;
            }
            if (ymin >= ymax) {
                ymax = ymin + (isOscSelected() ? 0.01 : 1.0);
            }
            ax->setRange(ymin, ymax);
        };
        setY(axisYSingle);
        for (auto& mp : multiPlots) {
            setY(mp.axisY);
        }
    }

    void updatePlot() {
        if (paused) {
            return;
        }
        const int minFrameMs = multiMode ? 33 : 16;
        if (limiter.elapsed() < minFrameMs) {
            return;
        }
        limiter.restart();
        if (multiMode) {
            updatePlotMulti();
        } else {
            updatePlotSingle();
        }
    }

    void onPauseToggled(bool p) {
        paused = p;
        pauseBtn->setText(paused ? QStringLiteral("▶ Run") : QStringLiteral("⏸ Pause"));
        zoomControlsWidget->setEnabled(paused);
        markerControlsWidget->setEnabled(paused);
        if (auto* ecv = qobject_cast<EnhancedChartView*>(singleChartView)) {
            ecv->setZoomEnabled(paused);
            ecv->setMarkersEnabled(paused);
        }
        for (auto& mp : multiPlots) {
            if (auto* ecv = qobject_cast<EnhancedChartView*>(mp.chartView)) {
                ecv->setZoomEnabled(paused);
                ecv->setMarkersEnabled(paused);
            }
        }
        if (auto* ecv = qobject_cast<EnhancedChartView*>(fftChartView)) {
            ecv->setZoomEnabled(paused);
            ecv->setMarkersEnabled(paused);
        }
        worker->setPaused(paused);
        if (paused) {
            limiter.restart();
        }
        updateStatusMessage();
    }

    void addMarker(double x, double y) {
        if (!paused) {
            return;
        }
        int markerId = nextMarkerId++;
        auto* series = new QScatterSeries();
        series->setName(QString("Marker %1").arg(markerId));
        series->setMarkerSize(10);
        series->setColor(QColor::fromHsv((markerId * 60) % 360, 255, 255));
        series->append(x, y);
        if (!multiMode) {
            chartSingle->addSeries(series);
            chartSingle->setAxisX(axisXSingle, series);
            chartSingle->setAxisY(axisYSingle, series);
        }
        auto* label = new QLabel(QString("M%1: (%2 µs, %3)").arg(markerId).arg(x, 0, 'f', 2).arg(y, 0, 'f', isOscSelected() ? 3 : 1));
        label->setStyleSheet("QLabel { color: " + series->color().name() + "; font-weight: bold; }");
        markerLayout->addWidget(label);
        markers.emplace_back(Marker{markerId, x, y});
    }

    void addFFTMarker(double x, double y) {
        if (!paused) {
            return;
        }
        int markerId = nextFFTMarkerId++;
        auto* series = new QScatterSeries();
        series->setName(QString("FFT Marker %1").arg(markerId));
        series->setMarkerSize(10);
        series->setColor(QColor::fromHsv((120 + markerId * 60) % 360, 255, 200));
        series->append(x, y);
        chartFFT->addSeries(series);
        chartFFT->setAxisX(axisXFFT, series);
        chartFFT->setAxisY(axisYFFT, series);
        auto* label = new QLabel(QString("F%1: (%2 kHz, %3 dB)").arg(markerId).arg(x, 0, 'f', FREQ_DECIMALS).arg(y, 0, 'f', 1));
        label->setStyleSheet("QLabel { color: " + series->color().name() + "; font-weight: bold; }");
        markerLayout->addWidget(label);
        fftMarkers.emplace_back(Marker{markerId, x, y});
    }

    void clearMarkers() {
        QLayoutItem* child;
        while ((child = markerLayout->takeAt(0)) != nullptr) {
            if (auto* w = child->widget()) {
                w->deleteLater();
            }
            delete child;
        }
        markers.clear();
        fftMarkers.clear();
        auto removeScatterSeries = [](QChart* c) {
            QList<QAbstractSeries*> toRemove;
            for (auto* s : c->series()) {
                if (qobject_cast<QScatterSeries*>(s)) {
                    toRemove.push_back(s);
                }
            }
            for (auto* s : toRemove) {
                c->removeSeries(s);
                delete s;
            }
        };
        removeScatterSeries(chartSingle);
        removeScatterSeries(chartFFT);
    }

    void zoomIn() {
        if (!paused) {
            return;
        }
        QSignalBlocker b1(xMinSpin);
        QSignalBlocker b2(xMaxSpin);
        QSignalBlocker b3(yMinSpin);
        QSignalBlocker b4(yMaxSpin);
        double center = (axisXSingle->max() + axisXSingle->min()) / 2.0;
        double newWidth = (axisXSingle->max() - axisXSingle->min()) * 0.5;
        xMinSpin->setValue(center - newWidth / 2.0);
        xMaxSpin->setValue(center + newWidth / 2.0);
        if (!autoYCheck->isChecked()) {
            double yCenter = (axisYSingle->max() + axisYSingle->min()) / 2.0;
            double newHeight = (axisYSingle->max() - axisYSingle->min()) * 0.8;
            yMinSpin->setValue(int(std::round(isOscSelected() ? (yCenter - newHeight / 2.0) / VOLTS_PER_COUNT : (yCenter - newHeight / 2.0))));
            yMaxSpin->setValue(int(std::round(isOscSelected() ? (yCenter + newHeight / 2.0) / VOLTS_PER_COUNT : (yCenter + newHeight / 2.0))));
        }
        updateScale();
    }

    void zoomOut() {
        if (!paused) {
            return;
        }
        QSignalBlocker b1(xMinSpin);
        QSignalBlocker b2(xMaxSpin);
        QSignalBlocker b3(yMinSpin);
        QSignalBlocker b4(yMaxSpin);
        double center = (axisXSingle->max() + axisXSingle->min()) / 2.0;
        double newWidth = (axisXSingle->max() - axisXSingle->min()) * 2.0;
        xMinSpin->setValue(center - newWidth / 2.0);
        xMaxSpin->setValue(center + newWidth / 2.0);
        if (!autoYCheck->isChecked()) {
            double yCenter = (axisYSingle->max() + axisYSingle->min()) / 2.0;
            double newHeight = (axisYSingle->max() - axisYSingle->min()) * 1.25;
            double yminV = yCenter - newHeight / 2.0;
            double ymaxV = yCenter + newHeight / 2.0;
            yMinSpin->setValue(int(std::round(isOscSelected() ? (yminV / VOLTS_PER_COUNT) : yminV)));
            yMaxSpin->setValue(int(std::round(isOscSelected() ? (ymaxV / VOLTS_PER_COUNT) : ymaxV)));
        }
        updateScale();
    }

    void resetZoom() {
        if (!paused || !originalRangesStored) {
            return;
        }
        QSignalBlocker b1(xMinSpin);
        QSignalBlocker b2(xMaxSpin);
        QSignalBlocker b3(yMinSpin);
        QSignalBlocker b4(yMaxSpin);
        xMinSpin->setValue(originalXMin);
        xMaxSpin->setValue(originalXMax);
        yMinSpin->setValue(originalYMin);
        yMaxSpin->setValue(originalYMax);
        updateScale();
    }

    void onRectangleZoom(const QRectF& rect) {
        if (!paused) {
            return;
        }
        QSignalBlocker b1(xMinSpin);
        QSignalBlocker b2(xMaxSpin);
        QSignalBlocker b3(yMinSpin);
        QSignalBlocker b4(yMaxSpin);
        xMinSpin->setValue(rect.left());
        xMaxSpin->setValue(rect.right());
        if (!autoYCheck->isChecked()) {
            double top = rect.top();
            double bottom = rect.bottom();
            if (isOscSelected()) {
                top /= VOLTS_PER_COUNT;
                bottom /= VOLTS_PER_COUNT;
            }
            yMinSpin->setValue(qBound(0, int(std::round(top)), 255));
            yMaxSpin->setValue(qBound(0, int(std::round(bottom)), 255));
        }
        updateScale();
    }

    void onFFTRectangleZoom(const QRectF& rect) {
        if (!paused) {
            return;
        }
        double xmin = std::max(1e-9, rect.left());
        double xmax = rect.right();
        double ymin = rect.top();
        double ymax = rect.bottom();
        if (xmin >= xmax) {
            xmax = xmin * 1.1;
        }
        if (ymin >= ymax) {
            ymax = ymin + 0.1;
        }
        axisXFFT->setRange(xmin, xmax);
        axisYFFT->setRange(ymin, ymax);
    }

    void onMousePosition(double x, double y) {
        if (paused) {
            cursorLabel->setText(QString("Cursor: (%1 µs, %2%3)").arg(x, 0, 'f', 2).arg(y, 0, 'f', isOscSelected() ? 3 : 1).arg(isOscSelected() ? " V" : ""));
        } else {
            cursorLabel->setText("");
        }
    }

    void onFFTMousePosition(double x, double y) {
        if (paused) {
            cursorLabel->setText(QString("Cursor (FFT): (%1 kHz, %2 dB)").arg(x, 0, 'f', FREQ_DECIMALS).arg(y, 0, 'f', 1));
        } else {
            cursorLabel->setText("");
        }
    }

    void onTrigSnapshot(QByteArray snap) {
        if (paused) {
            return;
        }
        applyTrigSnapshot(snap);
        startFFT(snap);
    }

    void applyTrigSnapshot(const QByteArray& snap) {
        accSingle.clear();
        seriesSingle->clear();
        const int N = snap.size();
        QVector<QPointF> pts;
        pts.reserve(N);
        const uint8_t* p = reinterpret_cast<const uint8_t*>(snap.constData());
        for (int i = 0; i < N; ++i) {
            pts.append(QPointF(i * TS_XADC_US, yScaled(p[i])));
        }
        seriesSingle->replace(pts);
        if (autoYCheck->isChecked() && N > 0) {
            auto minmax = std::minmax_element(p, p + N);
            double mn = yScaled(*minmax.first);
            double mx = yScaled(*minmax.second);
            if (mn == mx) {
                mx = mn + (isOscSelected() ? 1e-3 : 1.0);
            }
            axisYSingle->setRange(mn, mx);
        }
        if (!userChangedX) {
            axisXSingle->setRange(0.0, std::max(1.0, N * TS_XADC_US));
        }
    }

    void onFramesTargetChanged(int frames) {
        trigLogStrideSpin->setMaximum(std::max(1, frames));
        statusBar()->showMessage(QString("TRIG frames target now %1").arg(frames), 2000);
    }

private:
    enum class FFTWindow { Hann, BlackmanHarris, Kaiser };
    struct FFTResult { QVector<QPointF> psd; double peakHz; };

    static inline uint32_t nextPow2(uint32_t x) {
        if (x <= 1) {
            return 1;
        }
        --x;
        x |= x >> 1;
        x |= x >> 2;
        x |= x >> 4;
        x |= x >> 8;
        x |= x >> 16;
        return x + 1;
    }

    static void bitReverseReorder(std::vector<std::complex<double>>& a) {
        const size_t n = a.size();
        size_t j = 0;
        for (size_t i = 1; i < n; ++i) {
            size_t bit = n >> 1;
            for (; j & bit; bit >>= 1) {
                j ^= bit;
            }
            j ^= bit;
            if (i < j) {
                std::swap(a[i], a[j]);
            }
        }
    }

    static void fftIterative(std::vector<std::complex<double>>& a) {
        const size_t n = a.size();
        bitReverseReorder(a);
        for (size_t len = 2; len <= n; len <<= 1) {
            double ang = -2.0 * M_PI / double(len);
            std::complex<double> wlen(std::cos(ang), std::sin(ang));
            for (size_t i = 0; i < n; i += len) {
                std::complex<double> w(1.0, 0.0);
                for (size_t j = 0; j < len / 2; ++j) {
                    std::complex<double> u = a[i + j];
                    std::complex<double> v = a[i + j + len / 2] * w;
                    a[i + j] = u + v;
                    a[i + j + len / 2] = u - v;
                    w *= wlen;
                }
            }
        }
    }

    static FFTResult computeFFTResult(const QByteArray& snap, FFTWindow wtype) {
        const int Nraw = snap.size();
        const int maxN = 16384;
        int N = std::min(Nraw, maxN);
        int Nfft = (int)nextPow2((uint32_t)N);
        if (Nfft > N) {
            Nfft >>= 1;
        }
        if (Nfft < 256) {
            Nfft = std::min(256, std::max(64, Nraw));
        }
        N = std::min(Nraw, Nfft);
        const uint8_t* p = reinterpret_cast<const uint8_t*>(snap.constData());
        double mean = 0.0;
        for (int i = 0; i < N; ++i) {
            mean += double(p[i]);
        }
        mean /= double(N);
        std::vector<double> win(N, 1.0);
        switch (wtype) {
        case FFTWindow::Hann: {
            for (int i = 0; i < N; ++i) {
                win[i] = 0.5 - 0.5 * std::cos(2.0 * M_PI * i / (N - 1));
            }
            break;
        }
        case FFTWindow::BlackmanHarris: {
            const double a0 = 0.35875;
            const double a1 = 0.48829;
            const double a2 = 0.14128;
            const double a3 = 0.01168;
            for (int n = 0; n < N; ++n) {
                double t = 2.0 * M_PI * n / (N - 1);
                win[n] = a0 - a1 * std::cos(t) + a2 * std::cos(2 * t) - a3 * std::cos(3 * t);
            }
            break;
        }
        case FFTWindow::Kaiser: {
            auto i0 = [](double x) {
                double s = 1.0;
                double y = x * x / 4.0;
                double t = 1.0;
                for (int k = 1; k < 50; ++k) {
                    t *= y / (k * k);
                    s += t;
                    if (t < 1e-12) {
                        break;
                    }
                }
                return s;
            };
            const double beta = 8.6;
            const double denom = i0(beta);
            for (int n = 0; n < N; ++n) {
                double r = 2.0 * n / double(N - 1) - 1.0;
                win[n] = i0(beta * std::sqrt(std::max(0.0, 1.0 - r * r))) / denom;
            }
            break;
        }
        }
        std::vector<std::complex<double>> a(N);
        for (int i = 0; i < N; ++i) {
            a[i] = std::complex<double>((double(p[i]) - mean) * win[i], 0.0);
        }
        fftIterative(a);
        const double fs = XADC_FS_HZ;
        const int nh = N / 2;
        QVector<QPointF> pts;
        pts.reserve(nh);
        int kStart = 1;
        double peakHz = 0.0;
        double maxPow = -1.0;
        int kMax = 1;
        std::vector<double> powSpec(nh + 1);
        for (int k = 0; k <= nh; ++k) {
            double re = a[k].real();
            double im = a[k].imag();
            double pw = re * re + im * im;
            powSpec[k] = pw;
            if (k >= kStart && pw > maxPow) {
                maxPow = pw;
                kMax = k;
            }
        }
        double peakBin = double(kMax);
        if (kMax > 1 && kMax < nh) {
            double y1 = std::log(powSpec[kMax - 1] + 1e-18);
            double y2 = std::log(powSpec[kMax] + 1e-18);
            double y3 = std::log(powSpec[kMax + 1] + 1e-18);
            double denom = (y1 - 2 * y2 + y3);
            if (std::abs(denom) > 1e-12) {
                double delta = 0.5 * (y1 - y3) / denom;
                if (std::isfinite(delta) && std::abs(delta) < 1.0) {
                    peakBin = double(kMax) + delta;
                }
            }
        }
        peakHz = peakBin * fs / double(N);
        double winPow = 0.0;
        for (int i = 0; i < N; ++i) {
            winPow += win[i] * win[i];
        }
        winPow /= N;
        const double scale = 1.0 / (std::sqrt(winPow) * (N * 0.5));
        for (int k = kStart; k <= nh; ++k) {
            double mag = std::sqrt(powSpec[k]) * scale;
            double db = 20.0 * std::log10(mag + 1e-18);
            double fKHz = (k * fs / double(N)) * 1e-3;
            pts.append(QPointF(fKHz, db));
        }
        return FFTResult{std::move(pts), peakHz};
    }

    static QString humanFreq(double hz) {
        if (hz >= 1e9) {
            return QString("%1 GHz").arg(hz / 1e9, 0, 'f', 3);
        }
        if (hz >= 1e6) {
            return QString("%1 MHz").arg(hz / 1e6, 0, 'f', 3);
        }
        if (hz >= 1e3) {
            return QString("%1 kHz").arg(hz / 1e3, 0, 'f', 1);
        }
        return QString("%1 Hz").arg(hz, 0, 'f', 0);
    }

    void startFFT(const QByteArray& snap) {
        if (paused || !rbOSCT->isChecked()) {
            return;
        }
        if (fftBusy.exchange(true)) {
            return;
        }
        FFTWindow wtype = FFTWindow::Hann;
        switch (fftWindowCombo->currentIndex()) {
        case 0: {
            wtype = FFTWindow::Hann;
            break;
        }
        case 1: {
            wtype = FFTWindow::BlackmanHarris;
            break;
        }
        case 2: {
            wtype = FFTWindow::Kaiser;
            break;
        }
        default: {
            break;
        }
        }
        fftWatcher->setFuture(QtConcurrent::run(&MainWindow::computeFFTResult, snap, wtype));
    }

    void setupUI() {
        auto* cw = new QWidget;
        auto* v = new QVBoxLayout(cw);
        setCentralWidget(cw);
        setWindowTitle("FTDI Oscilloscope – Production Grade (SW, OSC, TRIG, Set-Frames, FFT)");
        auto* hPerf = new QHBoxLayout;
        bufferLbl = new QLabel("Buffer: 0 KB");
        rateLbl = new QLabel("Rate: –");
        cursorLabel = new QLabel("");
        cursorLabel->setStyleSheet("QLabel { color: blue; font-weight: bold; }");
        freqCaptionLbl = new QLabel("Freq:");
        freqValueLbl = new QLabel("–");
        freqCaptionLbl->setVisible(false);
        freqValueLbl->setVisible(false);
        freqCaptionLbl->setStyleSheet("QLabel { font-weight: bold; }");
        freqValueLbl->setStyleSheet("QLabel { color: #00897B; font-weight: bold; }");
        hPerf->addWidget(bufferLbl);
        hPerf->addWidget(rateLbl);
        hPerf->addStretch();
        hPerf->addWidget(cursorLabel);
        hPerf->addSpacing(16);
        hPerf->addWidget(freqCaptionLbl);
        hPerf->addWidget(freqValueLbl);
        v->addLayout(hPerf);
        auto* hM = new QHBoxLayout;
        hM->addWidget(new QLabel("Mode:"));
        rbS = new QRadioButton("Sine");
        rbT = new QRadioButton("Triangle");
        rbC = new QRadioButton("Constant");
        rbSW = new QRadioButton("SW");
        rbSWM = new QRadioButton("SW multiple");
        rbOSC = new QRadioButton("OSC");
        rbOSCM = new QRadioButton("OSC multiple");
        rbOSCT = new QRadioButton("OSC TRIG");
        rbS->setChecked(true);
        for (auto* rb : std::initializer_list<QRadioButton*>{rbS, rbT, rbC, rbSW, rbSWM, rbOSC, rbOSCM, rbOSCT}) {
            hM->addWidget(rb);
        }
        hM->addStretch();
        v->addLayout(hM);
        channelRow = new QWidget;
        auto* hChan = new QHBoxLayout(channelRow);
        hChan->setContentsMargins(0, 0, 0, 0);
        hChan->addWidget(new QLabel("Channel:"));
        channelSelector = new QComboBox;
        for (int i = 0; i < PAYLOAD_LEN; ++i) {
            channelSelector->addItem(QString::number(i));
        }
        hChan->addWidget(channelSelector);
        hChan->addStretch();
        v->addWidget(channelRow);
        constRow = new QWidget;
        auto* hC = new QHBoxLayout(constRow);
        hC->setContentsMargins(0, 0, 0, 0);
        hC->addWidget(new QLabel("Const (dec/0x):"));
        constEdit = new QLineEdit;
        constEdit->setFixedWidth(70);
        auto* apply = new QPushButton("Apply");
        hC->addWidget(constEdit);
        hC->addWidget(apply);
        hC->addStretch();
        v->addWidget(constRow);
        auto* hT = new QHBoxLayout;
        trigEnable = new QCheckBox("Trigger");
        trigLvl = new QSpinBox;
        trigLvl->setRange(0, 255);
        trigLvl->setValue(128);
        trigSpanBox = new QComboBox;
        trigSpanBox->addItems({"1","2","4"});
        pauseBtn = new QPushButton("⏸ Pause");
        pauseBtn->setCheckable(true);
        trigVerboseChk = new QCheckBox("Verbose TRIG logs");
        trigVerboseChk->setChecked(true);
        trigLogStrideSpin = new QSpinBox;
        trigLogStrideSpin->setRange(1, DEFAULT_TRIG_FRAMES);
        trigLogStrideSpin->setValue(1);
        framesEdit = new QLineEdit(QString::number(DEFAULT_TRIG_FRAMES));
        framesEdit->setFixedWidth(80);
        framesApplyBtn = new QPushButton("Apply Frames");
        fftWindowLbl = new QLabel("FFT Window:");
        fftWindowCombo = new QComboBox;
        fftWindowCombo->addItems({"Hanning (Hann)","Blackman-Harris","Kaiser"});
        fftWindowLbl->setVisible(false);
        fftWindowCombo->setVisible(false);
        hT->addWidget(trigEnable);
        hT->addWidget(new QLabel("Lvl:"));
        hT->addWidget(trigLvl);
        hT->addWidget(new QLabel("Per:"));
        hT->addWidget(trigSpanBox);
        hT->addSpacing(15);
        hT->addWidget(new QLabel("TRIG Frames:"));
        hT->addWidget(framesEdit);
        hT->addWidget(framesApplyBtn);
        hT->addSpacing(15);
        hT->addWidget(trigVerboseChk);
        hT->addWidget(new QLabel("Log every N frames:"));
        hT->addWidget(trigLogStrideSpin);
        hT->addSpacing(15);
        hT->addWidget(fftWindowLbl);
        hT->addWidget(fftWindowCombo);
        hT->addSpacing(15);
        hT->addWidget(pauseBtn);
        hT->addStretch();
        v->addLayout(hT);
        auto* hScale = new QHBoxLayout;
        hScale->addWidget(new QLabel("X-Min (µs):"));
        xMinSpin = new QDoubleSpinBox;
        xMinSpin->setRange(0.0, 1'000'000.0);
        xMinSpin->setDecimals(2);
        xMinSpin->setValue(0.0);
        hScale->addWidget(xMinSpin);
        hScale->addWidget(new QLabel("X-Max (µs):"));
        xMaxSpin = new QDoubleSpinBox;
        xMaxSpin->setRange(1.0, 1'000'000.0);
        xMaxSpin->setDecimals(2);
        xMaxSpin->setValue(200.0);
        hScale->addWidget(xMaxSpin);
        autoYCheck = new QCheckBox("Auto-Y");
        hScale->addWidget(autoYCheck);
        hScale->addWidget(new QLabel("Y-min:"));
        yMinSpin = new QSpinBox;
        yMinSpin->setRange(0, 255);
        yMinSpin->setValue(0);
        hScale->addWidget(yMinSpin);
        hScale->addWidget(new QLabel("Y-max:"));
        yMaxSpin = new QSpinBox;
        yMaxSpin->setRange(0, 255);
        yMaxSpin->setValue(255);
        hScale->addWidget(yMaxSpin);
        hScale->addStretch();
        v->addLayout(hScale);
        zoomControlsWidget = new QWidget;
        auto* hZoom = new QHBoxLayout(zoomControlsWidget);
        hZoom->setContentsMargins(0, 0, 0, 0);
        auto* zoomInBtn = new QPushButton("🔍+ Zoom In");
        auto* zoomOutBtn = new QPushButton("🔍− Zoom Out");
        auto* resetBtn = new QPushButton("⌂ Reset View");
        hZoom->addWidget(new QLabel("Zoom Controls:"));
        hZoom->addWidget(zoomInBtn);
        hZoom->addWidget(zoomOutBtn);
        hZoom->addWidget(resetBtn);
        hZoom->addWidget(new QLabel("(Drag rectangle to zoom area)"));
        hZoom->addStretch();
        zoomControlsWidget->setEnabled(false);
        v->addWidget(zoomControlsWidget);
        markerControlsWidget = new QWidget;
        auto* markerMainLayout = new QVBoxLayout(markerControlsWidget);
        markerMainLayout->setContentsMargins(0, 0, 0, 0);
        auto* hMarker = new QHBoxLayout;
        auto* clearMarkersBtn = new QPushButton("Clear All Markers");
        hMarker->addWidget(new QLabel("Markers:"));
        hMarker->addWidget(clearMarkersBtn);
        hMarker->addWidget(new QLabel("(Right-click on plot to add marker)"));
        hMarker->addStretch();
        markerMainLayout->addLayout(hMarker);
        markerScrollArea = new QScrollArea;
        auto* markerWidget = new QWidget;
        markerLayout = new QVBoxLayout(markerWidget);
        markerScrollArea->setWidget(markerWidget);
        markerScrollArea->setMaximumHeight(100);
        markerScrollArea->setWidgetResizable(true);
        markerMainLayout->addWidget(markerScrollArea);
        markerControlsWidget->setEnabled(false);
        v->addWidget(markerControlsWidget);
        plotStack = new QStackedWidget;
        v->addWidget(plotStack, 1);
        singlePlotWidget = new QWidget;
        auto* svl = new QVBoxLayout(singlePlotWidget);
        svl->setContentsMargins(0, 0, 0, 0);
        chartSingle = new QChart;
        chartSingle->legend()->hide();
        seriesSingle = new QLineSeries;
        chartSingle->addSeries(seriesSingle);
        axisXSingle = new QValueAxis;
        axisYSingle = new QValueAxis;
        chartSingle->setAxisX(axisXSingle, seriesSingle);
        chartSingle->setAxisY(axisYSingle, seriesSingle);
        axisXSingle->setTitleText("Time (µs)");
        axisYSingle->setTitleText("Level");
        axisYSingle->setRange(0, 255);
        singleChartView = new EnhancedChartView(chartSingle);
        singleChartView->setRenderHint(QPainter::Antialiasing, false);
        chartSingle->setAnimationOptions(QChart::NoAnimation);
        axisXSingle->setMinorTickCount(0);
        axisYSingle->setMinorTickCount(0);
#if QT_CONFIG(opengl)
        seriesSingle->setUseOpenGL(true);
#endif
        svl->addWidget(singleChartView);
        chartFFT = new QChart;
        chartFFT->legend()->hide();
        seriesFFT = new QLineSeries;
        chartFFT->addSeries(seriesFFT);
        axisXFFT = new QLogValueAxis;
        axisXFFT->setTitleText("Frequency (kHz)");
        axisXFFT->setBase(10.0);
        axisXFFT->setLabelFormat("%.5f");
        axisXFFT->setMinorTickCount(0);
        axisYFFT = new QValueAxis;
        axisYFFT->setTitleText("Magnitude (dBFS)");
        axisYFFT->setMinorTickCount(0);
        chartFFT->setAxisX(axisXFFT, seriesFFT);
        chartFFT->setAxisY(axisYFFT, seriesFFT);
        chartFFT->setAnimationOptions(QChart::NoAnimation);
        fftChartView = new EnhancedChartView(chartFFT);
        fftChartView->setRenderHint(QPainter::Antialiasing, false);
#if QT_CONFIG(opengl)
        seriesFFT->setUseOpenGL(true);
#endif
        svl->addWidget(fftChartView);
        svl->setStretch(0, 1);
        svl->setStretch(1, 1);
        fftChartView->setVisible(false);
        plotStack->addWidget(singlePlotWidget);
        multiPlotWidget = new QWidget;
        auto* grid = new QGridLayout(multiPlotWidget);
        grid->setContentsMargins(0, 0, 0, 0);
        for (int p = 0; p < 4; ++p) {
            MultiPlot mp;
            mp.chart = new QChart;
            mp.chart->legend()->hide();
            mp.series = new QLineSeries;
            mp.chart->addSeries(mp.series);
            mp.axisX = new QValueAxis;
            mp.axisY = new QValueAxis;
            mp.chart->setAxisX(mp.axisX, mp.series);
            mp.chart->setAxisY(mp.axisY, mp.series);
            mp.axisX->setTitleText("Time (µs)");
            mp.axisY->setTitleText("Level");
            mp.axisY->setRange(0, 255);
            mp.chartView = new EnhancedChartView(mp.chart);
            mp.chartView->setRenderHint(QPainter::Antialiasing, false);
            mp.chart->setAnimationOptions(QChart::NoAnimation);
            mp.axisX->setMinorTickCount(0);
            mp.axisY->setMinorTickCount(0);
#if QT_CONFIG(opengl)
            mp.series->setUseOpenGL(true);
#endif
            mp.selector = new QComboBox;
            for (int i = 0; i < PAYLOAD_LEN; ++i) {
                mp.selector->addItem(QString::number(i));
            }
            auto* cell = new QWidget;
            auto* lay = new QVBoxLayout(cell);
            lay->setContentsMargins(0, 0, 0, 0);
            lay->addWidget(mp.selector);
            lay->addWidget(mp.chartView);
            grid->addWidget(cell, p / 2, p % 2);
            multiPlots.push_back(std::move(mp));
        }
        plotStack->addWidget(multiPlotWidget);
        plotStack->setCurrentIndex(0);
        connect(zoomInBtn, &QPushButton::clicked, this, &MainWindow::zoomIn);
        connect(zoomOutBtn, &QPushButton::clicked, this, &MainWindow::zoomOut);
        connect(resetBtn, &QPushButton::clicked, this, &MainWindow::resetZoom);
        connect(clearMarkersBtn, &QPushButton::clicked, this, &MainWindow::clearMarkers);
        connect(apply, &QPushButton::clicked, this, &MainWindow::applyConst);
        connect(framesApplyBtn, &QPushButton::clicked, this, &MainWindow::applyFrames);
    }

    void setupWorker() {
        worker = new Worker;
        worker->start();
    }

    void connectSignals() {
        connect(worker, &Worker::newSamples, this, &MainWindow::updatePlot);
        connect(worker, &Worker::rateUpdate, this, [=](double m) { rateLbl->setText(QString("Rate: %1 Mbit/s").arg(m, 0, 'f', 2)); });
        connect(worker, &Worker::bufferLevel, this, [=](size_t l) { bufferLbl->setText(QString("Buffer: %1 KB").arg(l * PAYLOAD_LEN / 1024)); });
        connect(worker, &Worker::trigAckReceived, this, [=] {
            statusBar()->showMessage("OSC-TRIG: ACK received – capturing snapshot…");
        });
        connect(worker, &Worker::trigCaptureProgress, this, [=](int f, int tot) {
            bufferLbl->setText(QString("TRIG: %1/%2 frames").arg(f).arg(tot));
        });
        connect(worker, &Worker::trigSnapshotReady, this, &MainWindow::onTrigSnapshot);
        connect(worker, &Worker::trigFramesTargetChanged, this, &MainWindow::onFramesTargetChanged);
        connect(worker, &Worker::configAckReceived, this, [=](int frames) {
            framesEdit->setText(QString::number(frames));
            statusBar()->showMessage(QString("FPGA confirmed TRIG frames = %1").arg(frames), 1500);
        });
        auto selMode = [=](auto b, Worker::Mode m, bool multi, bool clearOnSelect = false) {
            connect(b, &QRadioButton::toggled, this, [=](bool c) {
                if (!c) {
                    return;
                }
                multiMode = multi;
                plotStack->setCurrentIndex(multi ? 1 : 0);
                if (clearOnSelect) {
                    clearMarkers();
                    clearAllPlots();
                    worker->clearAllChannels();
                    userChangedX = false;
                }
                worker->setMode(m);
                updateScale();
                const bool isTrig = (b == rbOSCT);
                fftChartView->setVisible(isTrig);
                freqCaptionLbl->setVisible(isTrig);
                freqValueLbl->setVisible(isTrig);
                fftWindowLbl->setVisible(isTrig);
                fftWindowCombo->setVisible(isTrig);
                channelRow->setVisible(!isTrig);
                constRow->setVisible(!isTrig);
                if (isTrig) {
                    seriesFFT->clear();
                    freqValueLbl->setText("–");
                    axisXFFT->setRange(0.1, 500.0);
                }
            });
        };
        selMode(rbS, Worker::Mode::Sine, false);
        selMode(rbT, Worker::Mode::Triangle, false);
        selMode(rbC, Worker::Mode::Constant, false);
        selMode(rbSW, Worker::Mode::SwEnterCmd, false, true);
        selMode(rbSWM, Worker::Mode::SwEnterCmd, true, true);
        selMode(rbOSC, Worker::Mode::OscEnterCmd, false, true);
        selMode(rbOSCM, Worker::Mode::OscEnterCmd, true, true);
        selMode(rbOSCT, Worker::Mode::OscTrigCmd, false, true);
        connect(pauseBtn, &QPushButton::toggled, this, &MainWindow::onPauseToggled);
        connect(channelSelector, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::changeChannel);
        auto connScale = [=] {
            updateScale();
            userChangedX = true;
        };
        connect(xMinSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, connScale);
        connect(xMaxSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, connScale);
        connect(yMinSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, connScale);
        connect(yMaxSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, connScale);
        connect(autoYCheck, &QCheckBox::toggled, this, connScale);
        connect(singleChartView, &EnhancedChartView::rectangleZoom, this, &MainWindow::onRectangleZoom);
        connect(singleChartView, &EnhancedChartView::markerAdded, this, &MainWindow::addMarker);
        connect(singleChartView, &EnhancedChartView::mousePosition, this, &MainWindow::onMousePosition);
        connect(fftChartView, &EnhancedChartView::rectangleZoom, this, &MainWindow::onFFTRectangleZoom);
        connect(fftChartView, &EnhancedChartView::markerAdded, this, &MainWindow::addFFTMarker);
        connect(fftChartView, &EnhancedChartView::mousePosition, this, &MainWindow::onFFTMousePosition);
        connect(trigVerboseChk, &QCheckBox::toggled, worker, &Worker::setTrigVerbose);
        connect(trigLogStrideSpin, QOverload<int>::of(&QSpinBox::valueChanged), worker, &Worker::setTrigLogStride);
    }

    void updateStatusMessage() {
        QString msg = "High-priority mode active";
        if (paused) {
            msg += " - PAUSED: Right-click for markers, drag rectangle for zoom, use zoom controls";
        }
        statusBar()->showMessage(msg);
    }

    static void setRangeIfChanged(QValueAxis* ax, double mn, double mx) {
        double curMin = ax->min();
        double curMax = ax->max();
        const double span = curMax - curMin;
        const double newSpan = mx - mn;
        bool need = (mn < curMin) || (mx > curMax) || (newSpan < span * 0.98) || (newSpan > span * 1.02);
        if (need) {
            ax->setRange(mn, (mx == mn) ? mn + 1e-3 : mx);
        }
    }

    void updatePlotSingle() {
        if (rbOSCT->isChecked()) {
            return;
        }
        std::vector<uint8_t> chunk;
        worker->readSamples(chunk, 4096, currentChannel);
        if (chunk.empty()) {
            return;
        }
        accSingle.insert(accSingle.end(), chunk.begin(), chunk.end());
        if (accSingle.size() > MAX_ACC) {
            size_t excess = accSingle.size() - MAX_ACC;
            accSingle.erase(accSingle.begin(), accSingle.begin() + excess);
        }
        int plotSamples = std::min(4000, int(accSingle.size()));
        if (plotSamples <= 0) {
            return;
        }
        const double ts = isOscSelected() ? TS_XADC_US : TS_FTDI_US;
        QVector<QPointF> pts;
        pts.reserve(plotSamples);
        size_t startIdx = accSingle.size() - plotSamples;
        for (int i = 0; i < plotSamples; ++i) {
            pts.append(QPointF(i * ts, yScaled(accSingle[startIdx + i])));
        }
        seriesSingle->replace(pts);
        if (autoYCheck->isChecked()) {
            auto start = accSingle.begin() + startIdx;
            auto end = accSingle.end();
            auto mnmx = std::minmax_element(start, end);
            double mn = yScaled(*mnmx.first);
            double mx = yScaled(*mnmx.second);
            setRangeIfChanged(axisYSingle, mn, mx);
        }
    }

    void updatePlotMulti() {
        const double ts = isOscSelected() ? TS_XADC_US : TS_FTDI_US;
        for (size_t p = 0; p < multiPlots.size(); ++p) {
            int ch = multiPlots[p].selector->currentIndex();
            std::vector<uint8_t> chunk;
            worker->readSamples(chunk, 2048, ch);
            if (chunk.empty()) {
                continue;
            }
            auto& acc = multiPlots[p].acc;
            acc.insert(acc.end(), chunk.begin(), chunk.end());
            if (acc.size() > MAX_ACC) {
                size_t excess = acc.size() - MAX_ACC;
                acc.erase(acc.begin(), acc.begin() + excess);
            }
            int plotSamples = std::min(4000, int(acc.size()));
            if (plotSamples <= 0) {
                continue;
            }
            QVector<QPointF> pts;
            pts.reserve(plotSamples);
            size_t startIdx = acc.size() - plotSamples;
            for (int i = 0; i < plotSamples; ++i) {
                pts.append(QPointF(i * ts, yScaled(acc[startIdx + i])));
            }
            multiPlots[p].series->replace(pts);
            if (autoYCheck->isChecked()) {
                auto start = acc.begin() + startIdx;
                auto end = acc.end();
                auto mnmx = std::minmax_element(start, end);
                double mn = yScaled(*mnmx.first);
                double mx = yScaled(*mnmx.second);
                setRangeIfChanged(multiPlots[p].axisY, mn, mx);
            }
        }
    }

    struct MultiPlot {
        QChart* chart;
        QLineSeries* series;
        QValueAxis* axisX;
        QValueAxis* axisY;
        QComboBox* selector;
        std::deque<uint8_t> acc;
        EnhancedChartView* chartView;
    };

    QLabel* bufferLbl;
    QLabel* rateLbl;
    QLabel* cursorLabel;
    QLabel* freqCaptionLbl;
    QLabel* freqValueLbl;
    QRadioButton* rbS;
    QRadioButton* rbT;
    QRadioButton* rbC;
    QRadioButton* rbSW;
    QRadioButton* rbSWM;
    QRadioButton* rbOSC;
    QRadioButton* rbOSCM;
    QRadioButton* rbOSCT;
    QComboBox* channelSelector;
    QComboBox* trigSpanBox;
    QLineEdit* constEdit;
    QCheckBox* trigEnable;
    QCheckBox* autoYCheck;
    QCheckBox* trigVerboseChk;
    QSpinBox* trigLogStrideSpin;
    QSpinBox* trigLvl;
    QSpinBox* yMinSpin;
    QSpinBox* yMaxSpin;
    QPushButton* pauseBtn;
    QDoubleSpinBox* xMinSpin;
    QDoubleSpinBox* xMaxSpin;
    QLineEdit* framesEdit;
    QPushButton* framesApplyBtn;
    QWidget* channelRow{nullptr};
    QWidget* constRow{nullptr};
    QLabel* fftWindowLbl{nullptr};
    QComboBox* fftWindowCombo{nullptr};
    QWidget* zoomControlsWidget;
    QWidget* markerControlsWidget;
    QScrollArea* markerScrollArea;
    QVBoxLayout* markerLayout;
    QStackedWidget* plotStack;
    QWidget* singlePlotWidget;
    QWidget* multiPlotWidget;
    QChart* chartSingle;
    QLineSeries* seriesSingle;
    QValueAxis* axisXSingle;
    QValueAxis* axisYSingle;
    EnhancedChartView* singleChartView;
    std::vector<MultiPlot> multiPlots;
    bool paused = false;
    bool multiMode = false;
    int currentChannel = 0;
    bool userChangedX = false;
    std::deque<uint8_t> accSingle;
    QElapsedTimer limiter;
    struct Marker { int id; double x; double y; };
    std::vector<Marker> markers;
    std::vector<Marker> fftMarkers;
    int nextMarkerId = 1;
    int nextFFTMarkerId = 1;
    bool originalRangesStored = false;
    double originalXMin;
    double originalXMax;
    int originalYMin;
    int originalYMax;
    Worker* worker;
    QChart* chartFFT;
    QLineSeries* seriesFFT;
    QLogValueAxis* axisXFFT;
    QValueAxis* axisYFFT;
    EnhancedChartView* fftChartView;
    QFutureWatcher<FFTResult>* fftWatcher = nullptr;
    std::atomic<bool> fftBusy{false};
};

int main(int argc, char** argv) {
    QApplication a(argc, argv);
    MainWindow w;
    w.resize(1200, 980);
    w.show();
    return a.exec();
}




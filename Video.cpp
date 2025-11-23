#pragma once

#include <cstdint>
#include <string>
#include <vector>

/*
 * Basic assumptions about how a frame looks in memory:
 * - Packed buffer, row after row.
 * - For example RGB8 (3 bytes per pixel) or some packed YUV.
 * - Frame is row-major. If there is padding at the end of a row,
 *   we get the stride in bytes from the SDK.
 */

enum class PixelFormat {
    RGB8,          // 3 bytes per pixel
    YUV422_10BIT,  // packed 10-bit YUV 4:2:2
    RAW8,
    RAW16
};

struct CameraConfig {
    std::string deviceName;  // e.g. "xiB-64-0"
    int         width;
    int         height;
    int         fps;
    PixelFormat format;
};


struct CropRegion {
    int x;      // left pixel index in source frame
    int y;      // top pixel index in source frame
    int width;  // width of cropped region
    int height; // height of cropped 
};


struct SdiConfig {
    int         deviceIndex;  // SDI card index
    PixelFormat format;       // e.g. YUV422_10BIT
    int         width;
    int         height;
    int         fps;
};

struct VideoFormat {
    int         width;
    int         height;
    int         fps;
    PixelFormat format;
};

/*
 * Very simple logging/diagnostics stub.
 * In a real system this would go to syslog / tracing / CLI, etc.
 */
class Diagnostics {
public:
    void logInfo(const std::string& msg);
    void logError(const std::string& msg);

    // Periodic frame stats (e.g. once per second)
    void logFrameStats(double fps, bool bufferOverrun);

    // Error codes from SDI driver / camera SDK
    void logSdiError(int errorCode);
    void logCameraError(int errorCode);
};


/*
 * PCIe camera wrapper.
 * Hides the vendor SDK (e.g. Ximea) behind a small C++ API.
 */
class PcieCamera {
public:
    PcieCamera();

    // Open device, set resolution / fps / format, allocate buffers if needed.
    bool init(const CameraConfig& cfg);

    bool start();  // start acquisition
    bool stop();   // stop acquisition

    /*
     * Acquire a pointer to the next frame from the SDK.
     * - Returns nullptr on failure / timeout.
     * - outStrideBytes tells us how many bytes per row (may be >= width*bpp).
     * - The pointer is owned by the SDK and is valid until releaseFrame().
     */
    uint8_t* acquireFrame(int& outStrideBytes);

    // Return a frame buffer to the SDK.
    void releaseFrame(uint8_t* framePtr);

    // Try to recover from “soft” errors (timeouts etc.).
    bool recoverFromError();

    // Tear down and re-open the device with the same config.
    bool reconnect();

    bool isStreaming() const { return m_streaming; }
    const CameraConfig& getConfig() const { return m_cfg; }

private:
    void*        m_handle = nullptr;  // camera SDK handle
    CameraConfig m_cfg{};
    bool         m_streaming = false;

    int          m_bytesPerPixel = 0; // derived from PixelFormat
};


/*
 * FrameCropper:
 * Copies a rectangular region from a source frame into an output buffer.
 * No allocations in the hot path – caller owns the buffers.
 */
class FrameCropper {
public:
    explicit FrameCropper(int bytesPerPixel)
        : m_bytesPerPixel(bytesPerPixel) {}

    /*
     * Crop a region from inFrame into outFrame.
     *
     * inFrame       - pointer to top-left byte of the full frame
     * inWidth/Height- full frame size in pixels
     * inStrideBytes - number of bytes per row (>= inWidth * bpp)
     * outFrame      - pre-allocated buffer for the cropped frame
     * region        - crop window in source coordinates
     *
     * Returns false if region is invalid or out of bounds.
     */
    bool crop(const uint8_t*   inFrame,
              int              inWidth,
              int              inHeight,
              int              inStrideBytes,
              uint8_t*         outFrame,
              const CropRegion& region);

    int bytesPerPixel() const { return m_bytesPerPixel; }

private:
    int m_bytesPerPixel;
};


/*
 * SDI output wrapper.
 * Could be backed by a driver API, GStreamer, FFmpeg, etc.
 */
class SdiOutput {
public:
    SdiOutput();

    // Basic SDI device init (open card, set defaults, etc.).
    bool init(const SdiConfig& cfg);

    /*
     * Negotiate the actual video format with the SDI stack.
     * May adjust size/fps/format to whatever the card actually supports.
     */
    bool negotiateFormat(const VideoFormat& requested,
                         VideoFormat&       agreed);

    // Send one frame to SDI. Buffer must match the negotiated format.
    bool sendFrame(const uint8_t* frameData, size_t sizeBytes);

    // Try to recover from SDI write errors (underrun, timeout, etc.).
    bool recoverFromError();

    void stop();

    const VideoFormat& currentFormat() const { return m_currentFormat; }

private:
    void*       m_deviceHandle = nullptr;   // SDI driver handle
    VideoFormat m_currentFormat{};
};


/*
 * PipelineController – single-threaded version.
 *
 * Responsibilities:
 * - Init camera, cropper, and SDI output.
 * - Main loop:
 *     1) Acquire frame from camera.
 *     2) Crop.
 *     3) Send to SDI.
 *     4) Update diagnostics.
 *
 * Thread-safety note:
 * - Here everything runs in one thread to keep it simple.
 * - In a real high-FPS setup we’d probably split:
 *     * capture thread (camera SDK only)
 *     * processing/output thread (crop + SDI)
 *     * small ring buffer or double-buffer between them
 *   with some mutex/lock-free mechanism around shared state.
 */
class PipelineController {
public:
    PipelineController(PcieCamera&   cam,
                       FrameCropper& cropper,
                       SdiOutput&    sdi,
                       Diagnostics&  diag)
        : m_cam(cam)
        , m_cropper(cropper)
        , m_sdi(sdi)
        , m_diag(diag)
        , m_running(false)
    {}

    /*
     * Pipeline init:
     * - Init camera.
     * - Init SDI.
     * - Negotiate common video format.
     * - Set crop region and allocate internal buffer.
     */
    bool init(const CameraConfig& camCfg,
              const SdiConfig&    sdiCfg,
              const CropRegion&   cropRegion);

    /*
     * Main loop. Runs until stop() is called or a fatal error occurs.
     *
     * Rough flow:
     *   while (m_running) {
     *       acquire frame
     *       crop into internal buffer
     *       send to SDI
     *       update FPS / diagnostics
     *   }
     */
    void runMainLoop();

    // Ask the loop to exit.
    void stop() { m_running = false; }

private:
    PcieCamera&   m_cam;
    FrameCropper& m_cropper;
    SdiOutput&    m_sdi;
    Diagnostics&  m_diag;

    CropRegion    m_cropRegion{};
    bool          m_running;

    // Single output buffer for cropped frames.
    // Can be replaced with double-buffer / ring buffer if needed.
    std::vector<uint8_t> m_croppedFrameBuffer;
};


/// ======= minimal inline implementations (for illustration only) =======

inline bool FrameCropper::crop(const uint8_t*   inFrame,
                               int              inWidth,
                               int              inHeight,
                               int              inStrideBytes,
                               uint8_t*         outFrame,
                               const CropRegion& region)
{
    // Basic bounds checks
    if (region.x < 0 || region.y < 0 ||
        region.width  <= 0 || region.height <= 0)
    {
        return false;
    }

    if (region.x + region.width  > inWidth ||
        region.y + region.height > inHeight)
    {
        return false;
    }

    const int bytesPerRow = region.width * m_bytesPerPixel;

    // Pointer to the first row inside the crop window
    const uint8_t* srcRowPtr =
        inFrame + region.y * inStrideBytes + region.x * m_bytesPerPixel;

    uint8_t* dstRowPtr = outFrame;

    // Copy row by row
    for (int row = 0; row < region.height; ++row) {
        // In real code we’d simply do: std::memcpy(dstRowPtr, srcRowPtr, bytesPerRow);
        for (int b = 0; b < bytesPerRow; ++b) {
            dstRowPtr[b] = srcRowPtr[b];
        }

        srcRowPtr += inStrideBytes;
        dstRowPtr += bytesPerRow;
    }

    return true;
}


/*
 * Example implementation of the main loop.
 * This is only structural – no real timing code.
 */
inline void PipelineController::runMainLoop()
{
    m_running = true;

    while (m_running) {
        int      strideBytes = 0;
        uint8_t* srcFrame    = m_cam.acquireFrame(strideBytes);

        if (!srcFrame) {
            m_diag.logCameraError(-1);
            if (!m_cam.recoverFromError()) {
                m_running = false;
                break;
            }
            continue;
        }

        const int bytesPerPixel = m_cropper.bytesPerPixel();
        const int outWidth      = m_cropRegion.width;
        const int outHeight     = m_cropRegion.height;

        const size_t outSize =
            static_cast<size_t>(outWidth) *
            static_cast<size_t>(outHeight) *
            static_cast<size_t>(bytesPerPixel);

        if (m_croppedFrameBuffer.size() < outSize) {
            m_croppedFrameBuffer.resize(outSize);
        }

        uint8_t* dstFrame = m_croppedFrameBuffer.data();

        const auto& camCfg = m_cam.getConfig();

        // Crop into our local buffer
        bool ok = m_cropper.crop(srcFrame,
                                 camCfg.width,
                                 camCfg.height,
                                 strideBytes,
                                 dstFrame,
                                 m_cropRegion);

        // Release camera buffer as early as possible
        m_cam.releaseFrame(srcFrame);

        if (!ok) {
            m_diag.logError("Crop failed (region or pointers invalid).");
            continue;
        }

        // Send to SDI
        if (!m_sdi.sendFrame(dstFrame, outSize)) {
            m_diag.logSdiError(-2);
            if (!m_sdi.recoverFromError()) {
                m_running = false;
                break;
            }
        }

        // FPS / stats handling would go here (using some clock API).
        // m_diag.logFrameStats(...);
    }

    // Any shutdown / stop logic for camera and SDI would go here.
}

/*
 * For high-FPS (>60fps) or heavy processing:
 * 
 * Thread 1 (Capture):           Thread 2 (Process):
 * ===================           ===================
 * while(running) {              while(running) {
 *   frame = acquire()             frame = ring.pop()
 *   ring.push(frame)              crop(frame)
 *   release(frame)                sdi.send(frame)
 * }                             }
 * 
 * Advantages:
 * - Camera never waits for SDI
 * - Better CPU utilization
 * - Can handle burst loads
 * 
 * Need to add:
 * - LockFreeRingBuffer<Frame, 4>
 * - std::thread objects
 * - Proper shutdown synchronization
 */
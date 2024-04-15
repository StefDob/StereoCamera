
#pragma once

#include <memory>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <string>
#include <chrono>

#include <opencv2/opencv.hpp>

using Clock = std::chrono::steady_clock;

struct MatWithTimestamp {
    cv::Mat mat;
    //PerformanceMetrics::TimePoint timestamp;
    std::chrono::time_point<Clock> timestamp ;
};

class VideoFrame final {
public:
    cv::Mat frame;
    std::chrono::time_point<Clock> timestamp;

    std::size_t sourceIdx = 0;
    
    VideoFrame() = default;

    VideoFrame& operator =(VideoFrame const& vf) = delete;
};


class VideoSource {
public:
    virtual bool isRunning() const = 0;

    virtual void start() = 0;

    virtual bool read(VideoFrame& frame) = 0;

    virtual float getAvgReadTime() const = 0;

    virtual ~VideoSource();
};

class GeneralCapture{
    
    std::thread workThread;
    const bool isAsync;
    std::atomic_bool running = {true};
    //std::atomic_bool takeFrame = {false};

    std::mutex mutex;
    //std::condition_variable condVar;
    std::condition_variable hasFrame;
    std::queue< MatWithTimestamp> queue;

    bool realFps;

    const size_t queueSize;
    //MatWithTimestamp curFrame;
    //template<bool CollectStats>
    MatWithTimestamp captureFrame();

    //template<bool CollectStats>
    void startImpl();

public:
    GeneralCapture(bool async, int deviceID, int apiID, size_t queueSize_);

    GeneralCapture();

    ~GeneralCapture() ;

    void start();

    bool isRunning() const;

    void stop();

    bool read(cv::Mat& frame, std::chrono::time_point<Clock>& timestamp);
    bool read(void);

    std::unique_ptr<cv::VideoCapture> cap;
    MatWithTimestamp curFrame;

    std::condition_variable condVar;
    //std::condition_variable hasFrame;

     std::atomic_bool takeFrame = {false};

    /*
    float getAvgReadTime() const override {
        return perfTimer.getValue();
    }
    */

private:
    
    static void thread_fn(GeneralCapture*);
};



#include <atomic>
#include <chrono>
#include <memory>
#include <numeric>
#include <queue>
#include <string>

#include "GeneralCapture.hpp"


/*
class GeneralCapture{
    
    std::thread workThread;
    const bool isAsync;
    std::atomic_bool running = {true};
    std::atomic_bool takeFrame = {false};

    std::mutex mutex;
    std::condition_variable condVar;
    std::condition_variable hasFrame;
    std::queue< MatWithTimestamp> queue;

    std::unique_ptr<cv::VideoCapture> cap;

    bool realFps;

    const size_t queueSize;
    MatWithTimestamp curFrame;
    //template<bool CollectStats>
    MatWithTimestamp captureFrame();

    //template<bool CollectStats>
    void startImpl();

public:
    GeneralCapture(bool async, int deviceID, int apiID, size_t queueSize_, bool realFps_);

    ~GeneralCapture() ;

    void start();

    bool isRunning() const;

    void stop();

    bool read(cv::Mat& frame, std::chrono::time_point<Clock>& timestamp);
    bool read(void);

    /*
    float getAvgReadTime() const override {
        return perfTimer.getValue();
    }
    */
/*
private:
    
    static void thread_fn(GeneralCapture*);
};
*/


namespace {
bool isNumeric(const std::string& str) {
    return std::strspn(str.c_str(), "0123456789") == str.length();
}
}  // namespace


MatWithTimestamp GeneralCapture::captureFrame() {
    const auto timestamp = std::chrono::steady_clock::now();
    cv::Mat img;
    if (!cap->read(img)) {
        throw std::runtime_error("The image can't be captured from the camera");
    }
    
    
    return {img, timestamp};
    //return { cap->read(), timestamp };
}


GeneralCapture::GeneralCapture(bool async, int deviceID, int apiID, size_t queueSize_):
    isAsync(async),
    //cap(openImagesCapture(name, loopVideo)),
    cap( std::unique_ptr<cv::VideoCapture>(new cv::VideoCapture() ) ),
    queueSize(queueSize_) 
    {
        cap->open(deviceID,apiID);
        if (!cap->isOpened()) {
            std::string errorMessage = std::string("Error oppening the camera with ID: ")+std::to_string(deviceID);
            throw std::runtime_error(errorMessage);
            delete this;
        }
    }

GeneralCapture::~GeneralCapture() {
    stop();
}

bool GeneralCapture::isRunning() const {
    return running;
}


void GeneralCapture::thread_fn(GeneralCapture *vs) {
    while (vs->running) {
        
        std::cout<<"OnThread-> Gonna Take a frame, queue size: " << vs->queue.size() << std::endl;

        MatWithTimestamp frame = vs->captureFrame();
        const bool result = frame.mat.data;
        if (!result) {
            vs->running = false; // stop() also affects running, so override it only when out of frames
        }
        

        std::unique_lock<std::mutex> lock(vs->mutex);
        vs->condVar.wait(lock, [&]() {
            return ((vs->queue.size() < vs->queueSize || !vs->running) && vs->takeFrame); // queue has space or source ran out of frames
        });
        
        vs->queue.push( frame );
        vs->hasFrame.notify_one();
        vs->takeFrame = false;
        std::cout<<"Frame Taked, queue size: " << vs->queue.size() << std::endl;
        /*
        MatWithTimestamp frame = vs->captureFrame();
        const bool result = frame.mat.data;
        if (!result) {
            vs->running = false; // stop() also affects running, so override it only when out of frames
            std::cout<<"OnThread - Could Not Take frame, queue size: " << vs->queue.size() << std::endl;
        }
        else{
            vs->queue.push( frame );
            vs->hasFrame.notify_one();
            std::cout<<"Frame Taked, queue size: " << vs->queue.size() << std::endl;
        }
        */
    }
}


void GeneralCapture::startImpl(){
    if (isAsync) {
        running = true;
        takeFrame = false;
        workThread = std::thread(&GeneralCapture::thread_fn, this);
    }
}

void GeneralCapture::start() {
   
    startImpl();
}

void GeneralCapture::stop() {
    if (isAsync) {
        running = false;
        condVar.notify_one();
        if (workThread.joinable()) {
            workThread.join();
        }
    }
}

bool GeneralCapture::read(cv::Mat& frame, std::chrono::time_point<Clock>& timestamp) {
    bool res;
    std::cout<<" Print a frame by second Camera queue size= "<<queue.size()<<std::endl;
           
    {
        std::unique_lock<std::mutex> lock(mutex);
        hasFrame.wait(lock, [&]() {
            return !queue.empty() || !running;
        });
        //res = queue.front().first;
        //frame = queue.front().second.mat;
        //timestamp = queue.front().second.timestamp;
        frame = queue.front().mat; // because it is not a pair as before
        timestamp = queue.front().timestamp;
        if ( queue.size() > 1 || queueSize == 1) {
            queue.pop();
        }
        lock.unlock();
        res = frame.data;
    }
    condVar.notify_one();  // notify tha kanw ekei pou tha tou lew takeFrame=true   
    //return res;
    

    return res;
}

bool GeneralCapture::read() {

    return read(curFrame.mat, curFrame.timestamp);
}
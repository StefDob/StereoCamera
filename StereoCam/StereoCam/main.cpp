#include <thread>

#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include "GeneralCapture.hpp"

using namespace std;
//using std::endl;

/**
 * @brief Function that runs on a separate thread to take a frame from a camera
 * 
 */
void thread_fn(cv::VideoCapture cap)
{
    cv::Mat frame;
    string txt = "FPS: ";
    cv::Point org(30,30);  //( (img.cols - textsize.width), vPos);
    cv::Scalar colorTxt = cv::Scalar(0,255,0);
    cv::namedWindow("thread2 Simple Demo", cv::WINDOW_AUTOSIZE);

    auto startTime = chrono::steady_clock::now(); 
    while(true)
    {
        //auto startTime = chrono::steady_clock::now();
        bool res = cap.read(frame);
        if(!res)
        {
            cerr<<this_thread::get_id()<<" Camera is not ready...Error"<<endl;
            break;
        }
        // check if we succeeded
        if (frame.empty()) {
            cerr <<this_thread::get_id()<< "thread2 - ERROR! blank frame grabbed\n";
            break;
        }
        
        // show live and wait for a key with timeout long enough to show images
        //cv::imshow("thread2 Simple Demo", frame);
        if (cv::waitKey(1) >= 0)
            break;

        auto endTime = chrono::steady_clock::now();
        double fps = ( 1.0/(chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count()) ) *1000.0;
        startTime = chrono::steady_clock::now(); 
        txt += (to_string(fps));
        cout<<this_thread::get_id()<<" = "<< txt<<endl;
        cv::putText(frame, txt, org, cv::FONT_HERSHEY_COMPLEX, 1, colorTxt, 1, cv::LINE_8);
        cv::imshow("thread2 Simple Demo", frame);

        cout <<this_thread::get_id()<< " Elapsed time in milliseconds: " 
        << chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count()
        << " ms" 
        <<", FPS = " << fps << " Hz." << endl;
        txt = "FPS: ";
    }

    cv::destroyWindow("thread2 Simple Demo");

}


/**
 * @brief VideoCapObject
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */




int main(int argc, char** argv) {

    try {
        if (argc != 1) {
            cout << "Expecting a image file to be passed to program" << endl;
            return -1;
        }

        int deviceID = 0;             // 0 = open default camera
        int apiID = cv::CAP_ANY;      // 0 = autodetect default API
        int queueSize = 4;

        //VideoFrame myCap();
        //GeneralCapture mGenCap(true, deviceID, cv::CAP_ANY, queueSize);
        std::unique_ptr<GeneralCapture> mGenCap;

        MatWithTimestamp secondCamFrame;

        cv::Mat frame;
        string txt = "FPS: ";
        cv::Point org(10,20);  //( (img.cols - textsize.width), vPos);
        cv::Scalar colorTxt = cv::Scalar(255,  0,  0);
        auto startTime = chrono::steady_clock::now();

        cv::VideoCapture cap;
        //cv::VideoCapture cap2;
        deviceID = 2;             // 0 = open default camera

        apiID = cv::CAP_ANY;      // 0 = autodetect default API
        // open selected camera using selected API
        cap.open(deviceID, apiID);

        // second camera, open it 
        bool stereo = true;
        if(stereo)
        {
            int deviceID = 0;             // 0 = open default camera
            int apiID = cv::CAP_ANY;      // 0 = autodetect default API
            int queueSize = 20;
            // open selected camera using selected API
            //cap2.open(deviceID, apiID);
            mGenCap = std::make_unique<GeneralCapture>(true, deviceID, cv::CAP_ANY, queueSize);
            mGenCap->cap->set(cv::CAP_PROP_FRAME_HEIGHT , 720.0);
            //mGenCap.cap->open(deviceID, apiID);
            //mGenCap(true, deviceID, cv::CAP_ANY, queueSize);
        }


        // check if we succeeded
        if (!cap.isOpened()) {
            cerr << "ERROR! Unable to open camera\n";
            return -1;
        }

            //--- GRAB AND WRITE LOOP
        cout <<this_thread::get_id<< "Start grabbing" << endl
            << "Press any key to terminate" << endl;

        cv::namedWindow("Simple Demo", cv::WINDOW_AUTOSIZE);
        
        // start second thread
        if(stereo)
        {
            //thread t2 (thread_fn, cap2);
            mGenCap->start();
            
            cv::namedWindow("SecondCam", cv::WINDOW_AUTOSIZE);
        }

        while(true)
        {
            auto startTime = chrono::steady_clock::now();
            bool res = cap.read(frame);
            if(!res)
            {
                cerr<<this_thread::get_id<<" Camera is not ready...Error"<<endl;
                break;
            }
            // check if we succeeded
            if (frame.empty()) {
                cerr<< this_thread::get_id << " ERROR! blank frame grabbed\n";
                break;
            }

            /**Trigger the thread to take a frame*/
            mGenCap->takeFrame = true;
            mGenCap->condVar.notify_one();

            /** For the second camera now*/
            res  = mGenCap->read();
            if(!res)
            {
                cerr<<this_thread::get_id<<"2nd Camera is not ready...Error"<<endl;
                break;
            }
            std::cout<<"First Cam Frame tooked "<<std::endl;
            secondCamFrame = mGenCap->curFrame;
            if(secondCamFrame.mat.empty()){
                cerr<< this_thread::get_id << "2nd ERROR! blank frame grabbed\n";
                break;
            }
            //notify to take a new frame
            //mGenCap->takeFrame = true;
            //mGenCap->condVar.notify_one();

            /**End For the second camera now*/
            
            // show live and wait for a key with timeout long enough to show images
            //cv::imshow("Simple Demo", frame);
            if (cv::waitKey(1) >= 0)
                break;

            auto endTime = chrono::steady_clock::now();
            double fps = ( 1.0/(chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count()) ) *1000.0;
            txt +=(to_string(fps));
            cv::putText(frame, txt, org, cv::FONT_HERSHEY_PLAIN, 1, colorTxt, 1, 2);
            cv::imshow("Simple Demo", frame);


            //cv::putText(secondCamFrame.mat, txt, org, cv::FONT_HERSHEY_PLAIN, 1, colorTxt, 1, 2);
            cv::imshow("SecondCam", secondCamFrame.mat);


            

            cout <<this_thread::get_id<< " -Main Elapsed time in milliseconds: " 
            << chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count()
            << " ms" 
            <<", FPS = " << fps << " Hz." << endl;
            txt = "FPS: ";
        }

        /*
        cv::Mat img = cv::imread(argv[1]);
        
        if (img.empty()) {
            cout << "Not a valid image file" << endl;
            return -1;
        }
        
        cv::namedWindow("Simple Demo", cv::WINDOW_AUTOSIZE);
        cv::imshow("Simple Demo", img);
        
        cv::waitKey(0);
        */
        cv::destroyAllWindows();
    }
    catch(std::runtime_error &e){
         std::cout << " a standard exception was caught, with message '\n"
                  << e.what() << "'\n";
    }
    return 0;
}
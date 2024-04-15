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
 * @brief Create a Rect object 
 * 
 * @param img 
 * @param startPt 
 * @param dims 
 */
void createRect(cv::Mat &img, cv::Point &startPt, cv::Point &dims )
{
    //create the rectangle by using the startPt as the Point of the top-left Corner
    // and the dims.x = width, dims.y = height
    cv::Rect myRect{startPt.x, startPt.y,dims.x, dims.y };
    int i=1;
    for(int x=myRect.x; x<myRect.x+myRect.width; x++, i++)
    {
        if(i==dims.x){
            std::cout<<" Rectangle Legth locos "<<i<<std::endl;
        }
    }
    //
    cv::rectangle(img, myRect, cv::Scalar(255,0,0), 2, cv::LINE_8, 0 );

} 

/**
 * @brief Finds Mathing between the inputImage and the template-object Image.
 *        The results is an array of size = W_inpImg - w_template +1 x H_inpImg - h_template +1 
 *        Matching method is specified by the integer value matchMethod
 * 
 * @param inputImg = Input image to be searched for a match
 * @param objImg = The object to be found to the input image
 * @param res =  It must be single-channel 32-bit floating-point. If image is W×H and templ is w×h , then result is (W−w+1)×(H−h+1)
 * @param matchMethod = Parameter specifying the comparison method, see TemplateMatchModes
 */
cv::Point MatchingMethod(const cv::Mat& inputImg, const cv::Mat& objImg, cv::Mat& res, int matchMethod)
{
    bool methodAcceptsMask = (cv::TM_CCORR == matchMethod || matchMethod == cv::TM_CCORR_NORMED);

    int result_cols = inputImg.cols - objImg.cols + 1;
    int result_rows = inputImg.rows - objImg.rows + 1;
    //chekc for type and then convert...
    cv::Mat grayImg;
    //cv::cvtColor(inputImg, grayImg, cv::COLOR_BGR2GRAY);
    std::cout<<"  objImg.cols ="<<  objImg.cols<<", objImg.rows"<<objImg.rows<<std::endl;
    inputImg.copyTo(grayImg);
    //cv::imshow( "grayImg Img", grayImg );
    cv::imshow( "Template", objImg );

    // check if it is the accepted method
    if(methodAcceptsMask)
    {
        res.create(result_rows, result_cols, CV_32FC1 );
        cv::matchTemplate( grayImg, objImg, res , matchMethod); 	 
        std::cout<<" Finding mathces finished "<<std::endl;
    }

    normalize( res, res, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );
    double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
    cv::Point matchLoc;
    // find the match position
    minMaxLoc( res, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );
    if( matchMethod  == cv::TM_SQDIFF || matchMethod == cv::TM_SQDIFF_NORMED )
        { matchLoc = minLoc; }
    else
        { matchLoc = maxLoc; }
    
     std::cout<<" Match Loc X ="<<  matchLoc.x<<", matchLoc.Y "<<matchLoc.y<<std::endl;
    // print the match to the sourche image
    rectangle( inputImg, matchLoc, cv::Point( matchLoc.x + objImg.cols , matchLoc.y + objImg.rows ), cv::Scalar::all(0), 2, 8, 0 );
    rectangle( res, matchLoc, cv::Point( matchLoc.x + objImg.cols , matchLoc.y + objImg.rows ), cv::Scalar::all(0), 2, 8, 0 );
    cv::imshow( "Found Match in ROI", inputImg );
    return matchLoc;
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
        int queueSize = 2;
        int matchMethod = cv::TM_CCORR_NORMED; // cv::TM_CCORR;

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
        deviceID = 0;             // 0 = open default camera

        apiID = cv::CAP_ANY;      // 0 = autodetect default API
        // open selected camera using selected API
        cap.open(deviceID, apiID);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT , 480.0);

        // second camera, open it 
        bool stereo = true;
        if(stereo)
        {
            int deviceID = 2;             // 0 = open default camera
            int apiID = cv::CAP_ANY;      // 0 = autodetect default API
            int queueSize = 4;//2;
            // open selected camera using selected API
            mGenCap = std::make_unique<GeneralCapture>(true, deviceID, cv::CAP_ANY, queueSize);
            mGenCap->cap->set(cv::CAP_PROP_FRAME_HEIGHT , 480.0);
        }


        // check if we succeeded
        if (!cap.isOpened()) {
        //while (!cap.isOpened()) {
            string msg="ERROR! Unable to open mains thread camera\n";
            cerr << msg;
            throw std::runtime_error(msg);
            return -1;
            //deviceID++;
            //cap.open(deviceID, apiID);
            //cout<<"Main thread Camera ID = "<< deviceID <<endl;
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

        //auto startTime = chrono::steady_clock::now();
        auto secondCameraTime = chrono::steady_clock::now();
        while(true)
        {

           auto startTime = chrono::steady_clock::now();
            
            /**Trigger the thread to take a frame*/
            mGenCap->takeFrame = true;
            mGenCap->condVar.notify_one();
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

            /** For the second camera now*/
            res  = mGenCap->read();
            if(stereo && !res)
            {
                cerr<<this_thread::get_id<<"2nd Camera is not ready...Error"<<endl;
                break;
            }
            std::cout<<"First Cam Frame tooked "<<std::endl;
            secondCamFrame = mGenCap->curFrame;
            
            if(stereo && secondCamFrame.mat.empty()){
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
            
            /** Create a rectangle at the top left cornen*/
            cv::Point topLeftCorner{20,30};
            cv::Point widthHeigthPt{70,50};
            cv::Rect myRect{topLeftCorner.x, topLeftCorner.y,widthHeigthPt.x, widthHeigthPt.y };
            cv::Mat templImg{myRect.width, myRect.height};
            //frame.copyTo(templImg, frame );
            do{templImg = frame(myRect).clone();}while(false); // do it once
            //templImg = frame(myRect).clone();
            //cvtColor(templImg, templImg, cv::COLOR_BGR2GRAY);
            // add rectangle to image from the main thread camera
            cv::rectangle(frame, myRect, cv::Scalar(255,0,0), 2, cv::LINE_8, 0 );  
            // do a search to find the rectangle to the working thread camera frame
            // find matches...
            cv::Mat resultMat;
            // now take a region of Interest from the image and search for a match
            //cv::Mat  maskROI{( widthHegithPt.y - topLeftCorner.y +10),secondCamFrame.mat.cols,secondCamFrame.mat.type()  };
            cv::Mat imgROI;
            std::cout<<"Yeah buddy"<<std::endl;
            //imgROI = secondCamFrame.mat(maskROI).clone();
            imgROI = secondCamFrame.mat.rowRange(topLeftCorner.y,topLeftCorner.y+widthHeigthPt.y+20).clone();

            std::cout<<"Yeah buddy imgROI col="<< imgROI.cols<<" , rows="<<imgROI.rows <<std::endl;
            //MatchingMethod(secondCamFrame.mat, templImg, resultMat,  matchMethod); 
            cv::Point matchPos = MatchingMethod(imgROI, templImg, resultMat,  matchMethod); 
            cv::rectangle( secondCamFrame.mat, cv::Rect(matchPos.x,matchPos.y+topLeftCorner.y,templImg.cols,templImg.rows), cv::Scalar::all(0), 2, 8, 0 );

            //createRect(frame, topLeftCorner, widthHegithPt);
            std::cout<<"Yeah buddy No"<<std::endl;

            auto endTime = chrono::steady_clock::now();
            double fps = ( 1.0/(chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count()) ) *1000.0;
            txt +=(to_string(fps));
            cv::putText(frame, txt, org, cv::FONT_HERSHEY_PLAIN, 1, colorTxt, 1, 2);
            cv::imshow("Simple Demo", frame);
             
            /** Second Camera */
            double fpsSecondCam =  ( 1.0/(chrono::duration_cast<chrono::milliseconds>(secondCamFrame.timestamp - secondCameraTime).count()) ) *1000.0;
            txt = "2nd FPS: " + to_string(fpsSecondCam);
            cv::putText(secondCamFrame.mat, txt, org, cv::FONT_HERSHEY_PLAIN, 1, colorTxt, 1, 2);
            cv::imshow("SecondCam", secondCamFrame.mat);
            secondCameraTime = secondCamFrame.timestamp ;
            cout <<"2nd - Elapsed time in milliseconds: " 
            << chrono::duration_cast<chrono::milliseconds>(secondCamFrame.timestamp - secondCameraTime).count()
            << " ms" 
            <<", FPS = " << fpsSecondCam << " Hz." << endl;
            secondCameraTime = secondCamFrame.timestamp ;
            

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
       cap.release();
       cv::destroyAllWindows();
        
        
    }
    catch(std::runtime_error &e){
        
        std::cout << " a standard exception was caught, with message '\n"
                  << e.what() << "'\n";

        cv::destroyAllWindows();
    }
    
    return 0;
}
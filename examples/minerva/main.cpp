// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "utilities.h"
#include <chrono>
#include <sys/stat.h>

bool captureIR = false;

bool PointBreak = false;

std::mutex mtConsole;
void writer_thread_funtion(const uint32_t device_number)
{
    std::stringstream ofilename; // file name for the image
    std::string directory = "KinectImagesTEST/";
    int count = 0;

    uint8_t* colorBuffer = NULL;
    uint8_t* depthBuffer = NULL;
    uint8_t* irBuffer = NULL;
    int rows;
    int cols;
    omp_lock_t countlock;
    std::string NewSN = mCameras[device_number].get_serialnum();

    while (PointBreak == false || mImageQueue[device_number].size() > 0)
    {
        if (mImageQueue[device_number].size() > 0)
        {
            omp_init_lock(&countlock);
            Frame currentFrame = mImageQueue[device_number].front();

            colorBuffer = currentFrame.mCapture.get_color_image().get_buffer();
            rows = currentFrame.mCapture.get_color_image().get_height_pixels();
            cols = currentFrame.mCapture.get_color_image().get_width_pixels();
            cv::Mat colorMat(rows , cols, CV_8UC4, (void*)colorBuffer, cv::Mat::AUTO_STEP);

            ofilename<<directory<<mFrameCount[device_number] 
                << "_RGB_" << NewSN
                << "_"<< currentFrame.ms
                << "_"<< currentFrame.mCapture.get_color_image().get_system_timestamp().count()
                << "_"<< currentFrame.mCapture.get_color_image().get_device_timestamp().count()<<".jpg"; // create file name for the picture
            cv::imwrite(ofilename.str() , colorMat);
            ofilename.str(""); 


            // convert the raw buffer to cv::Mat
            depthBuffer = currentFrame.mCapture.get_depth_image().get_buffer();
            rows = currentFrame.mCapture.get_depth_image().get_height_pixels();
            cols = currentFrame.mCapture.get_depth_image().get_width_pixels();
            cv::Mat depthMat(rows, cols, CV_16UC1, (void*)depthBuffer, cv::Mat::AUTO_STEP);
            ofilename<<directory<<mFrameCount[device_number] 
                << "_DEPTH_raw16_" << NewSN
                << "_"<< currentFrame.ms
                << "_"<< currentFrame.mCapture.get_depth_image().get_system_timestamp().count()
                << "_"<< currentFrame.mCapture.get_depth_image().get_device_timestamp().count()<<".tiff"; // create file name for the picture
            cv::imwrite(ofilename.str(), depthMat);
            ofilename.str(""); 


            // convert the raw buffer to cv::Mat
            if (captureIR) {
                irBuffer = currentFrame.mCapture.get_ir_image().get_buffer();
                rows = currentFrame.mCapture.get_ir_image().get_height_pixels();
                cols = currentFrame.mCapture.get_ir_image().get_width_pixels();
                cv::Mat irMat(rows, cols, CV_16UC1, (void*)irBuffer, cv::Mat::AUTO_STEP);
                ofilename<<directory<<mFrameCount[device_number] 
                    << "_IR_raw16_" << NewSN
                    << "_"<< currentFrame.ms
                    << "_"<< currentFrame.mCapture.get_ir_image().get_system_timestamp().count()
                    << "_"<< currentFrame.mCapture.get_ir_image().get_device_timestamp().count()<<".tiff"; // create file name for the picture
                cv::imwrite(ofilename.str(), irMat);
                ofilename.str(""); 
            }

            mImageQueue[device_number].pop();
            mFrameCount[device_number]++; 
            omp_unset_lock(&countlock);
            count=0;
        }
        else{
            count++;
            std::this_thread::sleep_for(std::chrono::milliseconds(33)); // 200
        }
    }

    return;
}



int main()
{
    try
    {
        Timer timer(true);

        // create folder for the camera and create it if it doesnt already exist
        float etime = 0.0f;
        char foldername[60];
        sprintf(foldername, "KinectImagesTEST"); 
        mkdir(foldername, 0777);
        int junk = 0;

        sprintf(foldername, "KinectImagesTEST/calibration/"); 
        mkdir(foldername, 0777);

        // Check for devices
        const uint32_t device_count = k4a::device::get_installed_count();
        if (device_count == 0)
        {
            throw std::runtime_error("No Azure Kinect devices detected!");
        } else if (device_count < 7) {
        }
        std::cout<<"Number of Kinect Video Devices detected: " << device_count <<"\n";
        
        OpenDetectedDevices(device_count);
        SetUpDataStorage(device_count);
        std::this_thread::sleep_for(std::chrono::milliseconds(33));

        //Check Output files are created.
        for(unsigned int i=0; i<outputFiles.size(); i++)
        if (!outputFiles[i]) {
            std::cerr << "Error: Unable to open output file! "<< i << std::endl;
            return 1;
        }  //end audio setup

        std::thread worker_thread0(writer_thread_funtion, 0); 
        std::thread worker_thread1(writer_thread_funtion, 1);
        std::thread worker_thread2(writer_thread_funtion, 2);
        std::thread worker_thread3(writer_thread_funtion, 3);
        std::thread worker_thread4(writer_thread_funtion, 4);
        std::thread worker_thread5(writer_thread_funtion, 5);
        std::thread worker_thread6(writer_thread_funtion, 6);


        // Poll the device for new image data.
            //
          // We set the timeout to 0 so we don't block if there isn't an available frame.
          //
          // This works here because we're doing the work on the same thread that we're
          // using for the UI, and the ViewerWindow class caps the framerate at the
          // display's refresh rate (the EndFrame() call blocks on the display's sync).
          //
          // If we don't have new image data, we'll just reuse the textures we generated
         // from the last time we got a capture.
        //
        k4a::capture capture;

         std::cout << "  Recording KINECT Images... \n";
        int i = 0;
        omp_set_num_threads(device_count);
        omp_lock_t writelock;
        omp_init_lock(&writelock);
        cKeyboard kb;

        using clock = std::chrono::steady_clock;
        auto next_frame = clock::now();


        while (i < 30) {
            next_frame += std::chrono::milliseconds(66); //33

            #pragma omp parallel for 
            for(uint j=0; j<device_count; j++)
            {   
                try {
                 mCameras[ omp_get_thread_num()].get_capture(&mCaptures[omp_get_thread_num()], std::chrono::milliseconds(66));               //ret = mCameras[omp_get_thread_num() ].get_capture(&mCaptures[omp_get_thread_num() ], std::chrono::milliseconds(-1));
                //std::cout<<i<<": " <<ret<<std::endl;
                }
                catch (const std::exception &e)
                {//std::cout << "Error-------------1\n";
                }
            }
            i++;
            if( kb.getKeyState(KEY_ESC) ) {
                PointBreak=true; break; }

        }

i=0;
std::cout<<"Warmed up!....\n";
timer.Reset();
        while (true){
            #pragma omp parallel for 
            for(uint j=0; j<device_count; j++)
            {   
                int threadID= omp_get_thread_num();
                try{
                mCameras[ threadID].get_capture(&mCaptures[threadID], std::chrono::milliseconds(33));
                }
                catch (const std::exception &e)
                {//std::cout << "Error-------------1\n";
                }

                try{
                Frame newFrame(mCaptures[threadID], std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
                mImageQueue[threadID].push( newFrame );
                }
                catch (const std::exception &e)
                {//std::cout << "Error -----------2";
                }
            }
            
            i++;
            if( kb.getKeyState(KEY_ESC) ){ 
               // std::cout << "CAUGHT A BREAK!" << std::endl; 
                PointBreak=true; break;}

        }
        etime = timer.Elapsed().count();

        std::cout << "Elapsed time: " << std::fixed << etime << "ms , (seconds: "<< etime/1000 <<")\n";

        junk = CloseDetectedDevices(device_count);
        PointBreak=true;
        std::cout << "waiting..." << junk << "\n";

        worker_thread0.join();
        worker_thread1.join();
        worker_thread2.join();
        worker_thread3.join();
        worker_thread4.join();
        worker_thread5.join();
        worker_thread6.join();
    
        std::cout << "Joined to main thread: "<< mFrameCount[0] << std::endl;

        // Close the output file
        for(unsigned int i=0; i<outputFiles.size(); i++)
        {
            outputFiles[i].close();
        }
     
        std::cout << "Exiting... " << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "Press [enter] to exit." << std::endl;
        std::cin.get();
        return 1;
    }

    std::cout<<mImageQueue[0].size()<<std::endl;

    std::cout << "Exiting cleanly... " << std::endl;
    return (int)0;
}

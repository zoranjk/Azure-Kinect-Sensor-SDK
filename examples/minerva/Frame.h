#ifndef Frame_H
#define Frame_H

#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <sys/stat.h>
#include <k4a/k4a.hpp>

class Frame {
    private:
        //uint32_t serialnumber;
        std::string serialnumber;
    public:
        Frame(const k4a::capture NewRGBcapture, uint64_t NewMS){   //const std::string NewSN, 
        mCapture=NewRGBcapture;
        ms=NewMS;
        //serialnumber = NewSN;
        }

        ~Frame() {
            
        }


        std::string getSerialNum(){
            return serialnumber;
        }


        k4a::capture mCapture;
        uint64_t ms;
};

#endif
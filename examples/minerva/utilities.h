#include <vector>
#include <queue>
#include <string>
#include <sstream>

#include <iostream>
#include <fstream>
#include <stdio.h>

#include <k4a/k4a.hpp>
#include <lzma.h>

#include "opencv2/core.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include "timer.hpp"
#include "keyboard.hpp"
#include "Frame.h"
#include "timer.hpp"

#include <future>
#include <chrono>
#include <thread>
#include <mutex>
#include <omp.h>

#include <malloc.h>
#include <stdbool.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>               // open, O_RDWR
#include <unistd.h>              // close
#include <sys/ioctl.h>           // ioctl
#include <asm/types.h>           // videodev2.h
#include <sys/mman.h>
#include <sys/types.h>

#include <cstring>
#include "RtAudio.h"
#include "lame.h"

std::vector<k4a::device> mCameras;
std::vector< std::queue<Frame> > mImageQueue;
std::vector< uint > mFrameCount;
std::vector< k4a::capture > mCaptures;

lzma_stream strm;



#define SSTR( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()
#define IN_BUF_MAX  409600
#define OUT_BUF_MAX 409600
#define CHUNK_SIZE 409600


//Audio type and structures #JTK
std::vector<unsigned int> kincet_device_ids;
std::vector<std::ofstream> outputFiles;
std::vector< RtAudio::Api > apis;
std::vector<lame_t> lames;  //its so lame to have a Lame per thread
std::string device_name = "Kinect";

typedef signed short MY_TYPE;
#define FORMAT RTAUDIO_SINT16

struct InputData {
  MY_TYPE* buffer;
  unsigned int dID;
  //unsigned int fID;
};

const int BUFFER_SIZE = 8192;
const int channels = 7;   //7 for Kincet array,  2 for likely other stero devices, 1 mono
const unsigned int sampleRate = 48000;
const int mp3BufferSize = BUFFER_SIZE*channels;
unsigned char mp3Buffer[mp3BufferSize];
const uint long_time = 1800;  
volatile bool do_capture = true;





void SetUpDataStorage(uint32_t device_count)
{
    for (uint8_t deviceIndex = 0; deviceIndex < device_count; deviceIndex++)
    {
        //Set up individual Queues for the camera
        std::queue<Frame> v1;
        mImageQueue.push_back(v1);
        mFrameCount.push_back(0);
        k4a::capture cap;
        cap.reset ();
        mCaptures.push_back(cap);
    }
}


void OpenDetectedDevices(uint32_t device_count)
{
    FILE* outfile;
    char filename[120]; 
    k4a_calibration_t calibration;

    for (uint8_t deviceIndex = 0; deviceIndex < device_count; deviceIndex++)
    {
        std::cout << "Started opening K4A device..." <<(int)deviceIndex<< std::endl;

        mCameras.push_back(k4a::device::open(deviceIndex)); // pushes handle from device into camera vector
        if (!mCameras[deviceIndex])
            throw std::runtime_error(SSTR("Failed to open device: "<<deviceIndex<< "\n"));
        


        // The start_cameras() function requires a device configuration which
        // specifies the modes in which to put the color and depth cameras.
        // See DeviceConfiguration, EImageFormat, EColorResolution, EDepthMode,
        // EFramesPerSecond, and EWiredSyncMode.
        //
        k4a_wired_sync_mode_t sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
        uint32_t delay_off_master_usec = 0; // Delay off master to avoid IR interference. 160 JTK**************
        if (mCameras[deviceIndex].is_sync_out_connected() == true && mCameras[deviceIndex].is_sync_in_connected () == false)
        {
            std::cout << "    Master Device.... \n";
            sync_mode = K4A_WIRED_SYNC_MODE_MASTER;
            delay_off_master_usec = 0; // The master must have 0 as subordinate delay.
        }


        // Start the device
        //
        k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        config.camera_fps = K4A_FRAMES_PER_SECOND_30;
        config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
        config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
        config.color_resolution = K4A_COLOR_RESOLUTION_1080P;  //_1080P // _1440P //_2160P //_3072P

        // This means that we'll only get captures that have both color and
        // depth images, so we don't need to check if the capture contains
        // a particular type of image.
        //
        config.synchronized_images_only = true;
        config.depth_delay_off_color_usec=0,
        config.wired_sync_mode=sync_mode;
        config.subordinate_delay_off_master_usec=delay_off_master_usec;
        config.disable_streaming_indicator=false;

        // Set color exposure time to manual. Recommended for synchronicity, otherwise
        // differences in auto exposure between devices may cause drift to bring the devices out of sync.
        // mCameras[deviceIndex].set_color_control(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE,
        //     K4A_COLOR_CONTROL_MODE_MANUAL,
        //     3000);   ///   33330

        mCameras[deviceIndex].set_color_control(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE,
            K4A_COLOR_CONTROL_MODE_MANUAL, 3000);
        //     3000);   ///   33330

        // open up a file to print the calibration stuff onto 
        sprintf(filename, "/home/sdl/Documents/Azure-Kinect-Sensor-SDK/build/KinectImagesTEST/calibration/calibration-%s.bin", mCameras[deviceIndex].get_serialnum().c_str());
        outfile = fopen(filename, "wb");
        if (outfile == NULL) {
            fprintf(stderr, "\nError opened file\n");
            exit(1);
        }

        // get calibration information
        k4a_result_t result = k4a_device_get_calibration(mCameras[deviceIndex].handle(), config.depth_mode, config.color_resolution, &calibration);

        if (result == K4A_RESULT_SUCCEEDED) {
            std::cout << "Got calibration\n" << calibration.color_resolution <<std::endl;
        }
        // write struct to file
        int flag = 0;
        flag = fwrite(&calibration, sizeof(struct _k4a_calibration_t), 1, outfile);
        if (flag) {
            printf("Contents of the structure written successfully\n");
        } else
            printf("Error Writing to File!");



        mCameras[deviceIndex].start_cameras(&config);
        std::cout << "    Serial Number: " << mCameras[deviceIndex].get_serialnum () << "\n";

        // close file
        fclose(outfile);

    }
}


int CloseDetectedDevices(uint32_t device_count)
{
    bool ret = true;
    for (uint8_t deviceIndex = 0; deviceIndex < device_count; deviceIndex++)
    {
        std::cout << "Closing K4A device..." <<(int)deviceIndex<< std::endl;

        mCameras[deviceIndex].stop_cameras();
        std::cout<<(int)deviceIndex<<": " <<ret<<std::endl;
        mCameras[deviceIndex].close();
        std::cout<<(int)deviceIndex<<": " <<ret<<std::endl;
    }
    std::cout<<"All Closed"<<std::endl;
    return 1;
}


bool IntializeEncoder(uint32_t preset)
{
    strm = LZMA_STREAM_INIT;
    lzma_ret ret = lzma_easy_encoder(&strm, preset, LZMA_CHECK_CRC64);

    if (ret == LZMA_OK) // Return successfully if the initialization went fine.
        return true;


    // Something went wrong. The possible errors are documented in
    // lzma/container.h (src/liblzma/api/lzma/container.h in the source
    // package or e.g. /usr/include/lzma/container.h depending on the
    // install prefix).
    const char *msg;
    switch (ret) {
    case LZMA_MEM_ERROR:
        msg = "Memory allocation failed";
        break;

    case LZMA_OPTIONS_ERROR:
        msg = "Specified preset is not supported";
        break;

    case LZMA_UNSUPPORTED_CHECK:
        msg = "Specified integrity check is not supported";
        break;

    default:
        // This is most likely LZMA_PROG_ERROR indicating a bug in
        // this program or in liblzma. It is inconvenient to have a
        // separate error message for errors that should be impossible
        // to occur, but knowing the error code is important for
        // debugging. That's why it is good to print the error code
        // at least when there is no good error message to show.
        msg = "Unknown error, possibly a bug";
        break;
    }

    fprintf(stderr, "Error initializing the encoder: %s (error code %u)\n",
            msg, ret);
    return false;

}




void compress(const uint8_t* buffer, size_t bsize) {
    const char* outputFile = "Image-compressed.xz";
    // Initialize the encoder
    //lzma_stream strm = LZMA_STREAM_INIT;
    lzma_ret ret = lzma_easy_encoder(&strm, LZMA_PRESET_DEFAULT, LZMA_CHECK_CRC64);
    if (ret != LZMA_OK) {
        //fprintf(stderr, "Error initializing the encoder: %s\n", lzma_strerror(ret));
        return;
    }

    // Set the input buffer
    strm.next_in = buffer;
    strm.avail_in = bsize;

    // Open the output file
    FILE* file = fopen(outputFile, "wb");
    if (file == NULL) {
        fprintf(stderr, "Error opening the output file\n");
        lzma_end(&strm);
        return;
    }

    // Create an output buffer
    uint8_t* compressedBuffer = (uint8_t*)malloc(CHUNK_SIZE);
    if (compressedBuffer == NULL) {
        fprintf(stderr, "Memory allocation error\n");
        fclose(file);
        lzma_end(&strm);
        return;
    }

    // Compress the data in chunks and write to the output file
    size_t compressedSize = 0;
    do {
        // Set the output buffer
        strm.next_out = compressedBuffer;
        strm.avail_out = CHUNK_SIZE;

        // Compress the chunk
        ret = lzma_code(&strm, LZMA_FINISH);
        if (ret != LZMA_OK && ret != LZMA_STREAM_END) {
            //fprintf(stderr, "Compression error: %s\n", lzma_strerror(ret));
            free(compressedBuffer);
            fclose(file);
            lzma_end(&strm);
            return;
        }

        // Calculate the compressed size
        size_t chunkSize = CHUNK_SIZE - strm.avail_out;
        compressedSize += chunkSize;

        // Write the compressed chunk to the output file
        fwrite(compressedBuffer, 1, chunkSize, file);

    } while (ret != LZMA_STREAM_END);

    // Cleanup
    free(compressedBuffer);
    fclose(file);
    lzma_end(&strm);
}


// Lots of Audio Helper Functions #JTK

/*********************************************
 * Does the actual work - ie writing the data
 * *******************************************/
// Function to handle audio duplex and write to a raw file
int audioDuplexCallback([[maybe_unused]]void *outputBuffer, 
                        void *inputBuffer,
                        unsigned int nBufferFrames,
                        [[maybe_unused]] double streamTime,
                        [[maybe_unused]] RtAudioStreamStatus status,  
                        void *userData) {

    short *pcmData = static_cast< short*>(inputBuffer);
    InputData *iData = (InputData *) userData;
    
    int mp3BytesWritten = lame_encode_buffer(lames[iData->dID], pcmData, pcmData, nBufferFrames*channels, mp3Buffer, sizeof(mp3Buffer));
    
    //std::cout<< mp3BytesWritten << "  " << nBufferFrames << std::endl;
    outputFiles[iData->dID].write(reinterpret_cast<char*>(mp3Buffer), mp3BytesWritten);
    

    return 0;
}


/*********************************************
 * Setup up the Thread and init the callback
 * *******************************************/
void writer_thread_funtion_audio(unsigned int device_number)
{
    RtAudio audio(apis[0]);
    std::vector<unsigned int> devices = audio.getDeviceIds();
    //std::cout << "Found " << devices.size() << " device(s) ...\n";
    //std::cout << "\nNumber:   "<< devices[device_number]<<" : " <<device_number<<std::endl;

    RtAudio::StreamParameters parameters;
    //parameters.deviceId = audio.getDefaultInputDevice();
    parameters.deviceId = kincet_device_ids[device_number];
    parameters.nChannels = channels;  
    parameters.firstChannel = 0;

    unsigned int bufferFrames = BUFFER_SIZE;  // Adjust buffer size as needed

    RtAudio::StreamOptions options;
    //options.flags |= RTAUDIO_NONINTERLEAVED;
    //options.flags |= RTAUDIO_SCHEDULE_REALTIME;
    unsigned int bufferBytes = bufferFrames * channels * sizeof( RTAUDIO_SINT16 );
    InputData data;
    data.buffer = 0;
    data.buffer = (MY_TYPE*)malloc( bufferBytes );
    data.dID = device_number;


    audio.openStream(nullptr, &parameters, RTAUDIO_SINT16, sampleRate,
          &bufferFrames, &audioDuplexCallback, (void *)&data, &options );
    audio.startStream();

        //std::this_thread::sleep_for(std::chrono::seconds(long_time));
        while(do_capture){};
        std::cout<<"BROKE out of sleep..."<<device_number<<"\n";

    // Stop and close the audio stream when done
    audio.stopStream();
    audio.closeStream();
}


/*********************************************
 * List the Currrent Audio APIs on the device
 * *******************************************/
std::vector< RtAudio::Api > listApis()
{
  std::vector< RtAudio::Api > apis;
  RtAudio :: getCompiledApi( apis );

  std::cout << "\nCompiled APIs:\n";
  for ( size_t i=0; i<apis.size(); i++ )
    std::cout << i << ". " << RtAudio::getApiDisplayName(apis[i])
              << " (" << RtAudio::getApiName(apis[i]) << ")" << std::endl;

  return apis;
}


/*********************************************
 * List the Connected devices that you need
 * *******************************************/
void listDevices( RtAudio& audio )
{
  RtAudio::DeviceInfo info;
  int audio_devs_found = 0;

  std::cout << "\nAPI: " << RtAudio::getApiDisplayName(audio.getCurrentApi()) << std::endl;

  std::vector<unsigned int> devices = audio.getDeviceIds();
  std::cout << "Found " << devices.size() << " device(s) ...\n";

  std::vector<unsigned int> deviceIds = audio.getDeviceIds();
  for (unsigned int i=0; i<devices.size(); i++) {
    info = audio.getDeviceInfo( devices[i] );

    if (info.name.find(device_name) != std::string::npos) {
      audio_devs_found = audio_devs_found+1;
      kincet_device_ids.push_back(deviceIds[i]);
      outputFiles.push_back(std::ofstream("KinectImagesTEST/audio/output"+std::to_string(i)+".mp3", std::ios::binary));

      std::cout << "\nDevice Name = " << info.name << '\n';
      std::cout << "Device Index = " << i << '\n';
      std::cout << "Device ID = " << deviceIds[i] << '\n';
      std::cout << "file ID = " <<"output"+std::to_string(i)+".mp3" << '\n';

      lame_t lame = lame_init();
      // Initialize LAME encoder
      lame_set_in_samplerate(lame, sampleRate*channels); // Set input sample rate (adjust as needed)
      lame_set_out_samplerate(lame, sampleRate); // Set output sample rate (adjust as needed)
      lame_set_num_channels(lame, channels); // Set number of channels (adjust as needed)
      //lame_set_mode(lame, STEREO); // Set output mode (e.g., stereo)
      lame_set_quality(lame, 2); // Set encoding quality (0-9, where 2 is good quality)
      lame_set_VBR(lame, vbr_default);//vbr_default
      lame_init_params(lame);
      lames.push_back(lame);
    }

  }
  std::cout << "\nNumber of AUDIO DEVICES("+device_name+ ") FOUND = " << audio_devs_found << '\n';
  std::cout << "\nNumber of FILES FOUND = " << outputFiles.size()  << '\n';
}

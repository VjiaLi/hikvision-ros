#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "filesystem"
#include "string"
// ch:海康相机的SDK头文件 | en:SDK header file of Hikvision Camera
#include "MvCameraControl.h"


class ImageSaverNode : public rclcpp::Node
{
    public:
        ImageSaverNode()
            : Node("image_saver_node")
        {
            // ch:指定参数 | en: declare parameter
            declare_parameter("sensor_hostname", std::string("192.168.3.101"));
            declare_parameter("time_interval", 10);
            
            // ch:创建定时器，设置为10Hz | en:Create a timer, set to 10Hz
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(this->get_parameter("time_interval").as_int() * 10), 
                std::bind(&ImageSaverNode::timer_callback, this));

            // ch:初始化相机 | en:Initialize the camera
            int nRet = MV_CC_Initialize();
            if (nRet != MV_OK)
            {
                RCLCPP_ERROR(this->get_logger(), "Initialize SDK failed! nRet [0x%x]", nRet);
                return;
            }

            // ch:枚举设备并选择设备 | en:Enumerate the devices and select them
            MV_CC_DEVICE_INFO_LIST stDeviceList;
            memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
            nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
            if (MV_OK != nRet)
            {
                printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
                return;
            }

            // ch:打印相机设备信息同时根据配置文件中的sensor_hostname选择启动相机 | en:Print the camera device information while starting the camera based on the 'sensor_hostname' selections in the profile.
            if (stDeviceList.nDeviceNum > 0)
            {
                for (int i = 0; i < stDeviceList.nDeviceNum; i++)
                {
                    printf("[device %d]:\n", i);
                    MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
                    if (NULL == pDeviceInfo)
                    {
                        return;
                    } 

                    PrintDeviceInfo(pDeviceInfo);   

                    if(CheckDeviceIp(pDeviceInfo, this->get_parameter("sensor_hostname").as_string()))
                    {
                        nIndex = i;
                    }      
                }  
            } 
            else
            {
                printf("Find No Devices!\n");
                return;
            }

            // ch:选择设备并创建句柄 | en:select device and create handle
            nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
            if (MV_OK != nRet)
            {
                printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
                return;
            }

            // ch:打开设备 | en: Open the device
            nRet = MV_CC_OpenDevice(handle);
            if (nRet != MV_OK)
            {
                RCLCPP_ERROR(this->get_logger(), "Open Device failed! nRet [0x%x]", nRet);
                return;
            }

            // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
            if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE)
            {
                int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
                if (nPacketSize > 0)
                {
                    nRet = MV_CC_SetIntValueEx(handle,"GevSCPSPacketSize",nPacketSize);
                    if(nRet != MV_OK)
                    {
                        printf("Warning: Set Packet Size fail nRet [0x%x]!\n", nRet);
                    }
                }
                else
                {
                    printf("Warning: Get Packet Size fail nRet [0x%x]!\n", nPacketSize);
                }
            }

            printf("Start import the camera properties from the file\n");
            printf("Wait......\n");
            // ch:从文件中导入相机属性 | en:Import the camera properties from the file
            nRet = MV_CC_FeatureLoad(handle, (current_dir + "/src/hikvision-ros/config/FeatureFile.ini").c_str());
            if (MV_OK != nRet)
            {
                printf("Load Feature fail! nRet [0x%x]\n", nRet);
                return;
            }
            printf("Finish import the camera properties from the file\n");

            // ch:开始取流 | en:start grab image
            nRet = MV_CC_StartGrabbing(handle);
            if (nRet != MV_OK)
            {
                RCLCPP_ERROR(this->get_logger(), "Start Grabbing failed! nRet [0x%x]", nRet);
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Camera initialized and grabbing started.");
        }

        ~ImageSaverNode()
        {
            // 停止相机并释放资源
            if (handle)
            {
                MV_CC_StopGrabbing(handle);
                MV_CC_CloseDevice(handle);
                MV_CC_DestroyHandle(handle);
            }

            MV_CC_Finalize();
        }

    private:
        void timer_callback()
        {
            MV_FRAME_OUT stImageInfo = {0};
            int nRet = MV_CC_GetImageBuffer(handle, &stImageInfo, 2000); // 2秒超时
            if (nRet == MV_OK)
            {
                // ch:处理图像 | en:Process Image 
                cv::Mat img(stImageInfo.stFrameInfo.nExtendHeight, stImageInfo.stFrameInfo.nExtendWidth, CV_8UC3, stImageInfo.pBufAddr);

                cv::Mat bgr_img;

                cv::cvtColor(img, bgr_img, cv::COLOR_RGB2BGR);

                // ch:指定保存图像的子目录 | en:Specifies the subdirectory where the image is saved.
                std::string save_dir = current_dir + "/images/camera_" + std::to_string(nIndex);

                // ch:检查并创建目录 | en:Check and create a catalogue.
                if (!std::filesystem::exists(save_dir)) {
                    std::filesystem::create_directories(save_dir); 
                }

                // ch:获取时间戳 | en:Get the timestamp
                uint64_t m_DevTimeStamp = 0; //设备产生图像的时间
                int64_t m_HostTimeStamp = 0; //图像包到达主机的时间
                m_DevTimeStamp = stImageInfo.stFrameInfo.nDevTimeStampHigh;
                m_DevTimeStamp = (m_DevTimeStamp << 32) + stImageInfo.stFrameInfo.nDevTimeStampLow;
                m_HostTimeStamp = stImageInfo.stFrameInfo.nHostTimeStamp;

                // ch:保存图像到文件 | en:Save the image to a file.
                std::string filename = save_dir + "/image_" + std::to_string(m_HostTimeStamp) + ".png";
                cv::imwrite(filename, bgr_img);

                RCLCPP_INFO(this->get_logger(), "Image saved as %s", filename.c_str());

                // ch:释放图像缓冲区 | en:Free up the image buffer.
                MV_CC_FreeImageBuffer(handle, &stImageInfo);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Get Image failed! nRet [0x%x]", nRet);
            }
        }
        
        bool CheckDeviceIp(MV_CC_DEVICE_INFO* pstMVDevInfo, const std::string ip)
        {
            if (NULL == pstMVDevInfo)
            {
                printf("The Pointer of pstMVDevInfo is NULL!\n");
                return false;
            }
            if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
            {
                // ch:将相机IP格式转换成四字节 | en:Convert the camera IP format to four bytes.
                int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
                int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
                int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
                int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

                // ch:拼接IP为字符串 | en:The concatenated IP address is a string.
                std::string ip_str = std::to_string(nIp1) + "." + std::to_string(nIp2) + "." + std::to_string(nIp3) + "." + std::to_string(nIp4);

                if(ip_str == ip)
                {
                    return true;
                }

                return false;
            }
        }

        bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
        {
            if (NULL == pstMVDevInfo)
            {
                printf("The Pointer of pstMVDevInfo is NULL!\n");
                return false;
            }
            if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
            {
                int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
                int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
                int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
                int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

                // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
                printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
                printf("CurrentIp: %d.%d.%d.%d\n" , nIp1, nIp2, nIp3, nIp4);
                printf("UserDefinedName: %s\n\n" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
            }
            else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
            {
                printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
                printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
            }
            else if (pstMVDevInfo->nTLayerType == MV_GENTL_GIGE_DEVICE)
            {
                printf("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
                printf("Serial Number: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chSerialNumber);
                printf("Model Name: %s\n\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
            }
            else if (pstMVDevInfo->nTLayerType == MV_GENTL_CAMERALINK_DEVICE)
            {
                printf("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stCMLInfo.chUserDefinedName);
                printf("Serial Number: %s\n", pstMVDevInfo->SpecialInfo.stCMLInfo.chSerialNumber);
                printf("Model Name: %s\n\n", pstMVDevInfo->SpecialInfo.stCMLInfo.chModelName);
            }
            else if (pstMVDevInfo->nTLayerType == MV_GENTL_CXP_DEVICE)
            {
                printf("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stCXPInfo.chUserDefinedName);
                printf("Serial Number: %s\n", pstMVDevInfo->SpecialInfo.stCXPInfo.chSerialNumber);
                printf("Model Name: %s\n\n", pstMVDevInfo->SpecialInfo.stCXPInfo.chModelName);
            }
            else if (pstMVDevInfo->nTLayerType == MV_GENTL_XOF_DEVICE)
            {
                printf("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stXoFInfo.chUserDefinedName);
                printf("Serial Number: %s\n", pstMVDevInfo->SpecialInfo.stXoFInfo.chSerialNumber);
                printf("Model Name: %s\n\n", pstMVDevInfo->SpecialInfo.stXoFInfo.chModelName);
            }
            else
            {
                printf("Not support.\n");
            }

            return true;
        }

        rclcpp::TimerBase::SharedPtr timer_;

        // ch:获取当前工作目录 | en:Get current workfolder
        std::string current_dir = std::filesystem::current_path().string(); 
        
        // ch:选择的相机序号 | en:The camera serial number selected
        unsigned int nIndex = 0;

        void *handle = nullptr;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSaverNode>());
    rclcpp::shutdown();
    return 0;
}


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include "filesystem"
#include "string"
#include "MvCameraControl.h" // 海康相机的SDK头文件


class ImageSaverNode : public rclcpp::Node
{
    public:
        ImageSaverNode()
            : Node("image_saver_node")
        {
            // ch:创建定时器，设置为10Hz | en:Create a timer, set to 10Hz
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100), 
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
                }  
            } 
            else
            {
                printf("Find No Devices!\n");
                return;
            }

            printf("Please Intput camera index: ");
            unsigned int nIndex = 0;
            scanf("%d", &nIndex);

            if (nIndex >= stDeviceList.nDeviceNum)
            {
                printf("Intput error!\n");
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

                // ch:转换为ROS Image消息 | en:Transform to ROS Image Messages
                sensor_msgs::msg::Image::SharedPtr image_msg = cv_bridge::CvImage(
                    std_msgs::msg::Header(), "bgr8", bgr_img).toImageMsg();

                // ch:指定保存图像的子目录 | en:Specifies the subdirectory where the image is saved.
                std::string save_dir = current_dir + "/saved_images/";

                // ch:检查并创建目录 | en:Check and create a catalogue.
                if (!std::filesystem::exists(save_dir)) {
                    std::filesystem::create_directories(save_dir); 
                }

                // ch:获取时间戳 | en:Get the timestamp
                unsigned long long timestamp = stImageInfo.stFrameInfo.nDevTimeStampHigh * 1000000000LL + stImageInfo.stFrameInfo.nDevTimeStampLow;

                // ch:保存图像到文件 | en:Save the image to a file.
                std::string filename = save_dir + "image_" + std::to_string(timestamp) + ".png";
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
        void *handle = nullptr;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSaverNode>());
    rclcpp::shutdown();
    return 0;
}


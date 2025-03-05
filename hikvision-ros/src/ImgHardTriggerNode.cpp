#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "filesystem"
#include "string"
// ch:海康相机的SDK头文件 | en:SDK header file of Hikvision Camera
#include "MvCameraControl.h"


class ImgHardTriggerNode : public rclcpp::Node
{
    public:
        ImgHardTriggerNode()
            : Node("img_trigger_node")
        {
            // ch:指定参数 | en: declare parameter
            declare_parameter("sensor_hostname", std::string("192.168.3.101"));
            cur_ip = this->get_parameter("sensor_hostname").as_string();

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

            // 设置触发模式为on
            // set trigger mode as on
            nRet = MV_CC_SetEnumValue(handle, "TriggerMode", MV_TRIGGER_MODE_ON);
            if (MV_OK != nRet)
            {
                printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
                return;
            }

            // 设置图像格式为BGR8
            // set PixelFormat as BGR8
            nRet = MV_CC_SetEnumValue(handle, "PixelFormat", 0x02180015);
            if (MV_OK != nRet)
            {
                printf("MV_CC_SetPixelFormat fail! nRet [%x]\n", nRet);
                return;
            }

            // 设置触发极性为上升沿
            nRet = MV_CC_SetEnumValue(handle, "TriggerActivation", 0);
            if (MV_OK != nRet)
            {
                printf("MV_CC_SetTriggerActivation fail! nRet [%x]\n", nRet);
                return;
            }

            // 设置触发源为线路0
            nRet = MV_CC_SetEnumValue(handle, "TriggerSource", 0);
            if (MV_OK != nRet)
            {
                printf("MV_CC_SetTriggerSource fail! nRet [%x]\n", nRet);
                return;
            }
            
            // 设置PTP为True
            nRet = MV_CC_SetBoolValue(handle, "GevIEEE1588", 1);
            if (MV_OK != nRet)
            {
                printf("MV_CC_SetPTP fail! nRet [%x]\n", nRet);
                return;
            }

            // 设置自动曝光为连续
            nRet = MV_CC_SetEnumValue(handle, "ExposureAuto", 2);
            if (MV_OK != nRet)
            {
                printf("MV_CC_SetExposureAuto fail! nRet [%x]\n", nRet);
                return;
            }

            // 设置自动增益为连续
            nRet = MV_CC_SetEnumValue(handle, "GainAuto", 2);
            if (MV_OK != nRet)
            {
                printf("MV_CC_SetGainAuto fail! nRet [%x]\n", nRet);
                return;
            }

            nRet = MV_CC_RegisterImageCallBackEx(handle, &ImgHardTriggerNode::ImageCallBackEx, this);
            if (MV_OK != nRet)
            {
                printf("MV_CC_RegisterImageCallBackEx fail! nRet [%x]\n", nRet);
                return; 
            }

            // ch:开始取流 | en:start grab image
            nRet = MV_CC_StartGrabbing(handle);
            if (nRet != MV_OK)
            {
                RCLCPP_ERROR(this->get_logger(), "Start Grabbing failed! nRet [0x%x]", nRet);
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Camera initialized and grabbing started.");
            
        }

        ~ImgHardTriggerNode()
        {
            // ch:停止取流 | en:end grab image
            int nRet = MV_CC_StopGrabbing(handle);
            if (MV_OK != nRet)
            {
                printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
                return;
            }

            // ch:关闭设备 | en:close device
            nRet = MV_CC_CloseDevice(handle);
            if (MV_OK != nRet)
            {
                printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
                return;
            }

            // ch:销毁句柄 | en:destroy handle
            nRet = MV_CC_DestroyHandle(handle);
            if (MV_OK != nRet)
            {
                printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
                return;
            }
            handle = NULL;

            if (handle != NULL)
            {
                MV_CC_DestroyHandle(handle);
                handle = NULL;
            }

            // ch:反初始化SDK | en:Finalize SDK
            MV_CC_Finalize();

            printf("exit\n");
        }

        static void __stdcall ImageCallBackEx(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser)
        {
            ImgHardTriggerNode* pThis = static_cast<ImgHardTriggerNode*>(pUser);  // 恢复类实例
            
            if (pFrameInfo)
            {

                // ch:处理图像 | en:Process Image 
                cv::Mat bgr_img(pFrameInfo->nExtendHeight, pFrameInfo->nExtendWidth, CV_8UC3, pData);

                // ch:指定保存图像的子目录 | en:Specifies the subdirectory where the image is saved.
                std::string save_dir = pThis->current_dir + "/images/camera_" + pThis->cur_ip;

                // ch:检查并创建目录 | en:Check and create a catalogue.
                if (!std::filesystem::exists(save_dir)) {
                    std::filesystem::create_directories(save_dir); 
                }

                // ch:获取时间戳 | en:Get the timestamp
                uint64_t m_DevTimeStamp = 0; //设备产生图像的时间
                int64_t m_HostTimeStamp = 0; //图像包到达主机的时间
                m_DevTimeStamp = pFrameInfo->nDevTimeStampHigh;
                m_DevTimeStamp = (m_DevTimeStamp << 32) + pFrameInfo->nDevTimeStampLow;
                m_DevTimeStamp = m_DevTimeStamp / 100;
                m_HostTimeStamp = pFrameInfo->nHostTimeStamp;
                
                // ch:保存图像到文件 | en:Save the image to a file.
                std::string filename = save_dir + "/image_" + std::to_string(m_HostTimeStamp) + ".png";
                cv::imwrite(filename, bgr_img);

                RCLCPP_INFO(pThis->get_logger(), "Image saved as %s", filename.c_str());
            }
            else
            {
                RCLCPP_ERROR(pThis->get_logger(), "Get Image failed!");
            }
            
        }

    private:
        
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

        // ch:等待用户输入enter键来结束取流或结束程序 | en:wait for user to input enter to stop grabbing or end the sample program
        void PressEnterToExit(void)
        {
            int c;
            while ( (c = getchar()) != '\n' && c != EOF );
            fprintf( stderr, "\nPress enter to exit.\n");
            while( getchar() != '\n');
        }

        // ch:获取当前工作目录 | en:Get current workfolder
        std::string current_dir = std::filesystem::current_path().string(); 
        
        // ch:选择的相机序号 | en:The camera serial number selected
        unsigned int nIndex = 0;

        // ch:当前选择的相机IP | en:current camera ip 
        std::string cur_ip;

        void *handle = nullptr;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImgHardTriggerNode>());
    rclcpp::shutdown();
    return 0;
}


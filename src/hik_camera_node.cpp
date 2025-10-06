#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include "MvCameraControl.h"
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>
#include <rclcpp/parameter_events_filter.hpp>

// 打印错误信息
void PrintCameraError(int error_code, const std::string& operation) {
    if (error_code != MV_OK) {
        RCLCPP_ERROR(rclcpp::get_logger("hik_camera_node"), 
                     "%s failed with error code: [0x%x]", operation.c_str(), error_code);
    }
}

class HikCameraNode : public rclcpp::Node
{
public:
    HikCameraNode() : Node("hik_camera_node")
    {
        RCLCPP_INFO(this->get_logger(), "Starting Hikvision Camera Node...");

        // 声明所有可配置的参数
        this->declare_parameter<double>("exposure_time", 8000.0);
        this->declare_parameter<double>("gain", 5.0);
        this->declare_parameter<double>("frame_rate", 30.0);
        this->declare_parameter<std::string>("pixel_format", "BGR8");
        this->declare_parameter<int64_t>("width", 640);
        this->declare_parameter<int64_t>("height", 480);

        // 初始化 ROS 接口
        image_pub_ = image_transport::create_publisher(this, "image_raw");
        parameters_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&HikCameraNode::parameters_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Parameters and callback registered.");

        // 连接硬件并启动
        if (!connectCamera()) {
            throw std::runtime_error("Failed to connect to camera during initialization!");
        }

        // 在确认硬件可用后，创建依赖硬件状态的定时器
        MVCC_FLOATVALUE stParam;
        memset(&stParam, 0, sizeof(MVCC_FLOATVALUE));
        int nRet = MV_CC_GetFloatValue(handle_, "ResultingFrameRate", &stParam);
        if (MV_OK != nRet) {
            RCLCPP_WARN(this->get_logger(), "Could not get resulting frame rate, defaulting to 30fps for timer.");
            timer_period_ms_ = 33;
        } else {
            float frame_rate = stParam.fCurValue > 0 ? stParam.fCurValue : 30.0;
            timer_period_ms_ = 1000.0 / frame_rate;
            RCLCPP_INFO(this->get_logger(), "Camera's resulting frame rate: %.2f fps, setting timer to %.2f ms", frame_rate, timer_period_ms_);
        }
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(timer_period_ms_)),
            std::bind(&HikCameraNode::grab_image_callback, this));

        RCLCPP_INFO(this->get_logger(), "Node initialization complete.");
    }

    ~HikCameraNode()
    {
        is_connected_ = false;
        if (handle_) {
            MV_CC_StopGrabbing(handle_);
            MV_CC_CloseDevice(handle_);
            MV_CC_DestroyHandle(handle_);
            RCLCPP_INFO(this->get_logger(), "Camera disconnected.");
        }
    }

private:
    bool is_connected_ = false;
    void* handle_ = nullptr;
    image_transport::Publisher image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double timer_period_ms_;
    OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
    
    // 高效内存管理：只分配一次缓冲区
    std::unique_ptr<unsigned char[]> image_buffer_ = nullptr;
    unsigned int buffer_size_ = 0;

    bool connectCamera()
    {
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (nRet != MV_OK || stDeviceList.nDeviceNum == 0) {
            RCLCPP_ERROR(this->get_logger(), "No devices found!");
            is_connected_ = false; return false;
        }
        RCLCPP_INFO(this->get_logger(), "Found %d devices.", stDeviceList.nDeviceNum);

        nRet = MV_CC_CreateHandle(&handle_, stDeviceList.pDeviceInfo[0]);
        if (nRet != MV_OK) {
            PrintCameraError(nRet, "CreateHandle");
            is_connected_ = false; return false;
        }

        nRet = MV_CC_OpenDevice(handle_);
        if (nRet != MV_OK) {
            PrintCameraError(nRet, "OpenDevice");
            is_connected_ = false; return false;
        }
        RCLCPP_INFO(this->get_logger(), "Camera connected successfully.");
        
        nRet = MV_CC_SetEnumValue(handle_, "TriggerMode", MV_TRIGGER_MODE_OFF);
        PrintCameraError(nRet, "Set TriggerMode to Off");
        
        RCLCPP_INFO(this->get_logger(), "Setting initial parameters...");
        
        // 应用启动时的分辨率设置
        int64_t width, height;
        this->get_parameter("width", width);
        this->get_parameter("height", height);
        nRet = MV_CC_SetIntValue(handle_, "Width", width);
        PrintCameraError(nRet, "Set Width");
        nRet = MV_CC_SetIntValue(handle_, "Height", height);
        PrintCameraError(nRet, "Set Height");

        // 应用其他参数
        double exposure_time, gain, frame_rate;
        this->get_parameter("exposure_time", exposure_time);
        this->get_parameter("gain", gain);
        this->get_parameter("frame_rate", frame_rate);
        nRet = MV_CC_SetFloatValue(handle_, "ExposureTime", exposure_time);
        PrintCameraError(nRet, "Set ExposureTime");
        nRet = MV_CC_SetFloatValue(handle_, "Gain", gain);
        PrintCameraError(nRet, "Set Gain");
        nRet = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", frame_rate);
        PrintCameraError(nRet, "Set AcquisitionFrameRate");

        std::string pixel_format_str;
        this->get_parameter("pixel_format", pixel_format_str);
        unsigned int pixel_format_enum;
        if (pixel_format_str == "Mono8") pixel_format_enum = PixelType_Gvsp_Mono8;
        else if (pixel_format_str == "BGR8") pixel_format_enum = PixelType_Gvsp_BGR8_Packed;
        else RCLCPP_ERROR(this->get_logger(), "Initial pixel format '%s' unsupported.", pixel_format_str.c_str());
        nRet = MV_CC_SetEnumValue(handle_, "PixelFormat", pixel_format_enum);
        PrintCameraError(nRet, "Set PixelFormat");

        // 为高效内存管理分配缓冲区
        MVCC_INTVALUE_EX stParam;
        memset(&stParam, 0, sizeof(MVCC_INTVALUE_EX));
        nRet = MV_CC_GetIntValueEx(handle_, "PayloadSize", &stParam);
        if (nRet == MV_OK) {
            buffer_size_ = stParam.nCurValue;
            image_buffer_ = std::make_unique<unsigned char[]>(buffer_size_);
            RCLCPP_INFO(this->get_logger(), "Image buffer allocated with size: %u bytes", buffer_size_);
        } else {
            PrintCameraError(nRet, "Get PayloadSize");
            is_connected_ = false; return false;
        }

        nRet = MV_CC_StartGrabbing(handle_);
        if (nRet != MV_OK) {
            PrintCameraError(nRet, "StartGrabbing");
            is_connected_ = false; return false;
        }

        RCLCPP_INFO(this->get_logger(), "Camera is grabbing.");
        is_connected_ = true;
        return true;
    }

    void grab_image_callback()
    {
        if (!is_connected_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Camera disconnected. Trying to reconnect...");
            if (handle_) {
                MV_CC_StopGrabbing(handle_);
                MV_CC_CloseDevice(handle_);
                MV_CC_DestroyHandle(handle_);
                handle_ = nullptr;
            }
            if (connectCamera()) {
                RCLCPP_INFO(this->get_logger(), "Camera reconnected successfully!");
            }
            return;
        }

        MV_FRAME_OUT_INFO_EX stImageInfo = {0};
        int nRet = MV_CC_GetOneFrameTimeout(handle_, image_buffer_.get(), buffer_size_, &stImageInfo, 1000);

        if (nRet == MV_OK) {
            auto image_msg = std::make_unique<sensor_msgs::msg::Image>();
            image_msg->header.stamp = this->get_clock()->now();
            image_msg->header.frame_id = "camera_optical_frame";
            image_msg->height = stImageInfo.nHeight;
            image_msg->width = stImageInfo.nWidth;
            
            if (stImageInfo.enPixelType == PixelType_Gvsp_Mono8) {
                image_msg->encoding = "mono8";
                image_msg->step = stImageInfo.nWidth;
            } else if (stImageInfo.enPixelType == PixelType_Gvsp_BGR8_Packed) {
                image_msg->encoding = "bgr8";
                image_msg->step = stImageInfo.nWidth * 3;
            } else {
                RCLCPP_WARN_ONCE(this->get_logger(), "Unsupported pixel format: 0x%lx", stImageInfo.enPixelType);
                return;
            }

            image_msg->data.resize(stImageInfo.nFrameLen);
            memcpy(image_msg->data.data(), image_buffer_.get(), stImageInfo.nFrameLen);
            
            image_pub_.publish(std::move(image_msg));

        } else {
            RCLCPP_ERROR(this->get_logger(), "GetOneFrameTimeout failed! [0x%x]. Camera might be disconnected.", nRet);
            is_connected_ = false;
        }
    }
    
    bool restart_grabbing_with_new_resolution(int64_t new_width, int64_t new_height)
    {
        int nRet = MV_CC_StopGrabbing(handle_);
        PrintCameraError(nRet, "StopGrabbing before changing resolution");

        nRet = MV_CC_SetIntValue(handle_, "Width", new_width);
        if (nRet != MV_OK) {
            PrintCameraError(nRet, "Set New Width");
            MV_CC_StartGrabbing(handle_); return false;
        }

        nRet = MV_CC_SetIntValue(handle_, "Height", new_height);
        if (nRet != MV_OK) {
            PrintCameraError(nRet, "Set New Height");
            MV_CC_StartGrabbing(handle_); return false;
        }

        MVCC_INTVALUE_EX stParam;
        memset(&stParam, 0, sizeof(MVCC_INTVALUE_EX));
        nRet = MV_CC_GetIntValueEx(handle_, "PayloadSize", &stParam);
        if (nRet == MV_OK) {
            buffer_size_ = stParam.nCurValue;
            image_buffer_ = std::make_unique<unsigned char[]>(buffer_size_);
            RCLCPP_INFO(this->get_logger(), "Resolution changed. New buffer allocated with size: %u", buffer_size_);
        } else {
            PrintCameraError(nRet, "Get PayloadSize after resolution change");
            MV_CC_StartGrabbing(handle_); return false;
        }
        
        nRet = MV_CC_StartGrabbing(handle_);
        if (nRet != MV_OK) {
            PrintCameraError(nRet, "Restart Grabbing after changing resolution");
            is_connected_ = false; return false;
        }
        return true;
    }

    bool restart_grabbing_with_new_pixel_format(const std::string& new_format_str)
    {
        int nRet = MV_CC_StopGrabbing(handle_);
        PrintCameraError(nRet, "StopGrabbing before changing pixel format");

        unsigned int pixel_format_enum;
        if (new_format_str == "Mono8") pixel_format_enum = PixelType_Gvsp_Mono8;
        else if (new_format_str == "BGR8") pixel_format_enum = PixelType_Gvsp_BGR8_Packed;
        else {
            RCLCPP_ERROR(this->get_logger(), "Unsupported pixel format: %s. Reverting.", new_format_str.c_str());
            MV_CC_StartGrabbing(handle_); return false;
        }

        nRet = MV_CC_SetEnumValue(handle_, "PixelFormat", pixel_format_enum);
        if (nRet != MV_OK) {
            PrintCameraError(nRet, "Set PixelFormat");
            MV_CC_StartGrabbing(handle_); return false;
        }
        RCLCPP_INFO(this->get_logger(), "Pixel Format has been set to %s.", new_format_str.c_str());

        nRet = MV_CC_StartGrabbing(handle_);
        if (nRet != MV_OK) {
            PrintCameraError(nRet, "Restart Grabbing after changing pixel format");
            is_connected_ = false; return false;
        }
        return true;
    }

    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        
        int64_t new_width = -1, new_height = -1;

        for (const auto &param : parameters) {
            std::string name = param.get_name();
            if (name == "exposure_time") {
                int nRet = MV_CC_SetFloatValue(handle_, "ExposureTime", param.as_double());
                if (nRet != MV_OK) { result.successful = false; result.reason = "Failed to set exposure time"; }
            } else if (name == "gain") {
                int nRet = MV_CC_SetFloatValue(handle_, "Gain", param.as_double());
                if (nRet != MV_OK) { result.successful = false; result.reason = "Failed to set gain"; }
            } else if (name == "frame_rate") {
                int nRet = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", param.as_double());
                if (nRet != MV_OK) { result.successful = false; result.reason = "Failed to set frame rate"; }
            } else if (name == "pixel_format") {
                if (!restart_grabbing_with_new_pixel_format(param.as_string())) {
                    result.successful = false; result.reason = "Failed to set pixel format";
                }
            } else if (name == "width") new_width = param.as_int();
            else if (name == "height") new_height = param.as_int();
        }

        if (new_width != -1 || new_height != -1) {
            if (new_width == -1) this->get_parameter("width", new_width);
            if (new_height == -1) this->get_parameter("height", new_height);
            if (!restart_grabbing_with_new_resolution(new_width, new_height)) {
                result.successful = false; result.reason = "Failed to set new resolution";
            }
        }
        return result;
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<HikCameraNode> node = nullptr;
    try {
        node = std::make_shared<HikCameraNode>();
        rclcpp::spin(node);
    } catch (const rclcpp::exceptions::RCLError &e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Unhandled RCL exception: %s", e.what());
    } catch (const std::runtime_error &e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Node initialization failed: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}

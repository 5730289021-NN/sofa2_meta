#include <inference_engine.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <string>
#include <cmath>

static InferenceEngine::Blob::Ptr wrapMat2Blob(const cv::Mat &mat) {
    size_t channels = mat.channels();
    size_t height = mat.size().height;
    size_t width = mat.size().width;

    size_t strideH = mat.step.buf[0];
    size_t strideW = mat.step.buf[1];

    bool is_dense =
            strideW == channels &&
            strideH == channels * width;

    if (!is_dense) THROW_IE_EXCEPTION
                << "Doesn't support conversion from not dense cv::Mat";

    InferenceEngine::TensorDesc tDesc(InferenceEngine::Precision::U8,
                                      {1, channels, height, width},
                                      InferenceEngine::Layout::NHWC);

    return InferenceEngine::make_shared_blob<uint8_t>(tDesc, mat.data);
}

namespace follow_me
{
    class FollowMe
    {
    public:
        FollowMe() : image_updated(false)
        {
            ros::NodeHandle nodeHandle("~");
            ros::Publisher cmd_vel_publisher = nodeHandle.advertise<geometry_msgs::Twist>("/follow_vel", 1);
            ros::Subscriber image_subscriber = nodeHandle.subscribe("/camera/image_raw", 1, &FollowMe::latchImageCallback, this);
            cv::namedWindow(OPENCV_WINDOW);

            ros::Rate loop_rate(15);

            bool success = true;
            if (!nodeHandle.getParam("/follow_me/Kp_x", Kp_x))
            {
                ROS_ERROR("No Kp_x Given");
                success = false;
            }
            if (!nodeHandle.getParam("/follow_me/Kp_a", Kp_a))
            {
                ROS_ERROR("No Kp_a Given");
                success = false;
            }
            if (!nodeHandle.getParam("/follow_me/vx_max", vx_max))
            {
                ROS_ERROR("No vx_max Given");
                success = false;
            }
            if (!nodeHandle.getParam("/follow_me/wz_max", wz_max))
            {
                ROS_ERROR("No wz_max Given");
                success = false;
            }
            if (!nodeHandle.getParam("/follow_me/min_dist_fol", min_dist_fol))
            {
                ROS_ERROR("No min_dist_fol Given");
                success = false;
            }
            if (!nodeHandle.getParam("/follow_me/max_dist_fol", max_dist_fol))
            {
                ROS_ERROR("No max_dist_fol Given");
                success = false;
            }
            if (!nodeHandle.getParam("/follow_me/prob_thres", prob_thres))
            {
                ROS_ERROR("No prob_thres Given");
                success = false;
            }
            if (!nodeHandle.getParam("/follow_me/people_model", people_model))
            {
                ROS_ERROR("No people_model Given");
                success = false;
            }
            if (!success)
            {
                ROS_ERROR("There's uninitialized variable, follow_me won't start");
                return;
            }
            else
            {
                ROS_INFO("Parameter is corrected, Preparing OPENVINO...");
            }

            InferenceEngine::Core core;
            InferenceEngine::CNNNetwork network = core.ReadNetwork(people_model);
            network.setBatchSize(1);
            InferenceEngine::InputInfo::Ptr input_info = network.getInputsInfo().begin()->second;
            std::string input_name = network.getInputsInfo().begin()->first;

            ROS_INFO_STREAM("Input Network " << input_name << " initialized...");

            input_info->getPreProcess().setResizeAlgorithm(InferenceEngine::RESIZE_BILINEAR);
            input_info->setLayout(InferenceEngine::Layout::NHWC);
            input_info->setPrecision(InferenceEngine::Precision::U8);

            InferenceEngine::DataPtr output_info = network.getOutputsInfo().begin()->second;
            std::string output_name = network.getOutputsInfo().begin()->first;
            output_info->setPrecision(InferenceEngine::Precision::FP32);
            ROS_INFO_STREAM("Output Network " << output_name << " initialized...");

            InferenceEngine::ExecutableNetwork executable_network = core.LoadNetwork(network, "GPU");
            InferenceEngine::InferRequest infer_request = executable_network.CreateInferRequest();
            ROS_INFO_STREAM("Network has been loaded into GPU...");

            while (ros::ok())
            {
                InferenceEngine::Blob::Ptr imgBlob = wrapMat2Blob(cv_ptr->image);
                infer_request.SetBlob(input_name, imgBlob);
                infer_request.Infer();
                InferenceEngine::Blob::Ptr output = infer_request.GetBlob(output_name);
                
                /*Do something with output Blob*/
                auto const memLocker = output->cbuffer(); // use const memory locker
                // output_buffer is valid as long as the lifetime of memLocker
                const float *output_buffer = memLocker.as<const float *>();
                

                cv::imshow(OPENCV_WINDOW, cv_ptr->image);
                //cv::waitKey(3);

                ros::spinOnce();
                loop_rate.sleep();
            }
        }
        ~FollowMe()
        {
            cv::destroyWindow(OPENCV_WINDOW);
        }

    private:
        const std::string OPENCV_WINDOW = "Image window";
        cv_bridge::CvImagePtr cv_ptr;
        double Kp_x;
        double Kp_a;
        double vx_max;
        double wz_max;
        double min_dist_fol;
        double max_dist_fol;
        double prob_thres;
        std::string people_model;
        bool image_updated;

        //Assume that the image is coming too fast, so we can't process every image
        //The process is performed on main thread
        void latchImageCallback(const sensor_msgs::ImageConstPtr &in_img)
        {
            image_updated = true;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(in_img, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception &e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
        }
    };
} // namespace follow_me

int main(int argc, char **argv)
{
    ros::init(argc, argv, "follow_me");
    follow_me::FollowMe fm;
    return 0;
}

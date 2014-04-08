#include "camera.h"

Camera::Camera(int id, ros::NodeHandle nh)
{
    hmin = 0;
    hmax = 100;
    vmin = 50;
    threshold1 = 50;
    threshold2 = 50;
    minarea = 1000;
    request_status = STATUS_AVAILABLE;
    this->nh = nh;
    // Open the camera
    std::cout << "Opening right hand camera: ";
    if(id == LEFT_HAND)
        open_camera.request.name = "left_hand_camera";
    else if(id == HEAD)
        open_camera.request.name = "head_camera";
    else if(id == RIGHT_HAND)
        open_camera.request.name = "right_hand_camera";
    else
        return;
    open_camera.request.settings.width = 1280;
    open_camera.request.settings.height = 800;
    open_camera.request.settings.fps = 20;
    camera_control.id = baxter_core_msgs::CameraControl::CAMERA_CONTROL_GAIN;
    camera_control.value = 10;
    open_camera.request.settings.controls.push_back(camera_control);
    if(!ros::service::call("/cameras/open", open_camera) || open_camera.response.err != 0)
    {
        std::cout << "\033[1;31mFailed\033[0m" << std::endl;
        return;
    }
    std::cout << "\033[1;32mOK\033[0m" << std::endl;
    // Register the camera callback that receive the images
    std::cout << "Registrating camera callback: ";
    it = new image_transport::ImageTransport(nh);
    is = it->subscribe("/cameras/right_hand_camera/image", 1, &Camera::callback, this);
    if(is == NULL)
    {
        std::cout << "\033[1;31mFailed\033[0m" << std::endl;
        return;
    }
    std::cout << "\033[1;32mOK\033[0m" << std::endl;
}

Camera::~Camera()
{
    delete it;
}

void Camera::callback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Image acquisition failed: %s", e.what());
        return;
    }
    cv::cvtColor(cv_ptr->image, img_hsv, cv::COLOR_RGB2HSV);
    img_gray.create(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
    for(int i = 0; i < img_hsv.total(); i++)
    {
        if(img_hsv.data[i*3]<hmin || img_hsv.data[i*3]>hmax || img_hsv.data[i*3+2]<vmin)
            img_gray.data[i] = 0;
        else
            img_gray.data[i] = img_hsv.data[i*3+2];
    }
    cv::blur(img_gray, img_gray, cv::Size(5,5));
    cv::Canny(img_gray, img_gray, threshold1, threshold2);
    cv::dilate(img_gray, img_gray, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)));
    cv::findContours(img_gray, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if(request_status == STATUS_REQUESTING)
    {
       request_result = new std::vector<std::vector<cv::Point> >();
       request_status = STATUS_IN_PROGRESS;
    }
    for(int i = 0; i<contours.size(); i++)
    {
        double area = cv::contourArea(contours[i]);
        if(area>minarea)
        {
            cv::approxPolyDP(contours[i], contours[i], sqrt(area)/10.0, true);
            cv::drawContours(cv_ptr->image, contours, i, cv::Scalar(0,0,255),3, CV_AA);
            for(int j = 0; j<contours[i].size(); j++)
                cv::circle(cv_ptr->image, contours[i][j], 5, cv::Scalar(255,0,0), -1, CV_AA);
            if(request_status == STATUS_IN_PROGRESS)
            {
                if(request_type == REQUEST_SELECTED_SHAPE)
                    if(cv::pointPolygonTest(contours[i], cv::Point(800, 200), false) != -1)
                        request_result->push_back(std::vector<cv::Point>(contours[i]));
                else if(request_type == REQUEST_SHAPES)
                    request_result->push_back(std::vector<cv::Point>(contours[i]));
            }
        }
    }
    if(request_status == STATUS_IN_PROGRESS)
        request_status = STATUS_AVAILABLE;
    cv::imshow("Baxter Pick And Learn", cv_ptr->image);
    cv::waitKey(3);
}

void Camera::request(int request_type)
{
    if(STATUS_AVAILABLE)
    {
        this->request_type = request_type;
        request_status = STATUS_REQUESTING;
    }
}

bool Camera::isResultAvailable()
{
    if(STATUS_AVAILABLE)
        return true;
    else
        return false;
}


std::vector<std::vector<cv::Point> >* Camera::getResult()
{
    if(request_status == STATUS_AVAILABLE)
        return request_result;
    else
        return NULL;
}

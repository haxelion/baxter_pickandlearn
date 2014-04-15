#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

class Object
{
public:
    Object(std::vector<cv::Point> *shape, float picking_height);
    void setDropPosition(float position[], float orientation);
    bool match(std::vector<cv::Point> *shape);

private:
    std::vector<cv::Point> *shape;
    float picking_height, drop_position[3], drop_orientation[4];
};

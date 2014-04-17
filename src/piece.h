#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <string>
#include <sstream>

class Piece
{
public:
    Piece(std::vector<cv::Point> *shape, float picking_height, float picking_orientation[4], std::string name);
    void setDropPosition(float position[], float orientation[]);
    bool match(std::vector<cv::Point> *shape);
    std::string getName();
    std::string serialize();

private:
    std::vector<cv::Point> shape;
    std::string name;
    float picking_height, picking_orientation[4], drop_position[3], drop_orientation[4];
};

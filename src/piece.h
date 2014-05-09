#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <string>
#include <sstream>

class Piece
{
public:
    Piece(std::vector<cv::Point> shape, float picking_height, float picking_orientation[4], std::string name);
    void setDropPosition(float position[], float orientation[]);
    double match(std::vector<cv::Point> &shape);
    float getPickingHeight();
    void getPickingOrientation(float picking_orientation[]);
    void getDropPosition(float drop_position[]);
    void getDropOrientation(float drop_orientation[]);
    std::string getName();
    std::string serialize();
    void deserialize(std::string s);

private:
    std::vector<cv::Point> shape;
    std::string name;
    float picking_height, picking_orientation[4], drop_position[3], drop_orientation[4];
};

int closestMatch(std::vector<Piece> &pieces, std::vector<cv::Point> &shape, double threshold);
void closestMatch(std::vector<Piece> &pieces, std::vector<std::vector<cv::Point> > &shapes, double threshold, int &idxp, int &idxs);

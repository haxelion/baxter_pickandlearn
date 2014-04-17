#include "piece.h"

Piece::Piece(std::vector<cv::Point> shape, float picking_height, float picking_orientation[4], std::string name)
{
    this->shape = shape;
    this->picking_height = picking_height;
    this->picking_orientation[0] = picking_orientation[0];
    this->picking_orientation[1] = picking_orientation[1];
    this->picking_orientation[2] = picking_orientation[2];
    this->picking_orientation[3] = picking_orientation[3];
    this->name = name;
    for(int i = 0; i<3; i++)
        drop_position[i] = 0;
    for(int i = 0; i<4; i++)
        drop_orientation[i] = 0;
}

void Piece::setDropPosition(float position[], float orientation[])
{
    drop_position[0] = position[0];
    drop_position[1] = position[1];
    drop_position[2] = position[2];
    drop_orientation[0] = orientation[0];
    drop_orientation[1] = orientation[1];
    drop_orientation[2] = orientation[2];
    drop_orientation[3] = orientation[3];
}

std::string Piece::getName()
{
    return name;
}

std::string Piece::serialize()
{
    std::stringstream s;
    s << "{" << "name: '" << name << "', ";
    s << "shape: [(" << shape[0].x << ", " << shape[0].y << ")";
    for(int i = 1; i < shape.size(); i++)
        s << ", (" << shape[i].x <<", "<< shape[i].y << ")";
    s << "], picking_height: " << picking_height;
    s << ", picking_orientation: [" << picking_orientation[0];
    for(int i = 1; i < 4; i++)
        s << ", " << picking_orientation[i];
    s << "], drop_position: [" << drop_position[0];
    for(int i = 1; i < 3; i++)
        s << ", " << drop_position[i];
    s << "], drop_orientation: [" << drop_position[0];
    for(int i = 1; i < 4; i++)
        s << ", " << drop_position[i];
    s << "]}";
    return s.str();
}

double Piece::match(std::vector<cv::Point> &shape)
{
    return cv::matchShapes(this->shape, shape, CV_CONTOURS_MATCH_I3, 0);
}

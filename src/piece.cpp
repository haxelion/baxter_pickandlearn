#include "piece.h"

Piece::Piece(std::vector<cv::Point> *shape, float picking_height)
{
    this->shape = new std::vector<cv::Point>(*shape);
    this->picking_height = picking_height;
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

bool Piece::match(std::vector<cv::Point> *shape)
{
    if(cv::matchShapes(*(this->shape), *shape, CV_CONTOURS_MATCH_I3, 0) <0.1)
        return true;
    return false;
}

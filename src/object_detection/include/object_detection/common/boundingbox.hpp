#pragma once
#ifndef BOUNDINGBOX_HPP
#define BOUNDINGBOX_HPP

#include <vector>
#include <iostream>
#include <algorithm>
#include <visualization_msgs/msg/marker.hpp>

typedef struct
{
    float x, y, z;  // X, Y, Z position
    int clusterID;  // clustered ID
} Point;

class bbox
{

public:
    bbox() { marker = std::make_shared<visualization_msgs::msg::Marker>(); }

    void update(std::vector<Point>& points, const int& ID, const std::string& frame_id, const std::string& ns)
    {
        marker->header.frame_id = frame_id;
        marker->ns = ns;
        marker->type = visualization_msgs::msg::Marker::CUBE;
        marker->action = visualization_msgs::msg::Marker::ADD;
        marker->id = ID;
        marker->pose.orientation.x = 0;
        marker->pose.orientation.y = 0.0;
        marker->pose.orientation.z = 0.0;
        marker->pose.orientation.w = 1.0;

        if (points.size())
        {
            auto bound = std::partition(points.begin(), points.end(), [ID](Point& pt) {return (pt.clusterID == ID);});
            if (bound != points.begin()) 
            {
                for (int ch = 0; ch < 3; ch++) get_maxmin(points, bound, ch);
                initialized = true;
            }
            else initialized = false;
        }
        else
        {
            printf("no point received. \n");
            initialized = false;
        }
    }

    void get_maxmin(std::vector<Point>& points, std::vector<Point>::iterator& mid, const int ch)
    {
        float max, min;
        switch (ch)
        {
            case 0:
                max = points.front().x;
                min = points.front().x;
                for (std::vector<Point>::iterator it=points.begin()+1; it!=mid; ++it)
                {
                    if (it->x > max) max = it->x;
                    if (it->x < min) min = it->x;
                }
                marker->scale.x = max - min + 0.2;
                marker->pose.position.x = min + marker->scale.x/2 - 0.1;
                break;
            case 1:
                max = points.front().y;
                min = points.front().y;
                for (std::vector<Point>::iterator it=points.begin()+1; it!=mid; ++it)
                {
                    if (it->y > max) max = it->y;
                    if (it->y < min) min = it->y;
                }
                marker->scale.y = max - min + 0.2;
                marker->pose.position.y =  min + marker->scale.y/2 - 0.1;
                break;
            case 2:
                max = points.front().z;
                min = points.front().z;
                for (std::vector<Point>::iterator it=points.begin()+1; it!=mid; ++it)
                {
                    if (it->z > max) max = it->z;
                    if (it->z < min) min = it->z;
                }
                marker->scale.z = max - min + 0.2;
                marker->pose.position.z =  min + marker->scale.z/2 - 0.1;
                break;
        }   
    }

public:
    visualization_msgs::msg::Marker::SharedPtr marker;
    bool initialized = false;
};

#endif
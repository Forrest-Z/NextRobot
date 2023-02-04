//
// Created by waxz on 23-1-10.
//

#ifndef SCAN_REPUBLISHER_GEOMETRY_TYPES_H
#define SCAN_REPUBLISHER_GEOMETRY_TYPES_H


namespace geometry{
    struct Point{
        float x = 0.0;
        float y = 0.0;
        float z = 0.0;
        float r = 0.0; // distance to origin
        float b = 0.0; // angle to origin
        int i = -1; // cluster id

        inline float SquareDist(const Point& rhv){
            //std::sqrt
            return   ((x - rhv.x)*(x - rhv.x) + (y - rhv.y)*(y - rhv.y)) ;
        }
    };

    struct DetectTarget{
        // x,y
        float pose_in_laser[2];
        float pose_in_base[2];

        float match_error;
    };

}

#endif //SCAN_REPUBLISHER_GEOMETRY_TYPES_H

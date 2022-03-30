//PCL
#include <pcl/point_types.h>
//STL
#include <math.h>
#include <iostream>
#define pi 3.1415926535897932384

template <typename T>
/**
 * @brief polarCoord
 * convert from polar coordinates to cartesian
 * @param range, laser scanned length
 * @param angle, defined from 0 to pi (I./II. quadrant) and 0 to -pi (III./IV quadrant)
 * @param intensity, reflectivity of laser scan
 * @return
 */
inline pcl::PointXYZI polar2cartesian(T range,T angle, T intensity)
{
    pcl::PointXYZI p;
    p.x = range * std::cos(angle);
    p.y = range * std::sin(angle);
    p.z = 0.0;
    p.intensity = intensity;
    return p;
}

template <typename T>
/**
 * @brief polarCoord
 * convert from polar coordinates to cartesian
 * @param range, laser scanned length
 * @param angle, defined from 0 to pi (I./II. quadrant) and 0 to -pi (III./IV quadrant)
 * @return
 */
inline pcl::PointXYZ polar2cartesian(T range,T angle)
{
    pcl::PointXYZ p;
    p.x = range * std::cos(angle);
    p.y = range * std::sin(angle);
    p.z = 0.0;
    return p;
}

template <typename T>
/**
 * @brief polar2cartesian
 * color a point relative to the point which is farest aways
 * yellow = near, red = far away
 * @param range
 * @param angle
 * @param farest
 * @param alpha
 * @return
 */
inline pcl::PointXYZRGBA polar2cartesian(T range,T angle,T farest,T alpha)
{
    pcl::PointXYZRGBA p;
    p.x = range * std::cos(angle);
    p.y = range * std::sin(angle);
    p.z = 0.0;

    //really easy colorinterpolation
    p.r = 255;
    p.g = 255 * (range / farest);;
    p.b = 0;

    p.a = alpha;

    return p;
}

template <typename T>
/**
 * @brief polar2cartesianThreshold
 * color each point beyond a certain threshold
 * everything beyond a certain threshold is red
 * everything before is green
 * @param range
 * @param angle
 * @param threshold
 * @param alpha
 * @return
 */
inline pcl::PointXYZRGBA polar2cartesianThreshold(T range,T angle,T threshold,T alpha)
{
    pcl::PointXYZRGBA p;
    p.x = range * std::cos(angle);
    p.y = range * std::sin(angle);
    p.z = 0.0;

    if(range >= threshold)
    {
        p.r = 255;
        p.g = 0;
        p.b = 0;
    }
    else
    {
        p.r = 0;
        p.g = 255;
        p.b = 0;
    }

    p.a = alpha;

    return p;
}


template <typename T>
/**
 * @brief cartesian2polar
 * convert cartesian coordinates to polar coordinates
 * @param x
 * @param y
 * @param range
 * @param angle
 */
inline void cartesian2polar(T x, T y, T& range, T& angle)
{
    range = std::sqrt(x * x + y * y);

    //edge cases like x == 0 not checked
    if(x > 0.0 && y > 0.0) //I. quadrant
    {
        angle = std::atan(y / x);
    }
    else if(x < 0.0 && y > 0.0) //II. quadrant
    {
        angle = std::atan(y / x) + pi;
    }
    else if(x < 0.0 && y < 0.0) //II. quadrant
    {
        angle = std::atan(y / x) + pi;

    }
    else if(x > 0.0 && y < 0.0) //IV. quadrant
    {
        angle = std::atan(y / x) + 2 * pi;
    }
}

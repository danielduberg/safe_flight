#ifndef POINT_H
#define POINT_H

struct Point {
    float x;
    float y;
    
    static float getDistance(Point p);
    
    static float getDistance(Point p1, Point p2);
    
    static Point getMidpoint(Point p1, Point p2);
    
    static Point getPointFromVector(float direction, float magnitude);
    
    static Point getPointFromVectorDegrees(float direction, float magnitude);
    
    static void getVectorFromPoint(Point p, float & direction, float & magnitude);
    
    static void getVectorFromPointDegrees(Point p, float & direction, float & magnitude);
    
    static float getDirection(Point p);
    
    static float GetDirectionDegrees(Point p);
    
};


#endif // POINT_H

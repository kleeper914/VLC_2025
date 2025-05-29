#include "rss.h"
#include <math.h>

// 计算两个圆的交点
// 返回值: 交点数量(0,1,2)
int circleIntersection(Point p1, float r1, Point p2, float r2, Point* result) {
    // 圆心距离
    float d = sqrtf((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p1.y)*(p1.y - p2.y));
    
    // 特殊情况：圆心重合
    if (d < 0.0001f) {
        if (fabsf(r1 - r2) < 0.0001f) {
            // 完全重合的圆，选择一个方向上的点
            result[0].x = p1.x + r1;
            result[0].y = p1.y;
            return 1;
        }
        return 0; // 同心圆无交点
    }
    
    // 圆太远，不相交
    if (d > r1 + r2) { 
        // 按比例处理成一个虚拟交点
        result[0].x = p1.x + (p2.x - p1.x) * r1 / (r1 + r2);
        result[0].y = p1.y + (p2.y - p1.y) * r1 / (r1 + r2);
        return 1;
    }
    
    // 一个圆在另一个圆内部，无交点
    if (d < fabsf(r1 - r2)) {
        // 按比例处理成一个虚拟交点
        float ratio = (r1 < r2) ? 0.0f : 1.0f; // 选择靠近小圆圆周的点
        result[0].x = p1.x + (p2.x - p1.x) * ratio - r1 + (r2 + r1) * ratio;
        result[0].y = p1.y + (p2.y - p1.y) * ratio;
        return 1;
    }
    
    // 恰好外切，只有一个交点
    if (fabsf(d - (r1 + r2)) < 0.0001f) {
        result[0].x = p1.x + (p2.x - p1.x) * r1 / d;
        result[0].y = p1.y + (p2.y - p1.y) * r1 / d;
        return 1;
    }
    
    // 恰好内切，只有一个交点
    if (fabsf(d - fabsf(r1 - r2)) < 0.0001f) {
        float sign = (r1 > r2) ? 1.0f : -1.0f;
        result[0].x = p1.x + sign * (p2.x - p1.x) * r1 / d;
        result[0].y = p1.y + sign * (p2.y - p1.y) * r1 / d;
        return 1;
    }
    
    // 正常相交，两个交点
    float a = (r1*r1 - r2*r2 + d*d) / (2*d);
    float h = sqrtf(r1*r1 - a*a);
    
    float x3 = p1.x + a * (p2.x - p1.x) / d;
    float y3 = p1.y + a * (p2.y - p1.y) / d;
    
    // 两个交点
    result[0].x = x3 + h * (p2.y - p1.y) / d;
    result[0].y = y3 - h * (p2.x - p1.x) / d;
    
    result[1].x = x3 - h * (p2.y - p1.y) / d;
    result[1].y = y3 + h * (p2.x - p1.x) / d;
    
    return 2;
}

// 计算点到圆心的距离
float distToCenter(Point pt, Point center) {
    return sqrtf((pt.x - center.x)*(pt.x - center.x) + (pt.y - center.y)*(pt.y - center.y));
}

// 三点定位法
Point threePoints(float *dis, Point *ps) 
{
    // 固定点坐标：点0(0,10)、点1(-10,0)、点2(10,0)
    Point fixedPs[3] = {{0, 10}, {-10, 0}, {10, 0}};
    
    Point result = {0, 0};
    
    if (dis == NULL) 
        return result;
    
    // 使用固定坐标替换传入的坐标
    ps = fixedPs;
    
    // 检查距离是否有效
    for (int i = 0; i < 3; ++i) {
        if (dis[i] < 0)
            return result;
    }
    
    // 存储所有可能的交点和置信度
    Point finalPoints[3]; // 最多3个交点
    float confidence[3] = {0}; 
    int validPoints = 0;
    
    // 主要计算：使用点1和点2，即(-10,0)和(10,0)
    Point intersections[2];
    int numPoints = circleIntersection(ps[1], dis[1], ps[2], dis[2], intersections);
    
    if (numPoints > 0) {
        if (numPoints == 1) {
            // 只有一个交点
            finalPoints[validPoints] = intersections[0];
            // 计算与点0圆的匹配度
            float diff = fabsf(distToCenter(intersections[0], ps[0]) - dis[0]);
            confidence[validPoints] = 1.0f / (diff + 0.0001f); // 避免除以零
            validPoints++;
        } else {
            // 有两个交点，使用点0判断哪个更准确
            float diff1 = fabsf(distToCenter(intersections[0], ps[0]) - dis[0]);
            float diff2 = fabsf(distToCenter(intersections[1], ps[0]) - dis[0]);
            
            if (diff1 < diff2) {
                finalPoints[validPoints] = intersections[0];
                confidence[validPoints] = 1.0f / (diff1 + 0.0001f);
            } else {
                finalPoints[validPoints] = intersections[1];
                confidence[validPoints] = 1.0f / (diff2 + 0.0001f);
            }
            validPoints++;
        }
    }
    
    // 辅助计算1：点0和点1（即(0,10)和(-10,0)）
    numPoints = circleIntersection(ps[0], dis[0], ps[1], dis[1], intersections);
    if (numPoints > 0) {
        if (numPoints == 1) {
            finalPoints[validPoints] = intersections[0];
            float diff = fabsf(distToCenter(intersections[0], ps[2]) - dis[2]);
            confidence[validPoints] = 0.5f / (diff + 0.0001f); // 较低权重
            validPoints++;
        } else if (numPoints == 2) {
            float diff1 = fabsf(distToCenter(intersections[0], ps[2]) - dis[2]);
            float diff2 = fabsf(distToCenter(intersections[1], ps[2]) - dis[2]);
            
            if (diff1 < diff2) {
                finalPoints[validPoints] = intersections[0];
                confidence[validPoints] = 0.5f / (diff1 + 0.0001f);
            } else {
                finalPoints[validPoints] = intersections[1];
                confidence[validPoints] = 0.5f / (diff2 + 0.0001f);
            }
            validPoints++;
        }
    }
    
    // 辅助计算2：点0和点2（即(0,10)和(10,0)）
    numPoints = circleIntersection(ps[0], dis[0], ps[2], dis[2], intersections);
    if (numPoints > 0) {
        if (numPoints == 1) {
            finalPoints[validPoints] = intersections[0];
            float diff = fabsf(distToCenter(intersections[0], ps[1]) - dis[1]);
            confidence[validPoints] = 0.5f / (diff + 0.0001f); // 更低权重
            validPoints++;
        } else if (numPoints == 2) {
            float diff1 = fabsf(distToCenter(intersections[0], ps[1]) - dis[1]);
            float diff2 = fabsf(distToCenter(intersections[1], ps[1]) - dis[1]);
            
            if (diff1 < diff2) {
                finalPoints[validPoints] = intersections[0];
                confidence[validPoints] = 0.5f / (diff1 + 0.0001f);
            } else {
                finalPoints[validPoints] = intersections[1];
                confidence[validPoints] = 0.5f / (diff2 + 0.0001f);
            }
            validPoints++;
        }
    }
    
    // 如果没有有效点，返回原点
    if (validPoints == 0) {
        return result;
    }
    
    // 根据置信度加权平均计算最终结果
    float totalConfidence = 0;
    for (int i = 0; i < validPoints; i++) {
        totalConfidence += confidence[i];
    }
    
    for (int i = 0; i < validPoints; i++) {
        float weight = confidence[i] / totalConfidence;
        result.x += finalPoints[i].x * weight;
        result.y += finalPoints[i].y * weight;
    }
    
    return result;
}

Point getPDlocation(float* distances, Point *points)
{
    // 调用三点定位法
    Point result = threePoints(distances, points);
    return result;
}


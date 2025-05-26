#include "rss.h"

//三点定位法
//dis:半径
//points：圆心
Point threePoints(float *dis, Point *ps) 
{
	Point p ={0,0}; //初始化点为无效值
	if (dis == NULL || ps== NULL)
		return p;

	for (int i = 0; i < 3; ++i)
     {
	    //检查距离是否有问题
		if (dis[i] < 0)
			return p;

		for (int j = i + 1; j < 3; ++j) 
        {
		    //圆心距离PQ
			float p2p = (float)sqrt((ps[i].x - ps[j].x)*(ps[i].x - ps[j].x) +
				                    (ps[i].y - ps[j].y)*(ps[i].y - ps[j].y));
		    //判断两圆是否相交
			if (dis[i] + dis[j] <= p2p) 
            {
			    //不相交，按比例求
				p.x += ps[i].x + (ps[j].x - ps[i].x)*dis[i] / (dis[i] + dis[j]);
				p.y += ps[i].y + (ps[j].y - ps[i].y)*dis[i] / (dis[i] + dis[j]);
			}
			else
             {
			    //相交则套用公式
                //PC
				float dr = p2p / 2 + (dis[i] * dis[i] - dis[j] * dis[j]) / (2 * p2p); 
                //x = xp + (xq-xp) * PC / PQ
				p.x += ps[i].x + (ps[j].x - ps[i].x)*dr / p2p;
                //y = yp + (yq-yp) * PC / PQ
				p.y += ps[i].y + (ps[j].y - ps[i].y)*dr / p2p;
			}
		}
	}
	
    //三个圆两两求点，最终得到三个点，求其均值
	p.x /= 3;
	p.y /= 3;

	return p;
}

Point getPDlocation(float* distances, Point *points)
{
    // 调用三点定位法
    Point result = threePoints(distances, points);
    return result;
}


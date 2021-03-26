/***********************************
 * iou.h
 *
 * Calculate the iou (Intersection over Union) ratio.
 * For ONLY Convex Polygons.
 *
 * Author: WeiQM (weiquanmao@hotmail.com)
 * Github: https://github.com/CheckBoxStudio/IoU
 *
 * 2018
 ***********************************/

#ifndef _IOU_HPP_FILE_
#define _IOU_HPP_FILE_

#include "math.h"
#include <assert.h>
#include <vector>
#include <algorithm>

namespace IOU
{
    const double EPS = 1e-6;

    enum WiseType
    {
        NoneWise,
        ClockWise,
        AntiClockWise
    };
    enum LocPosition
    {
        Outside,
        OnEdge,
        Inside
    };


    template <typename T>
    struct Vec2 {
        // Members
        union {
            struct
            {
                T x;
                T y;
            };
            T D[2];
        };

        // Constructors
        Vec2() : x(0), y(0) {}
        Vec2(T _x, T _y) : x(_x), y(_y) {}

        // Access component
        inline T& operator[](unsigned int i) { assert(i < 2); return D[i]; }
        inline const T& operator[](unsigned int i) const { assert(i < 2); return D[i]; }

        // Operations
        inline bool operator==(const Vec2 &p) const {
            return (abs(x - p.x) <= EPS && abs(y - p.y) <= EPS);
        }

        template<typename TT>
        inline Vec2 operator*(TT t) const { return Vec2(x * t, y * t); }
        template<typename TT>
        inline Vec2 operator/(TT t) const { return Vec2(x / t, y / t); }
        template<typename TT>
        inline Vec2& operator*=(TT t) { x *= t; y *= t; return *this; }
        template<typename TT>
        inline Vec2& operator/=(TT t) { x /= t; y /= t; return *this; }

        inline Vec2 operator+(const Vec2 &p) const { return Vec2(x + p.x, y + p.y); }
        inline Vec2 operator-(const Vec2 &p) const { return Vec2(x - p.x, y - p.y); }
        inline Vec2& operator+=(const Vec2 &p) { x += p.x; y += p.y; return *this; }
        inline Vec2& operator-=(const Vec2 &p) { x -= p.x; y -= p.y; return *this; }

        inline bool isZero() { return (abs(x) <= EPS && abs(y) <= EPS); }

        inline Vec2 dmul(const Vec2 &p) const { return Vec2(x * p.x, y * p.y); }
        inline Vec2 ddiv(const Vec2 &p) const { return Vec2(x / p.x, y / p.y); }

        inline double dot(const Vec2 &p) const { return x * p.x + y * p.y; }
        inline double operator*(const Vec2 &p) const { return x * p.x + y * p.y; }

        inline double cmul(const Vec2 &p) const { return x * p.y - y * p.x; }
        inline double operator^(const Vec2 &p) const { return x * p.y - y * p.x; }

        inline double norm() const { return sqrt(x*x + y*y); }
        inline double normSquared() const { return x*x + y*y; }

        void normalize() { *this /= norm(); }
        Vec2 normalized() const { return *this / norm(); }

        double distance(const Vec2 &p) const {
            return (*this - p).norm();
        }
        double squareDistance(const Vec2 &p) const {
            return (*this - p).normSquared();
        }
        double angle(const Vec2 &r) const {
            return acos( dot(r) / (norm() * r.norm()) ); }
        double theta() const {
            return atan2(y, x);
        }
    };
    template <typename T>
    double norm(const Vec2<T> &p) { return p.norm(); }
    template <typename T>
    double normSquared(const Vec2<T> &p) { return p.normSquared(); }
    template <typename T>
    void normalize(Vec2<T> &p) { p.normalize(); }
    template <typename T>
    Vec2<T> normalized(const Vec2<T> &p) { return p.normalized(); }
    template <typename T, typename TT>
    inline Vec2<T> operator*(TT t, const Vec2<T>& v) { return Vec2<T>(v.x * t, v.y * t); }
    template <typename T>
    inline double distance(const Vec2<T> &p1, const Vec2<T> &p2) { return p1.distance(p2); }
    template <typename T>
    inline double squareDistance(const Vec2<T> &p1, const Vec2<T> &p2) { return p1.squareDistance(p2); }
    template <typename T>
    inline double angle(const Vec2<T> &p1, const Vec2<T> &p2) { return p1.angle(p2); }
    template <typename T>
    inline double theta(const Vec2<T> &p) { return p.theta(); }

    typedef Vec2<int> Vec2i;
    typedef Vec2<float> Vec2f;
    typedef Vec2<double> Vec2d;


    typedef Vec2d Point;
    typedef std::vector<Point> Vertexes;


    class Line {
    public:
        // Members
        Point p1;
        Point p2;

        // Constructors
        Line() : p1(Point()), p2(Point()) {}
        Line(const Point &_p1, const Point &_p2) : p1(_p1), p2(_p2) {}
        Line(const Point _vert[2]) : p1(_vert[0]), p2(_vert[1]) {}

        // Methods
        double length() const {return p1.distance(p2); }
        bool isOnEdge(const Point &p) const;
        Point intersection(const Line &line, bool *bOnEdge = 0) const;
    };
    inline bool isOnEdge(const Line &line, const Point &p) {
        return line.isOnEdge(p); }
    inline bool isOnEdge(const Point &p, const Line &line) {
        return line.isOnEdge(p); }
    inline Point intersection(const Line &line1, const Line &line2, bool *bOnEdge = 0) {
        return line1.intersection(line2,bOnEdge); }


    class Quad {
    public:
        // Members [in clockwise]
        Point p1;
        Point p2;
        Point p3;
        Point p4;

        // Constructors
        Quad() : p1(Point()), p2(Point()), p3(Point()), p4(Point()) {}
        Quad(const Point &_p1, const Point &_p2, const Point &_p3, const Point &_p4)
            : p1(_p1), p2(_p2), p3(_p3), p4(_p4) {}
        Quad(const Point _vert[4])
            : p1(_vert[0]), p2(_vert[1]), p3(_vert[2]), p4(_vert[3]) {}

        // Methods
        void flip() {
            Point tmp = p2;
            p2 = p4;
            p4 = tmp; }
        void getVertList(Vertexes &_vert) const;

        double area() const;
        WiseType whichWise() const;
        bool isInClockWise() const { return ClockWise == whichWise(); }
        bool isInAntiClockWise() const { return AntiClockWise == whichWise(); }
        void beInSomeWise(const WiseType wiseType);
        void beInClockWise() { beInSomeWise(ClockWise); }
        void beInAntiClockWise() { beInSomeWise(AntiClockWise); }

        LocPosition location(const Point &p) const;
        int interPts(const Line &line, Vertexes &pts) const;
    };
    inline LocPosition location(const Quad &quad, const Point &p) {
        return quad.location(p); }
    inline int interPts(const Quad &quad, const Line &line, Vertexes &pts) {
        return quad.interPts(line, pts); }









    // For any convex polygon
    double areaEx(const Vertexes &C)
    WiseType whichWiseEx(const Vertexes &C);
    void beInSomeWiseEx(Vertexes &C, const WiseType wiseType);
    LocPosition locationEx(const Vertexes &C, const Point &p);
    int interPtsEx(const Vertexes &C, const Line &line, Vertexes &pts);


    // For any convex polygon
    int findInterPointsEx(const Vertexes &C1, const Vertexes &C2, Vertexes &vert);
    int findInnerPointsEx(const Vertexes &C1, const Vertexes &C2, Vertexes &vert);
    double areaIntersectionEx(const Vertexes &C1, const Vertexes &C2);
    double areaUnionEx(const Vertexes &C1, const Vertexes &C2);
    double iouEx(const Vertexes &C1, const Vertexes &C2);


    // For convex quadrilateral
    int findInterPoints(const Quad &Q1, const Quad &Q2, Vertexes &vert);
    int findInnerPoints(const Quad &Q1, const Quad &Q2, Vertexes &vert);
    double areaIntersection(const Quad&Q1, const Quad &Q2);
    double areaUnion(const Quad &Q1, const Quad &Q2);
    double iou(const Quad &Q1, const Quad &Q2);






    // ### Definition


    bool Line::isOnEdge(const Point &p) const
    {
        if (p1 == p2)
            return (p == (p1 + p2) / 2.0);

        Point pp1 = p - p1;
        Point pp2 = p - p2;

        if (abs(pp1^pp2) < EPS &&
            pp1*pp2 < EPS)
            return true;
        else
            return false;
    }
    Point Line::intersection(const Line &line, bool *bOnEdge) const
    {
        Point pInter(0,0);
        bool bOn = false;

        if (p1 == p2 && line.p1 == line.p2){
            // Both lines are actually points.
            bOn =((p1 + p2) / 2.0 == (line.p1 + line.p2) / 2.0);
            if (bOn)
                pInter = (p1 + p2 + line.p1 + line.p2) / 4.0;
        }
        else if (p1 == p2) {
            // This line is actually a point.
            bOn = line.isOnEdge((p1 + p2) / 2.0);
            if (bOn)
                pInter = (p1 + p2) / 2.0;
        }
        else if (line.p1 == line.p2) {
            // The input line is actually a point.
            bOn = isOnEdge((line.p1 + line.p2) / 2.0);
            if (bOn)
                pInter = (line.p1 + line.p2) / 2.0;
        }
        else {
            // Normal cases.
            Point a12 = p2 - p1;
            Point b12 = line.p2 - line.p1;
            double ang = angle(a12, b12);
            if (ang < EPS || abs(3.141592653 - ang) < EPS)
                bOn = false; // Collinear!!
            else {
                // a1_x + m*a12_x = b1_x + n*b12_x
                // a1_y + m*a12_y = b1_y + n*b12_y
                // n = ( (a1_y-b1_y)*a12_x - (a1_x-b1_x)*a12_y ) / (a12_x*b12_y - b12_x*a12_y)
                // m = ( (a1_y-b1_y)*b12_x - (a1_x-b1_x)*b12_y ) / (a12_x*b12_y - b12_x*a12_y)
                // 0 < m < 1
                // 0 < n < 1
                double abx = p1.x - line.p1.x;
                double aby = p1.y - line.p1.y;
                double ab = a12.x*b12.y - b12.x*a12.y;
                assert(abs(ab)>EPS);
                double n = (aby*a12.x - abx*a12.y) / ab;
                double m = (aby*b12.x - abx*b12.y) / ab;

                if (n >= -EPS && n-1.0 <= EPS &&
                    m >= -EPS && m-1.0 <= EPS) {
                    Point ip1 = p1 + m*a12;
                    Point ip2 = line.p1 + n*b12;
                    pInter = (ip1 + ip2) / 2.0;
                    bOn = true;
                }
                else
                    bOn = false;
            }
        }
        if (bOnEdge != 0)
            *bOnEdge = bOn;
        return pInter;
    }

    void Quad::getVertList(Vertexes &_vert) const
    {
        Vertexes vertTemp;
        vertTemp.reserve(4);
        vertTemp.push_back(p1);
        vertTemp.push_back(p2);
        vertTemp.push_back(p3);
        vertTemp.push_back(p4);
        _vert.swap(vertTemp);
    }

    double Quad::area() const
    {
        Vertexes vertTemp;
        getVertList(vertTemp);

        return areaEx(vertTemp);
    }

    WiseType Quad::whichWise() const
    {
        Vertexes vertTemp;
        getVertList(vertTemp);
        return whichWiseEx(vertTemp);
    }
    void Quad::beInSomeWise(const WiseType wiseType)
    {
        if (wiseType != NoneWise) {
            Vertexes vertTemp;
            getVertList(vertTemp);
            beInSomeWiseEx(vertTemp,wiseType);
            p1 = vertTemp[0];
            p2 = vertTemp[1];
            p3 = vertTemp[2];
            p4 = vertTemp[3];
        }
    }

    LocPosition Quad::location(const Point &p) const
    {
        Vertexes vertTemp;
        getVertList(vertTemp);
        return locationEx(vertTemp, p);
    }
    int Quad::interPts(const Line &line, Vertexes &pts) const
    {
        Vertexes vertTemp;
        getVertList(vertTemp);
        return interPtsEx(vertTemp, line, pts);
    }

    double areaEx(const Vertexes &C)
    {
        if (whichWiseEx(C) == NoneWise)
            return -1.0;

        double sArea = 0.0;
        const int N = C.size();
        if (N > 2) {
            const Point &p0 = C.at(0);
            for (int i = 1; i < N-1; ++i) {
                const Point &p1 = C.at(i);
                const Point &p2 = C.at(i + 1);
                Point p01 = p1 - p0;
                Point p02 = p2 - p0;
                sArea += abs(p01^p02)*0.5;
            }
        }
        return sArea;
    }
    WiseType whichWiseEx(const Vertexes &C)
    {
        WiseType wiseType = NoneWise;
        const int N = C.size();

        if (N > 2) {
            Point p0 = C.at(N - 1);
            Point p1 = C.at(0);
            Point p2 = C.at(1);
            Point p01 = p1 - p0;
            Point p12 = p2 - p1;
            if ((abs(p01^p12) <= EPS) && p01*p12 < 0.0)
                return NoneWise;
            else
                wiseType = (p01^p12) > 0.0 ? AntiClockWise : ClockWise;

            const double flip = (wiseType == ClockWise) ? 1.0 : -1.0;
            for (int i = 1; i < N ; ++i) {
                p0 = C.at((i-1)%N);
                p1 = C.at(i%N);
                p2 = C.at((i+1)%N);
                p01 = p1 - p0;
                p12 = p2 - p1;
                if ((p01^p12)*flip > 0.0 ||
                    ((abs(p01^p12) <= EPS) && p01*p12 < 0.0)) {
                    return NoneWise;
                }
            }
        }
        return wiseType;
    }
    typedef std::pair<double, Point> AngPoint;
    bool angIncrease(const AngPoint &p1, const AngPoint &p2)
    {
        return p1.first < p2.first;
    }
    bool angDecrease(const AngPoint &p1, const AngPoint &p2)
    {
        return p1.first > p2.first;
    }
    void beInSomeWiseEx(Vertexes &C, const WiseType wiseType)
    {
        if (wiseType != NoneWise) {
            const int N = C.size();
            if (N>2) {
                Point pO(0.0,0.0);
                for (int i = 0; i < N; ++i)
                    pO += C.at(i);
                pO /= N;
                std::vector<AngPoint> APList;
                APList.reserve(N);
                for (int i = 0; i < N; ++i) {
                    Point op = C.at(i) - pO;
                    double ang = op.theta();
                    APList.push_back(AngPoint(ang, C.at(i)));
                }
                if (wiseType == AntiClockWise)
                    std::sort(APList.begin(), APList.end(), angIncrease);
                else
                    std::sort(APList.begin(), APList.end(), angDecrease);
                Vertexes vertTemp;
                for (int i = 0; i < N; ++i)
                    vertTemp.push_back(APList.at(i).second);
                C.swap(vertTemp);
            }
        }
    }

    LocPosition locationEx(const Vertexes &C, const Point &p)
    {
        const int N = C.size();
        // Special cases.
        if (N == 0)
            return Outside;
        if (N == 1) {
            if (C[0] == p)
                return Inside;
            else
                return Outside;
        }
        if (N == 2) {
            if (isOnEdge(Line(C[0],C[1]),p))
                return OnEdge;
            else
                return Outside;
        }

        // Normal cases.
        // Check OnEdge.
        for (int i=0; i<N; ++i) {
            if (isOnEdge(Line(C[i%N],C[(i+1)%N]),p))
                return OnEdge;
        }
        // Check Outside.
        Point pO(0.0,0.0);
        for (int i=0; i<N; ++i) {
            pO += C[i];
        }
        pO /= N;
        Line op(pO,p);
        bool bIntersection = true;
        for (int i=0; i<N; ++i) {
            intersection(Line(C[i%N],C[(i+1)%N]),op,&bIntersection);
            if (bIntersection)
                return Outside;
        }

        return Inside;
    }
    int interPtsEx(const Vertexes &C, const Line &line, Vertexes &pts)
    {
        Vertexes vertTemp;
        const int N = C.size();
        bool bIntersection = false;
        for (int i=0; i<N; ++i) {
            Point p = intersection(Line(C[i%N],C[(i+1)%N]),line,&bIntersection);
            if (bIntersection)
                vertTemp.push_back(p);
        }
        pts.swap(vertTemp);

        return pts.size();
    }

    int findInterPointsEx(const Vertexes &C1, const Vertexes &C2, Vertexes &vert)
    {
        Vertexes _vert;
        const int N = C2.size();
        for (int i=0; i<N; ++i) {
            Vertexes pts;
            interPtsEx(C1,Line(C2[i%N],C2[(i+1)%N]),pts);
            for (int i = 0; i < pts.size(); ++i)
                _vert.push_back(pts.at(i));
        }
        vert.swap(_vert);
        return vert.size();
    }
    int findInnerPointsEx(const Vertexes &C1, const Vertexes &C2, Vertexes &vert)
    {
        Vertexes _vert;
        for (int i=0; i<C2.size(); ++i) {
            if (locationEx(C1,C2[i]) != Outside)
                _vert.push_back(C2[i]);
        }
        vert.swap(_vert);
        return vert.size();
    }
    double areaIntersectionEx(const Vertexes &C1, const Vertexes &C2)
    {
        if (whichWiseEx(C1) == NoneWise ||
            whichWiseEx(C2) == NoneWise )
            return -1.0;

        Vertexes interVert;
        Vertexes innerVert12;
        Vertexes innerVert21;
        Vertexes allVerts;
        //---------------
        findInterPointsEx(C1, C2, interVert);
        findInnerPointsEx(C1, C2, innerVert12);
        findInnerPointsEx(C2, C1, innerVert21);
        //---------------
        // TODO : Check conditions
        for (int i = 0; i < interVert.size(); ++i)
            allVerts.push_back(interVert.at(i));
        for (int i = 0; i < innerVert12.size(); ++i)
            allVerts.push_back(innerVert12.at(i));
        for (int i = 0; i < innerVert21.size(); ++i)
            allVerts.push_back(innerVert21.at(i));

        if (allVerts.empty())
            return 0.0;
        else {
            assert(allVerts.size() >= 3);
            beInSomeWiseEx(allVerts, ClockWise);
            if (whichWiseEx(allVerts) == NoneWise)
                return -1.0;
            else
                return areaEx(allVerts);
        }
        return -1.0;
    }
    double areaUnionEx(const Vertexes &C1, const Vertexes &C2)
    {
        return areaEx(C1) + areaEx(C2) - areaIntersectionEx(C1, C2);
    }
    double iouEx(const Vertexes &C1, const Vertexes &C2)
    {
        return areaIntersectionEx(C1,C2)/areaUnionEx(C1,C2);
    }

    int findInterPoints(const Quad &Q1, const Quad &Q2, Vertexes &vert)
    {
        Vertexes V1, V2;
        Q1.getVertList(V1);
        Q2.getVertList(V2);
        return findInterPointsEx(V1,V2,vert);
    }
    int findInnerPoints(const Quad &Q1, const Quad &Q2, Vertexes &vert)
    {
        Vertexes V1, V2;
        Q1.getVertList(V1);
        Q2.getVertList(V2);
        return findInnerPointsEx(V1,V2,vert);
    }
    double areaIntersection(const Quad&Q1, const Quad &Q2)
    {
        if (Q1.whichWise() == NoneWise ||
            Q2.whichWise() == NoneWise )
            return -1.0;

        Vertexes interVert;
        Vertexes innerVert12;
        Vertexes innerVert21;
        Vertexes allVerts;
        //---------------
        findInterPoints(Q1, Q2, interVert);
        findInnerPoints(Q1, Q2, innerVert12);
        findInnerPoints(Q2, Q1, innerVert21);
        //---------------
        // TODO : Check conditions
        for (int i = 0; i < interVert.size(); ++i)
            allVerts.push_back(interVert.at(i));
        for (int i = 0; i < innerVert12.size(); ++i)
            allVerts.push_back(innerVert12.at(i));
        for (int i = 0; i < innerVert21.size(); ++i)
            allVerts.push_back(innerVert21.at(i));

        if (allVerts.empty())
            return 0.0;
        else {
            assert(allVerts.size() >= 3);
            beInSomeWiseEx(allVerts, ClockWise);
            if (whichWiseEx(allVerts) == NoneWise)
                return -1.0;
            else
                return areaEx(allVerts);
        }
        return -1.0;
    }
    double areaUnion(const Quad &Q1, const Quad &Q2){
        return Q1.area()+Q2.area()-areaIntersection(Q1,Q2);
    }
    double iou(const Quad &Q1, const Quad &Q2)
    {
        return areaIntersection(Q1,Q2)/areaUnion(Q1,Q2);
    }


}
#endif // !_IOU_HPP_FILE_

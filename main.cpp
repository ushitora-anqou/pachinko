#include <optional>
#include <tuple>
#include <vector>
#include <SFML/Graphics.hpp>
#include "hoolib.hpp"
#include "canvas.hpp"

#define NANOSVG_IMPLEMENTATION
#include "3rd/nanosvg.h"

#include <iostream>

using HooLib::equal, HooLib::equal0, HooLib::between, HooLib::betweenEq;
using namespace HooLib::Operator;

std::optional<std::pair<double, double>> solveQuadrantic(double a, double b, double c)
{
    if(equal0(a))   return std::nullopt;
    double D = b * b - 4 * a * c;
    if(D < 0)   return std::nullopt;
    double sqrtD = std::sqrt(D);
    return std::make_pair((-b - sqrtD) / (2 * a), (-b + sqrtD) / (2 * a));
}

// solve such vector equation for t :
//   |A + tB| = k
std::optional<std::pair<double, double>> solveLinerVectorSizeEq(const Vec2d& a, const Vec2d& b, double k)
{
    return solveQuadrantic(b.lengthSq(), 2 * dot(a, b), a.lengthSq() - k * k);
}

// struct representing collision result between line and moving circle
struct StaticLineVSMovingCircleCollRes
{
    double elapsedTime; // 0 <= t <= 1
    Point circlePosOnHit, contactPos;
};
std::optional<StaticLineVSMovingCircleCollRes> calcCollisionPos(const Line& line, const Circle& circle, const Segment& move)
{
    auto q = line.p, v = line.v.norm(), s0 = move.p, e = move.v;
    auto a = -e + dot(e, v) * v, c = q - s0 + dot(s0 - q, v) * v;
    double k = circle.r;
    if(auto res = solveLinerVectorSizeEq(c, a, k)){
        if(betweenEq(0., res->first, 1.)){
            double t = res->first;
            auto p = s0 + t * e, n = t * a + c;
            return StaticLineVSMovingCircleCollRes{t, p, p + n};
        }
    }
    return std::nullopt;
}

// struct representing collision result between segment and moving circle
struct StaticSegmentVSMovingCircleCollRes : public StaticLineVSMovingCircleCollRes
{};
std::optional<StaticSegmentVSMovingCircleCollRes> calcCollisionPos(const Segment& segment, const Circle& circle, const Segment& move)
{
    // First, check collision in the segment's body, not on its ends
    if(auto res = calcCollisionPos(Line{segment.p, segment.v}, circle, move)){
        auto cp = res->contactPos;
        if(betweenEq(segment.left(), cp.x, segment.right()) && betweenEq(segment.top(), cp.y, segment.bottom()))
            return StaticSegmentVSMovingCircleCollRes{res->elapsedTime, res->circlePosOnHit, res->contactPos};
    }

    // Then, check on ends
    auto p0 = circle.p, e = move.v;
    double k = circle.r;
    if(auto res = solveLinerVectorSizeEq(p0 - segment.from(), e, k))
        if(betweenEq(0., res->first, 1.))
            return StaticSegmentVSMovingCircleCollRes{res->first, p0 + res->first * e, segment.from()};
    if(auto res = solveLinerVectorSizeEq(p0 - segment.to(), e, k))
        if(betweenEq(0., res->first, 1.))
            return StaticSegmentVSMovingCircleCollRes{res->first, p0 + res->first * e, segment.to()};

    return std::nullopt;
}

bool sharpAngle(const Point& p1, const Point& p2, const Point& p3)
{
    return sharpAngle(p1 - p2, p3 - p2);
}

// calculate distance vector
//      * (p)
//      |
//      |    <= calulate this vector
//      v
// ------------ (l)
Vec2d calcDistVec(const Point& p, const Line& l)
{
    double t = dot(l.v, p - l.p) / l.v.lengthSq();
    return (l.p + t * l.v) - p;
}

Vec2d calcDistVec(const Point& p, const Segment& s)
{
    if(!sharpAngle(p, s.from(), s.to()))    return s.from() - p;
    if(!sharpAngle(p, s.to(), s.from()))    return s.to() - p;
    return calcDistVec(p, Line{s.p, s.v});
}

Vec2d getResilienceNorm(const Line& line, const Vec2d& v)
{
    auto nv = Vec2d(line.v.y, -line.v.x).norm();
    if(!sameSide(line.v, -v, nv))   nv *= -1.;
    return nv;
}

// return points of path except for p1
std::vector<Point> calcCubicBezPath(const Point& p1, const Point& p2, const Point& p3, const Point& p4, double tol, int level = 0)
{
    if(level > 12)  return {};
    auto p12 = (p1 + p2) * 0.5,
         p23 = (p2 + p3) * 0.5,
         p34 = (p3 + p4) * 0.5,
         p123 = (p12 + p23) * 0.5,
         p234 = (p23 + p34) * 0.5,
         p1234 = (p123 + p234) * 0.5;
    if(calcDistVec(p1234, makeSegment(p1, p4)).lengthSq() <= tol * tol)
        return {p4};
    return calcCubicBezPath(p1, p12, p123, p1234, tol, level + 1) +
           calcCubicBezPath(p1234, p234, p34, p4, tol, level + 1);
}

class SVGParser
{
private:
    std::shared_ptr<NSVGimage> image_;

public:
    SVGParser(const std::string& filepath)
        : image_(nsvgParseFromFile(filepath.c_str(), "px", 96), nsvgDelete)
    {
        HOOLIB_THROW_UNLESS(image_, "can't read SVG file: " + filepath);
    }

    std::pair<std::vector<Point>, std::vector<Point>> getControllPoints() const
    {
        std::vector<Point> pts14, pts23;
        for(auto shape = image_->shapes;shape != NULL;shape = shape->next){
            for(auto path = shape->paths;path != NULL;path = path->next){
                for(int i = 0;i < path->npts - 1;i += 3){
                    float *p = &path->pts[i * 2];
                    pts14.emplace_back(p[0], p[1]);
                    pts23.emplace_back(p[2], p[3]);
                    pts23.emplace_back(p[4], p[5]);
                    pts14.emplace_back(p[6], p[7]);
                }
            }
        }
        return std::make_pair(pts14, pts23);

    }

    std::vector<Segment> createSegments() const
    {
        std::vector<Segment> ret;
        for(auto shape = image_->shapes;shape != NULL;shape = shape->next){
            for(auto path = shape->paths;path != NULL;path = path->next){
                std::vector<Point> points({Point(path->pts[0], path->pts[1])});
                for(int i = 0;i < path->npts - 1;i += 3){
                    float *p = &path->pts[i * 2];
                    points += calcCubicBezPath({p[0], p[1]}, {p[2], p[3]}, {p[4], p[5]}, {p[6], p[7]}, 1);
                }
                if(path->closed)    points.push_back(points.front());
                for(int i = 1;i < points.size();i++)
                    ret.push_back(makeSegment(points[i - 1], points[i]));
            }
        }
        return ret;
    }
};

class Ball
{
    constexpr static double SEGMENT_THICKNESS = 0.0001;

private:
    Circle c_;
    Vec2d a_, v_;

public:
    Ball(const Circle& c, const Vec2d& a, const Vec2d& v)
        : c_(c), a_(a), v_(v)
    {}

    const Circle& circle() const { return c_; }
    const Vec2d& v() const { return v_; }
    const Vec2d& a() const { return a_; }

private:
    void avoidPresentCollision(const std::vector<Segment>& bars)
    {
        while(true){
            bool hasNoCollision = true;
            for(auto&& bar : bars){
                auto distVec = calcDistVec(c_.p, bar);
                double x = c_.r - distVec.length();
                if(x < 0)   continue;
                c_.p += (x + SEGMENT_THICKNESS) * -distVec.norm();
                hasNoCollision = false;
            }
            if(hasNoCollision)  break;
        }
    }

    void processCollision(double dt, const std::vector<Segment>& bars)
    {
        avoidPresentCollision(bars);

        // check collisions in moving
        Segment move{c_.p, v_ * dt};
        while(!equal0(move.length())){
            std::vector<StaticLineVSMovingCircleCollRes> colls;
            for(auto&& bar : bars){
                auto res = calcCollisionPos(bar, Circle{move.from(), c_.r}, move);
                if(!res)  continue;
                colls.push_back(*res);
            }
            auto it = std::min_element(HOOLIB_RANGE(colls), [&move](const auto& lhs, const auto& rhs) {
                return distanceSq(move.from(), lhs.circlePosOnHit) < distanceSq(move.from(), rhs.circlePosOnHit);
            });

            if(it == colls.end())    break;

            double t = it->elapsedTime;
            auto p = it->circlePosOnHit, nv = (it->circlePosOnHit - it->contactPos).norm();
            v_ += 2 * dot(-v_, nv) * nv * 0.8;
            move = Segment{p, v_ * (1 - t) * dt};
        }

        c_.p = move.to();
    }

public:
    void update(double dt, const std::vector<Segment>& bars)
    {
        processCollision(dt, bars);
        v_ += a_ * dt;
    }
};

int main()
{
    Ball ball{Circle{{153, 200}, 5}, {0, 300}, {0, -500}};
    SVGParser parser("field.svg");
    //auto contPts = parser.getControllPoints();
    auto bars = parser.createSegments();

    sf::Clock clock;
    Canvas().run([&](auto& window) {
        // update
        double dt = clock.restart().asSeconds();
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
            ball.update(dt, bars);

        // draw
        window.draw(SfCircle(ball.circle()));
        for(auto&& bar : bars)
            window.draw(SfSegment(bar));
        DebugPrinter printer(Point(0, 500));
        printer << "x = " << ball.circle().p << std::endl
                << "v = " << ball.v() << std::endl
                << "a = " << ball.a() << std::endl;
        window.draw(printer);
        //for(auto&& pt : contPts.first)
        //    window.draw(SfDot(pt));
    });
    return 0;
}

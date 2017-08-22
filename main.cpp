#include <optional>
#include <tuple>
#include <vector>
#include <SFML/Graphics.hpp>
#include "hoolib.hpp"
#include "canvas.hpp"

#include <iostream>

using HooLib::equal, HooLib::equal0, HooLib::between, HooLib::betweenEq;

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
    std::vector<Segment> bars = {
        {Point(100, 500), Vec2d(500, 80)},
        {Point(100, 800), Vec2d(500, -500)}
    };
    Ball ball{Circle{{100.01, 400}, 5}, {0, 300}, {0, 0}};

    sf::Clock clock;
    Canvas().run([&](auto& window) {
        // update
        double dt = clock.restart().asSeconds();
        ball.update(dt, bars);

        // draw
        window.draw(SfCircle(ball.circle()));
        for(auto&& bar : bars)
            window.draw(SfSegment(bar));
        DebugPrinter printer(Point(0, 0));
        printer << "x = " << ball.circle().p << std::endl
                << "v = " << ball.v() << std::endl
                << "a = " << ball.a() << std::endl;
        window.draw(printer);
    });
    return 0;
}

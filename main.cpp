#include <optional>
#include <tuple>
#include <vector>
#include <SFML/Graphics.hpp>
#include "hoolib.hpp"
#include "canvas.hpp"

#include <iostream>

using HooLib::equal, HooLib::equal0, HooLib::between, HooLib::betweenEq;

// struct representing collision result between line and moving circle
struct StaticLineVSMovingCircleCollRes
{
    double elapsedTime; // 0 <= t <= 1
    Point contactPos;
};
std::optional<StaticLineVSMovingCircleCollRes> calcCollisionPos(const Line& line, const Circle& circle, const Segment& move)
{
    auto q = line.p, v = line.v.norm(), s0 = move.p, e = move.v;
    auto a = -e + dot(e, v) * v, c = q - s0 + dot(s0 - q, v) * v;
    double alpha = a.lengthSq();
    if(equal0(alpha))   return std::nullopt;
    double beta = dot(a, c), gamma = c.lengthSq() - circle.r * circle.r;
    double Dquarter = beta * beta - alpha * gamma;
    if(Dquarter < 0) return std::nullopt;
    double t = (-beta - std::sqrt(Dquarter)) / alpha;
    if(!(0 <= t && t <= 1))    return std::nullopt;
    return StaticLineVSMovingCircleCollRes{t, s0 + t * e};
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

    void update(double dt, const std::vector<Segment>& bars)
    {
        // avoid present collision
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

        // check collisions in moving
        Segment move{c_.p, v_ * dt};
        while(!equal0(move.length())){
            std::vector<std::tuple<Point, double, Vec2d>> colls;    // contact, elapsed, norm of bar
            for(auto&& bar : bars){
                auto res = calcCollisionPos(bar, Circle{move.from(), c_.r}, move);
                if(!res)  continue;
                auto p = res->contactPos;
                auto nv = getResilienceNorm(bar, move.v);
                auto cp = p - nv * c_.r;
                if(!betweenEq(bar.left(), cp.x, bar.right()) || !betweenEq(bar.top(), cp.y, bar.bottom()))
                    continue;
                colls.emplace_back(p, res->elapsedTime, nv);
            }
            auto it = std::min_element(HOOLIB_RANGE(colls), [&move](const auto& lhs, const auto& rhs) {
                return distanceSq(move.from(), std::get<0>(lhs)) < distanceSq(move.from(), std::get<0>(rhs));
            });

            if(it == colls.end())    break;

            auto [p, t, nv] = *it;
            v_ += 2 * dot(-v_, nv) * nv * 0.8;
            move = Segment{p + nv * SEGMENT_THICKNESS, v_ * (1 - t) * dt};
        }

        // check collisions after moving
        auto rv = Vec2d::zero();
        for(auto&& bar : bars){
            auto distVec = calcDistVec(move.to(), bar);
            double x = c_.r - distVec.length();
            if(x < 0)   continue;
            auto ndv = distVec.norm();
            v_ += 2 * dot(-v_, ndv) * ndv * 0.8;
        }

        c_.p = move.to();
        v_ += a_ * dt;
    }
};

int main()
{
    std::vector<Segment> bars = {
        {Point(100, 500), Vec2d(500, 50)},
        //{Point(100, 800), Vec2d(500, -500)}
    };
    Ball ball{Circle{{100.01, 400}, 10}, {0, 300}, {0, 0}};

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

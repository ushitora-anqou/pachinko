#include <optional>
#include <vector>
#include <SFML/Graphics.hpp>
#include "hoolib.hpp"
#include "canvas.hpp"

#include <iostream>

using
    HooLib::Vec2d, HooLib::Point,
    HooLib::Segment, HooLib::Circle, HooLib::Line,
    HooLib::equal, HooLib::equal0, HooLib::between, HooLib::betweenEq, HooLib::distance, HooLib::distanceSq, HooLib::sharpAngle;

std::optional<Point> calcCollisionPos(const Line& line, const Circle& circle, const Segment& move)
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
    return s0 + t * e;
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

int main()
{
    std::vector<Segment> bars = {
        {Point(100, 500), Vec2d(500, 50)},
        //{Point(100, 800), Vec2d(500, -500)}
    };
    Circle ball{Point{99.99, 400}, 10};
    Vec2d a{0, 300}, v{0, 0};

    sf::Clock clock;
    Canvas().run([&](auto& window) {
        // update
        double dt = clock.restart().asSeconds();

        // avoid present collision
        while(true){
            bool hasNoCollision = true;
            for(auto&& bar : bars){
                auto distVec = calcDistVec(ball.p, bar);
                double x = ball.r - distVec.length();
                if(x < 0)   continue;
                ball.p += (x + 0.01) * -distVec.norm();
                hasNoCollision = false;
            }
            if(hasNoCollision)  break;
        }

        // check collisions in moving
        Segment move{ball.p, v * dt};
        std::vector<std::pair<Point, Vec2d>> colls;
        for(auto&& bar : bars){
            auto p = calcCollisionPos(bar, ball, move);
            if(!p)  continue;
            auto nv = getResilienceNorm(bar, move.v);
            auto cp = *p - nv * ball.r;
            if(!betweenEq(bar.left(), cp.x, bar.right()) || !betweenEq(bar.top(), cp.y, bar.bottom()))
                continue;
            colls.emplace_back(*p, nv);
        }
        auto it = std::min_element(HOOLIB_RANGE(colls), [&](const auto& lhs, const auto& rhs) {
            return distanceSq(ball.p, lhs.first) < distanceSq(ball.p, rhs.first);
        });
        if(it != colls.end()){
            v += 2 * dot(-v, it->second) * it->second * 0.8;
            move = makeSegment(move.from(), it->first + it->second * 0.01);  //TODO
        }

        // check collisions after moving
        auto rv = Vec2d::zero();
        for(auto&& bar : bars){
            auto distVec = calcDistVec(move.to(), bar);
            double x = ball.r - distVec.length();
            if(x < 0)   continue;
            auto ndv = distVec.norm();
            v += 2 * dot(-v, ndv) * ndv * 0.8;
        }

        ball.p = move.to();
        v += a * dt;

        // draw
        window.draw(SfCircle(ball.p, ball.r));
        for(auto&& bar : bars)
            window.draw(SfSegment(bar));
        DebugPrinter printer(Point(0, 0));
        printer << "x = " << ball.p << std::endl
                << "v = " << v << std::endl
                << "a = " << a << std::endl;
        window.draw(printer);
    });
    return 0;
}

#include <optional>
#include <vector>
#include <SFML/Graphics.hpp>
#include "hoolib.hpp"
#include "canvas.hpp"

#include <iostream>

using
    HooLib::Vec2d, HooLib::Point,
    HooLib::Segment, HooLib::Circle, HooLib::Line,
    HooLib::equal, HooLib::equal0, HooLib::distance, HooLib::distanceSq;

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

double distance(const Point& p, const Line& l)
{
    auto nv = l.v.norm();
    return std::abs(cross(p, nv) + cross(nv, l.p));
}

int main()
{
    std::vector<Segment> bars = {
        {Point(100, 500), Vec2d(500, 50)},
        {Point(100, 800), Vec2d(500, -500)}
    };
    Circle ball{Point{10, 400}, 10};
    Vec2d a{0, 300}, v{0, 0};

    sf::Clock clock;
    Canvas().run([&](auto& window) {
        // update
        double dt = clock.restart().asSeconds();
        Segment move{ball.p, v * dt};

        std::vector<std::pair<Point, Vec2d>> colls;
        for(auto&& bar : bars){
            auto nv = Vec2d(bar.v.y, -bar.v.x).norm();
            if(!sameSide(bar.v, -move.v, nv))   nv *= -1.;

            auto p = calcCollisionPos(bar, ball, move);
            if(p){
                colls.emplace_back(*p, nv);
            }
            else{
                double x = ball.r - distance(move.from(), bar);
                if(x > 0){
                    ball.p += (x + 1) * nv;
                    v = dot(v, bar.v.norm()) * bar.v.norm();
                }
            }
        }
        auto it = std::min_element(HOOLIB_RANGE(colls), [&](const auto& lhs, const auto& rhs) {
            return distanceSq(ball.p, lhs.first) < distanceSq(ball.p, rhs.first);
        });
        if(it != colls.end()){
            v += 2 * dot(-v, it->second) * it->second * 0.8;
            move = makeSegment(move.from(), it->first + it->second * 0.01);  //TODO
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

#include <optional>
#include <vector>
#include <SFML/Graphics.hpp>
#include "hoolib.hpp"
#include "canvas.hpp"

#include <iostream>

using HooLib::Vec2d, HooLib::Point, HooLib::Segment, HooLib::equal, HooLib::equal0, HooLib::distance, HooLib::distanceSq;

std::optional<Point> checkCollision(const Segment& s0, const Segment& s1)
{
    const static std::optional<Point> none;
    auto a = s0.from(), b = s0.to(), c = s1.from(), d = s1.to();
    double det = (b.x - a.x) * (d.y - c.y) - (b.y - a.y) * (d.x - c.x);
    if(equal0(det)) return none;
    double x = ((b.x - a.x) * (c.x * d.y - c.y * d.x) - (d.x - c.x) * (a.x * b.y - a.y * b.x)) / det,
           y = ((b.y - a.y) * (c.x * d.y - c.y * d.x) - (d.y - c.y) * (a.x * b.y - a.y * b.x)) / det;
    if(std::max(std::min(a.x, b.x), std::min(c.x, d.x)) <= x &&
        x <= std::min(std::max(a.x, b.x), std::max(c.x, d.x)) &&
        std::max(std::min(a.y, b.y), std::min(c.y, d.y)) <= y &&
        y <= std::min(std::max(a.y, b.y), std::max(c.y, d.y)))
            return Point(x, y);
    return none;
}

struct SegmentsCollision
{
    std::array<Segment, 2> segment;
    std::array<double, 2> distSq2Pos;
    Point pos;
};

std::vector<SegmentsCollision> checkCollisions(const Segment& seg, const std::vector<Segment>& walls)
{
    std::vector<SegmentsCollision> colls;
    for(auto&& wall : walls){
        auto p = checkCollision(seg, wall);
        if(!p)  continue;
        colls.push_back(SegmentsCollision{{seg, wall}, {distanceSq(seg.from(), *p), distanceSq(wall.from(), *p)}, *p});
    }
    return colls;
}

class Player
{
private:
    Point x_;
    Vec2d v_, a_;

public:
    Player(const Point& x, const Vec2d& v, const Vec2d& a)
        : x_(x), v_(v), a_(a)
    {}

    Point getPos() const { return x_; }
    Vec2d getVelocity() const { return v_; }
    Vec2d getAccel() const { return a_; }

    void update(double dt, const std::vector<Segment>& walls)
    {
        Segment move{x_, v_ * dt};
        while(!equal0(move.length())){
            // select wall with needed values which collides nearest.
            auto collisions = checkCollisions(move, walls);
            auto it = std::min_element(HOOLIB_RANGE(collisions), [move](auto target, auto smallest) {
                if(equal(target.pos, move.from()))  return false;
                return target.distSq2Pos[0] < smallest.distSq2Pos[0];
            });

            if(it == collisions.end() || equal(it->pos, move.from()))   break;

            // calculate the next position
            static const double e = 0.8;
            auto vn = v_.norm();
            auto tn = Vec2d(it->segment[1].v.y, -it->segment[1].v.x).norm();
            auto d = vn - 2 * dot(vn, tn) * tn;
            v_ = d * v_.length() * e;
            move = Segment{it->pos, d * (move.length() - std::sqrt(it->distSq2Pos[0])) * e};
        }

        x_ = move.to();
        v_ += a_ * dt;
    }
};

///


int main()
{
    std::vector<Segment> bars = {
        {Point(50, 200), Vec2d(100, 85)},
        {Point(600, 500), Vec2d(-500, 30)},
        {Point(600, 500), Vec2d(300, -500)},
        {Point(105, 535), Vec2d(-100, -300)},
    };
    Player player(Point(80, 0), Vec2d(-1, 5), Vec2d(0, 200));

    sf::Clock clock;
    Canvas().run([&](auto& window) {
        // update
        double dt = clock.restart().asSeconds();
        player.update(dt, bars);

        // draw
        window.draw(SfDot(player.getPos()));
        for(auto&& bar : bars)
            window.draw(SfSegment(bar));
        DebugPrinter printer(Point(500, 500));
        printer << "x = " << player.getPos() << std::endl
                << "v = " << player.getVelocity() << std::endl
                << "a = " << player.getAccel() << std::endl;
        window.draw(printer);
    });
    return 0;
}

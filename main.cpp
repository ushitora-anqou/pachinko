#include <optional>
#include <vector>
#include <SFML/Graphics.hpp>
#include "hoolib.hpp"
#include "canvas.hpp"

#include <iostream>

using HooLib::Vec2d, HooLib::Point, HooLib::Segment, HooLib::equal, HooLib::equal0, HooLib::distance, HooLib::distanceSq;

namespace Pachinko
{

class Object
{
public:
    struct Info
    {
        double m;
        Point x;
        Vec2d v = Vec2d::zero(), a = Vec2d::zero();
    };

    static double getSpringConst() { return 6000; }
    static double getViscosityConst() { return 150; }

private:
    Info info_;

protected:

    virtual Vec2d getGivenForce(const std::vector<Segment>& bars) = 0;

public:
    Object(const Info& info)
        : info_(info)
    {}

    double m() const { return info_.m; }
    const Point& x() const { return info_.x; }
    const Vec2d& v() const { return info_.v; }
    const Vec2d& a() const { return info_.a; }

    void update(double dt, const std::vector<Segment>& bars)
    {
        info_.a = getGivenForce(bars) / info_.m;
        info_.v += info_.a * dt;
        info_.x += info_.v * dt;
    }
};

class GravitizedObject : public Object
{
private:
    static constexpr double GRAV_ACCEL = 300;

protected:
    virtual Vec2d getGivenForceWithoutGravity(const std::vector<Segment>& bars) = 0;

private:
    Vec2d getGivenForce(const std::vector<Segment>& bars)
    {
        return getGivenForceWithoutGravity(bars) + Vec2d(0, GRAV_ACCEL * m());
    }

public:
    GravitizedObject(const Object::Info& info)
        : Object(info)
    {}
};

class Circle : public GravitizedObject
{
private:
    double r_;

protected:
    Vec2d getGivenForceWithoutGravity(const std::vector<Segment>& bars) override
    {
        std::optional<double> minD_;
        Vec2d na, sv, snv;
        for(auto&& bar : bars){
            auto a = x() - bar.from(), b = x() - bar.to(), s = bar.v;
            double d = std::abs(cross(s, a) / s.length());
            if(d > r_)  continue;
            if(!(dot(a, s) * dot(b, s) <= 0 || (r_ > a.length() || r_ > b.length())))   continue;
            minD_ = d;
            sv = bar.v.norm();
            snv = Vec2d(sv.y, -sv.x);
            na = a.norm();
        }

        if(!minD_)  return Vec2d::zero();
        auto d_size = (getSpringConst() * (r_ - *minD_) - getViscosityConst() * std::abs(dot(v(), snv)));
        auto d = snv * d_size;
        //std::cout << *minD_ << ", " << d.x << ", " << d.y << ", " << d_size << std::endl;
        return cross(sv, d) * cross(sv, na) > 0 ? d : -d;
    }

public:
    Circle(double r, const Object::Info& info)
        : GravitizedObject(info), r_(r)
    {}

    double r() const { return r_; }
};

}

int main()
{
    std::vector<Segment> bars = {
        {Point(100, 500), Vec2d(200, 0)},
    };
    Pachinko::Circle ball{10, Pachinko::Object::Info{1, Point{150, 70}}};

    sf::Clock clock;
    Canvas().run([&](auto& window) {
        // update
        double dt = clock.restart().asSeconds();
        ball.update(dt, bars);

        // draw
        window.draw(SfCircle(ball.x(), ball.r()));
        for(auto&& bar : bars)
            window.draw(SfSegment(bar));
        DebugPrinter printer(Point(0, 0));
        printer << "x = " << ball.x() << std::endl
                << "v = " << ball.v() << std::endl
                << "a = " << ball.a() << std::endl;
        window.draw(printer);
    });
    return 0;
}

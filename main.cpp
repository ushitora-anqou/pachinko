#include <memory>
#include <optional>
#include <SFML/Graphics.hpp>

class Canvas
{
private:
    std::shared_ptr<sf::RenderWindow> window_;

public:
    Canvas()
    {
        sf::ContextSettings settings;
        settings.antialiasingLevel = 8;
        window_ = std::make_shared<sf::RenderWindow>(
            sf::VideoMode(800, 600),
            "Pachinko",
            sf::Style::Default,
            settings
        );
    }

    template<class Func>
    bool run(Func func)
    {
        while (window_->isOpen())
        {
            sf::Event event;
            while (window_->pollEvent(event))
                if (event.type == sf::Event::Closed)
                    window_->close();

            window_->clear(sf::Color::Black);
            func(*window_);
            window_->display();
        }
    }
};

#include <cmath>
template<class T>
struct Vec2
{
    using type = T;

    T x, y;

    Vec2()
    {}
    Vec2(T x, T y)
        : x(x), y(y)
    {}

    Vec2& operator+=(const Vec2<T>& rhs)
    {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }
    Vec2& operator-=(const Vec2<T>& rhs)
    {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }
    Vec2& operator*=(T k)
    {
        x *= k;
        y *= k;
        return *this;
    }
    Vec2& operator/=(T k)
    {
        x /= k;
        y /= k;
        return *this;
    }

    T lengthSq() const
    {
        return x * x + y * y;
    }
    T length() const
    {
        using std::sqrt;
        return sqrt(lengthSq());
    }
    Vec2<T> norm() const
    {
        const T len = length();
        if(len > 0) return Vec2<T>(x / len, y / len);
        return Vec2(0, 0);
    }
};

template<class T>
Vec2<T> operator+(const Vec2<T>& lhs, const Vec2<T>& rhs)
{
    Vec2<T> ret(lhs);
    ret += rhs;
    return ret;
}

template<class T>
Vec2<T> operator-(const Vec2<T>& lhs, const Vec2<T>& rhs)
{
    Vec2<T> ret(lhs);
    ret -= rhs;
    return ret;
}

template<class T>
Vec2<T> operator*(const Vec2<T>& lhs, T rhs)
{
    Vec2<T> ret(lhs);
    ret *= rhs;
    return ret;
}

template<class T>
Vec2<T> operator*(T lhs, const Vec2<T>& rhs)
{
    Vec2<T> ret(rhs);
    ret *= lhs;
    return ret;
}

template<class T>
Vec2<T> operator/(const Vec2<T>& lhs, T rhs)
{
    Vec2<T> ret(lhs);
    ret /= rhs;
    return ret;
}

template<class T>
Vec2<T> operator/(T lhs, const Vec2<T>& rhs)
{
    Vec2<T> ret(rhs);
    ret /= lhs;
    return ret;
}

template<class T>
double distance(const Vec2<T>& lhs, const Vec2<T>& rhs)
{
    return (lhs - rhs).length();
}

template<class T>
double distanceSq(const Vec2<T>& lhs, const Vec2<T>& rhs)
{
    return (lhs - rhs).lengthSq();
}

template<class T>
double dot(const Vec2<T>& lhs, const Vec2<T>& rhs)
{
    return lhs.x * rhs.x + lhs.y * rhs.y;
}

template<class T>
double cross(const Vec2<T>& lhs, const Vec2<T>& rhs)
{
    return lhs.x * rhs.y - lhs.y * rhs.x;
}

using Vec2d = Vec2<double>;
using Point = Vec2d;

struct Segment
{
    Point p;
    Vec2d v;

    Point from() const { return p; }
    Point to() const { return Point(p.x + v.x, p.y + v.y); }
    double length() const { return v.length(); }
};

const static double EQUAL_ERROR = 0.000000001;

bool equal(double x, double y, double e = EQUAL_ERROR)
{
    return std::abs(x - y) < e;
}

bool equal0(double x, double e = EQUAL_ERROR)
{
    return equal(x, 0, e);
}

std::optional<Point> checkCollision(const Segment& s0, const Segment& s1)
{
    const static std::optional<Point> none;
    auto a = s0.from(), b = s0.to(), c = s1.from(), d = s1.to();
    double det = (b.x - a.x) * (d.y - c.y) - (b.y - a.y) * (d.x - c.x);
    if(equal0(det)) return none;
    double x = ((b.x - a.x) * (c.x * d.y - c.y * d.x) - (d.x - c.x) * (a.x * b.y - a.y * b.x)) / det,
           y = ((b.y - a.y) * (c.x * d.y - c.y * d.x) - (d.y - c.y) * (a.x * b.y - a.y * b.x)) / det;
    if(std::max(std::min(a.x, b.x), std::min(c.x, d.x)) <= x &&
        x < std::min(std::max(a.x, b.x), std::max(c.x, d.x)) &&
        std::max(std::min(a.y, b.y), std::min(c.y, d.y)) <= y &&
        y < std::min(std::max(a.y, b.y), std::max(c.y, d.y)))
            return Point(x, y);
    return none;
}

sf::Vector2f toSfVec(const Vec2d& src)
{
    return sf::Vector2f(src.x, src.y);
}

class SfSegment : public sf::Drawable
{
private:
    std::array<sf::Vertex, 2> vertices_;

public:
    SfSegment(const Segment& src)
        : vertices_({toSfVec(src.from()), toSfVec(src.to())})
    {}

private:
    void draw(sf::RenderTarget& target, sf::RenderStates states) const override
    {
        target.draw(vertices_.data(), 2, sf::Lines, states);
    }
};

class SfDot : public sf::Shape
{
private:
    sf::CircleShape dot_;

public:
    SfDot(const Point& pos)
        : dot_(10)
    {
        dot_.setPosition(pos.x, pos.y);
        dot_.setPointCount(4);
    }

    std::size_t getPointCount() const override
    {
        return dot_.getPointCount();
    }

    sf::Vector2f getPoint(std::size_t index) const
    {
        return dot_.getPoint(index);
    }
};

#include <iostream>

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
        Segment plMove{x_, v_ * dt};

        // select wall with needed values which collides nearest.
        std::optional<Segment> target;
        Point colPos;
        double minDistanceSq;
        for(auto&& wall : walls){
            auto res = checkCollision(plMove, wall);
            if(!res)    continue;
            auto p = *res;  // collision pos
            double dist = distanceSq(p, x_);
            if(target && dist >= minDistanceSq) continue;
            target = wall;
            colPos = p;
            minDistanceSq = dist;
        }

        // calculate the next position
        static const double e = 0.8;
        auto vn = v_.norm();
        auto tn = Vec2d(target->v.y, -target->v.x).norm();
        auto d = vn - 2 * dot(vn, tn) * tn;
        x_ = colPos + d * (plMove.length() - std::sqrt(minDistanceSq)) * e;
        v_ = d * v_.length() * e;
    }
};

int main()
{
    Segment bar = {Point(50, 50), Vec2d(100, 85)};
    Segment player = {Point(75, 0), Vec2d(-1, 5)};
    Vec2d d(-1, 5);

    sf::Clock clock;
    Canvas().run([&](auto& window) {
        window.draw(SfSegment(bar));
        window.draw(SfSegment(player));

        if(auto result = checkCollision(bar, player)){
            auto p = *result;
            double rest = d.length() - distance(p, player.p + player.v - d);
            auto n = Vec2d(bar.v.y, -bar.v.x).norm();
            d = d - 2 * dot(d, n) * n;
            player.p = p + d.norm();
            player.v = d;
        }

        if(clock.getElapsedTime().asMilliseconds() >= 200){
            player.v += d;
            clock.restart();
        }
    });
    return 0;
}

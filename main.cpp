#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <sstream>
#include <SFML/Graphics.hpp>


namespace HooLib {

std::string createErrorMsg(const std::string& what, const char *file, int line)
{
    std::stringstream ss;
    ss << "(<" << file << "," << line << ">" << what << ")";
    return ss.str();
}

#define HOOLIB_ERROR(msg) HooLib::createErrorMsg((msg), __FILE__, __LINE__)
#define HOOLIB_THROW(msg) { throw std::runtime_error(HOOLIB_ERROR(msg)); };
#define HOOLIB_THROW_IF(ret, msg) if((ret)){HOOLIB_THROW((msg));}
#define HOOLIB_THROW_UNLESS(ret, msg) HOOLIB_THROW_IF(!(ret), msg);

#define unless(cond) if(!(cond))
#define until(cond) while(!(cond))


template<class T> std::string to_str(T t)
{
    std::stringstream ss;
    ss << t;
    return ss.str();
}

std::vector<std::string> splitStrByChars(const std::string& src, const std::string& delimChars)
{
    std::shared_ptr<char> data(new char[src.size() + 1], std::default_delete<char[]>());
    std::vector<std::string> ret;

    std::strcpy(data.get(), src.c_str());

    char *p = std::strtok(data.get(), delimChars.c_str());
    while(p != nullptr){
        ret.emplace_back(p);
        p = std::strtok(nullptr, delimChars.c_str());
    }

    return std::move(ret);
}
}

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
        x <= std::min(std::max(a.x, b.x), std::max(c.x, d.x)) &&
        std::max(std::min(a.y, b.y), std::min(c.y, d.y)) <= y &&
        y <= std::min(std::max(a.y, b.y), std::max(c.y, d.y)))
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

class SfDot : public sf::CircleShape
{
    constexpr static double RADIUS = 5, POINT_COUNT = 6;
public:
    SfDot(const Point& pos)
        : sf::CircleShape(RADIUS)
    {
        setOrigin(RADIUS, RADIUS);
        setPosition(pos.x, pos.y);
        setPointCount(POINT_COUNT);
        setFillColor(sf::Color::White);
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

        if(target){
            // calculate the next position
            static const double e = 0.8;
            auto vn = v_.norm();
            auto tn = Vec2d(target->v.y, -target->v.x).norm();
            auto d = vn - 2 * dot(vn, tn) * tn;
            x_ = colPos + d * (plMove.length() - std::sqrt(minDistanceSq)) * e;
            v_ = d * v_.length() * e;
        }
        else{
            x_ += v_ * dt;
            v_ += a_ * dt;
        }
    }
};

class DebugPrinter : public sf::Drawable
{
private:
    Point pos_;
    sf::Font font_;
    std::stringstream ss_;

public:
    DebugPrinter(const Point& pos)
        : pos_(pos)
    {
        HOOLIB_THROW_UNLESS(
            font_.loadFromFile("/home/anqou/.fonts/Ricty-Regular.ttf"),
            "Can't load font for debug"
        );
    }

    template<class T> DebugPrinter& operator<<(T t)
    {
        ss_ << t;
        return *this;
    }

    DebugPrinter& operator <<(std::ostream& (*manip)(std::ostream&)) {
        manip(ss_);
        return *this;
    }

    DebugPrinter& operator<<(const Vec2d& v)
    {
        ss_ << "(" << v.x << ", " << v.y << ")";
        return *this;
    }

private:
    void draw(sf::RenderTarget& target, sf::RenderStates states) const override
    {
        auto src = HooLib::splitStrByChars(ss_.str(), "\n");
        for(int i = 0;i < src.size();i++){
            sf::Text text;
            text.setFont(font_);
            text.setString(src[i]);
            text.setCharacterSize(24);
            text.setColor(sf::Color::White);
            text.setPosition(pos_.x, pos_.y + i * 30);
            target.draw(text, states);
        }
    }
};

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

#include <functional>
#include <optional>
#include <tuple>
#include <vector>
#include <SFML/Graphics.hpp>
#include "hoolib.hpp"
#include "canvas.hpp"

#define NANOSVG_IMPLEMENTATION
#include "3rd/nanosvg.h"

#include <iostream>
#include <bitset>

using HooLib::equal, HooLib::equal0, HooLib::between, HooLib::betweenEq, HooLib::deg2rad;
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

class Bar
{
private:
    Segment seg_;
    Vec2d n_;
    double e_;
    std::function<double(double)> vFunc_;

public:
    Bar(const Segment& seg, double e, const std::function<double(double)>& vFunc = [](double) { return 0; })
        : seg_(seg), n_(Vec2d(seg.v.y, -seg.v.x).norm()), e_(e), vFunc_(vFunc)
    {}

    const Segment& segment() const { return seg_; }
    double e() const { return e_; }
    Vec2d v(double r) const { return vFunc_(r) * n_; }
};

class FieldSVGParser
{
    constexpr static unsigned int FLIPPER_FILL_COLOR = 0xff0000ff;

private:
    std::shared_ptr<NSVGimage> image_;

public:
    FieldSVGParser(const std::string& filepath)
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

    std::vector<Point> getFlipperPoints() const
    {
        std::vector<Point> ret;
        for(auto shape = image_->shapes;shape != NULL;shape = shape->next)
            if(shape->fill.color == FLIPPER_FILL_COLOR)
                ret.emplace_back(shape->paths->pts[0], shape->paths->pts[1]);
        std::sort(HOOLIB_RANGE(ret), [](auto& p1, auto& p2) { return p1.x < p2.x; });
        return ret;
    }

    std::vector<Bar> createBars() const
    {
        std::vector<Bar> ret;
        for(auto shape = image_->shapes;shape != NULL;shape = shape->next){
            if(shape->fill.color == FLIPPER_FILL_COLOR) continue;
            double e = HooLib::divd((shape->stroke.color & (0xff << 24)) >> 24, 0xff * 1.25);
            for(auto path = shape->paths;path != NULL;path = path->next){
                std::vector<Point> points({Point(path->pts[0], path->pts[1])});
                for(int i = 0;i < path->npts - 1;i += 3){
                    float *p = &path->pts[i * 2];
                    points += calcCubicBezPath({p[0], p[1]}, {p[2], p[3]}, {p[4], p[5]}, {p[6], p[7]}, 1);
                }
                if(path->closed)    points.push_back(points.front());
                for(int i = 1;i < points.size();i++)
                    ret.emplace_back(
                        makeSegment(points[i - 1], points[i]),
                        e
                    );
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
    void avoidPresentCollision(const std::vector<Bar>& bars)
    {
        for(int i = 0;i < 10;i++){
            bool hasNoCollision = true;
            for(auto&& bar : bars){
                auto distVec = calcDistVec(c_.p, bar.segment());
                double x = c_.r - distVec.length();
                if(x < 0)   continue;
                c_.p += (x + SEGMENT_THICKNESS) * -distVec.norm();
                hasNoCollision = false;
            }
            if(hasNoCollision)  break;
            if(i == 9)
                std::cerr << "DEBUG: CAN'T AVOID PRESENT COLLISION." << std::endl;
        }
    }

    void processCollision(double dt, const std::vector<Bar>& bars)
    {
        avoidPresentCollision(bars);

        // check collisions in moving
        Segment move{c_.p, v_ * dt};
        for(int i = 0;i < 10;i++){
            if(equal0(move.length()))   break;
            std::vector<std::pair<StaticLineVSMovingCircleCollRes, const Bar&>> colls;
            for(auto&& bar : bars){
                auto res = calcCollisionPos(bar.segment(), Circle{move.from(), c_.r}, move);
                if(!res)  continue;
                colls.push_back(std::make_pair(*res, bar));
            }
            auto it = std::min_element(HOOLIB_RANGE(colls), [&move](const auto& lhs, const auto& rhs) {
                return distanceSq(move.from(), lhs.first.circlePosOnHit) < distanceSq(move.from(), rhs.first.circlePosOnHit);
            });

            if(it == colls.end())    break;

            auto& res = it->first;
            double t = res.elapsedTime, e = it->second.e();
            auto p = res.circlePosOnHit, nv = (res.circlePosOnHit - res.contactPos).norm(),
                 v0 = dot(v_, nv) * nv, V = dot(it->second.v((res.contactPos - it->second.segment().p).length()), nv) * nv;
            v_ += -v0 + (e + 1) * V - e * v0;
            move = Segment{p, v_ * (1 - t) * dt};
        }

        c_.p = move.to();
    }

public:
    void update(double dt, const std::vector<Bar>& bars)
    {
        processCollision(dt, bars);
        v_ += a_ * dt;
    }
};

class Flipper
{
private:
    const Point pos_;
    const double omega_, maxAngle_;
    double angle_;
    Bar nowBar_;
    int rotateDir_;

    bool isRotating() const
    {
        return 
            (rotateDir_ == 1 && std::abs(angle_) < std::abs(maxAngle_)) ||
            (rotateDir_ == -1 && omega_ * angle_ > 0);
    }
    double getVerocity(double r) const
    {
        if(isRotating())
            return std::abs(omega_ * r);
        else
            return 0;
    }

public:
    Flipper(const Bar& bar, double omega, double maxAngle)
        : pos_(bar.segment().p), omega_(omega), maxAngle_(maxAngle),
          angle_(0), nowBar_(bar), rotateDir_(0)
    {}

    const Bar& bar() const { return nowBar_; }

    void setDir(int dir) { rotateDir_ = dir; }

    void update(double dt)
    {
        if(isRotating()){
            double dAngle = rotateDir_ * omega_ * dt;
            angle_ += dAngle;
            nowBar_ = Bar(Segment{pos_, rotate(nowBar_.segment().v, dAngle)}, nowBar_.e(), [this](double r) { return getVerocity(r); });
        }
    }
};

class Pachinko
{
private:
    Ball ball_;
    std::array<Flipper, 2> flippers_;
    std::vector<Bar> bars_;
    sf::Clock clock_;

public:
    Pachinko(const Ball& ball, const std::array<Flipper, 2>& flippers, const std::vector<Bar>& bars)
        : ball_(ball), flippers_(flippers), bars_(bars)
    {}

    void run()
    {
        clock_.restart();
        Canvas().run([&](auto& window) {
            // controll
            bool executed = sf::Keyboard::isKeyPressed(sf::Keyboard::Space);
            flippers_[0].setDir(sf::Keyboard::isKeyPressed(sf::Keyboard::Left) ? 1 : -1);
            flippers_[1].setDir(sf::Keyboard::isKeyPressed(sf::Keyboard::Right) ? 1 : -1);

            // update
            double dt = clock_.restart().asSeconds();
            if(executed)
                ball_.update(dt, bars_ + std::vector<Bar>({flippers_[0].bar(), flippers_[1].bar()}));
            for(auto&& flipper : flippers_)
                flipper.update(dt);

            // draw
            window.draw(SfCircle(ball_.circle()));
            for(auto&& bar : bars_)
                window.draw(SfSegment(bar.segment()));
            for(auto&& flipper : flippers_)
                window.draw(SfSegment(flipper.bar().segment()));

            DebugPrinter printer(Point(400, 0));
            printer << "x = " << ball_.circle().p << std::endl
                    << "v = " << ball_.v() << std::endl
                    << "a = " << ball_.a() << std::endl;
            window.draw(printer);
            //for(auto&& pt : contPts.first)
            //    window.draw(SfDot(pt));
        });
    }
};

int main()
{
    FieldSVGParser field("field.svg");
    Pachinko pachinko(
        Ball{Circle{{353, 300}, 7}, {0, 200}, {0, -500}},
        std::array<Flipper, 2>{
            Flipper{
                Bar{
                    Segment{field.getFlipperPoints().at(0), {30, 10}},
                    1.
                },
                deg2rad(-720),
                deg2rad(20)
            },
            Flipper{
                Bar{
                    Segment{field.getFlipperPoints().at(1), {-30, 10}},
                    1.
                },
                deg2rad(720),
                deg2rad(20)
            }
        },
        field.createBars()
    );
    pachinko.run();

    return 0;
}

#pragma once
#ifndef ANQOU_CANVAS_HPP
#define ANQOU_CANVAS_HPP

#include <array>
#include <memory>
#include <sstream>
#include <SFML/Graphics.hpp>
#include "hoolib.hpp"

inline sf::Vector2f toSfVec(const HooLib::Vec2d& src)
{
    return sf::Vector2f(src.x, src.y);
}

class SfSegment : public sf::Drawable
{
private:
    std::array<sf::Vertex, 2> vertices_;

public:
    SfSegment(const HooLib::Segment& src)
        : vertices_({toSfVec(src.from()), toSfVec(src.to())})
    {}

private:
    void draw(sf::RenderTarget& target, sf::RenderStates states) const override
    {
        target.draw(vertices_.data(), 2, sf::Lines, states);
    }
};

class SfCircle : public sf::CircleShape
{
public:
    SfCircle(const HooLib::Point& pos, double radius)
        : sf::CircleShape(radius)
    {
        setOrigin(radius, radius);
        setPosition(pos.x, pos.y);
        setFillColor(sf::Color::White);
    }
};

class SfDot : public sf::CircleShape
{
    constexpr static double RADIUS = 3, POINT_COUNT = 6;
public:
    SfDot(const HooLib::Point& pos)
        : sf::CircleShape(RADIUS)
    {
        setOrigin(RADIUS, RADIUS);
        setPosition(pos.x, pos.y);
        setPointCount(POINT_COUNT);
        setFillColor(sf::Color::White);
    }
};

class DebugPrinter : public sf::Drawable
{
private:
    HooLib::Point pos_;
    sf::Font font_;
    std::stringstream ss_;

public:
    DebugPrinter(const HooLib::Point& pos)
        : pos_(pos)
    {
        HOOLIB_THROW_UNLESS(
            font_.loadFromFile("Ricty-Regular.ttf"),
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

    DebugPrinter& operator<<(const HooLib::Vec2d& v)
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
            text.setFillColor(sf::Color::White);
            text.setPosition(pos_.x, pos_.y + i * 30);
            target.draw(text, states);
        }
    }
};

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

#endif

main: main.cpp
	g++ -O0 -g -std=c++17 -o $@ $< -lsfml-graphics -lsfml-system -lsfml-window

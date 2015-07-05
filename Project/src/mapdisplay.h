#ifndef MAPDISPLAY_H
#define MAPDISPLAY_H

#include <SDL.h>
#include <pthread.h>
#include <unistd.h>
#include <iostream>
#include <math.h>
#include "interpreter.h"

class MapDisplay
{
public:

    MapDisplay(const Interpreter::Map& map, int screenW, int screenH);
    ~MapDisplay();

    SDL_Surface* GetDisplay() const;
    SDL_Surface* UpdateDisplay();

private:
    const Interpreter::Map& m_map;
    int m_screenW;
    int m_screenH;
    SDL_Surface *m_bgSurf;

    void ConvertScreenCoordToMatrixCoord(int x, int y, int *i, int *j);

};

#endif // MAPDISPLAY_H

#ifndef MAPDISPLAY_H
#define MAPDISPLAY_H

#include <SDL.h>
#include <pthread.h>
#include <unistd.h>
#include <iostream>
#include <math.h>
#include "interpreter.h"

/**
 * @brief This class is responsible for the display of the map on screen.
 *
 */
class MapDisplay
{
public:

    MapDisplay(const Interpreter::Map& map, int screenW, int screenH);
    ~MapDisplay();

    SDL_Surface* GetDisplay() const;
    SDL_Surface* UpdateDisplay();

private:
    const Interpreter::Map& m_map;  /**< map given for the interpreter */
    int m_screenW;                  /**< Width of the screen */
    int m_screenH;                  /**< Height of the screen */
    SDL_Surface *m_bgSurf;          /**< SDL surface */

    void ConvertScreenCoordToMatrixCoord(int x, int y, int *i, int *j);

};

#endif // MAPDISPLAY_H

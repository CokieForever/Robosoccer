#include "mapdisplay.h"
#include "interpreter.h"
#include "matrix.h"
#include "sdlutilities.h"

using namespace std;


/**
 * @brief Constructor of the mapdisplay class. Initializes the variables.
 *
 * @param map Map to draw.
 * @param screenW Width of the surface in which the map will be drawn.
 * @param screenH Height of the surface in which the map will be drawn.
 */
MapDisplay::MapDisplay(const Interpreter::Map& map, int screenW, int screenH) : m_map(map)
{
    m_screenW = screenW;
    m_screenH = screenH;
    m_bgSurf = NULL;
}

/**
 * @brief Destructor of the class mapdisplay.
 *
 */
MapDisplay::~MapDisplay()
{
    if (m_bgSurf)
        SDL_FreeSurface(m_bgSurf);
}

/**
 * @brief This function gets the SDL_surface on which the map is drawn.
 *
 * @return SDL_Surface * The surface.
 */
SDL_Surface* MapDisplay::GetDisplay() const
{
    return m_bgSurf;
}

/**
 * @brief This function updates the SDL surface
 *
 * @return SDL_Surface * SDL_surface on which the map is drawn.
 */
SDL_Surface* MapDisplay::UpdateDisplay()
{
    if (!m_bgSurf)
        m_bgSurf = SDL_CreateRGBSurface(SDL_SWSURFACE, m_screenW, m_screenH, 32,0,0,0,0);

    if (!m_bgSurf)
        return NULL;

    Interpreter::Map map(m_map); //We use a copy to avoid modifications during the update

    SDL_LockSurface(m_bgSurf);
    for (int x=0; x < m_screenW ; x++)
    {
        for (int y=0 ; y < m_screenH ; y++)
        {
            int i=0,j=0;
            ConvertScreenCoordToMatrixCoord(x, y, &i, &j);

            int n = map[i][j];
            if (n == 0)
                PutPixel(m_bgSurf, x, y, SDL_MapRGB(m_bgSurf->format, 0, 255, 0));
            else if (n == 1)
                PutPixel(m_bgSurf, x, y, SDL_MapRGB(m_bgSurf->format, 255, 0, 0));
            else if (n == 2)
                PutPixel(m_bgSurf, x, y, SDL_MapRGB(m_bgSurf->format, 255, 255, 0));
            else
                PutPixel(m_bgSurf, x, y, SDL_MapRGB(m_bgSurf->format, 0, 0, 0));
        }
    }
    SDL_UnlockSurface(m_bgSurf);

    return m_bgSurf;
}

/**
 * @brief This function converts surface coordinates into matrix coordinates
 *
 * @param x surface coordinate x
 * @param y surface coordinate y
 * @param i column number in the matrix.
 * @param j row number in the matrix.
 */
void MapDisplay::ConvertScreenCoordToMatrixCoord(int x, int y, int *i, int *j)
{
    double dx = 2 * (x / (double)(m_screenW-1) - 0.5);
    double dy = 2 * (y / (double)(m_screenH-1) - 0.5);

    *i = Interpreter::coord2mapX(dx);
    *j = Interpreter::coord2mapY(dy);
}


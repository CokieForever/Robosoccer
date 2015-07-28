#include "matrix.h"
#include "sdl_gfx/SDL_gfxPrimitives.h"
#include "interpreter.h"
#include "sdlutilities.h"
#include <queue>

/**
 * @brief Wrapper to create a @ref Matrix::Point
 *
 * @param x coordinate
 * @param y coordinate
 * @return Matrix::Point Point
 */
Matrix::Point Matrix::CreatePoint(int x, int y)
{
    Point pt = {x, y};
    return pt;
}

/**
 * @brief Basic constructor.
 *
 * @param height Matrix height. Default to Interpreter::MAP_HEIGHT.
 * @param width Matrix width. Default to Interpreter::MAP_WIDTH.
 */
Matrix::Matrix(int height, int width)
{
    if (height <= 0)
        height = Interpreter::MAP_HEIGHT;
    if (width <= 0)
        width = Interpreter::MAP_WIDTH;

    m_surface = SDL_CreateRGBSurface(SDL_SWSURFACE, width, height, 32, 0xFF000000, 0xFF0000, 0xFF00, 0xFF);
    SDL_FillRect(m_surface, NULL, 0);
    SDL_LockSurface(m_surface);

    pthread_mutex_init(&m_mutex, NULL);
}

/**
 * @brief Copy constructor
 *
 * @param matrix
 */
Matrix::Matrix(const Matrix& matrix)
{
    pthread_mutex_lock((pthread_mutex_t*)&(matrix.m_mutex));
    SDL_UnlockSurface(matrix.m_surface);
    m_surface = SDL_ConvertSurface(matrix.m_surface, matrix.m_surface->format, SDL_SWSURFACE);
    SDL_LockSurface(matrix.m_surface);
    pthread_mutex_unlock((pthread_mutex_t*)&(matrix.m_mutex));

    SDL_LockSurface(m_surface);
    pthread_mutex_init(&m_mutex, NULL);
}

/**
 * @brief Destructor
 *
 */
Matrix::~Matrix()
{
    SDL_UnlockSurface(m_surface);
    SDL_FreeSurface(m_surface);
    pthread_mutex_destroy(&m_mutex);
}

/**
 * @brief Copy operator overload
 *
 * @param matrix
 * @return Matrix & Matrix::operator
 */
Matrix& Matrix::operator=(const Matrix& matrix)
{
    if (&matrix == this)
        return *this;

    SDL_Surface* surf = m_surface;

    pthread_mutex_lock((pthread_mutex_t*)&m_mutex);
    pthread_mutex_lock((pthread_mutex_t*)&(matrix.m_mutex));
    SDL_UnlockSurface(matrix.m_surface);
    m_surface = SDL_ConvertSurface(matrix.m_surface, matrix.m_surface->format, SDL_SWSURFACE);
    SDL_LockSurface(matrix.m_surface);
    pthread_mutex_unlock((pthread_mutex_t*)&(matrix.m_mutex));
    pthread_mutex_unlock((pthread_mutex_t*)&m_mutex);

    if (surf)
    {
        SDL_UnlockSurface(surf);
        SDL_FreeSurface(surf);
    }

    return *this;
}

/**
 * @brief Fills the whole matrix with the given number.
 *
 * @param number The number to use to fill the matrix.
 */
void Matrix::Fill(Uint8 number)
{
    pthread_mutex_lock((pthread_mutex_t*)&m_mutex);
    SDL_UnlockSurface(m_surface);
    SDL_FillRect(m_surface, NULL, number << 24);
    SDL_LockSurface(m_surface);
    pthread_mutex_unlock((pthread_mutex_t*)&m_mutex);
}

/**
 * @brief This function draws a filled rectangle in the matrix.
 *
 * @param ul Upper left corner of the rectangle
 * @param lr Lower right corner of the rectangle
 * @param number The number to use to fill the rectangle.
 */
void Matrix::DrawRectangle(Point ul, Point lr, Uint8 number)
{
    SDL_Rect rect = {ul.x, ul.y, lr.x - ul.x + 1, lr.y - ul.y + 1};
    pthread_mutex_lock((pthread_mutex_t*)&m_mutex);
    SDL_UnlockSurface(m_surface);
    SDL_FillRect(m_surface, &rect, number << 24);
    SDL_LockSurface(m_surface);
    pthread_mutex_unlock((pthread_mutex_t*)&m_mutex);
}

/**
 * @brief This function draws a filled thick line (rotated rectangle) in the matrix.
 *
 * @param start Start point of the line.
 * @param end End point of the line.
 * @param thickness Thickness of the line.
 * @param number The number to use to fill the thick line.
 */
void Matrix::DrawThickLine(Point start, Point end, int thickness, Uint8 number)
{
    pthread_mutex_lock((pthread_mutex_t*)&m_mutex);
    thickLineColor(m_surface, start.x, start.y, end.x, end.y, thickness, number << 24 | 0xFF);
    pthread_mutex_unlock((pthread_mutex_t*)&m_mutex);
}

/**
 * @brief This function draws a filled circle in the matrix.
 *
 * @param center The centre of the circle.
 * @param radius The radius of the circle.
 * @param number The number to use to fill the circle.
 */
void Matrix::DrawCircle(Point center, int radius, Uint8 number)
{
    pthread_mutex_lock((pthread_mutex_t*)&m_mutex);
    filledCircleColor(m_surface, center.x, center.y, radius, number << 24 | 0xFF);
}

/**
 * @brief This function draws a filled polygon in the matrix.
 * @param points Vertices of the polygon.
 * @param n Number of vertices in the array "points".
 * @param number The number to use to fill the polygon.
 */
void Matrix::DrawPolygon(Point *points, int n, Uint8 number)
{
    Sint16 *vx = new Sint16[n];
    Sint16 *vy = new Sint16[n];

    for (int i=0 ; i < n ; i++)
    {
        vx[i] = points[i].x;
        vy[i] = points[i].y;
    }

    filledPolygonColor(m_surface, vx, vy, n, number << 24 | 0xFF);
    delete vx;
    delete vy;
}

//From http://will.thimbleby.net/scanline-flood-fill/
/**
 * @brief Performs a flood fill in the matrix from the given starting point. The "accessible" values are the ones equal to the one at the starting point.
 *
 * @param start The starting point.
 * @param number The number to use to fill the matrix.
 */
void Matrix::FloodFill(Point start, Uint8 number)
{
    Uint8 i = Get(start.x, start.y);

    // xMin, xMax, y, down[true] / up[false], extendLeft, extendRight
    queue<FloodFillStruct> ranges;
    FloodFillStruct ffs = {start.x, start.x, start.y, 0, true, true};
    ranges.push(ffs);

    (*this)[start.x][start.y] = number;

    while (ranges.size() > 0)
    {
        ffs = ranges.front();
        ranges.pop();

        bool down = ffs.direction < 0;
        bool up = ffs.direction > 0;

        // extendLeft
        int minX = ffs.xMin;
        int y = ffs.y;
        if (ffs.extendLeft)
        {
            while (minX>0 && Get(minX-1, y)==i)
            {
                minX--;
                (*this)[minX][y] = number;
            }
        }

        int maxX = ffs.xMax;
        // extendRight
        if(ffs.extendRight)
        {
            while (maxX<m_surface->w-1 && Get(maxX+1, y)==i)
            {
                maxX++;
                (*this)[maxX][y] = number;
            }
        }

        // extend range ignored from previous line
        (ffs.xMin)--;
        (ffs.xMax)++;

        if(y < m_surface->h-1)
            flood_AddNextLine(y+1, !up, true, minX, maxX, i, number, ffs, ranges);
        if(y > 0)
            flood_AddNextLine(y-1, !down, false, minX, maxX, i, number, ffs, ranges);
    }
}

/**
 * @brief
 *
 * @param newY
 * @param isNext
 * @param downwards
 * @param minX
 * @param maxX
 * @param i
 * @param number
 * @param ffs
 * @param ranges
 */
void Matrix::flood_AddNextLine(int newY, bool isNext, bool downwards, int minX, int maxX, int i, int number, FloodFillStruct ffs, std::queue<FloodFillStruct>& ranges)
{
    int rMinX = minX;
    bool inRange = false;
    int x;
    for (x=minX ; x<=maxX ; x++)
    {
        // skip testing, if testing previous line within previous range
        bool empty = (isNext || (x<ffs.xMin || x>ffs.xMax)) && Get(x, newY)==i;
        if (!inRange && empty)
        {
            rMinX = x;
            inRange = true;
        }
        else if (inRange && !empty)
        {
            FloodFillStruct ffs2 = {rMinX, x-1, newY, downwards ? -1 : 1, rMinX==minX, false};
            ranges.push(ffs2);
            inRange = false;
        }

        if (inRange)
            (*this)[x][newY] = number;

        // skip
        if(!isNext && x==ffs.xMin) {
            x = ffs.xMax;
        }
    }

    if (inRange)
    {
        FloodFillStruct ffs2 = {rMinX, x-1, newY, downwards ? -1 : 1, rMinX==minX, true};
        ranges.push(ffs2);
    }
}

/**
 * @brief Gets an element of the matrix.
 *
 * @param x x-coordinates of the element to retrieve
 * @param y y-coordinates of the element to retrieve
 * @return Uint8 & The element. If coordinates are out of range, a reference to an internal garbage value is returned.
 */
Uint8& Matrix::Get(int x, int y) const
{
    if (x >= m_surface->w || x < 0  || y >= m_surface->h || y < 0)
        return m_default;

    Uint8* p = ((Uint8*)m_surface->pixels + y * m_surface->pitch + x * 4);

    return *(p + 3);
}

Uint8 Matrix::m_default = 0;

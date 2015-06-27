#include "matrix.h"
#include "sdl_gfx/SDL_gfxPrimitives.h"
#include "interpreter.h"
#include "sdlutilities.h"

Matrix::Point Matrix::CreatePoint(double x, double y)
{
    Point pt = {x, y};
    return pt;
}

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

Matrix::~Matrix()
{
    SDL_UnlockSurface(m_surface);
    SDL_FreeSurface(m_surface);
    pthread_mutex_destroy(&m_mutex);
}

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

void Matrix::Fill(Uint8 number)
{
    pthread_mutex_lock((pthread_mutex_t*)&m_mutex);
    SDL_UnlockSurface(m_surface);
    SDL_FillRect(m_surface, NULL, number << 24);
    SDL_LockSurface(m_surface);
    pthread_mutex_unlock((pthread_mutex_t*)&m_mutex);
}

void Matrix::DrawRectangle(Point ul, Point lr, Uint8 number)
{
    SDL_Rect rect = {ul.x, ul.y, lr.x - ul.x + 1, lr.y - ul.y + 1};
    pthread_mutex_lock((pthread_mutex_t*)&m_mutex);
    SDL_UnlockSurface(m_surface);
    SDL_FillRect(m_surface, &rect, number << 24);
    SDL_LockSurface(m_surface);
    pthread_mutex_unlock((pthread_mutex_t*)&m_mutex);
}

void Matrix::DrawThickLine(Point start, Point end, int thickness, Uint8 number)
{
    pthread_mutex_lock((pthread_mutex_t*)&m_mutex);
    thickLineColor(m_surface, start.x, start.y, end.x, end.y, thickness, number << 24 | 0xFF);
    pthread_mutex_unlock((pthread_mutex_t*)&m_mutex);
}

void Matrix::DrawCircle(Point center, int radius, Uint8 number)
{
    pthread_mutex_lock((pthread_mutex_t*)&m_mutex);
    filledCircleColor(m_surface, center.x, center.y, radius, number << 24 | 0xFF);
}

Uint8& Matrix::Get(int x, int y) const
{
    if (x >= m_surface->w || x < 0  || y >= m_surface->h || y < 0)
        return m_default;

    Uint8* p = ((Uint8*)m_surface->pixels + y * m_surface->pitch + x * 4);

    return *(p + 3);
}

Uint8 Matrix::m_default = 0;

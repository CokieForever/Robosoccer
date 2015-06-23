#include "matrixdisplay.h"
#include "interpreter.h"

using namespace std;

MatrixDisplay::Matrix MatrixDisplay::ConvertToMatrix(const int *array, int rows, int cols)
{
    Matrix matrix = {array, rows, cols};
    return matrix;
}

MatrixDisplay::MatrixDisplay(Matrix matrix, int width, int height)
{
    m_matrix = matrix;
    m_isDisplaying = false;
    m_displayThread = NULL;
    m_keepGoing = true;
    m_screenW = width;
    m_screenH = height;
    m_bgSurf = NULL;
}

MatrixDisplay::~MatrixDisplay()
{
    if (m_bgSurf)
        SDL_FreeSurface(m_bgSurf);
}

SDL_Surface* MatrixDisplay::GetDisplay() const
{
    return m_bgSurf;
}

SDL_Surface* MatrixDisplay::UpdateDisplay()
{
    if (!m_bgSurf)
        m_bgSurf = SDL_CreateRGBSurface(SDL_SWSURFACE, m_screenW, m_screenH, 32,0,0,0,0);
    if (!m_bgSurf)
        return NULL;

    SDL_LockSurface(m_bgSurf);
    for (int x=0; x < m_screenW ; x++)
    {
        for (int y=0 ; y < m_screenH ; y++)
        {
            int i,j;
            ConvertScreenCoordToMatrixCoord(x, y, &i, &j);

            int n = m_matrix.data[i * m_matrix.cols + j];
            if (n == 0)
                PutPixel(m_bgSurf, x, y, SDL_MapRGB(m_bgSurf->format, 0, 255, 0));
            else if (n == 1)
                PutPixel(m_bgSurf, x, y, SDL_MapRGB(m_bgSurf->format, 255, 0, 0));
        }
    }
    SDL_UnlockSurface(m_bgSurf);

    return m_bgSurf;
}

void MatrixDisplay::ConvertScreenCoordToMatrixCoord(int x, int y, int *i, int *j)
{
    double dx = 2 * (x / (double)(m_screenW-1) - 0.5);
    double dy = 2 * (y / (double)(m_screenH-1) - 0.5);

    *i = Interpreter::coord2mapX(dx);
    *j = Interpreter::coord2mapY(dy);
}


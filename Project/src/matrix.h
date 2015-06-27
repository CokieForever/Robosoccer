#ifndef MATRIX_H
#define MATRIX_H

#include <SDL.h>

class Interpreter;

class Matrix
{
public:
    /*** [] operator overloading ***/
    class MatrixProxy
    {
    private:
        const Matrix& m_matrix;
        int m_i;
    public:
        MatrixProxy(const Matrix& m, int i) : m_matrix(m)
        {
        m_i = i;
        }

        Uint8& operator[](int j)
        {
        return m_matrix.Get(m_i, j);
        }
    };

    MatrixProxy operator[](int i) const
    {
        return MatrixProxy(*this, i);
    }
    /*** End of overloading section ***/

    struct Point
    {
        int x, y;
    };

    static Point CreatePoint(double x, double y);

    Matrix(int height = 0, int width = 0);
    ~Matrix();
    Matrix(const Matrix& matrix);
    Matrix& operator=(const Matrix& matrix);
    Uint8& Get(int i, int j) const;

    void Fill(Uint8 number);
    void DrawRectangle(Point ul, Point lr, Uint8 number);
    void DrawThickLine(Point start, Point end, int thickness, Uint8 number);
    void DrawCircle(Point center, int radius, Uint8 number);

private:
    static Uint8 m_default;
    SDL_Surface *m_surface;
    pthread_mutex_t m_mutex;

};

#endif // MATRIX_H

#ifndef MATRIX_H
#define MATRIX_H

#include <SDL.h>
#include <queue>

class Interpreter;

/**
 * @brief
 *
 */
class Matrix
{
public:
    /*** [] operator overloading ***/
    /**
     * @brief
     *
     */
    class MatrixProxy
    {
    private:
        const Matrix& m_matrix;     /**< TODO */
        int m_i;                    /**< TODO */
    public:
        /**
         * @brief
         *
         * @param m
         * @param i
         */
        MatrixProxy(const Matrix& m, int i) : m_matrix(m)
        {
            m_i = i;
        }

        /**
         * @brief
         *
         * @param j
         * @return Uint8 & operator
         */
        Uint8& operator[](int j)
        {
            return m_matrix.Get(m_i, j);
        }
    };

    /**
     * @brief
     *
     * @param i
     * @return MatrixProxy operator
     */
    MatrixProxy operator[](int i) const
    {
        return MatrixProxy(*this, i);
    }
    /*** End of overloading section ***/

    /**
     * @brief
     *
     */
    struct Point
    {
        int x;  /**< TODO */
        int y;  /**< TODO */
    };

    static Point CreatePoint(int x, int y);

    Matrix(int height = 0, int width = 0);
    ~Matrix();
    Matrix(const Matrix& matrix);
    Matrix& operator=(const Matrix& matrix);
    Uint8& Get(int i, int j) const;

    void Fill(Uint8 number);
    void DrawRectangle(Point ul, Point lr, Uint8 number);
    void DrawThickLine(Point start, Point end, int thickness, Uint8 number);
    void DrawCircle(Point center, int radius, Uint8 number);
    void DrawPolygon(Point *points, int n, Uint8 number);
    void FloodFill(Point start, Uint8 number);

private:
    static Uint8 m_default;     /**< TODO */

    /**
     * @brief
     *
     */
    struct FloodFillStruct
    {
        int xMin;           /**< TODO */
        int xMax;           /**< TODO */
        int y;              /**< TODO */
        int direction;      /**< TODO */
        bool extendLeft;    /**< TODO */
        bool extendRight;   /**< TODO */
    };

    SDL_Surface *m_surface;     /**< TODO */
    pthread_mutex_t m_mutex;    /**< TODO */

    void flood_AddNextLine(int newY, bool isNext, bool downwards, int minX, int maxX, int i, int number, FloodFillStruct ffs, std::queue<FloodFillStruct>& ranges);

};

#endif // MATRIX_H

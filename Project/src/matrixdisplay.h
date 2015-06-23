#ifndef MATRIXDISPLAY_H
#define MATRIXDISPLAY_H

#include <SDL.h>
#include <pthread.h>
#include <unistd.h>
#include <iostream>
#include "sdlutilities.h"
#include <math.h>

class MatrixDisplay
{
public:
    typedef struct
    {
        const int *data;
        int rows, cols;
    } Matrix;

    static Matrix ConvertToMatrix(const int *array, int rows, int cols);

    MatrixDisplay(Matrix matrix, int width, int height);
    ~MatrixDisplay();

    SDL_Surface* GetDisplay() const;
    SDL_Surface* UpdateDisplay();

private:
    Matrix m_matrix;
    bool m_isDisplaying;
    pthread_t m_displayThread;
    bool m_keepGoing;
    int m_screenW;
    int m_screenH;
    SDL_Surface *m_bgSurf;

    void ConvertScreenCoordToMatrixCoord(int x, int y, int *i, int *j);

};

#endif // MATRIXDISPLAY_H

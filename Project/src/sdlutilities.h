#ifndef SDLUTILITIES_H
#define SDLUTILITIES_H

#include <SDL.h>
#include <math.h>
#include <algorithm>

#define IsPointInRect(p,r) ((p).x >= (r).x && (p).x <= (r).x+(r).w && (p).y >= (r).y && (p).y <= (r).y+(r).h)

#if SDL_BYTEORDER == SDL_BIG_ENDIAN

#define RED_MASK   0xFF000000
#define GREEN_MASK 0x00FF0000
#define BLUE_MASK  0x0000FF00
#define ALPHA_MASK 0x000000FF

#else

#define RED_MASK   0x000000FF
#define GREEN_MASK 0x0000FF00
#define BLUE_MASK  0x00FF0000
#define ALPHA_MASK 0xFF000000

#endif

typedef enum
{
    BS_NONE, BS_HOVERED, BS_PUSHED, BS_CLICKED
} ButtonStatus;

typedef struct
{
    SDL_Rect area;
    SDL_Color colorHovered;
    SDL_Color colorPushed;

    bool pushed;
} Button;

typedef enum
{
    DDS_NONE, DDS_HOVERED, DDS_DRAGGED, DDS_DROPPED
} DragDropStatus;

typedef struct
{
    SDL_Rect area;
    SDL_Color colorHovered;
    SDL_Surface *ghostSurf;

    bool dragged;
} DragDrop;

void DrawLine(SDL_Surface *surf, float x1, float y1, float x2, float y2, SDL_Color color);
SDL_Color CreateColor(int r, int g, int b);
Uint32 GetPixel(SDL_Surface *surface, int x, int y);
bool PutPixelCheck(SDL_Surface *surf, int x, int y, Uint32 pixel);
void PutPixel(SDL_Surface *surface, int x, int y, Uint32 pixel);
SDL_Rect GetMousePos();

Button CreateButton(SDL_Rect area, SDL_Color colorHovered = CreateColor(255,128,0), SDL_Color colorPushed = CreateColor(255,0,0));
ButtonStatus ManageButton(Button *button, SDL_Event event);

DragDrop CreateDragDrop(SDL_Rect area, SDL_Surface *ghostSurf = NULL, SDL_Color colorHovered = CreateColor(255,128,0));
DragDropStatus ManageDragDrop(DragDrop *dd, SDL_Event event);

void ComputeVectorEnd(double startX, double startY, double angle, double length, double *endX, double *endY);
void ComputeVectorEnd(double startX, double startY, double cosAngle, double sinAngle, double length, double *endX, double *endY);
void ComputeLineAngle(double startX, double startY, double endX, double endY, double *angle);
void ComputeLineAngle(double startX, double startY, double endX, double endY, double *cosAngle, double *sinAngle);
void ComputeThickLineRect(double startX, double startY, double endX, double endY, double thickness, double (&rectX)[4], double (&rectY)[4]);

#endif // SDLUTILITIES_H

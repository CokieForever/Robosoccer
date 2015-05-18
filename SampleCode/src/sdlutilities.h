#ifndef SDLUTILITIES_H
#define SDLUTILITIES_H

#include <SDL.h>
#include <math.h>
#include <algorithm>

void DrawLine(SDL_Surface *surf, float x1, float y1, float x2, float y2, SDL_Color color);
SDL_Color CreateColor(int r, int g, int b);
Uint32 GetPixel(SDL_Surface *surface, int x, int y);
bool PutPixelCheck(SDL_Surface *surf, int x, int y, Uint32 pixel);
void PutPixel(SDL_Surface *surface, int x, int y, Uint32 pixel);

#endif // SDLUTILITIES_H

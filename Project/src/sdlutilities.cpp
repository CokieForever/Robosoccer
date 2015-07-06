#include "sdlutilities.h"
#include "pathfinder.h"


//From here: http://stackoverflow.com/questions/11737988/how-to-draw-a-line-using-sdl-without-using-external-libraries
/**
 * @brief
 *
 * @param surf
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 * @param color
 */
void DrawLine(SDL_Surface *surf, float x1, float y1, float x2, float y2, SDL_Color color)
{
    Uint32 pixel = SDL_MapRGB(surf->format, color.r, color.g, color.b);

    SDL_LockSurface(surf);

    //Bresenham's line algorithm
    const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
    if(steep)
    {
        std::swap(x1, y1);
        std::swap(x2, y2);
    }

    if(x1 > x2)
    {
        std::swap(x1, x2);
        std::swap(y1, y2);
    }

    const float dx = x2 - x1;
    const float dy = fabs(y2 - y1);

    float error = dx / 2.0f;
    const int ystep = (y1 < y2) ? 1 : -1;
    int y = (int)y1;

    const int maxX = (int)x2;

    for(int x=(int)x1; x<maxX; x++)
    {
        if(steep)
            PutPixelCheck(surf, y,x, pixel);
        else
            PutPixelCheck(surf, x,y, pixel);

        error -= dy;
        if(error < 0)
        {
            y += ystep;
            error += dx;
        }
    }

    SDL_UnlockSurface(surf);
}

/**
 * @brief
 *
 * @param r
 * @param g
 * @param b
 * @return SDL_Color
 */
SDL_Color CreateColor(int r, int g, int b)
{
    SDL_Color color = {r,g,b,0};
    return color;
}

/*
 * Return the pixel value at (x, y)
 * NOTE: The surface must be locked before calling this!
 */
/**
 * @brief
 *
 * @param surface
 * @param x
 * @param y
 * @return Uint32
 */
Uint32 GetPixel(SDL_Surface *surface, int x, int y)
{
    int bpp = surface->format->BytesPerPixel;
    /* Here p is the address to the pixel we want to retrieve */
    Uint8 *p = (Uint8 *)surface->pixels + y * surface->pitch + x * bpp;

    switch(bpp) {
    case 1:
        return *p;

    case 2:
        return *(Uint16 *)p;

    case 3:
        if(SDL_BYTEORDER == SDL_BIG_ENDIAN)
            return p[0] << 16 | p[1] << 8 | p[2];
        else
            return p[0] | p[1] << 8 | p[2] << 16;

    case 4:
        return *(Uint32 *)p;

    default:
        return 0;       /* shouldn't happen, but avoids warnings */
    }
}

/**
 * @brief
 *
 * @param surf
 * @param x
 * @param y
 * @param pixel
 * @return bool
 */
bool PutPixelCheck(SDL_Surface *surf, int x, int y, Uint32 pixel)
{
    if (x < 0 || y < 0 || x >= surf->w || y >= surf->h)
        return false;
    PutPixel(surf, x, y, pixel);
    return true;
}

/*
 * Set the pixel at (x, y) to the given value
 * NOTE: The surface must be locked before calling this!
 */
/**
 * @brief
 *
 * @param surface
 * @param x
 * @param y
 * @param pixel
 */
void PutPixel(SDL_Surface *surface, int x, int y, Uint32 pixel)
{
    int bpp = surface->format->BytesPerPixel;
    /* Here p is the address to the pixel we want to set */
    Uint8 *p = (Uint8 *)surface->pixels + y * surface->pitch + x * bpp;

    switch(bpp) {
    case 1:
        *p = pixel;
        break;

    case 2:
        *(Uint16 *)p = pixel;
        break;

    case 3:
        if(SDL_BYTEORDER == SDL_BIG_ENDIAN) {
            p[0] = (pixel >> 16) & 0xff;
            p[1] = (pixel >> 8) & 0xff;
            p[2] = pixel & 0xff;
        } else {
            p[0] = pixel & 0xff;
            p[1] = (pixel >> 8) & 0xff;
            p[2] = (pixel >> 16) & 0xff;
        }
        break;

    case 4:
        *(Uint32 *)p = pixel;
        break;
    }
}

/**
 * @brief
 *
 * @param surf
 * @param rect
 * @param color
 */
void DrawRect(SDL_Surface *surf, SDL_Rect rect, SDL_Color color)
{
    Uint32 pixel = SDL_MapRGB(surf->format, color.r, color.g, color.b);
    SDL_LockSurface(surf);

    for (int x=rect.x ; x <= rect.x+rect.w ; x++)
    {
        PutPixelCheck(surf, x, rect.y, pixel);
        PutPixelCheck(surf, x, rect.y+rect.h, pixel);
    }

    for (int y=rect.y ; y <= rect.y+rect.h ; y++)
    {
        PutPixelCheck(surf, rect.x, y, pixel);
        PutPixelCheck(surf, rect.x+rect.w, y, pixel);
    }

    SDL_UnlockSurface(surf);
}

/**
 * @brief
 *
 * @return SDL_Rect
 */
SDL_Rect GetMousePos()
{
    int x, y;
    SDL_GetMouseState(&x, &y);
    SDL_Rect rect = {x, y, 0, 0};
    return rect;
}

/**
 * @brief
 *
 * @param area
 * @param colorHovered
 * @param colorPushed
 * @return Button
 */
Button CreateButton(SDL_Rect area, SDL_Color colorHovered, SDL_Color colorPushed)
{
    Button button = {area, colorHovered, colorPushed, false};
    return button;
}

/**
 * @brief
 *
 * @param button
 * @param event
 * @return ButtonStatus
 */
ButtonStatus ManageButton(Button *button, SDL_Event event)
{
    ButtonStatus status = BS_NONE;

    switch (event.type)
    {
        case SDL_MOUSEBUTTONDOWN:
            if (IsPointInRect(event.button, button->area))
            {
                button->pushed = true;
                status = BS_PUSHED;
            }
            break;

        case SDL_MOUSEBUTTONUP:
            if (button->pushed && IsPointInRect(event.button, button->area))
                status = BS_CLICKED;
            button->pushed = false;
            break;

        default:
            break;
    }

    SDL_Rect mousePos = GetMousePos();

    if (IsPointInRect(mousePos, button->area))
    {
        DrawRect(SDL_GetVideoSurface(), button->area, button->pushed ? button->colorPushed : button->colorHovered);
        if (button->pushed)
            status = BS_PUSHED;
        else if (status == BS_NONE)
            status = BS_HOVERED;
    }
    else
        button->pushed = false;

    return status;
}

/**
 * @brief
 *
 * @param area
 * @param ghostSurf
 * @param colorHovered
 * @return DragDrop
 */
DragDrop CreateDragDrop(SDL_Rect area, SDL_Surface *ghostSurf, SDL_Color colorHovered)
{
    DragDrop dd = {area, colorHovered, ghostSurf, false};
    return dd;
}

/**
 * @brief
 *
 * @param dd
 * @param event
 * @return DragDropStatus
 */
DragDropStatus ManageDragDrop(DragDrop *dd, SDL_Event event)
{
    DragDropStatus status = DDS_NONE;

    switch (event.type)
    {
        case SDL_MOUSEBUTTONDOWN:
            if (IsPointInRect(event.button, dd->area))
            {
                dd->dragged = true;
                status = DDS_DRAGGED;
            }
            break;

        case SDL_MOUSEBUTTONUP:
            if (dd->dragged)
                status = DDS_DROPPED;
            dd->dragged = false;
            break;

        default:
            break;
    }

    SDL_Rect mousePos = GetMousePos();

    if (dd->dragged)
    {
        if (dd->ghostSurf)
        {
            SDL_Rect pos = {mousePos.x - dd->ghostSurf->w/2, mousePos.y - dd->ghostSurf->h/2, 0, 0};
            SDL_BlitSurface(dd->ghostSurf, NULL, SDL_GetVideoSurface(), &pos);
        }
        status = DDS_DRAGGED;
    }
    else if (IsPointInRect(mousePos, dd->area))
    {
        DrawRect(SDL_GetVideoSurface(), dd->area, dd->colorHovered);
        if (status == DDS_NONE)
            status = DDS_HOVERED;
    }

    return status;
}


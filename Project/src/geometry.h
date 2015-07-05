#ifndef GEOMETRY_H
#define GEOMETRY_H

void ComputeVectorEnd(double startX, double startY, double angle, double length, double *endX, double *endY);
void ComputeVectorEnd(double startX, double startY, double cosAngle, double sinAngle, double length, double *endX, double *endY);
void ComputeLineAngle(double startX, double startY, double endX, double endY, double *angle);
void ComputeLineAngle(double startX, double startY, double endX, double endY, double *cosAngle, double *sinAngle);
void ComputeThickLineRect(double startX, double startY, double endX, double endY, double thickness, double (&rectX)[4], double (&rectY)[4]);
bool DoSegmentsIntersect(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4);
bool ComputeLinesIntersection(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4, double *isectX, double *isectY);
bool GetLineRectIntersections(double xMin, double yMin, double xMax, double yMax, double a, double b, double (&isectX)[2], double (&isectY)[2]);
int GetSegmentRectIntersections(double ulX, double ulY, double lrX, double lrY, double startX, double startY, double endX, double endY, double (&isectX)[2], double (&isectY)[2]);

#endif // GEOMETRY_H

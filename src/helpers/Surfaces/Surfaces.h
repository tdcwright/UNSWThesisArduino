#ifndef SURFACES_H
#define SURFACES_H

// have a look at https://wokwi.com/projects/344915694795620948

#include "../MatrixOperations/Matrix.h"
#include "../MovePoints.h"

#include "../../dualSerial/dualSerial.h"
extern DualSerial dualSerial;

#define NUMBER_OF_LEAST_SQUARES_POINTS 9

int fitTypeToNumVars(surfaceFitType fitType);

// Surface coeffients in order (defined by surfaceFitType)
struct SurfaceCoefficients
{
    float a;
    float b;
    float c;
    float d;
    float e;
    float g;
    SurfaceCoefficients() { a, b, c, d, e, g = 0; }
    SurfaceCoefficients(float aVal, float bVal, float cVal, float dVal, float eVal, float gVal)
    {
        a = aVal;
        b = bVal;
        c = cVal;
        d = dVal;
        e = eVal;
        g = gVal;
    }
    SurfaceCoefficients(surfaceFitType fitType, MatrixObj RowEchelon)
    {
        SurfaceCoefficients();
        int numRows = fitTypeToNumVars(fitType);
        float *vars[] = {&a,
                         &b,
                         &c,
                         &d,
                         &e,
                         &g};

        for (int i = 0; i < numRows; i++)
        {
            *(vars[i]) = RowEchelon.get(i, numRows);
        }
    }
};

class SurfaceData
{
private:
    int nextRowNumber;
    MatrixObj AT;  // For curve fitting. A'
    MatrixObj ATA; // For curve fitting. A'A
    MatrixObj ATB; // For curve fitting. A'B

    void incrementRow();

public:
    MatrixObj A;          // For curve fitting. Ax=B
    MatrixObj B;          // For curve fitting. Ax=B
    MatrixObj RowEchelon; // For curve fitting. A'B
    bool operationCompleted;
    bool validRowEchelonOperation;
    bool validSurface; // indicates whether the surface is of parabolid like shape (has a minimum)
    int numberOfPoints;
    SurfaceCoefficients coefficients;

    surfaceFitType fitType;

    SurfaceData(surfaceFitType sfitType);

    // Adds readings to A and B matracies. Implements z^zPower to increase values.
    void addReadings(float x, float y, float z, int zPower);

    // Adds readings to A and B matracies.
    void addReadings(float x, float y, float z);

    // Sets boolean values to 'not completed' so that the process can restart
    void reset();

    XYPoint getLocalMinima();

    XYPoint directionOfSteepestDecent(XYPoint point);

    // Provides the differential value at a point
    XYPoint differentiateAtAPoint(XYPoint point);

    void performLeastSquaresOperation();
};

#endif
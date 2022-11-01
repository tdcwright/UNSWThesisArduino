#ifndef SURFACES_H
#define SURFACES_H

// have a look at https://wokwi.com/projects/344915694795620948

#include "../MatrixOperations/Matrix.h"
#include "../BasicLinearAlgebra/BasicLinearAlgebra.h"

#define USING_BLA_LIBRARY true

#define POSITION_SCALE_ADJUSTMENT 100.0

#include "../MovePoints.h"

#include "../../dualSerial/dualSerial.h"
extern DualSerial dualSerial;

// #define NUMBER_OF_LEAST_SQUARES_POINTS 9

int fitTypeToNumVars(surfaceFitType fitType);

template <int rows, int cols>
String BLAMatrixAsString(BLA::Matrix<rows, cols> M, String title = "");

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
    SurfaceCoefficients(surfaceFitType fitType, MatrixObj solvedMatrix)
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
            *(vars[i]) = solvedMatrix.get(i, numRows);
        }
    }
    template <int rows, int cols>
    SurfaceCoefficients(surfaceFitType fitType, BLA::Matrix<rows, cols> solvedMatrix)
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
            *(vars[i]) = solvedMatrix(i);
        }
    }
};

template <int numVars, int numReadings>
class SurfaceData
{
private:
    int nextRowNumber;
#if USING_BLA_LIBRARY
    BLA::Matrix<numVars, numReadings> AT; // For curve fitting. A'
    BLA::Matrix<numVars, numVars> ATA;    // For curve fitting. A'A
    BLA::Matrix<numVars> ATB;             // For curve fitting. A'B
#else
    MatrixObj AT;           // For curve fitting. A'
    MatrixObj ATA;          // For curve fitting. A'A
    MatrixObj ATB;          // For curve fitting. A'B
#endif

    void incrementRow();
    bool checkValidSurface();

public:
#if USING_BLA_LIBRARY
    BLA::Matrix<numReadings, numVars> A; // For curve fitting. Ax=B
    BLA::Matrix<numReadings> B;          // For curve fitting. Ax=B
    BLA::Matrix<numVars> solvedMatrix;   // For curve fitting. A'B
#else
    MatrixObj A;            // For curve fitting. Ax=B
    MatrixObj B;            // For curve fitting. Ax=B
    MatrixObj solvedMatrix; // For curve fitting. A'B
#endif
    bool operationCompleted;
    bool validReductionOperation;
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
    void printSurfaceCoefficients();
};

#endif
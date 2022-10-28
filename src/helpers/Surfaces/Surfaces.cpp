#include "Surfaces.h"

int fitTypeToNumVars(surfaceFitType fitType)
{
    switch (fitType)
    {
    case surfaceFitType::poly12:
    case surfaceFitType::poly21:
        return POLY21_NUM_VARS;
    case surfaceFitType::poly11:
        return POLY11_NUM_VARS;
    case surfaceFitType::poly22:
    default:
        return POLY22_NUM_VARS;
    }
}

SurfaceData::SurfaceData(surfaceFitType sfitType) : A(NUMBER_OF_LEAST_SQUARES_POINTS, fitTypeToNumVars(sfitType)),
                                                    B(NUMBER_OF_LEAST_SQUARES_POINTS, 1),
                                                    AT(fitTypeToNumVars(sfitType), NUMBER_OF_LEAST_SQUARES_POINTS),
                                                    ATA(fitTypeToNumVars(sfitType), fitTypeToNumVars(sfitType)),
                                                    ATB(fitTypeToNumVars(sfitType), 1),
                                                    RowEchelon(fitTypeToNumVars(sfitType), fitTypeToNumVars(sfitType) + 1),
                                                    coefficients()
{
    reset();

    fitType = sfitType;
}

void SurfaceData::incrementRow()
{
    nextRowNumber++;
    if (nextRowNumber >= NUMBER_OF_LEAST_SQUARES_POINTS)
        nextRowNumber = 0;

    if (numberOfPoints < NUMBER_OF_LEAST_SQUARES_POINTS)
        numberOfPoints++;
}

void SurfaceData::addReadings(float x, float y, float z, int zPower)
{
    dualSerial.println("adding readings to surface");

    float normalisedZ = z;

    for (int i = 0; i < zPower - 1; i++)
        normalisedZ *= z;
    MMath.addReadingsToABMatrix(fitType, A, B, nextRowNumber, x, y, normalisedZ);
    incrementRow();
}

void SurfaceData::addReadings(float x, float y, float z)
{
    addReadings(x, y, z, 1);
}

void SurfaceData::reset()
{
    operationCompleted = false;
    validRowEchelonOperation = false;
    validSurface = false;
    nextRowNumber = 0;
    numberOfPoints = 0;
}

XYPoint SurfaceData::getLocalMinima()
{
    if (fitType == surfaceFitType::poly22)
    {
        double yMin = (((2.0 * coefficients.a * coefficients.e) / coefficients.c) - coefficients.d) / (coefficients.c - 4 * coefficients.a * coefficients.b / coefficients.c);
        double xMin = -(2 * coefficients.b * yMin + coefficients.e) / coefficients.c;

        return XYPoint(xMin, yMin);
    }
    else
    {
        dualSerial.println("*ERROR* Local minima not avalible for this surface type");
        return XYPoint(0, 0);
    }
}

XYPoint SurfaceData::directionOfSteepestDecent(XYPoint point)
{
    XYPoint diff = differentiateAtAPoint(point);
    return XYPoint(diff.x * -1, diff.y * -1);
}

XYPoint SurfaceData::differentiateAtAPoint(XYPoint point)
{
    double x = point.x;
    double y = point.y;
    double dx = 0;
    double dy = 0;
    switch (fitType)
    {
    case surfaceFitType::poly22: // z = ax^2+by^2+cxy+dx+ey+g
        dx = 2 * coefficients.a * x + coefficients.c * y + coefficients.d;
        dy = 2 * coefficients.b * y + coefficients.c * x + coefficients.e;
        break;
    case surfaceFitType::poly21: // z = ax^2+bxy+cx+dy+e
        dx = 2 * coefficients.a * x + coefficients.b * y + coefficients.c;
        dy = coefficients.b * x + coefficients.d;
        break;
    case surfaceFitType::poly12: // z = ay^2+bxy+cx+dy+e
        dx = coefficients.b * y + coefficients.c;
        dy = 2 * coefficients.a * y + coefficients.b * x + coefficients.d;
        break;
    case surfaceFitType::poly11: // z = ax+by+c
        dx = coefficients.a;
        dy = coefficients.b;
        break;
    }
    return XYPoint(dx, dy);
}

void SurfaceData::performLeastSquaresOperation()
{
    MMath.Transpose(A, AT);
    MMath.Multiply(AT, A, ATA);
    MMath.Multiply(AT, B, ATB);
    MMath.Copy(ATA, RowEchelon);
    MMath.setCol(RowEchelon, fitTypeToNumVars(fitType), ATB);
    validRowEchelonOperation = MMath.reducedRowEchelon(RowEchelon);
    coefficients = SurfaceCoefficients(fitType, RowEchelon);
    operationCompleted = true;

    switch (fitType)
    {
    case surfaceFitType::poly22:
        validSurface = coefficients.a * coefficients.b > 0;
        break;
    case surfaceFitType::poly21:
    case surfaceFitType::poly12:
        validSurface = coefficients.a > 0;
        break;
    default:
        validSurface = true;
        break;
    }
}
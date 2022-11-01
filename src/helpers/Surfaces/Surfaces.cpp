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
                                                    solvedMatrix(fitTypeToNumVars(sfitType), fitTypeToNumVars(sfitType) + 1),
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
    float normalisedZ = z;

    for (int i = 0; i < zPower - 1; i++)
        normalisedZ *= z;

    float xAdjusted = x / POSITION_SCALE_ADJUSTMENT;
    float yAdjusted = y / POSITION_SCALE_ADJUSTMENT;
#if USING_BLA_LIBRARY
    if (fitType == surfaceFitType::poly22)
    {
        A(nextRowNumber, 0) = sq(xAdjusted);
        A(nextRowNumber, 1) = sq(yAdjusted);
        A(nextRowNumber, 2) = xAdjusted * yAdjusted;
        A(nextRowNumber, 3) = xAdjusted;
        A(nextRowNumber, 4) = yAdjusted;
        A(nextRowNumber, 5) = 1;
    }
    else if (fitType == surfaceFitType::poly11)
    {
        A(nextRowNumber, 3) = xAdjusted;
        A(nextRowNumber, 4) = yAdjusted;
        A(nextRowNumber, 5) = 1;
    }
    else if (fitType == surfaceFitType::poly21 || fitType == surfaceFitType::poly12)
    {
        A(nextRowNumber, 0) = (fitType == surfaceFitType::poly21 ? sq(xAdjusted) : sq(yAdjusted));
        A(nextRowNumber, 2) = xAdjusted * yAdjusted;
        A(nextRowNumber, 3) = xAdjusted;
        A(nextRowNumber, 4) = yAdjusted;
        A(nextRowNumber, 5) = 1;
    }

    B(nextRowNumber) = normalisedZ;
#else
    MMath.addReadingsToABMatrix(fitType, A, B, nextRowNumber, xAdjusted, yAdjusted, normalisedZ);
#endif

    incrementRow();
}

void SurfaceData::addReadings(float x, float y, float z)
{
    addReadings(x, y, z, 1);
}

void SurfaceData::reset()
{
    operationCompleted = false;
    validReductionOperation = false;
    validSurface = false;
    nextRowNumber = 0;
    numberOfPoints = 0;
}

XYPoint SurfaceData::getLocalMinima()
{
    if (fitType == surfaceFitType::poly22)
    {
        // double yMin = (((2.0 * coefficients.a * coefficients.e) / coefficients.c) - coefficients.d) / (coefficients.c - 4 * coefficients.a * coefficients.b / coefficients.c);
        // double xMin = -(2 * coefficients.b * yMin + coefficients.e) / coefficients.c;
        double yMin = (coefficients.c * coefficients.d - 2 * coefficients.a * coefficients.e - coefficients.c * coefficients.e) / (2 * coefficients.b * (2 * coefficients.a + coefficients.c));
        double xMin = -(coefficients.d / (2 * coefficients.a + coefficients.c));

        return XYPoint(xMin * POSITION_SCALE_ADJUSTMENT, yMin * POSITION_SCALE_ADJUSTMENT);
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

#if USING_BLA_LIBRARY
    AT = ~A;
    ATA = AT * A;
    ATB = AT * B;
    auto ATA_decomp = ATA; // LUDecompose will destroy A here so we'll pass in a copy so we can refer back to A later
    auto decomp = LUDecompose(ATA_decomp);
    solvedMatrix = LUSolve(decomp, ATB);
    validReductionOperation = true;
#else
    MMath.Transpose(A, AT);
    MMath.Multiply(AT, A, ATA);
    MMath.Multiply(AT, B, ATB);
    MMath.Copy(ATA, solvedMatrix);
    MMath.setCol(solvedMatrix, fitTypeToNumVars(fitType), ATB);
    validReductionOperation = MMath.reducedRowEchelon(solvedMatrix);
#endif

    coefficients = SurfaceCoefficients(fitType, solvedMatrix);
    operationCompleted = true;

    switch (fitType)
    {
    case surfaceFitType::poly22:
        validSurface = coefficients.a > 0 && coefficients.b > 0;
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

void SurfaceData::printSurfaceCoefficients()
{
    if (operationCompleted)
    {
        dualSerial.println("Surface Info:");
        dualSerial.print("\tValid Row Echelon Operation: ");
        dualSerial.println(validReductionOperation ? "TRUE" : "FALSE");
        dualSerial.print("\tValid Surface: ");
        dualSerial.println(validSurface ? "TRUE" : "FALSE");
        if (validSurface)
        {
            XYPoint minima = getLocalMinima();
            dualSerial.println("\tMinima: ");
            dualSerial.print("\t\tX Min: ");
            dualSerial.println(minima.x);
            dualSerial.print("\t\tY Min: ");
            dualSerial.println(minima.y);
        }
        dualSerial.println("\tEquation: ");
        dualSerial.print("\t\tz = ");
        switch (fitType)
        {
        case surfaceFitType::poly12:
        case surfaceFitType::poly21:
            dualSerial.print(coefficients.a);
            if (fitType == surfaceFitType::poly12)
                dualSerial.print("*y^2 + ");
            else
                dualSerial.print("*x^2 + ");
            dualSerial.print(coefficients.b);
            dualSerial.print("*x*y + ");
            dualSerial.print(coefficients.c);
            dualSerial.print("*x + ");
            dualSerial.print(coefficients.d);
            dualSerial.print("*y + ");
            dualSerial.print(coefficients.e);
            dualSerial.println();
            break;
        case surfaceFitType::poly11:
            dualSerial.print(coefficients.a);
            dualSerial.print("*x + ");
            dualSerial.print(coefficients.b);
            dualSerial.print("*y + ");
            dualSerial.print(coefficients.c);
            dualSerial.println();
            break;
        case surfaceFitType::poly22:
            dualSerial.print(coefficients.a);
            dualSerial.print("*x^2 + ");
            dualSerial.print(coefficients.b);
            dualSerial.print("*y^2 + ");
            dualSerial.print(coefficients.c);
            dualSerial.print("*x*y + ");
            dualSerial.print(coefficients.d);
            dualSerial.print("*x + ");
            dualSerial.print(coefficients.e);
            dualSerial.print("*y + ");
            dualSerial.print(coefficients.g);
            dualSerial.println();
            break;
        default:
            dualSerial.print('\t\ta: ');
            dualSerial.println(coefficients.a);
            dualSerial.print('\t\tb: ');
            dualSerial.println(coefficients.b);
            dualSerial.print('\t\tc: ');
            dualSerial.println(coefficients.c);
            dualSerial.print('\t\td: ');
            dualSerial.println(coefficients.d);
            dualSerial.print('\t\te: ');
            dualSerial.println(coefficients.e);
            dualSerial.print('\t\tg: ');
            dualSerial.println(coefficients.g);
        }
    }
    else
    {
        dualSerial.println("Surface not yet created");
    }
}
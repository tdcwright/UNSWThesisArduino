/*
 *  MatrixMath.cpp Library for Matrix Math
 *
 *  Created by Charlie Matlack on 12/18/10.
 *  Modified from code by RobH45345 on Arduino Forums, algorithm from
 *  NUMERICAL RECIPES: The Art of Scientific Computing.
 */

#include "Matrix.h"

#define NR_END 1

MatrixMath MMath; // MatrixMath object. Pre-instantiate

// Matrix Printing Routine
// Uses tabs to separate numbers under assumption printed mtx_type width won't cause problems
void MatrixMath::Print(MatrixObj &A, String label)
{
    int m = A.nRows;
    int n = A.nCols;
    // A = input matrix (m x n)
    int i, j;
    Serial.println();
    Serial.println(label);
    for (i = 0; i < m; i++)
    {
        for (j = 0; j < n; j++)
        {
            Serial.print(A.get(i, j));
            Serial.print("\t");
        }
        Serial.println();
    }
}

void MatrixMath::Copy(MatrixObj &A, MatrixObj &B)
{
    int m = A.nRows;
    int n = A.nCols;
    int i, j;
    for (i = 0; i < m; i++)
        for (j = 0; j < n; j++)
        {
            B.set(i, j, A.get(i, j));
        }
}

// Matrix Multiplication Routine
//  C = A*B
void MatrixMath::Multiply(MatrixObj &A, MatrixObj &B, MatrixObj &C)
{
    // A = input matrix (m x p)
    // B = input matrix (p x n)
    // m = number of rows in A
    // p = number of columns in A = number of rows in B
    // n = number of columns in B
    // C = output matrix = A*B (m x n)

    if (A.nCols != B.nRows)
    {
        dualSerial.println("Matrix dimensions don't match");
        return;
    }

    int m = A.nRows;
    int p = A.nCols;
    int n = B.nCols;

    int i, j, k;
    mtx_type newVal;
    for (i = 0; i < m; i++)
        for (j = 0; j < n; j++)
        {
            C.set(i, j, 0);
            for (k = 0; k < p; k++)
                newVal = C.get(i, j) + A.get(i, k) * B.get(k, j);
            C.set(i, j, newVal);
        }
}

// Matrix Addition Routine
void MatrixMath::Add(MatrixObj &A, MatrixObj &B, MatrixObj &C)
{
    // A = input matrix (m x n)
    // B = input matrix (m x n)
    // m = number of rows in A = number of rows in B
    // n = number of columns in A = number of columns in B
    // C = output matrix = A+B (m x n)
    int m = B.nRows;
    int n = B.nCols;

    int i, j;
    for (i = 0; i < m; i++)
        for (j = 0; j < n; j++)
            C.set(i, j, (A.get(i, j) + B.get(i, j)));
}

// Matrix Subtraction Routine
void MatrixMath::Subtract(MatrixObj &A, MatrixObj &B, MatrixObj &C)
{
    // A = input matrix (m x n)
    // B = input matrix (m x n)
    // m = number of rows in A = number of rows in B
    // n = number of columns in A = number of columns in B
    // C = output matrix = A-B (m x n)

    int m = B.nRows;
    int n = B.nCols;
    int i, j;
    for (i = 0; i < m; i++)
        for (j = 0; j < n; j++)
            C.set(i, j, (A.get(i, j) - B.get(i, j)));
}

// Matrix Transpose Routine
void MatrixMath::Transpose(MatrixObj &A, MatrixObj &C)
{
    // A = input matrix (m x n)
    // m = number of rows in A
    // n = number of columns in A
    // C = output matrix = the transpose of A (n x m)
    int m = A.nRows;
    int n = A.nCols;

    int i, j;
    for (i = 0; i < m; i++)
        for (j = 0; j < n; j++)
            C.set(j, i, A.get(i, j));
}

void MatrixMath::setRow(MatrixObj &A, int rowNumber, MatrixObj &newRow)
{
    int m = A.nRows;
    int n = A.nCols;

    if (rowNumber >= A.nRows)
    {
        dualSerial.println("Row number too high");
        return;
    }

    if (newRow.nCols != A.nCols)
    {
        dualSerial.println("Unmatched number of cols");
        return;
    }

    for (int i = 0; i < n; i++)
    {
        A.set(rowNumber, i, newRow.get(0, i));
    }
}

void MatrixMath::setCol(MatrixObj &A, int colNumber, MatrixObj &newCol)
{
    if (colNumber >= A.nCols)
    {
        dualSerial.println("Col number too high");
        return;
    }

    if (newCol.nRows != A.nRows)
    {
        dualSerial.println("Unmatched number of rows");
        return;
    }

    for (int i = 0; i < A.nRows; i++)
    {
        A.set(i, colNumber, newCol.get(i, 0));
    }
}

bool MatrixMath::reducedRowEchelon(MatrixObj &A)
{
    int lead = 0;

    while (lead < A.nRows)
    {
        float d, m;

        for (int r = 0; r < A.nRows; r++)
        { // for each row ...
            /* calculate divisor and multiplier */
            d = A.get(lead, lead);
            m = A.get(r, lead) / d;

            for (int c = 0; c < A.nCols; c++)
            { // for each column ...
                if (r == lead)
                    A.set(r, c, A.get(r, c) / d); // make pivot = 1
                else
                    A.set(r, c, A.get(r, c) - A.get(lead, c) * m); // make other = 0

                if (isnan(A.get(r, c)))
                    return false;
            }
        }

        lead++;
    }

    return true;
}

void MatrixMath::addReadingsToABMatrix(surfaceFitType fitType, MatrixObj &A, MatrixObj &B, int rowNumber, float x, float y, float z)
{
    MatrixObj newRow(1, A.nCols);
    if (fitType == surfaceFitType::poly22)
    {
        if (A.nCols != 6)
        {
            dualSerial.println("Surface fit matrix is not the right size");
            return;
        }

        mtx_type a = sq(x);
        mtx_type b = sq(y);
        mtx_type c = x * y;
        mtx_type d = x;
        mtx_type e = y;
        mtx_type g = 1;
        newRow.set(0, 0, a);
        newRow.set(0, 1, b);
        newRow.set(0, 2, c);
        newRow.set(0, 3, d);
        newRow.set(0, 4, e);
        newRow.set(0, 5, g);
    }
    else if (fitType == surfaceFitType::poly11)
    {
        if (A.nCols != 3)
        {
            dualSerial.println("Surface fit matrix is not the right size");
            return;
        }
        mtx_type a = x;
        mtx_type b = y;
        mtx_type c = 1;
        newRow.set(0, 0, a);
        newRow.set(0, 1, b);
        newRow.set(0, 2, c);
    }
    else if (fitType == surfaceFitType::poly21 || fitType == surfaceFitType::poly12)
    {
        if (A.nCols != 5)
        {
            dualSerial.println("Surface fit matrix is not the right size");
            return;
        }
        mtx_type a = (fitType == surfaceFitType::poly21 ? sq(x) : sq(y));
        mtx_type b = x * y;
        mtx_type c = x;
        mtx_type d = y;
        mtx_type e = 1;
        newRow.set(0, 0, a);
        newRow.set(0, 1, b);
        newRow.set(0, 2, c);
        newRow.set(0, 3, d);
        newRow.set(0, 4, e);
    }

    setRow(A, rowNumber, newRow);
    B.set(rowNumber, 0, z);
}
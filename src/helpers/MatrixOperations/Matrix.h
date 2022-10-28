/*
 *  MatrixMath.h Library for Matrix Math
 *
 *  Created by Charlie Matlack on 12/18/10.
 *  Modified from code by RobH45345 on Arduino Forums, algorithm from
 *  NUMERICAL RECIPES: The Art of Scientific Computing.
 *  Modified to work with Arduino 1.0/1.5 by randomvibe & robtillaart
 *  Made into a real library on GitHub by Vasilis Georgitzikis (tzikis)
 *  so that it's easy to use and install (March 2015)
 */

#ifndef MatrixMath_h
#define MatrixMath_h

#include "../../dualSerial/dualSerial.h"

extern DualSerial dualSerial;

typedef double mtx_type;

#define POLY22_NUM_VARS 6
#define POLY22_MIN_NUM_POINTS 6
#define POLY21_NUM_VARS 5
#define POLY21_MIN_NUM_POINTS 5
#define POLY11_NUM_VARS 3
#define POLY11_MIN_NUM_POINTS 3
// Selects which surface equation to use
enum class surfaceFitType
{
    poly22, // In the form: f(x,y)=ax^2+by^2+cxy+dx+ey+g
    poly21, // In the form: f(x,y)=ax^2+bxy+cx+dy+e
    poly12, // In the form: f(x,y)=ay^2+bxy+cx+dy+e
    poly11  // In the form: f(x,y)=ax+by+c
};

// Object containing mtx_type[rows][cols]
class MatrixObj
{
private:
    mtx_type **matrix;

public:
    MatrixObj(int rows, int cols) : nRows(rows),
                                    nCols(cols)
    {
        matrix = new mtx_type *[rows];
        for (int i = 0; i < rows; ++i)
            matrix[i] = new mtx_type[cols];
    };
    ~MatrixObj()
    {
        for (int i = 0; i < nRows; ++i)
            delete matrix[i];
        delete matrix;
    }

    mtx_type get(int row, int col)
    {
        return (matrix[row])[col];
    }
    void set(int row, int col, mtx_type value)
    {
        (matrix[row])[col] = value;
    }

    int nRows;
    int nCols;
};

class MatrixMath
{

public:
    // MatrixMath();
    void Print(MatrixObj &A, String label);
    void Copy(MatrixObj &A, MatrixObj &B);
    void Multiply(MatrixObj &A, MatrixObj &B, MatrixObj &C);
    void Add(MatrixObj &A, MatrixObj &B, MatrixObj &C);
    void Subtract(MatrixObj &A, MatrixObj &B, MatrixObj &C);
    void Transpose(MatrixObj &A, MatrixObj &C);

    // ADITIONAL
    void setRow(MatrixObj &A, int rowNumber, MatrixObj &newRow);
    void setCol(MatrixObj &A, int colNumber, MatrixObj &newCol);

    // With reference to: https://math.stackexchange.com/questions/2010758/how-do-i-fit-a-paraboloid-surface-to-nine-points-and-find-the-minimum
    void addReadingsToABMatrix(surfaceFitType fitType, MatrixObj &A, MatrixObj &B, int rowNumber, float x, float y, float z);

    // Returns successful value
    bool reducedRowEchelon(MatrixObj &A);
};

extern MatrixMath MMath;
#endif
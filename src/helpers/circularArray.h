#ifndef CIRCULAR_ARRAY_H
#define CIRCULAR_ARRAY_H

#include <Arduino.h>

template <typename T>
struct minMaxVals
{
    T maxValue;
    T minValue;

    minMaxVals() : maxValue(0), minValue(0){};
    minMaxVals(T max, T min) : maxValue(max), minValue(min){};
};

template <typename T>
class CircularArray
{
private:
    T *buffer;
    uint16_t maxSize;
    uint16_t currPastValueIndex;
    uint16_t numPastValuesStored;

public:
    CircularArray(uint16_t sizeOfArray)
    {
        buffer = new T[sizeOfArray];
        maxSize = sizeOfArray;
        currPastValueIndex = 0;
        numPastValuesStored = 0;
    }
    ~CircularArray()
    {
        delete buffer;
    }

    // gets FIFO value at index
    T getFIFO(uint16_t index)
    {
        if (index + 1 > numPastValuesStored)
            return 0.0;

        short trueIndex = currPastValueIndex - (numPastValuesStored - index);

        if (trueIndex < 0)
            trueIndex = numPastValuesStored + trueIndex;

        return buffer[trueIndex];
    }

    // gets FILO value at index
    T getFILO(uint16_t index)
    {
        if (index + 1 > numPastValuesStored)
            return 0.0;

        short trueIndex = currPastValueIndex - (index + 1);

        if (trueIndex < 0)
            trueIndex = numPastValuesStored + trueIndex;

        return buffer[trueIndex];
    }

    void addValue(T value)
    {
        buffer[currPastValueIndex] = value;
        currPastValueIndex++;

        if (currPastValueIndex == maxSize)
            currPastValueIndex = 0;

        if (numPastValuesStored < maxSize)
            numPastValuesStored++;
    }

    void reset()
    {
        currPastValueIndex = 0;
        numPastValuesStored = 0;
    }

    minMaxVals<T> getMaxMin()
    {
        minMaxVals<T> result(getFIFO(0), getFIFO(0));

        for (uint16_t i = 0; i < numPastValuesStored; i++)
        {
            T currValue = getFIFO(i);
            if (currValue > result.maxValue)
                result.maxValue = currValue;
            else if (currValue < result.minValue)
                result.minValue = currValue;
        }
        return result;
    }

    uint16_t numElements() { return numPastValuesStored; }
};

#endif
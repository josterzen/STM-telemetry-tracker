#ifndef MODEL_HPP
#define MODEL_HPP
#include <stdint.h>

class ModelListener;

class Model
{
public:
    Model();

    void bind(ModelListener* listener)
    {
        modelListener = listener;
    }

    void tick();
protected:
    ModelListener* modelListener;
    short gyroYval;
    short engineRPMval;
    short accYval;
    short accXval;
    short engineTempval;
};

#endif // MODEL_HPP

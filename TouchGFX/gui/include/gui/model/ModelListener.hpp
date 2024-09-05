#ifndef MODELLISTENER_HPP
#define MODELLISTENER_HPP

#include <gui/model/Model.hpp>

class ModelListener
{
public:
    ModelListener() : model(0) {}
    
    virtual ~ModelListener() {}

    void bind(Model* m)
    {
        model = m;
    }
    virtual void setGyro_Y(short value);
    virtual void setEngine_RPM(short value);
    virtual void setAcc_Y(short value);
    virtual void setAcc_X(short value);
    virtual void setEngine_Temp(short value);
protected:
    Model* model;
};

#endif // MODELLISTENER_HPP

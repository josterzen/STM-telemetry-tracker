#include <gui/screen1_screen/Screen1View.hpp>
#include <gui/screen1_screen/Screen1Presenter.hpp>

Screen1Presenter::Screen1Presenter(Screen1View& v)
    : view(v)

{

}

void Screen1Presenter::activate()
{

}

void Screen1Presenter::deactivate()
{

}

void Screen1Presenter::setGyro_Y(short val)
{
	view.setGyro_Y(val);
}

void Screen1Presenter::setEngine_RPM(short val)
{
	view.setEngine_RPM(val);
}

void Screen1Presenter::setAcc_Y(short val)
{
	view.setAcc_Y(val);
}

void Screen1Presenter::setAcc_X(short val)
{
	view.setAcc_X(val);
}

void Screen1Presenter::setEngine_Temp(short val)
{
	view.setEngine_Temp(val);
}

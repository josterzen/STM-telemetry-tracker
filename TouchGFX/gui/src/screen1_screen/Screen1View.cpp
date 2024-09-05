#include <gui/screen1_screen/Screen1View.hpp>

Screen1View::Screen1View()
{

}

void Screen1View::setupScreen()
{
    Screen1ViewBase::setupScreen();
}

void Screen1View::tearDownScreen()
{
    Screen1ViewBase::tearDownScreen();
}


void Screen1View::setGyro_Y(short val)
 {
	if (val > 0) {
		gyplus_bar.setValue(val);
		gyplus_bar.invalidate();
		gyminus_bar.setValue(0);
		gyminus_bar.invalidate();
	} else if (val < 0) {
		gyminus_bar.setValue(-val);
		gyminus_bar.invalidate();
		gyplus_bar.setValue(0);
		gyplus_bar.invalidate();
	} else {
		gyplus_bar.setValue(1);
		gyminus_bar.setValue(1);
		gyplus_bar.invalidate();
		gyminus_bar.invalidate();
	}
}

void Screen1View::setEngine_RPM(short val)
{
	enginerpm_bar.setValue(val);
}

void Screen1View::setAcc_Y(short val) {
	if (val > 0) {
		accelyplus_bar.setValue(val);
		accelyplus_bar.invalidate();
		accelyminus_bar.setValue(0);
		accelyminus_bar.invalidate();
	} else if (val < 0) {
		accelyminus_bar.setValue(-val);
		accelyminus_bar.invalidate();
		accelyplus_bar.setValue(0);
		accelyplus_bar.invalidate();
	} else {
		accelyplus_bar.setValue(1);
		accelyminus_bar.setValue(1);
		accelyplus_bar.invalidate();
		accelyminus_bar.invalidate();
	}
}

void Screen1View::setAcc_X(short val)
{
	if (val > 0) {
		accelxplus_bar.setValue(val);
		accelxplus_bar.invalidate();
		accelxminus_bar.setValue(0);
		accelxminus_bar.invalidate();
	} else if (val < 0) {
		accelxminus_bar.setValue(-val);
		accelxminus_bar.invalidate();
		accelxplus_bar.setValue(0);
		accelxplus_bar.invalidate();
	} else {
		accelxplus_bar.setValue(1);
		accelxminus_bar.setValue(1);
		accelxplus_bar.invalidate();
		accelxminus_bar.invalidate();
	}
}

void Screen1View::setEngine_Temp(short val)
{
	enginetemp_bar.setValue(val);
}

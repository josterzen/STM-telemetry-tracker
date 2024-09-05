#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>

#include <cmsis_os2.h>
#include "main.h"

extern "C" {
extern osMessageQueueId_t gyroYqueueHandle;
extern osMessageQueueId_t engineRPMqueueHandle;
extern osMessageQueueId_t accYqueueHandle;
extern osMessageQueueId_t accXqueueHandle;
extern osMessageQueueId_t engineTempQueueHandle;
}
Model::Model() :
		modelListener(0), gyroYval(0), engineRPMval(0), accYval(0), accXval(0), engineTempval(
				0) {

}

void Model::tick() {

	if (osMessageQueueGet(gyroYqueueHandle, &gyroYval, 0U, 0) == osOK) {
		modelListener->setGyro_Y(gyroYval);
	}
	if (osMessageQueueGet(engineRPMqueueHandle, &engineRPMval, 0U, 0) == osOK) {
		modelListener->setEngine_RPM(engineRPMval);
	}
	if (osMessageQueueGet(accYqueueHandle, &accYval, 0U, 0) == osOK) {
		modelListener->setAcc_Y(accYval);
	}
	if (osMessageQueueGet(accXqueueHandle, &accXval, 0U, 0) == osOK) {
		modelListener->setAcc_X(accXval);
	}
	if (osMessageQueueGet(engineTempQueueHandle, &engineTempval, 0U, 0)
			== osOK) {
		modelListener->setEngine_Temp(engineTempval);
	}

}

#ifndef MY_OBJECT_SIMULATOR_H
#define MY_OBJECT_SIMULATOR_H

#include <GLModel/GLModel.h>
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"

#include "BaseSimulator.h"
#include "ObjectSystem.h"
#include "Hermite.h"

#define SCENE_TIME 10.0

class ObjectSimulator : public BaseSimulator
{
public:
	ObjectSimulator(const std::string& name, ObjectSystem* system);
	int step(double time);
	int init(double time);

protected:
	ObjectSystem* m_system;
};


#endif

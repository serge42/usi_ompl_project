#include "omplapp/apps/AppBase.h"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ControlSpace.h>

namespace ompl
{
    class TestCtrlSpace : public control::ControlSpace
    {
        public:
            double *getValueAddressAtIndex(control::Control *control, unsigned int index) const override
            {
                if(index > 2)
                    return nullptr;
                return values[index];
            }

        protected:
            double *values[3];
    };
}
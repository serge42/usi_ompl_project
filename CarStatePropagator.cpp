#include "omplapp/apps/AppBase.h"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <valarray>
#include <limits>

namespace oc = ompl::control;
namespace ob = ompl::base;

class CarStatePropagator : public oc::StatePropagator
{
    public:
        CarStatePropagator(const oc::SpaceInformationPtr &si) : oc::StatePropagator(si)
        {
            space_ = si->getStateSpace();
            carLength_ = 0.2;
            timeStep_ = 0.01;
        }

        void propagate(const ob::State *state, const oc::Control *control, double duration, ob::State *result) const override
        {
            EulerIntegration(state, control, duration, result);
        }

        // bool steer(const ob::State *from, const ob::State *to, oc::Control *result, double &duration) const override
        // {
        //     // TODO
        //     return false;
        // }

        // bool canSteer() const override { return true; }

        bool canPropagateBackward() const override { return false; }

    protected:
        void EulerIntegration(const ob::State *start, const oc::Control *control, const double duration, ob::State *result) const
        {
            double t = timeStep_;
            std::valarray<double> dstate;
            space_->copyState(result, start);
            while(t < duration + std::numeric_limits<double>::epsilon())
            {
                ode(result, control, dstate);
                update(result, timeStep_ * dstate);
                t += timeStep_;
            }
            if (t + std::numeric_limits<double>::epsilon() > duration)
            {
                ode(result, control, dstate);
                update(result, (t - duration) * dstate);
            }
        }

        void ode(const ob::State *state, const oc::Control *control, std::valarray<double> &dstate) const
        {
            const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
            const double theta = state->as<ob::SE2StateSpace::StateType>()->getYaw();

            dstate.resize(3);
            dstate[0] = u[0] * cos(theta);
            dstate[1] = u[0] * sin(theta);
            dstate[2] = u[0] * tan(u[1]) / carLength_;
        }

        void update(ob::State *state, const std::valarray<double> &dstate) const
        {
            ob::SE2StateSpace::StateType &s = *state->as<ob::SE2StateSpace::StateType>();
            s.setX(s.getX() + dstate[0]);
            s.setY(s.getY() + dstate[1]);
            s.setYaw(s.getYaw() + dstate[2]);
            space_->enforceBounds(state);
        }

        ob::StateSpacePtr space_;
        double carLength_;
        double timeStep_;
    //     oc::SpaceInformation *si_;
};

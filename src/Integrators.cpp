#include "Integrators.h"
#include "Node.h"
#include "cinder\Vector.h"

using namespace ci;

void integrateByEulerMethod(Bird *boid, State &state, float dTime, const Derivative &dFunc)
{
	State st = boid->mState;
	state.mCOM = st.mCOM + dFunc.V*dTime;
	state.mR = st.mR + dFunc.dOmegaR*dTime;
	state.mP = st.mP + dFunc.force*dTime;
	state.mL = st.mL + dFunc.torque*dTime;
	state.mI = state.mR * st.mInitI * state.mR.transposed();
	state.mInvI = state.mR * st.mInitInvI * state.mR.transposed();
}

Derivative evaluate(Bird *boid, const State &oldState, float dTime, const Derivative &dFunc)
{
    State state = oldState;
	state.mCOM = oldState.mCOM + dFunc.V*dTime;
	state.mR = oldState.mR + dFunc.dOmegaR*dTime;
	state.mP = oldState.mP + dFunc.force*dTime;
	state.mL = oldState.mL + dFunc.torque*dTime;	
	state.mI = state.mR * oldState.mInitI * state.mR.transposed();
	state.mInvI = state.mR * oldState.mInitInvI * state.mR.transposed();
	return boid->accelerate(state);
}

void integrateByRungeKutta4(Bird *boid, State &state, float dTime, const Derivative &dFunc)
{
	State st = boid->mState;
	Derivative a = evaluate(boid, st, dTime*0.0f, dFunc);
	Derivative b = evaluate(boid, st, dTime*0.5f, a);
	Derivative c = evaluate(boid, st, dTime*0.5f, b);
	Derivative d = evaluate(boid, st, dTime, c);

	const Vec3f dV				=	(1.0f/6.0f) * (a.V + 2.0f*(b.V + c.V) + d.V);
	const Matrix33f dDOmegaR	=	(a.dOmegaR + (b.dOmegaR + c.dOmegaR)*2.0f + d.dOmegaR)*(1.0f/6.0f);
	const Vec3f dForce			=	(1.0f/6.0f) * (a.force + 2.0f*(b.force + c.force) + d.force);
	const Vec3f dTorque			=	(1.0f/6.0f) * (a.torque + 2.0f*(b.torque + c.torque) + d.torque);
	
	state.mCOM = st.mCOM + dV*dTime;
	state.mR = st.mR + dDOmegaR*dTime;
	state.mP = st.mP + dForce*dTime;
	state.mL = st.mL + dTorque*dTime;
	state.mI = state.mR * st.mInitI * state.mR.transposed();
	state.mInvI = state.mR * st.mInitInvI * state.mR.transposed();
}
//==============================================================================
/*
    CPSC 599.86 / 601.86 - Computer Haptics
    Winter 2018, University of Calgary

    This class extends the cAlgorithmFingerProxy class in CHAI3D that
    implements the god-object/finger-proxy haptic rendering algorithm.
    It allows us to modify or recompute the force that is ultimately sent
    to the haptic device.

    Your job for this assignment is to implement the updateForce() method
    in this class to support for two new effects: force shading and haptic
    textures. Methods for both are described in Ho et al. 1999.
*/
//==============================================================================

#ifndef MYPROXYALGORITHM_H
#define MYPROXYALGORITHM_H

#include "chai3d.h"

//------------------------------------------------------------------------------

class MyProxyAlgorithm : public chai3d::cAlgorithmFingerProxy
{
public:
	chai3d::cVector3d m_debugVector;
	chai3d::cColorb m_colorAtCollision;
	chai3d::cColorb m_roughnessAtCollision;
	chai3d::cColorb m_heightAtCollision;
	chai3d::cColorb m_normalColorAtCollision;


	chai3d::cVector3d normalMapNorm;
	chai3d::cVector3d surfaceNorm;
	chai3d::cVector3d perturbedNorm;
	chai3d::cVector3d deltaHVector;
	double penDepthDebug, dHx, dHy, dHz;

	MyProxyAlgorithm();
	void setFrictionOn(bool iWantItOn);

protected:


	chai3d::cVector3d previousPerturbedNormal;
	bool frictionOn;


    //! This method computes the resulting force which will be sent to the haptic device.
    virtual void updateForce();

    //! This method attempts to move the proxy, subject to friction constraints.
    virtual void testFrictionAndMoveProxy(const chai3d::cVector3d& a_goal,
                                          const chai3d::cVector3d& a_proxy,
                                          chai3d::cVector3d& a_normal,
                                          chai3d::cGenericObject* a_parent);
};

//------------------------------------------------------------------------------
#endif

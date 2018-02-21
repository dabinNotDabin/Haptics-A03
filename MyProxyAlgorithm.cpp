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

#include "MyProxyAlgorithm.h"
#include "MyMaterial.h"

using namespace chai3d;

//==============================================================================
/*!
    This method uses the information computed earlier in
    computeNextBestProxyPosition() to calculate the force to be rendered.
    It first calls cAlgorithmFingerProxy::updateForce() to compute the base
    force from contact geometry and the constrained proxy position. That
    force can then be modified or recomputed in this function.

    Your implementation of haptic texture mapping will likely end up in this
    function. When this function is called, collision detection has already
    been performed, and the proxy point has already been updated based on the
    constraints found. Your job is to compute a force with all that information
    available to you.

    Useful variables to read:
        m_deviceGlobalPos   - current position of haptic device
        m_proxyGlobalPos    - computed position of the constrained proxy
        m_numCollisionEvents- the number of surfaces constraining the proxy
        m_collisionRecorderConstraint0,1,2
                            - up to three cCollisionRecorder structures with
                              cCollisionEvents that contain very useful
                              information about each contact

    Variables that this function should set/reset:
        m_normalForce       - computed force applied in the normal direction
        m_tangentialForce   - computed force along the tangent of the surface
        m_lastGlobalForce   - this is what the operator ultimately feels!!!
*/
//==============================================================================

void MyProxyAlgorithm::updateForce()
{
    // get the base class to do basic force computation first
    cAlgorithmFingerProxy::updateForce();

    // TODO: compute force shading and texture forces here

    if (m_numCollisionEvents > 0)
    {
        // this is how you access collision information from the first constraint
        cCollisionEvent* c0 = &m_collisionRecorderConstraint0.m_nearestCollision;

        if (MyMaterialPtr material = std::dynamic_pointer_cast<MyMaterial>(c0->m_object->m_material))
        {
			material->setUseHapticShading(true);

			std::cout << "Here\n";

			// you can access your custom material properties here
			material->m_myMaterialProperty;

			c0->m_globalNormal;
			c0->m_triangles;

			c0->m_object->getBoundaryMax();
			c0->m_object->getBoundaryMin();
			c0->m_object->setShowBoundaryBox(true, false);

			int triangleIndex0 = c0->m_index;
			int vertexIndex0 = c0->m_triangles->getVertexIndex0(triangleIndex0);

			cColorf color = c0->m_object->m_texture->m_color;
			
			cImagePtr image = c0->m_object->m_texture->m_image;
			
			double pixelX, pixelY;
			cColorb pixelColor;
			cVector3d texCoord;
			texCoord = c0->m_triangles->getTexCoordAtPosition(c0->m_index, c0->m_localPos);
			image->getPixelLocationInterpolated(texCoord, pixelX, pixelY, true);
			image->getPixelColorInterpolated(pixelX, pixelY, pixelColor);


			// For bumps
			//	Say black is height 0 and white is height 1.
			//	Take the degree of whiteness normalize(R, G, B), then take (R + G + B) / 3.0
			//	Perturbation factor = min(whiteness, 1.0 - whiteness)
			//	Perturb normal towards tangent at contact point by a degree of the computed factor
			//	Normalizing the color values may not work.
			//		Might have to find max values through testing to use for scaling the colors.
			//		Where objects are initialized and textures applied, set to white to test max values



//			m_debugColor.

			m_debugVector = c0->m_triangles->m_vertices->getTexCoord(vertexIndex0);
			m_debugVector = c0->m_triangles->m_vertices->getTangent(vertexIndex0);
        }
    }
}


//==============================================================================
/*!
    This method attempts to move the proxy, subject to friction constraints.
    This is called from computeNextBestProxyPosition() when the proxy is
    ready to move along a known surface.

    Your implementation of friction mapping will likely need to modify or
    replace the CHAI3D implementation in cAlgorithmFingerProxy. You may
    either copy the implementation from the base class and modify it to take
    into account a friction map, or use your own friction rendering from your
    previous assignment.

    The most important thing to do in this method is to write the desired
    proxy position into the m_nextBestProxyGlobalPos member variable.

    The input parameters to this function are as follows, all provided in the
    world (global) coordinate frame:

    \param  a_goal    The location to which we'd like to move the proxy.
    \param  a_proxy   The current position of the proxy.
    \param  a_normal  The surface normal at the obstructing surface.
    \param  a_parent  The surface along which we're moving.
*/
//==============================================================================
void MyProxyAlgorithm::testFrictionAndMoveProxy(const cVector3d& a_goal,
                                                const cVector3d& a_proxy,
                                                cVector3d &a_normal,
                                                cGenericObject* a_parent)
{
    cAlgorithmFingerProxy::testFrictionAndMoveProxy(a_goal, a_proxy, a_normal, a_parent);
}

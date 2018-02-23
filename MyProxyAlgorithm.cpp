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
			// you can access your custom material properties here
			cColorf color = c0->m_object->m_texture->m_color;

			double pixelX, pixelY;
			cColorb pixelColor;
			cVector3d texCoord;
			cVector3d perturbedNormal;

			std::string textureFilename;
			cImagePtr image = c0->m_object->m_texture->m_image;

			if (image == NULL || c0 == NULL)
			{
				std::cout << "Null Ptr Update Forces.\n";
				return;
			}

			texCoord = c0->m_triangles->getTexCoordAtPosition(c0->m_index, c0->m_localPos);

			if (material->objectID == 3)
			{
				image = c0->m_object->m_texture->m_image;
				image->getPixelLocationInterpolated(texCoord, pixelX, pixelY, true);
				image->getPixelColorInterpolated(pixelX, pixelY, pixelColor);

				m_colorAtCollision = pixelColor;

				double g, b;
				g = (double)pixelColor.getG();
				b = (double)pixelColor.getB();

				double height = (g + b) / (255.0*2.0);


				double distance = texCoord.x();

				// Texture wrapping in effect, need to get value between 0 and 1.
				// If greater than 1, decrease by 1 until between 0 and 1.  
				while (distance > 1.0)
					distance -= 1.0;

				// If less than 1, increase until between -1 and 0. Then take 1.0 + distance. (if texCoord is -0.25, this is extracting 0.75 from the texture)
				while (distance < -1.0)
					distance += 1.0;

				if (distance < 0.0)
					distance = 1.0 + distance;

				double yVariant = sin(19.5*M_PI*distance);
//				std::cout << "Sin Tex coord Clamped: " << yVariant << std::endl;

				double magnitudeOfForce = m_lastGlobalForce.length();
				m_lastGlobalForce += cVector3d(0.0, yVariant*2.0, 0.0);
				m_lastGlobalForce.normalize();
				m_lastGlobalForce = m_lastGlobalForce * (magnitudeOfForce + height*3.0);
			}
			else if (material->objectID != 5)
			{
				std::cout << "Here Update Forces OthersA\n";
				material->normalMap->m_image->getPixelLocationInterpolated(texCoord, pixelX, pixelY, true);
				material->normalMap->m_image->getPixelColorInterpolated(pixelX, pixelY, pixelColor);

				m_normalColorAtCollision = pixelColor;

				perturbedNormal = cVector3d(pixelColor.getR() / 255.0, pixelColor.getG() / 255.0, pixelColor.getB() / 255.0);

				double height = 0.0;
			
				std::cout << "Here Update Forces OthersB\n";

				material->heightMap->m_image->getPixelLocationInterpolated(texCoord, pixelX, pixelY, true);
				material->heightMap->m_image->getPixelColorInterpolated(pixelX, pixelY, pixelColor);

				std::cout << "Here Update Forces OthersC\n";

				m_heightAtCollision = pixelColor;

				double r, g, b;
				r = pixelColor.getR();
				g = pixelColor.getG();
				b = pixelColor.getB();

				height = (r + g + b) / 255.0*3.0;

				perturbedNormal.normalize();
				m_lastGlobalForce = perturbedNormal * m_lastGlobalForce.length() + perturbedNormal * height;
			}
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
	cCollisionEvent* c0 = &m_collisionRecorderConstraint0.m_nearestCollision;

	if (c0)
	{
		MyMaterialPtr material = std::dynamic_pointer_cast<MyMaterial>(c0->m_object->m_material);
		std::string textureFilename;
		cImagePtr image = c0->m_object->m_texture->m_image;

		if (image == NULL || material == NULL)
		{
			std::cout << "Null Ptr Friction.\n";
			return;
		}

		textureFilename = image->getFilename();

		double pixelX, pixelY;
		cColorb pixelColor;
		cVector3d texCoord;
		double r, g, b;

		texCoord = c0->m_triangles->getTexCoordAtPosition(c0->m_index, c0->m_localPos);
		if (material->objectID == 5)
		{
			double distance = texCoord.y();

			// Texture wrapping in effect, need to get value between 0 and 1.
			// If greater than 1, decrease by 1 until between 0 and 1.  
			while (distance > 1.0)
				distance -= 1.0;

			// If less than 1, increase until between -1 and 0. Then take 1.0 + distance. (if texCoord is -0.25, this is extracting 0.75 from the texture)
			while (distance < -1.0)
				distance += 1.0;

			if (distance < 0.0)
				distance = 1.0 + distance;

			double frictionVariant = sin(9.75*M_PI*distance + 0.5);
//			std::cout << "Sin Tex coord Clamped Friction: " << frictionVariant << std::endl;

			double staticFric;
			double dynamicFric;

			frictionVariant = ((frictionVariant > 0.0) ? frictionVariant : 0.0);

			double frictionMultiplier = pow((1.0 + frictionVariant), 3);
			std::cout << "Friction Multiplier: " << frictionMultiplier << std::endl;

			staticFric = material->baseStaticFriction * frictionMultiplier;
			dynamicFric = material->baseDynamicFriction * frictionMultiplier;
			
			a_parent->setFriction(staticFric, dynamicFric, true);
		}
		else if (material->objectID != 3)
		{
			std::cout << "Here Friction OthersA.\n";

			material->roughnessMap->m_image->getPixelLocationInterpolated(texCoord, pixelX, pixelY, true);
			material->roughnessMap->m_image->getPixelColorInterpolated(pixelX, pixelY, pixelColor);

			r = pixelColor.getR();
			g = pixelColor.getG();
			b = pixelColor.getB();

			//				material->maxStaticFriction

			double maxStaticFriction = 2.0;
			double maxDynamicFriction = 1.5;
			double roughness = (r + g + b) / 3.0;

			a_parent->setFriction(maxStaticFriction * roughness, maxDynamicFriction * roughness, true);
		}
	}


	cAlgorithmFingerProxy::testFrictionAndMoveProxy(a_goal, a_proxy, a_normal, a_parent);
}

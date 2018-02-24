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

				double yVariant = sin(0.7 + 19.5*M_PI*distance);
				double negator = sin(0.7 +1.5*M_PI + 19.5*M_PI*distance);

//				std::cout << "Sin Tex coord Clamped: " << yVariant << std::endl;

				dHx = yVariant;
				dHy = negator;

				double magnitudeOfForce = m_lastGlobalForce.length();
				double blendDistance = 0.15;
				double blendAmount = 1.0;

				if (yVariant > 0.0)
				{
					if (yVariant < blendAmount)
						blendAmount = yVariant / blendDistance;

					yVariant = 1.0 - yVariant;

					yVariant *= blendAmount;

					if (negator < 0.0)
						yVariant = -yVariant;

					magnitudeOfForce += height * 2.0;
				}
				else
					yVariant = 0.0;

				m_lastGlobalForce += cVector3d(0.0, yVariant*1.5, 0.0);
				m_lastGlobalForce.normalize();
				m_lastGlobalForce = m_lastGlobalForce * magnitudeOfForce;


			}
			else if (material->objectID != 5)
			{
				cVector3d deltaH, deltaHx, deltaHy, deltaHz, meshSurfaceNormal;
				double epsilon, penetrationDepth, height;
				cVector3d 
					texCoord_XplusE_YZ, texCoord_XminusE_YZ, 
					texCoordX_YPlusE_Z, texCoordX_YminusE_Z,
					texCoordXY_ZPlusE, texCoordXY_ZminusE;

				cColorb colorXplusE, colorXminusE, colorYplusE, colorYminusE, colorZplusE, colorZminusE;

				double hXplusE, hXminusE, hYplusE, hYminusE, hZplusE, hZminusE;

				epsilon = 0.01;

				// Calculate pixel locations for texture coordinates shifted by epsilon along coorinate axes
				// in both directions for each of the three axes.
				texCoord_XplusE_YZ = cVector3d(texCoord.x() + epsilon, texCoord.y(), texCoord.z());
				texCoord_XminusE_YZ = cVector3d(texCoord.x() - epsilon, texCoord.y(), texCoord.z());
				texCoordX_YPlusE_Z = cVector3d(texCoord.x(), texCoord.y() + epsilon, texCoord.z());
				texCoordX_YminusE_Z = cVector3d(texCoord.x(), texCoord.y() - epsilon, texCoord.z());
				texCoordXY_ZPlusE = cVector3d(texCoord.x(), texCoord.y(), texCoord.z() + epsilon);
				texCoordXY_ZminusE = cVector3d(texCoord.x(), texCoord.y(), texCoord.z() - epsilon);

				// Get the color values at those locations.
				material->heightMap->m_image->getPixelLocationInterpolated(texCoord_XplusE_YZ, pixelX, pixelY, true);
				material->heightMap->m_image->getPixelColor(pixelX, pixelY, colorXplusE);

				material->heightMap->m_image->getPixelLocationInterpolated(texCoord_XminusE_YZ, pixelX, pixelY, true);
				material->heightMap->m_image->getPixelColor(pixelX, pixelY, colorXminusE);

				material->heightMap->m_image->getPixelLocationInterpolated(texCoordX_YPlusE_Z, pixelX, pixelY, true);
				material->heightMap->m_image->getPixelColor(pixelX, pixelY, colorYplusE);

				material->heightMap->m_image->getPixelLocationInterpolated(texCoordX_YminusE_Z, pixelX, pixelY, true);
				material->heightMap->m_image->getPixelColor(pixelX, pixelY, colorYminusE);

				material->heightMap->m_image->getPixelLocationInterpolated(texCoordXY_ZPlusE, pixelX, pixelY, true);
				material->heightMap->m_image->getPixelColor(pixelX, pixelY, colorZplusE);

				material->heightMap->m_image->getPixelLocationInterpolated(texCoordXY_ZminusE, pixelX, pixelY, true);
				material->heightMap->m_image->getPixelColor(pixelX, pixelY, colorZminusE);


				//material->heightMap->m_image->getPixelLocation(texCoord_XplusE_YZ, pX, pY, true);
				//material->heightMap->m_image->getPixelColor(pX, pY, colorXplusE);

				//material->heightMap->m_image->getPixelLocation(texCoord_XminusE_YZ, pX, pY, true);
				//material->heightMap->m_image->getPixelColor(pX, pY, colorXminusE);

				//material->heightMap->m_image->getPixelLocation(texCoordX_YPlusE_Z, pX, pY, true);
				//material->heightMap->m_image->getPixelColor(pX, pY, colorYplusE);

				//material->heightMap->m_image->getPixelLocation(texCoordX_YminusE_Z, pX, pY, true);
				//material->heightMap->m_image->getPixelColor(pX, pY, colorYminusE);

				//material->heightMap->m_image->getPixelLocation(texCoordXY_ZPlusE, pX, pY, true);
				//material->heightMap->m_image->getPixelColor(pX, pY, colorZplusE);

				//material->heightMap->m_image->getPixelLocation(texCoordXY_ZminusE, pX, pY, true);
				//material->heightMap->m_image->getPixelColor(pX, pY, colorZminusE);



				// Calculate the gradient (deltaH) using those color values.
				hXplusE = (colorXplusE.getLuminance() - 20) / 255.0;
				hXminusE = (colorXminusE.getLuminance() - 20) / 255.0;

				hYplusE = (colorYplusE.getLuminance() - 20) / 255.0;
				hYminusE = (colorYminusE.getLuminance() - 20) / 255.0;

				hZplusE = (colorZplusE.getLuminance() - 20) / 255.0;
				hZminusE = (colorZminusE.getLuminance() - 20) / 255.0;

				hXplusE = ((hXplusE > 0.0) ? hXplusE : 0.0);
				hXminusE = ((hXminusE > 0.0) ? hXminusE : 0.0);
				hYplusE = ((hYplusE > 0.0) ? hYplusE : 0.0);
				hYminusE = ((hYminusE > 0.0) ? hYminusE : 0.0);
				hZplusE = ((hZplusE > 0.0) ? hZplusE : 0.0);
				hZminusE = ((hZminusE > 0.0) ? hZminusE : 0.0);

				m_heightAtCollision = colorXplusE;

				deltaHx = ((hXplusE - hXminusE) / (2.0*epsilon)) * cVector3d(0.0, -1.0, 0.0);
				deltaHy = ((hYplusE - hYminusE) / (2.0*epsilon)) * cVector3d(1.0, 0.0, 0.0);
				deltaHz = ((hZplusE - hZminusE) / (2.0*epsilon)) * cVector3d(0.0, 0.0, 1.0);

				//deltaHx = ((((hXplusE - hXminusE) > (hXminusE - hXplusE)) ? (hXplusE - hXminusE) : (hXminusE - hXplusE)) / (2.0*epsilon)) * cVector3d(1.0, 0.0, 0.0);
				//deltaHy = ((((hYplusE - hYminusE) > (hYminusE - hYplusE)) ? (hYplusE - hYminusE) : (hYminusE - hYplusE)) / (2.0*epsilon)) * cVector3d(0.0, 1.0, 0.0);
				//deltaHz = ((((hZplusE - hZminusE) > (hZminusE - hZplusE)) ? (hZplusE - hZminusE) : (hZminusE - hZplusE)) / (2.0*epsilon)) * cVector3d(0.0, 0.0, 1.0);

				//dHx = deltaHx.y();
				//dHy = deltaHy.x();
				//dHz = deltaHz.z();

				//dHx = colorXplusE.getLuminance() - 20;
				//dHy = colorXminusE.getLuminance() - 20;
				//dHz = deltaHz.z();


				deltaH = deltaHx + deltaHy + deltaHz;
				if (deltaH.length() > 1.0)
					deltaH.normalize();
				deltaHVector = deltaH;


				material->normalMap->m_image->getPixelLocationInterpolated(texCoord, pixelX, pixelY, true);
				material->normalMap->m_image->getPixelColorInterpolated(pixelX, pixelY, pixelColor);
//				m_normalColorAtCollision = pixelColor;
//				meshSurfaceNormal = cVector3d(pixelColor.getB(), pixelColor.getR(), pixelColor.getG());
//				meshSurfaceNormal.normalize();

				meshSurfaceNormal = computeShadedSurfaceNormal(c0);
				meshSurfaceNormal.normalize();
				surfaceNormal = meshSurfaceNormal;

//				deltaH = cVector3d(deltaH.x(), deltaH.y(), meshSurfaceNormal.z());

//				perturbedNormal = meshSurfaceNormal - deltaH + (deltaH.dot(meshSurfaceNormal))*meshSurfaceNormal;
//				perturbedNormal = deltaH;
				perturbedNormal = meshSurfaceNormal + deltaH;
				perturbedNormal.normalize();
				perturbedNorm = perturbedNormal;


				penetrationDepth = (m_proxyGlobalPos - m_deviceGlobalPos).length();

				material->heightMap->m_image->getPixelLocationInterpolated(texCoord, pixelX, pixelY, true);
				material->heightMap->m_image->getPixelColorInterpolated(pixelX, pixelY, pixelColor);
				height = pixelColor.getLuminance() / 255.0;

				penetrationDepth += height;
				penDepthDebug = penetrationDepth;

				double perturbedNormalFactor = material->smoothnessConstant * height;
				double meshNormalFactor = penetrationDepth - perturbedNormalFactor;

				meshNormalFactor = ((meshNormalFactor >= 0.0) ? meshNormalFactor : 0.0);
							
				dHx = meshNormalFactor;
				dHy = perturbedNormalFactor;
				dHz = material->smoothnessConstant;

				
				
				double forceMagnitude = m_lastGlobalForce.length();
				if (penetrationDepth > perturbedNormalFactor)
				{
					m_lastGlobalForce =  
						(penetrationDepth - perturbedNormalFactor)*meshSurfaceNormal +
						perturbedNormalFactor * perturbedNormal;
					m_lastGlobalForce.normalize();
				}
				else
				{
					m_lastGlobalForce = perturbedNormalFactor * perturbedNormal;
				}

				m_lastGlobalForce = cVector3d(m_lastGlobalForce.x(), m_lastGlobalForce.y(), m_lastGlobalForce.z());

				m_lastGlobalForce *= forceMagnitude;

//				std::cout << "Last Global Force: " << m_lastGlobalForce.str() << std::endl;
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
			//std::cout << "Here Friction OthersA.\n";

			//material->roughnessMap->m_image->getPixelLocationInterpolated(texCoord, pixelX, pixelY, true);
			//material->roughnessMap->m_image->getPixelColorInterpolated(pixelX, pixelY, pixelColor);

			//m_roughnessAtCollision = pixelColor;

			//r = pixelColor.getR();
			//g = pixelColor.getG();
			//b = pixelColor.getB();

			////				material->maxStaticFriction

			//double maxStaticFriction = 2.0;
			//double maxDynamicFriction = 1.5;
			//double roughness = (r + g + b) / (3.0*255.0);

			//a_parent->setFriction(maxStaticFriction * roughness, maxDynamicFriction * roughness, true);
		}
	}


	cAlgorithmFingerProxy::testFrictionAndMoveProxy(a_goal, a_proxy, a_normal, a_parent);
}

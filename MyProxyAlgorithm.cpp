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

#define GLM_ENABLE_EXPERIMENTAL


#include <../glm/glm/vec3.hpp>
#include <../glm/glm/gtc/matrix_transform.hpp>
#include <../glm/glm/mat4x4.hpp>
#include <../glm/glm/gtx/rotate_vector.hpp>




using namespace glm;
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

			if (texCoord.x() > 1.0)
				texCoord = cVector3d(texCoord.x() - 1.0, texCoord.y(), texCoord.z());
			if (texCoord.y() > 1.0)
				texCoord = cVector3d(texCoord.x(), texCoord.y() - 1.0, texCoord.z());
			if (texCoord.x() < 0.0)
				texCoord = cVector3d(1.0 + texCoord.x(), texCoord.y(), texCoord.z());
			if (texCoord.y() < 0.0)
				texCoord = cVector3d(texCoord.x(), 1.0 + texCoord.y(), texCoord.z());


			// For Bumps texture -- procedural implementation
			if (material->objectID == 3)
			{
				image = c0->m_object->m_texture->m_image;
				image->getPixelLocationInterpolated(texCoord, pixelX, pixelY, true);
				image->getPixelColorInterpolated(pixelX, pixelY, pixelColor);

				m_colorAtCollision = pixelColor;

				double g, b;
				g = (double)pixelColor.getG();
				b = (double)pixelColor.getB();

				// Height used to scale force when passing over bumps.
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

				// yVariant is used to vary the force in the y direction.
				// negator is used to negate the y variant when passing over the middle of the bump.
				double yVariant = sin(0.7 + 19.5*M_PI*distance);
				double negator = sin(0.7 +1.5*M_PI + 19.5*M_PI*distance);

//				std::cout << "Sin Tex coord Clamped: " << yVariant << std::endl;

				// Save the magnitude of force.
				double magnitudeOfForce = m_lastGlobalForce.length();
				double blendDistance = 0.15;
				double blendAmount = 1.0;

				// yVariant is between 0 and 1 when passing over white bands.
				if (yVariant > 0.0)
				{
					// Blend perturbation over short distance to avoid sharp changes in force direction.
					if (yVariant < blendAmount)
						blendAmount = yVariant / blendDistance;

					yVariant = 1.0 - yVariant;

					yVariant *= blendAmount;

					if (negator < 0.0)
						yVariant = -yVariant;

					// Use height to increase magnitude of force.
					magnitudeOfForce += height*2.0;
				}
				else
					yVariant = 0.0;

				// Add to the y component of the global force to simulate bumps
				m_lastGlobalForce += cVector3d(0.0, yVariant*magnitudeOfForce*0.25, 0.0);
				m_lastGlobalForce.normalize();
				m_lastGlobalForce = m_lastGlobalForce * magnitudeOfForce;
			}
			else if (material->objectID != 5)
			{
				cVector3d meshSurfaceNormal, normalMapNormal;
				double epsilon, penetrationDepth, height;

				savedTangentialForce = getTangentialForce();

				material->normalMap->m_image->getPixelLocationInterpolated(texCoord, pixelX, pixelY, true);
				material->normalMap->m_image->getPixelColorInterpolated(pixelX, pixelY, pixelColor);
				m_normalColorAtCollision = pixelColor;

				// Get normal relative to implicit (127.5, 127.5, 127.5) normal origin.
				// This is because normals are directions expressed in values ranging from 0 to 255.
				// If a value is 255, it is maximum in that direction, 0 is maximum in opposite direction.
				normalMapNormal = cVector3d(pixelColor.getG() - 127.5, pixelColor.getR() - 127.5, pixelColor.getB() - 127.5);
				normalMapNormal.normalize();


				meshSurfaceNormal = computeShadedSurfaceNormal(c0);
				meshSurfaceNormal.normalize();
		
				
				// Get angle between surface normal and implicit global unit axes
				float thetaX, thetaY, thetaZ;
				thetaX = acos(meshSurfaceNormal.dot(cVector3d(1.0, 0.0, 0.0)));
				thetaY = acos(meshSurfaceNormal.dot(cVector3d(0.0, 1.0, 0.0)));
				thetaZ = acos(meshSurfaceNormal.dot(cVector3d(0.0, 0.0, 1.0)));

				// Rotate the normal map normal by the same angle that the surface normal deviates from these
				// axes.
				vec3 glmNormalMapNormal = vec3(normalMapNormal.y(), normalMapNormal.z(), normalMapNormal.x());

				// If normal is 80 degrees off of positive x in Chai3d, we rotate -10 degrees about positive
				// x in glm.
				// If normal is 170 degrees off of positive x in Chai3d, we rotate 80 degrees about positive
				// x in glm.
				if (thetaX < (M_PI * 0.5))
				{
					thetaX = (M_PI * 0.5) - thetaX;
				}
				else
				{
					thetaX = thetaX - (M_PI * 0.5);
					thetaX = -thetaX;
				}
				vec3 xAxis = vec3(1.0, 0.0, 0.0);
				glmNormalMapNormal = rotateX(glmNormalMapNormal, thetaX);

				// If normal is 80 degrees off of positive y in Chai3d, we rotate -10 degrees about positive
				// z in glm.
				// If normal is 170 degrees off of positive y in Chai3d, we rotate 80 degrees about positive
				// z in gml
				if (thetaY < (M_PI * 0.5))
				{
					thetaY = (M_PI * 0.5) - thetaY;
					thetaY = -thetaY;
				}
				else
					thetaY = thetaY - (M_PI * 0.5);

				vec3 zAxis = vec3(0.0, 0.0, 1.0);
				glmNormalMapNormal = rotateZ(glmNormalMapNormal, thetaY);


				normalMapNormal = cVector3d(glmNormalMapNormal.z, glmNormalMapNormal.x, glmNormalMapNormal.y);
				normalMapNormal.normalize();
				// NEW CALCULATIONS END
				
				// Get the height at the collision point and use to scale the penetration depth.
				penetrationDepth = (m_proxyGlobalPos - m_deviceGlobalPos).length();

				material->heightMap->m_image->getPixelLocationInterpolated(texCoord, pixelX, pixelY, true);
				material->heightMap->m_image->getPixelColorInterpolated(pixelX, pixelY, pixelColor);
				height = pixelColor.getLuminance() / 255.0;

				penetrationDepth += height;
				penetrationDepth += (1.0 - material->smoothnessConstant);

				// Calculate the blending factors to blend the normal map normal with the surface normal.
				double perturbedNormalFactor = material->smoothnessConstant * height;
				double meshNormalFactor = penetrationDepth - perturbedNormalFactor;
				meshNormalFactor = ((meshNormalFactor >= 0.0) ? meshNormalFactor : 0.0);
				meshNormalFactor = ((meshNormalFactor <= 1.0) ? meshNormalFactor : 1.0);


				surfaceNorm = meshSurfaceNormal;
				normalMapNorm = normalMapNormal;
				

				double forceMagnitude = m_lastGlobalForce.length();
				perturbedNormal = normalMapNormal;

				// If penetration depth is large, blend normal map normal with surface normal to avoid force
				// direction discontinuitues.
				if (penetrationDepth > perturbedNormalFactor)
				{
					m_lastGlobalForce =  
						(penetrationDepth - perturbedNormalFactor)*meshSurfaceNormal +
						perturbedNormalFactor * perturbedNormal;
				}
				else
				{
					m_lastGlobalForce = perturbedNormalFactor * perturbedNormal;
				}

				m_lastGlobalForce = cVector3d(m_lastGlobalForce.x(), m_lastGlobalForce.y(), m_lastGlobalForce.z() + (height * (1.5 - material->smoothnessConstant)));
				
				// If friction is on, use the previously saved tangential force to alter the global force.
				if (frictionOn)
					m_lastGlobalForce += (savedTangentialForce * 0.25);
	
				m_lastGlobalForce.normalize();
				m_lastGlobalForce *= forceMagnitude;
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
		
		if (texCoord.x() > 1.0)
			texCoord = cVector3d(texCoord.x() - 1.0, texCoord.y(), texCoord.z());
		if (texCoord.y() > 1.0)
			texCoord = cVector3d(texCoord.x(), texCoord.y() - 1.0, texCoord.z());
		if (texCoord.x() < 0.0)
			texCoord = cVector3d(1.0 + texCoord.x(), texCoord.y(), texCoord.z());
		if (texCoord.y() < 0.0)
			texCoord = cVector3d(texCoord.x(), 1.0 + texCoord.y(), texCoord.z());

		// Procedural friction modulation for rocky/blue banded texture
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

			double staticFric;
			double dynamicFric;

			// Friction variant is > 0.0 when over the rocky surfaces.
			frictionVariant = ((frictionVariant > 0.0) ? frictionVariant : 0.0);

			double frictionMultiplier = pow((1.0 + frictionVariant), 3);
			frictionMultiplier -= 1.0;
			std::cout << "Friction Multiplier: " << frictionMultiplier << std::endl;

			// Use friction variant to modulate fricton.
			staticFric = material->baseStaticFriction * frictionMultiplier;
			dynamicFric = material->baseDynamicFriction * frictionMultiplier;
			
			a_parent->setFriction(staticFric, dynamicFric, true);
		}
		else if (material->objectID != 3)
		{
			// Get the roughness value from the roughness map.
			material->roughnessMap->m_image->getPixelLocationInterpolated(texCoord, pixelX, pixelY, true);
			material->roughnessMap->m_image->getPixelColorInterpolated(pixelX, pixelY, pixelColor);

			m_roughnessAtCollision = pixelColor;

			r = pixelColor.getR();
			g = pixelColor.getG();
			b = pixelColor.getB();

			double roughness = (r + g + b) / (3.0*255.0);

			roughness *= 0.25;

			// Modulate friction using material properties and roughness map values.
			if (frictionOn)
				a_parent->setFriction(material->maxStaticFriction * roughness * material->frictionFactor, material->maxDynamicFriction * roughness * material->frictionFactor, true);
			else
				a_parent->setFriction(0.0, 0.0, true);
		}
	}


	cAlgorithmFingerProxy::testFrictionAndMoveProxy(a_goal, a_proxy, a_normal, a_parent);
}




MyProxyAlgorithm::MyProxyAlgorithm()
{
	frictionOn = false;
}


void MyProxyAlgorithm::setFrictionOn(bool iWantItOn)
{
	frictionOn = iWantItOn;
}
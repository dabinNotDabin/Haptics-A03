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
			std::cout << "Here\n";

			// you can access your custom material properties here
			material->m_myMaterialProperty;

			c0->m_object->getBoundaryMax();
			c0->m_object->getBoundaryMin();
			c0->m_object->setShowBoundaryBox(true, false);
			cColorf color = c0->m_object->m_texture->m_color;

			double pixelX, pixelY;
			cColorb pixelColor;
			cVector3d texCoord;
			cVector3d perturbedNormal;

			std::string textureFilename;
			cImagePtr image;
			textureFilename = image->getFilename();


			texCoord = c0->m_triangles->getTexCoordAtPosition(c0->m_index, c0->m_localPos);
			if (textureFilename == "bumps.png")
			{
				image = c0->m_object->m_texture->m_image;
				image->getPixelLocationInterpolated(texCoord, pixelX, pixelY, true);
				image->getPixelColorInterpolated(pixelX, pixelY, pixelColor);

				m_colorAtCollision = pixelColor;

				double height = (pixelColor.getR() + pixelColor.getG() + pixelColor.getB()) / 255.0*3.0;
				double blendFactor = (height < (1.0 - height) ? height : (1.0 - height));


				int vertexIndex0 = c0->m_triangles->getVertexIndex0(c0->m_index);
				int vertexIndex1 = c0->m_triangles->getVertexIndex1(c0->m_index);

				//cVector3d tangentAtCollisionPt =
				//	c0->m_triangles->m_vertices->getLocalPos(1) - c0->m_triangles->m_vertices->getLocalPos(0);
				cVector3d tangentAtCollisionPt =
					c0->m_triangles->m_vertices->getLocalPos(vertexIndex1) - c0->m_triangles->m_vertices->getLocalPos(vertexIndex1);
				tangentAtCollisionPt.normalize();

				cVector3d normalAtCollisionPoint = c0->m_localNormal;
				normalAtCollisionPoint.normalize();

				perturbedNormal =
					(tangentAtCollisionPt * blendFactor) + (normalAtCollisionPoint * (1.0 - blendFactor));

				perturbedNormal.normalize();
				m_lastGlobalForce = perturbedNormal * m_lastGlobalForce.length();
			}
			else if (textureFilename != "friction.jpg")
			{
				image = c0->m_object->m_normalMap->m_image;
				image->getPixelLocationInterpolated(texCoord, pixelX, pixelY, true);
				image->getPixelColorInterpolated(pixelX, pixelY, pixelColor);

				m_normalColorAtCollision = pixelColor;

				perturbedNormal = cVector3d(pixelColor.getR() / 255.0, pixelColor.getG() / 255.0, pixelColor.getB() / 255.0);

				double height = 0.0;
				int pos = textureFilename.find_last_of('_');
				if (pos < textureFilename.length() && pos >= 0)
				{
					std::string fileBeginning = textureFilename.substr(0, pos);
					cTexture2dPtr heightMap = cTexture2d::create();
					heightMap->loadFromFile("images/" + fileBeginning + "_height.jpg");
					heightMap->setWrapModeS(GL_REPEAT);
					heightMap->setWrapModeT(GL_REPEAT);
					heightMap->setUseMipmaps(true);

					heightMap->m_image->getPixelLocationInterpolated(texCoord, pixelX, pixelY, true);
					heightMap->m_image->getPixelColorInterpolated(pixelX, pixelY, pixelColor);

					m_heightAtCollision = pixelColor;

					double r, g, b;
					r = pixelColor.getR();
					g = pixelColor.getG();
					b = pixelColor.getB();

					height = (r + g + b) / 255.0*3.0;
				}

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
		cImagePtr image;
		textureFilename = image->getFilename();


		cMultiMesh* object = (cMultiMesh*)a_parent;
		cMesh* mesh = object->getMesh(0);

		double pixelX, pixelY;
		cColorb pixelColor;
		cVector3d texCoord;
		double r, g, b;

		texCoord = c0->m_triangles->getTexCoordAtPosition(c0->m_index, c0->m_localPos);


		if (textureFilename == "friction.jpg")
		{
			texCoord = c0->m_triangles->getTexCoordAtPosition(c0->m_index, c0->m_localPos);
			image = a_parent->m_texture->m_image;
			image->getPixelLocationInterpolated(texCoord, pixelX, pixelY, true);
			image->getPixelColorInterpolated(pixelX, pixelY, pixelColor);

			m_roughnessAtCollision = pixelColor;

			// High blue AND low red/green = low friction;
			// Low blue AND high red/green = high friction;
			r = pixelColor.getR();
			g = pixelColor.getG();
			b = pixelColor.getB();

			double frictionModifier;
			double baseStaticFriction = 1.5 * (r + g - b);
			double baseDynamicFriction = (r + g - b);
			frictionModifier = (((r + g) > 1.0) ? (r + g) : 1.0);

			//		double currentStatic = a_parent->m_material->getStaticFriction();
			//		double currentDynamic = a_parent->m_material->getDynamicFriction();

			baseStaticFriction *= frictionModifier;
			baseDynamicFriction *= frictionModifier;

			baseStaticFriction = ((baseStaticFriction >= 0.0) ? baseStaticFriction : 0.0);
			baseDynamicFriction = ((baseDynamicFriction >= 0.0) ? baseDynamicFriction : 0.0);

			a_parent->setFriction(baseStaticFriction * frictionModifier, baseDynamicFriction * frictionModifier, true);
		}
		else if (textureFilename != "bumps.png")
		{
			int pos = textureFilename.find_last_of('_');
			if (pos < textureFilename.length() && pos >= 0)
			{
				std::string fileBeginning = textureFilename.substr(0, pos);
				cTexture2dPtr roughnessMap = cTexture2d::create();
				roughnessMap->loadFromFile("images/" + fileBeginning + "_roughness.jpg");
				roughnessMap->setWrapModeS(GL_REPEAT);
				roughnessMap->setWrapModeT(GL_REPEAT);
				roughnessMap->setUseMipmaps(true);

				roughnessMap->m_image->getPixelLocationInterpolated(texCoord, pixelX, pixelY, true);
				roughnessMap->m_image->getPixelColorInterpolated(pixelX, pixelY, pixelColor);

				r = pixelColor.getR();
				g = pixelColor.getG();
				b = pixelColor.getB();

//				material->maxStaticFriction

				double maxStaticFriction = 2.0;
				double maxDynamicFriction = 1.5;
				double roughness = (r + g + b) / 3.0;

				a_parent->setFriction(maxStaticFriction * roughness, maxDynamicFriction * roughness, true);
			}
			else
			{
				std::cout << "Failure loading roughness map.\n";
			}
		}
	}


	cAlgorithmFingerProxy::testFrictionAndMoveProxy(a_goal, a_proxy, a_normal, a_parent);
}

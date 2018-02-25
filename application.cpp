//==============================================================================
/*
\author    Your Name
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "chai3d.h"
#include "MyProxyAlgorithm.h"
#include "MyMaterial.h"
#include <iostream>

//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
C_STEREO_DISABLED:            Stereo is disabled
C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;


//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cSpotLight* light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a label to display the rates [Hz] at which the simulation is running
cLabel* labelRates;

// a small sphere (cursor) representing the haptic device 
cToolCursor* tool;

// a pointer to the custom proxy rendering algorithm inside the tool
MyProxyAlgorithm* proxyAlgorithm;

// nine objects with different surface textures that we want to render
cMultiMesh *objects[3][3];

// flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// flag to indicate if the haptic simulation has terminated
bool simulationFinished = false;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;

// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;


chai3d::cVector3d cameraPosition;
chai3d::cVector3d cameraLookAt;


chai3d::cVector3d devicePosOld;

double workspaceRadius = 0.0375;




cLabel* colorCollisionLabel;
cLabel* heightCollisionLabel;
cLabel* roughnessCollisionLabel;
cLabel* normalCollisionLabel;


cLabel *deltaHLabel;
cLabel *penDepthLabel;
cLabel *normalVectorLabel;
cLabel *normalMapNormalLabel;
cLabel *perturbedNormalVectorLabel;
cLabel *infoLabel;




cMesh* normalMapNormalArrow = new cMesh();
cMesh* surfaceNormalArrow = new cMesh();
cMesh* globalForceArrow = new cMesh();


bool showNormals;
bool frictionOn;

//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function renders the scene
void updateGraphics(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);


//==============================================================================
/*
TEMPLATE:    application.cpp

Description of your application.
*/
//==============================================================================

int main(int argc, char* argv[])
{
	//--------------------------------------------------------------------------
	// INITIALIZATION
	//--------------------------------------------------------------------------

	cout << endl;
	cout << "-----------------------------------" << endl;
	cout << "CHAI3D" << endl;
	cout << "-----------------------------------" << endl << endl << endl;
	cout << "Keyboard Options:" << endl << endl;
	cout << "[f] - Enable/Disable full screen mode" << endl;
	cout << "[m] - Enable/Disable vertical mirroring" << endl;
	cout << "[q] - Exit application" << endl;
	cout << endl << endl;


	//--------------------------------------------------------------------------
	// OPENGL - WINDOW DISPLAY
	//--------------------------------------------------------------------------

	// initialize GLFW library
	if (!glfwInit())
	{
		cout << "failed initialization" << endl;
		cSleepMs(1000);
		return 1;
	}

	// set error callback
	glfwSetErrorCallback(errorCallback);

	// compute desired size of window
	const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
	int w = 0.8 * mode->height;
	int h = 0.5 * mode->height;
	int x = 0.5 * (mode->width - w);
	int y = 0.5 * (mode->height - h);

	// set OpenGL version
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

	// set active stereo mode
	if (stereoMode == C_STEREO_ACTIVE)
	{
		glfwWindowHint(GLFW_STEREO, GL_TRUE);
	}
	else
	{
		glfwWindowHint(GLFW_STEREO, GL_FALSE);
	}

	// create display context
	window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
	if (!window)
	{
		cout << "failed to create window" << endl;
		cSleepMs(1000);
		glfwTerminate();
		return 1;
	}

	// get width and height of window
	glfwGetWindowSize(window, &width, &height);

	// set position of window
	glfwSetWindowPos(window, x, y);

	// set key callback
	glfwSetKeyCallback(window, keyCallback);

	// set resize callback
	glfwSetWindowSizeCallback(window, windowSizeCallback);

	// set current display context
	glfwMakeContextCurrent(window);

	// sets the swap interval for the current display context
	glfwSwapInterval(swapInterval);

#ifdef GLEW_VERSION
	// initialize GLEW library
	if (glewInit() != GLEW_OK)
	{
		cout << "failed to initialize GLEW library" << endl;
		glfwTerminate();
		return 1;
	}
#endif


	//--------------------------------------------------------------------------
	// WORLD - CAMERA - LIGHTING
	//--------------------------------------------------------------------------

	// create a new world.
	world = new cWorld();

	// set the background color of the environment
	world->m_backgroundColor.setBlack();

	// create a camera and insert it into the virtual world
	camera = new cCamera(world);
	world->addChild(camera);


	cameraPosition = cVector3d(0.1, 0.0, 0.07);
	cameraLookAt = cVector3d(0.0, 0.0, 0.0);

	// position and orient the camera
	camera->set(cameraPosition,    // camera position (eye)
		cameraLookAt,    // look at position (target)
		cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

									 // set the near and far clipping planes of the camera
	camera->setClippingPlanes(0.01, 1.0);

	// set stereo mode
	camera->setStereoMode(stereoMode);

	// set stereo eye separation and focal length (applies only if stereo is enabled)
	camera->setStereoEyeSeparation(0.01);
	camera->setStereoFocalLength(0.5);

	// set vertical mirrored display mode
	camera->setMirrorVertical(mirroredDisplay);

	// create a directional light source
	light = new cSpotLight(world);

	// insert light source inside world
	world->addChild(light);

	// enable light source
	light->setEnabled(true);

	// position the light source
	light->setLocalPos(0.7, 0.3, 1.0);

	// define the direction of the light beam
	light->setDir(-0.5, -0.2, -0.8);

	// enable this light source to generate shadows
	light->setShadowMapEnabled(true);

	// set the resolution of the shadow map
	light->m_shadowMap->setQualityHigh();

	// set light cone half angle
	light->setCutOffAngleDeg(10);

	// use a point avatar for this scene
	double toolRadius = 0.0;



	showNormals = false;
	frictionOn = false;


	//--------------------------------------------------------------------------
	// [CPSC.86] TEXTURED OBJECTS
	//--------------------------------------------------------------------------

	const double objectSpacing = 0.09;

	const std::string textureFiles[3][3] = 
	{
		{ "Organic_Scales_001_colour.jpg", "unknown.png", "Fabric_002_colour.jpg" },
		{ "bumps.png", "Metal_plate_001_colour.jpg", "friction.jpg" },
		{ "Leather_padded_001_colour.jpg", "unknown.png", "Cork_001_colour.jpg" }
	};

	const std::string normalMaps[3][3] =
	{
		{ "Organic_Scales_001_normal.jpg", "unknown.png", "Fabric_002_normal.jpg" },
		{ "bumps.png", "Metal_plate_001_normal.jpg", "friction.jpg" },
		{ "Leather_padded_001_normal.jpg", "unknown.png", "Cork_001_normal.jpg" }
	};


	const std::string heightMaps[3][3] =
	{
		{ "Organic_Scales_001_height.jpg", "unknown.png", "Fabric_002_height.jpg" },
		{ "bumps.png", "Metal_plate_001_height.jpg", "friction.jpg" },
		{ "Leather_padded_001_height.jpg", "unknown.png", "Cork_001_height.jpg" }
	};


	const std::string roughnessMaps[3][3] =
	{
		{ "Organic_Scales_001_roughness.jpg", "unknown.png", "Fabric_002_roughness.jpg" },
		{ "bumps.png", "Metal_plate_001_roughness.jpg", "friction.jpg" },
		{ "Leather_padded_001_roughness.jpg", "unknown.png", "Cork_001_roughness.jpg" }
	};



	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			objects[i][j] = new cMultiMesh();
			cMultiMesh* object = objects[i][j];

			// load geometry from file and compute additional properties
			object->loadFromFile("tray.obj");
			object->createAABBCollisionDetector(toolRadius);
			object->computeBTN();

			// obtain the first (and only) mesh from the object
			cMesh* mesh = object->getMesh(0);

			// replace the object's material with a custom one
			MyMaterialPtr material = MyMaterial::create();
			mesh->m_material = material;
			mesh->m_material->setWhite();
			mesh->m_material->setUseHapticShading(true);
			mesh->m_material->setUseHapticTexture(true);
			object->setStiffness(2000.0, true);

			// create a colour texture map for this mesh object
			cTexture2dPtr albedoMap = cTexture2d::create();
			albedoMap->loadFromFile("images/" + textureFiles[i][j]);
			albedoMap->setWrapModeS(GL_REPEAT);
			albedoMap->setWrapModeT(GL_REPEAT);
			albedoMap->setUseMipmaps(true);

			// assign textures to the mesh
			mesh->m_texture = albedoMap;
			mesh->setUseTexture(true);


			cTexture2dPtr normalMap = cTexture2d::create();
			normalMap->loadFromFile("images/" + normalMaps[i][j]);
			normalMap->setWrapModeS(GL_REPEAT);
			normalMap->setWrapModeT(GL_REPEAT);
			normalMap->setUseMipmaps(true);

			cTexture2dPtr heightMap = cTexture2d::create();
			heightMap->loadFromFile("images/" + heightMaps[i][j]);
			heightMap->setWrapModeS(GL_REPEAT);
			heightMap->setWrapModeT(GL_REPEAT);
			heightMap->setUseMipmaps(true);


			cTexture2dPtr roughnessMap = cTexture2d::create();
			roughnessMap->loadFromFile("images/" + roughnessMaps[i][j]);
			roughnessMap->setWrapModeS(GL_REPEAT);
			roughnessMap->setWrapModeT(GL_REPEAT);
			roughnessMap->setUseMipmaps(true);


			material->normalMap = normalMap;
			material->heightMap = heightMap;
			material->roughnessMap = roughnessMap;
			material->objectID = i*3 + j;
			material->baseStaticFriction = 0.3;
			material->baseDynamicFriction = 0.1;
			material->maxStaticFriction = 2.0;
			material->maxDynamicFriction = 1.7;


			// (0)Scales -- (1)Unknown -- (2)Fabric -- (3)Bumps -- (4)Metal -- (5)Friction -- (6)Leather -- (7)Unknown -- (8)Cork
			switch (material->objectID)
			{
				case 0: 
					material->frictionFactor = 0.4;
					material->smoothnessConstant = 0.6;
					break;
				case 1: 
					material->frictionFactor = 1.0;
					material->smoothnessConstant = 1.0;
					break;
				case 2:
					material->frictionFactor = 0.5;
					material->smoothnessConstant = 0.8;
					break;
				case 3:
					material->frictionFactor = 0.0;
					material->smoothnessConstant = 1.0;
					break;
				case 4:
					material->frictionFactor = 0.4;
					material->smoothnessConstant = 0.85;
					break;
				case 5:
					material->frictionFactor = 1.0;
					material->smoothnessConstant = 1.0;
					break;
				case 6:
					material->frictionFactor = 0.25;
					material->smoothnessConstant = 0.4;
					break;
				case 7:
					material->frictionFactor = 1.0;
					material->smoothnessConstant = 1.0;
					break;
				case 8:
					material->frictionFactor = 0.8;
					material->smoothnessConstant = 0.35;
					break;

				default:
					material->frictionFactor = 1.0;
					material->smoothnessConstant = 0.5;
			}



//			mesh->setShowNormals(true);

			// set the position of this object
			double xpos = -objectSpacing + i * objectSpacing;
			double ypos = -objectSpacing + j * objectSpacing;
			object->setLocalPos(xpos, ypos);

			world->addChild(object);
		}
	}

	//--------------------------------------------------------------------------
	// HAPTIC DEVICE
	//--------------------------------------------------------------------------

	// create a haptic device handler
	handler = new cHapticDeviceHandler();

	// get a handle to the first haptic device
	handler->getDevice(hapticDevice, 0);

	// if the device has a gripper, enable the gripper to simulate a user switch
	hapticDevice->setEnableGripperUserSwitch(true);

	tool = new cToolCursor(world);
	world->addChild(tool);

	// [CPSC.86] replace the tool's proxy rendering algorithm with our own
	proxyAlgorithm = new MyProxyAlgorithm;
	delete tool->m_hapticPoint->m_algorithmFingerProxy;
	tool->m_hapticPoint->m_algorithmFingerProxy = proxyAlgorithm;

	tool->m_hapticPoint->m_sphereProxy->m_material->setWhite();

	tool->setRadius(0.001, toolRadius);

	tool->setHapticDevice(hapticDevice);

	tool->setWaitForSmallForce(true);

	tool->start();


	//--------------------------------------------------------------------------
	// WIDGETS
	//--------------------------------------------------------------------------

	// create a font
	cFontPtr font = NEW_CFONTCALIBRI20();

	// create a label to display the haptic and graphic rates of the simulation
	labelRates = new cLabel(font);
	labelRates->m_fontColor.setWhite();
	camera->m_frontLayer->addChild(labelRates);


	colorCollisionLabel = new cLabel(font);
	colorCollisionLabel->m_fontColor.setWhite();
	camera->m_frontLayer->addChild(colorCollisionLabel);

	heightCollisionLabel = new cLabel(font);
	heightCollisionLabel->m_fontColor.setWhite();
	camera->m_frontLayer->addChild(heightCollisionLabel);

	roughnessCollisionLabel = new cLabel(font);
	roughnessCollisionLabel->m_fontColor.setWhite();
	camera->m_frontLayer->addChild(roughnessCollisionLabel);

	normalCollisionLabel = new cLabel(font);
	normalCollisionLabel->m_fontColor.setWhite();
	camera->m_frontLayer->addChild(normalCollisionLabel);




	penDepthLabel = new cLabel(font);
	penDepthLabel->m_fontColor.setWhite();
	camera->m_frontLayer->addChild(penDepthLabel);


	deltaHLabel = new cLabel(font);
	deltaHLabel->m_fontColor.setWhite();
	camera->m_frontLayer->addChild(deltaHLabel);


	normalVectorLabel = new cLabel(font);
	normalVectorLabel->m_fontColor.setWhite();
	camera->m_frontLayer->addChild(normalVectorLabel);


	normalMapNormalLabel = new cLabel(font);
	normalMapNormalLabel->m_fontColor.setWhite();
	camera->m_frontLayer->addChild(normalMapNormalLabel);


	perturbedNormalVectorLabel = new cLabel(font);
	perturbedNormalVectorLabel->m_fontColor.setWhite();
	camera->m_frontLayer->addChild(perturbedNormalVectorLabel);


	infoLabel = new cLabel(font);
	infoLabel->m_fontColor.setWhite();
	camera->m_frontLayer->addChild(infoLabel);




	//--------------------------------------------------------------------------
	// START SIMULATION
	//--------------------------------------------------------------------------

	// create a thread which starts the main haptics rendering loop
	hapticsThread = new cThread();
	hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

	// setup callback when application exits
	atexit(close);


	//--------------------------------------------------------------------------
	// MAIN GRAPHIC LOOP
	//--------------------------------------------------------------------------

	// call window size callback at initialization
	windowSizeCallback(window, width, height);

	// main graphic loop
	while (!glfwWindowShouldClose(window))
	{
		// get width and height of window
		glfwGetWindowSize(window, &width, &height);

		// render graphics
		updateGraphics();

		// swap buffers
		glfwSwapBuffers(window);

		// process events
		glfwPollEvents();

		// signal frequency counter
		freqCounterGraphics.signal(1);
	}

	// close window
	glfwDestroyWindow(window);

	// terminate GLFW library
	glfwTerminate();

	// exit
	return 0;
}

//------------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
	// update window size
	width = a_width;
	height = a_height;
}

//------------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description)
{
	cout << "Error: " << a_description << endl;
}

//------------------------------------------------------------------------------

void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
	// filter calls that only include a key press
	if (a_action != GLFW_PRESS)
	{
		return;
	}

	// option - exit
	else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
	{
		glfwSetWindowShouldClose(a_window, GLFW_TRUE);
	}

	// option - toggle fullscreen
	else if (a_key == GLFW_KEY_F)
	{
		// toggle state variable
		fullscreen = !fullscreen;

		// get handle to monitor
		GLFWmonitor* monitor = glfwGetPrimaryMonitor();

		// get information about monitor
		const GLFWvidmode* mode = glfwGetVideoMode(monitor);

		// set fullscreen or window mode
		if (fullscreen)
		{
			glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
			glfwSwapInterval(swapInterval);
		}
		else
		{
			int w = 0.8 * mode->height;
			int h = 0.5 * mode->height;
			int x = 0.5 * (mode->width - w);
			int y = 0.5 * (mode->height - h);
			glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
			glfwSwapInterval(swapInterval);
		}
	}

	// option - toggle vertical mirroring
	else if (a_key == GLFW_KEY_M)
	{
		mirroredDisplay = !mirroredDisplay;
		camera->setMirrorVertical(mirroredDisplay);
	}
	else if (a_key == GLFW_KEY_N)
	{
		showNormals = !showNormals;
	}
	else if (a_key == GLFW_KEY_O)
	{
		frictionOn = !frictionOn;

		proxyAlgorithm->setFrictionOn(frictionOn);
	}


}

//------------------------------------------------------------------------------

void close(void)
{
	// stop the simulation
	simulationRunning = false;

	// wait for graphics and haptics loops to terminate
	while (!simulationFinished) { cSleepMs(100); }

	// close haptic device
	hapticDevice->close();

	// delete resources
	delete hapticsThread;
	delete world;
	delete handler;
}

//------------------------------------------------------------------------------

void updateGraphics(void)
{
	/////////////////////////////////////////////////////////////////////
	// UPDATE WIDGETS
	/////////////////////////////////////////////////////////////////////

	if (normalMapNormalArrow != NULL)
		world->deleteChild(normalMapNormalArrow);
	if (surfaceNormalArrow != NULL)
		world->deleteChild(surfaceNormalArrow);
	if (globalForceArrow != NULL)
		world->deleteChild(globalForceArrow);

	normalMapNormalArrow = new cMesh();
	surfaceNormalArrow = new cMesh();
	globalForceArrow = new cMesh();

	cCreateArrow(normalMapNormalArrow, 0.05, 0.0002, 0.001, 0.001, false, 32, proxyAlgorithm->normalMapNorm, proxyAlgorithm->getProxyGlobalPosition(), cColorf(0.0, 1.0, 0.0, 0.0));
	cCreateArrow(surfaceNormalArrow, 0.05, 0.0002, 0.001, 0.001, false, 32, proxyAlgorithm->surfaceNorm, proxyAlgorithm->getProxyGlobalPosition(), cColorf(0.0, 1.0, 0.0, 0.0));
	cCreateArrow(globalForceArrow, 0.05, 0.0002, 0.001, 0.001, false, 32, proxyAlgorithm->getForce(), proxyAlgorithm->getProxyGlobalPosition(), cColorf(0.0, 1.0, 0.0, 0.0));

	normalMapNormalArrow->m_material->setGreenLime();
	surfaceNormalArrow->m_material->setBlack();
	globalForceArrow->m_material->setRedCrimson();

	if (showNormals)
	{
		world->addChild(normalMapNormalArrow);
		world->addChild(surfaceNormalArrow);
		world->addChild(globalForceArrow);
	}



	// update haptic and graphic rate data
	labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
		cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");
	labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);

//	penDepthLabel->setText("Penetration Depth: " + to_string(proxyAlgorithm->penDepthDebug));
//	penDepthLabel->setLocalPos((int)(0.5 * (width - penDepthLabel->getWidth())), 115);

//	deltaHLabel->setText("DeltaH: " + proxyAlgorithm->deltaHVector.str());
//	deltaHLabel->setLocalPos((int)(0.5 * (width - deltaHLabel->getWidth())), 95);

//	normalVectorLabel->setText("Mesh Normal At Collision: " + proxyAlgorithm->surfaceNorm.str());
//	normalVectorLabel->setLocalPos((int)(0.5 * (width - normalVectorLabel->getWidth())), 75);

//	normalMapNormalLabel->setText("Normal Map Normal At Collision: " + proxyAlgorithm->normalMapNorm.str());
//	normalMapNormalLabel->setLocalPos((int)(0.5 * (width - normalMapNormalLabel->getWidth())), 55);

//	perturbedNormalVectorLabel->setText("Perturbed Normal At Collision: " + proxyAlgorithm->perturbedNorm.str());
//	perturbedNormalVectorLabel->setLocalPos((int)(0.5 * (width - perturbedNormalVectorLabel->getWidth())), 35);

	if (showNormals)
		infoLabel->setText
		(
			"Friction is currently: " + string((frictionOn) ? "ON" : "OFF") + "\n" +
			"    Press \"O\" to toggle friction." + "\n\n" +
			"Render normals is currently: " + string((showNormals) ? "ON" : "OFF") + "\n" +
			"    Press \"N\" to toggle normal rendering." + "\n" +
			"    Red is the global force.\n    Black is the surface normal.\n    Green is the normal map normal."
		);
	else
		infoLabel->setText
		(
			"Friction is currently: " + string((frictionOn) ? "ON" : "OFF") + "\n" +
			"    Press \"O\" to toggle friction." + "\n\n" +
			"Render normals is currently: " + string((showNormals) ? "ON" : "OFF") + "\n" +
			"    Press \"N\" to toggle normal rendering." + "\n"
		);

	infoLabel->setLocalPos(10, height - 200);

//	cColorb color;

	/*
	color = proxyAlgorithm->m_colorAtCollision;
	colorCollisionLabel->setText
	(
		"Color at collision: " +
		to_string(color.getR()) + ", " + to_string(color.getG()) + ", " + to_string(color.getB())
	);
	colorCollisionLabel->setLocalPos((int)(0.1 * (width - colorCollisionLabel->getWidth())), height - 15);


	color = proxyAlgorithm->m_heightAtCollision;
	heightCollisionLabel->setText
	(
		"Height at collision: " +
		to_string(color.getR()) + ", " + to_string(color.getG()) + ", " + to_string(color.getB())
	);
	heightCollisionLabel->setLocalPos((int)(0.1 * (width - heightCollisionLabel->getWidth())), height - 40);


	color = proxyAlgorithm->m_roughnessAtCollision;
	roughnessCollisionLabel->setText
	(
		"Roughness at collision: " +
		to_string(color.getR()) + ", " + to_string(color.getG()) + ", " + to_string(color.getB())
	);
	roughnessCollisionLabel->setLocalPos((int)(0.1 * (width - roughnessCollisionLabel->getWidth())), height - 65);


	color = proxyAlgorithm->m_normalColorAtCollision;
	normalCollisionLabel->setText
	(
		"Normal at collision: " +
		to_string(color.getR()) + ", " + to_string(color.getG()) + ", " + to_string(color.getB())
	);
	normalCollisionLabel->setLocalPos((int)(0.1 * (width - normalCollisionLabel->getWidth())), height - 90);
	*/

	/////////////////////////////////////////////////////////////////////
	// RENDER SCENE
	/////////////////////////////////////////////////////////////////////

	// update shadow maps (if any)
	world->updateShadowMaps(false, mirroredDisplay);

	// render world
	camera->renderView(width, height);

	// wait until all GL commands are completed
	glFinish();

	// check for any OpenGL errors
	GLenum err;
	err = glGetError();
	if (err != GL_NO_ERROR) cout << "Error:  %s\n" << gluErrorString(err);
}

//------------------------------------------------------------------------------

void updateHaptics(void)
{
	// simulation in now running
	simulationRunning = true;
	simulationFinished = false;

	// main haptic simulation loop
	while (simulationRunning)
	{
		/////////////////////////////////////////////////////////////////////
		// READ HAPTIC DEVICE
		/////////////////////////////////////////////////////////////////////

		// read position 
		cVector3d toolPos = tool->getLocalPos();
		cVector3d position;
		hapticDevice->getPosition(position);

		// read orientation 
		cMatrix3d rotation;
		hapticDevice->getRotation(rotation);

		// read user-switch status (button 0)
		bool button = false;
		hapticDevice->getUserSwitch(0, button);


		world->computeGlobalPositions();

		/////////////////////////////////////////////////////////////////////
		// UPDATE 3D CURSOR MODEL
		/////////////////////////////////////////////////////////////////////

		tool->updateFromDevice();


		/////////////////////////////////////////////////////////////////////
		// UPDATE CAMERA WITH RESPECT TO AVATAR POSITION
		/////////////////////////////////////////////////////////////////////

		position = cVector3d(position.x(), position.y(), 0.0);

		if (position.x() < 0.0)
			position = cVector3d(position.x() - 0.01, position.y(), 0.0);
		else
			position = cVector3d(position.x() + 0.02, position.y(), 0.0);

		chai3d::cVector3d positionDirection = position;
		positionDirection.normalize();
		if (position.length() > workspaceRadius)
		{
			tool->setLocalPos(tool->getLocalPos() + positionDirection * min((max((position.length() - workspaceRadius) * 0.015, 0.00001)), 0.0001));
			//			tool->setLocalPos(tool->getLocalPos() + positionDirection * (position.length() - workspaceRadius) * 0.01);

			// A) The camera mimics the avatar's movement along the x and y plane.
			cVector3d toolPosDxDy = tool->getLocalPos() - toolPos;
			toolPosDxDy = cVector3d(toolPosDxDy.x(), toolPosDxDy.y(), 0.0);

			cameraPosition = cameraPosition + toolPosDxDy;
			cameraLookAt = cameraLookAt + toolPosDxDy;

			camera->set
			(
				cameraPosition,				// camera position (eye)
				cameraLookAt,				// look at position (target)
				cVector3d(0.0, 0.0, 1.0)	// direction of the (up) vector
			);
			// End A)
		}





		/////////////////////////////////////////////////////////////////////
		// COMPUTE FORCES
		/////////////////////////////////////////////////////////////////////

		tool->computeInteractionForces();

		cVector3d force(0, 0, 0);
		cVector3d torque(0, 0, 0);
		double gripperForce = 0.0;


		/////////////////////////////////////////////////////////////////////
		// APPLY FORCES
		/////////////////////////////////////////////////////////////////////

		tool->applyToDevice();

		// signal frequency counter
		freqCounterHaptics.signal(1);
	}

	// exit haptics thread
	simulationFinished = true;
}

//------------------------------------------------------------------------------

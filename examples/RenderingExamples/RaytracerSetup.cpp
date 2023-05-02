
#include "RaytracerSetup.h"

#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3AlignedObjectArray.h"
#include "../CommonInterfaces/CommonRenderInterface.h"

#include "../CommonInterfaces/Common2dCanvasInterface.h"
//#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h"
//#include "BulletCollision/NarrowPhaseCollision/btGjkConvexCast.h"
//#include "BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.h"
#include "../CommonInterfaces/CommonExampleInterface.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "btBulletCollisionCommon.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"

struct RaytracerPhysicsSetup : public CommonExampleInterface
{
	struct CommonGraphicsApp* m_app;
	struct RaytracerInternalData* m_internalData;

	RaytracerPhysicsSetup(struct CommonGraphicsApp* app);

	virtual ~RaytracerPhysicsSetup();

	virtual void initPhysics();

	virtual void exitPhysics();

	virtual void stepSimulation(btScalar deltaTime);

	virtual void physicsDebugDraw(int debugFlags);

	virtual void syncPhysicsToGraphics(struct GraphicsPhysicsBridge& gfxBridge);

	///worldRaytest performs a ray versus all objects in a collision world, returning true is a hit is found (filling in worldNormal and worldHitPoint)
	bool worldRaytest(const btVector3& rayFrom, const btVector3& rayTo, btVector3& worldNormal, btVector3& worldHitPoint);

	///singleObjectRaytest performs a ray versus one collision shape, returning true is a hit is found (filling in worldNormal and worldHitPoint)
	bool singleObjectRaytest(const btVector3& rayFrom, const btVector3& rayTo, btVector3& worldNormal, btVector3& worldHitPoint);

	///lowlevelRaytest performs a ray versus convex shape, returning true is a hit is found (filling in worldNormal and worldHitPoint)
	bool lowlevelRaytest(const btVector3& rayFrom, const btVector3& rayTo, btVector3& worldNormal, btVector3& worldHitPoint);

	virtual bool mouseMoveCallback(float x, float y);

	virtual bool mouseButtonCallback(int button, int state, float x, float y);

	virtual bool keyboardCallback(int key, int state);

	virtual void renderScene()
	{
	}
};

struct RaytracerInternalData
{
	int m_canvasIndex;
	struct Common2dCanvasInterface* m_canvas;

	int m_width;
	int m_height;

	btAlignedObjectArray<btConvexShape*> m_shapePtr;
	btAlignedObjectArray<btTransform> m_transforms;
	btVoronoiSimplexSolver m_simplexSolver;
	btScalar m_pitch;
	btScalar m_roll;
	btScalar m_yaw;

	RaytracerInternalData()
		: m_canvasIndex(-1),
		  m_canvas(0),
#ifdef _DEBUG
		  m_width(64),
		  m_height(64),
#else
		  m_width(128),
		  m_height(128),
#endif
		  m_pitch(0),
		  m_roll(0),
		  m_yaw(0)
	{
		btConeShape* cone = new btConeShape((btScalar)1, (btScalar)1);
		btSphereShape* sphere = new btSphereShape((btScalar)1);
		btBoxShape* box = new btBoxShape(btVector3(1, 1, 1));
		m_shapePtr.push_back(cone);
		m_shapePtr.push_back(sphere);
		m_shapePtr.push_back(box);

		updateTransforms();
	}
	void updateTransforms()
	{
		int numObjects = m_shapePtr.size();
		m_transforms.resize(numObjects);
		for (int i = 0; i < numObjects; i++)
		{
			m_transforms[i].setIdentity();
			btVector3 pos(0.f, 0.f, -(2.5 * numObjects * 0.5) + i * 2.5f);
			m_transforms[i].setIdentity();
			m_transforms[i].setOrigin(pos);
			btQuaternion orn;
			if (i < 2)
			{
				orn.setEuler(m_yaw, m_pitch, m_roll);
				m_transforms[i].setRotation(orn);
			}
		}
		m_pitch += (btScalar)0.005f;
		m_yaw += (btScalar)0.01f;
	}
};

RaytracerPhysicsSetup::RaytracerPhysicsSetup(struct CommonGraphicsApp* app)
{
	m_app = app;
	m_internalData = new RaytracerInternalData;
}

RaytracerPhysicsSetup::~RaytracerPhysicsSetup()
{
	delete m_internalData;
}

void RaytracerPhysicsSetup::initPhysics()
{
	//request a visual bitma/texture we can render to

	m_internalData->m_canvas = m_app->m_2dCanvasInterface;

	if (m_internalData->m_canvas)
	{
		m_internalData->m_canvasIndex = m_internalData->m_canvas->createCanvas("raytracer", m_internalData->m_width, m_internalData->m_height, 15, 55);
		for (int i = 0; i < m_internalData->m_width; i++)
		{
			for (int j = 0; j < m_internalData->m_height; j++)
			{
				unsigned char red = 255;
				unsigned char green = 255;
				unsigned char blue = 255;
				unsigned char alpha = 255;
				m_internalData->m_canvas->setPixel(m_internalData->m_canvasIndex, i, j, red, green, blue, alpha);
			}
		}
		m_internalData->m_canvas->refreshImageData(m_internalData->m_canvasIndex);

		//int bitmapId = gfxBridge.createRenderBitmap(width,height);
	}
}

///worldRaytest performs a ray versus all objects in a collision world, returning true is a hit is found (filling in worldNormal and worldHitPoint)
bool RaytracerPhysicsSetup::worldRaytest(const btVector3& rayFrom, const btVector3& rayTo, btVector3& worldNormal, btVector3& worldHitPoint)
{
	return false;
}

///singleObjectRaytest performs a ray versus one collision shape, returning true is a hit is found (filling in worldNormal and worldHitPoint)
bool RaytracerPhysicsSetup::singleObjectRaytest(const btVector3& rayFrom, const btVector3& rayTo, btVector3& worldNormal, btVector3& worldHitPoint)
{
	return false;
}

///lowlevelRaytest performs a ray versus convex shape, returning true is a hit is found (filling in worldNormal and worldHitPoint)
bool RaytracerPhysicsSetup::lowlevelRaytest(const btVector3& rayFrom, const btVector3& rayTo, btVector3& worldNormal, btVector3& worldHitPoint)
{
	btScalar closestHitResults = (btScalar)1.f;

	bool hasHit = false;
	btConvexCast::CastResult rayResult;
	btSphereShape pointShape((btScalar)0.0f);
	btTransform rayFromTrans;
	btTransform rayToTrans;

	rayFromTrans.setIdentity();
	rayFromTrans.setOrigin(rayFrom);
	rayToTrans.setIdentity();
	rayToTrans.setOrigin(rayTo);

	int numObjects = m_internalData->m_shapePtr.size();

	for (int s = 0; s < numObjects; s++)
	{
		//do some culling, ray versus aabb
		btVector3 aabbMin, aabbMax;
		m_internalData->m_shapePtr[s]->getAabb(m_internalData->m_transforms[s], aabbMin, aabbMax);
		btScalar hitLambda = (btScalar)1.f;
		btVector3 hitNormal;
		btCollisionObject tmpObj;
		tmpObj.setWorldTransform(m_internalData->m_transforms[s]);

		if (btRayAabb(rayFrom, rayTo, aabbMin, aabbMax, hitLambda, hitNormal))
		{
			//reset previous result

			//choose the continuous collision detection method
			btSubsimplexConvexCast convexCaster(&pointShape, m_internalData->m_shapePtr[s], &m_internalData->m_simplexSolver);
			//btGjkConvexCast convexCaster(&pointShape,shapePtr[s],&simplexSolver);
			//btContinuousConvexCollision convexCaster(&pointShape,shapePtr[s],&simplexSolver,0);

			if (convexCaster.calcTimeOfImpact(rayFromTrans, rayToTrans, m_internalData->m_transforms[s], m_internalData->m_transforms[s], rayResult))
			{
				if (rayResult.m_fraction < closestHitResults)
				{
					closestHitResults = rayResult.m_fraction;

					worldNormal = m_internalData->m_transforms[s].getBasis() * rayResult.m_normal;
					worldNormal.normalize();
					hasHit = true;
				}
			}
		}
	}

	return hasHit;
}

void RaytracerPhysicsSetup::exitPhysics()
{
	if (m_internalData->m_canvas && m_internalData->m_canvasIndex >= 0)
	{
		m_internalData->m_canvas->destroyCanvas(m_internalData->m_canvasIndex);
	}
}

void RaytracerPhysicsSetup::stepSimulation(btScalar deltaTime)
{
	m_internalData->updateTransforms();

	btScalar top = (btScalar)1.f;
	btScalar bottom = (btScalar)-1.f;
	btScalar nearPlane = (btScalar)1.f;

	btScalar tanFov = (top - bottom) * (btScalar)0.5f / nearPlane;

	btScalar fov = (btScalar)2.0 * atan(tanFov);

	float camPos[] = { 5, 0, 0 };
	float camTarget[] = { 0, 0, 0 };

	if (m_app->m_renderer && m_app->m_renderer->getActiveCamera())
	{
		m_app->m_renderer->getActiveCamera()->getCameraPosition(camPos);
		m_app->m_renderer->getActiveCamera()->getCameraTargetPosition(camTarget);
	}

	btVector3 rayFrom = btVector3((btScalar)camPos[0], (btScalar)camPos[1], (btScalar)camPos[2]);

	btVector3 rayForward = (btVector3((btScalar)camTarget[0], (btScalar)camTarget[1], (btScalar)camTarget[2])
		- btVector3((btScalar)camPos[0], (btScalar)camPos[1], (btScalar)camPos[2]));

	rayForward.normalize();
	btScalar farPlane = (btScalar)600.f;
	rayForward *= (btScalar)farPlane;

	btVector3 rightOffset;
	btVector3 vertical(0.f, 1.f, 0.f);
	btVector3 hor;
	hor = rayForward.cross(vertical);
	hor.normalize();
	vertical = hor.cross(rayForward);
	vertical.normalize();

	btScalar tanfov = tan((btScalar)0.5f * fov);

	hor *= (btScalar)2.f * farPlane * tanfov;
	vertical *= (btScalar)2.f * farPlane * tanfov;

	btVector3 rayToCenter = rayFrom + rayForward;

	btVector3 dHor = hor * (btScalar)1.f / (btScalar)(m_internalData->m_width);
	btVector3 dVert = vertical * (btScalar)1.f / (btScalar)(m_internalData->m_height);

	//	int	mode = 0;
	int x, y;

	for (x = 0; x < m_internalData->m_width; x++)
	{
		for (y = 0; y < m_internalData->m_height; y++)
		{
			btVector4 rgba((btScalar)0, (btScalar)0, (btScalar)0, (btScalar)0);
			btVector3 rayTo = rayToCenter - (btScalar)0.5f * hor + (btScalar)0.5f * vertical;
			rayTo += (btScalar)x * dHor;
			rayTo -= (btScalar)y * dVert;
			btVector3 worldNormal(0, 0, 0);
			btVector3 worldPoint(0, 0, 0);

			bool hasHit = false;
			int mode = 0;
			switch (mode)
			{
				case 0:
					hasHit = lowlevelRaytest(rayFrom, rayTo, worldNormal, worldPoint);
					break;
				case 1:
					hasHit = singleObjectRaytest(rayFrom, rayTo, worldNormal, worldPoint);
					break;
				case 2:
					hasHit = worldRaytest(rayFrom, rayTo, worldNormal, worldPoint);
					break;
				default:
				{
				}
			}

			if (hasHit)
			{
				btScalar lightVec0 = worldNormal.dot(btVector3(0, -1, -1));  //0.4f,-1.f,-0.4f));
				btScalar lightVec1 = worldNormal.dot(btVector3(-1, 0, -1));  //-0.4f,-1.f,-0.4f));

				rgba = btVector4(lightVec0, lightVec1, (btScalar)0, (btScalar)1.f);
				rgba.setMin(btVector3(1, 1, 1));
				rgba.setMax(btVector3(0.2, 0.2, 0.2));
				rgba[3] = (btScalar)1.f;
				unsigned char red = (int)rgba[0] * 255;
				unsigned char green = (int)rgba[1] * 255;
				unsigned char blue = (int)rgba[2] * 255;
				unsigned char alpha = 255;
				m_internalData->m_canvas->setPixel(m_internalData->m_canvasIndex, x, y, red, green, blue, alpha);
			}
			else
			{
				//	btVector4 rgba = raytracePicture->getPixel(x,y);
			}
			if (!rgba.length2())
			{
				m_internalData->m_canvas->setPixel(m_internalData->m_canvasIndex, x, y, 255, 0, 0, 255);
			}
		}
	}
	m_internalData->m_canvas->refreshImageData(m_internalData->m_canvasIndex);
}

void RaytracerPhysicsSetup::physicsDebugDraw(int debugDrawFlags)
{
}

bool RaytracerPhysicsSetup::mouseMoveCallback(float x, float y)
{
	return false;
}

bool RaytracerPhysicsSetup::mouseButtonCallback(int button, int state, float x, float y)
{
	return false;
}

bool RaytracerPhysicsSetup::keyboardCallback(int key, int state)
{
	return false;
}

void RaytracerPhysicsSetup::syncPhysicsToGraphics(GraphicsPhysicsBridge& gfxBridge)
{
}

CommonExampleInterface* RayTracerCreateFunc(struct CommonExampleOptions& options)
{
	return new RaytracerPhysicsSetup(options.m_guiHelper->getAppInterface());
}

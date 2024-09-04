#include "GLViewFinalProject.h"

#include "WorldList.h" //This is where we place all of our WOs
#include "ManagerOpenGLState.h" //We can change OpenGL State attributes with this
#include "Axes.h" //We can set Axes to on/off with this
#include "PhysicsEngineODE.h"

//Different WO used by this module
#include "WO.h"
#include "WOStatic.h"
#include "WOStaticPlane.h"
#include "WOStaticTrimesh.h"
#include "WOTrimesh.h"
#include "WOHumanCyborg.h"
#include "WOHumanCal3DPaladin.h"
#include "WOWayPointSpherical.h"
#include "WOLight.h"
#include "WOSkyBox.h"
#include "WOCar1970sBeater.h"
#include "Camera.h"
#include "CameraStandard.h"
#include "CameraChaseActorSmooth.h"
#include "CameraChaseActorAbsNormal.h"
#include "CameraChaseActorRelNormal.h"
#include "Model.h"
#include "ModelDataShared.h"
#include "ModelMesh.h"
#include "ModelMeshDataShared.h"
#include "ModelMeshSkin.h"
#include "WONVStaticPlane.h"
#include "WONVPhysX.h"
#include "WONVDynSphere.h"
#include "WOImGui.h" //GUI Demos also need to #include "AftrImGuiIncludes.h"
#include "AftrImGuiIncludes.h"
#include "AftrGLRendererBase.h"
#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include "PxConfig.h"
#include "extensions\PxDefaultErrorCallback.h"
#include "extensions\PxDefaultAllocator.h"
#include "extensions/PxDefaultStreams.h"
#include "PxPhysicsAPI.h"
#include "cooking/PxCooking.h"
#include <cmath>
#include <GL/glew.h>

using namespace Aftr;
using namespace physx;

physx::PxFoundation* GLViewFinalProject::gFoundation = nullptr;
physx::PxPhysics* GLViewFinalProject::gPhysics = nullptr;
physx::PxMaterial* GLViewFinalProject::gMaterial = nullptr;

WOHeightField* WOHeightField::New(const std::vector<float>& heightData, int numRows, int numCols)
{
    WOHeightField* wo = new WOHeightField(heightData, numRows, numCols);
    return wo;
}

WOHeightField::WOHeightField(const std::vector<float>& heightData, int numRows, int numCols)
    : WOGrid(), IFace(this), heightData(heightData), numRows(numRows), numCols(numCols)
{
    // Create the height field mesh
    createHeightFieldMesh(heightData, numRows, numCols);
}

void WOHeightField::createHeightFieldMesh(const std::vector<float>& heightData, int numRows, int numCols)
{
    // Find the minimum and maximum height values to normalize
    float minHeight = *std::min_element(heightData.begin(), heightData.end());
    float maxHeight = *std::max_element(heightData.begin(), heightData.end());

    std::vector<std::vector<VectorD>> grid(numRows, std::vector<VectorD>(numCols));
    std::vector<std::vector<aftrColor4ub>> colorGrid(numRows, std::vector<aftrColor4ub>(numCols));

    for (int row = 0; row < numRows; ++row)
    {
        for (int col = 0; col < numCols; ++col)
        {
            int index = row * numCols + col;
            grid[row][col] = VectorD(col, row, heightData[index]); // Set height to 0 for flat terrain

            // Normalize height to [0, 1]
            float normalizedHeight = (heightData[index] - minHeight) / (maxHeight - minHeight);

            // Map normalized height to a color
            GLubyte r = static_cast<GLubyte>(normalizedHeight * 255);
            GLubyte g = 255 - r; // Inverse of red
            GLubyte b = static_cast<GLubyte>(128 + normalizedHeight * 127); // Adjust blue based on height
            GLubyte a = 255; // Fully opaque

            colorGrid[row][col] = aftrColor4ub(r, g, b, a);
        }
    }

    this->onCreate(grid, VectorD(1, 1, 1), colorGrid);
}


void WOHeightField::createPhysXHeightField(physx::PxPhysics* physics, physx::PxMaterial* material)
{
    // Create height field samples
    std::vector<physx::PxHeightFieldSample> samples(numRows * numCols);
    for (int i = 0; i < numRows * numCols; ++i)
    {
        samples[i].height = static_cast<physx::PxI16>(heightData[i]);
        samples[i].materialIndex0 = 0;  // Default material
        samples[i].materialIndex1 = 0;  // Default material
        samples[i].clearTessFlag();
    }

    // Create height field descriptor
    physx::PxHeightFieldDesc hfDesc;
    hfDesc.nbRows = numRows;
    hfDesc.nbColumns = numCols;
    hfDesc.samples.data = samples.data();
    hfDesc.samples.stride = sizeof(physx::PxHeightFieldSample);

    // Cook the height field
    this->heightField = PxCreateHeightField(hfDesc, physics->getPhysicsInsertionCallback());

    // Create the height field geometry
    physx::PxHeightFieldGeometry hfGeom(heightField, physx::PxMeshGeometryFlags(), 1.0f, 1.0f, 1.0f);

    // Create the static actor
    physx::PxTransform transform(physx::PxVec3(this->getPosition().x, this->getPosition().y, this->getPosition().z));

    this->heightFieldActor = physics->createRigidStatic(transform);
    physx::PxShape* shape = physics->createShape(hfGeom, *material);
    this->heightFieldActor->attachShape(*shape);
    shape->release();
}

WOPhysX::WOPhysX(WO* wo, physx::PxPhysics* physics, physx::PxMaterial* material, bool isStatic)
    : WO(), IFace(woUsingThisInterface), wo(wo), physics(physics), material(material), actor(nullptr), isStatic(isStatic)
{
    initPhysicsActor();

    // Set the initial global pose of the physics actor to match the initial position of the world object
    physx::PxTransform initialTransform(physx::PxVec3(wo->getPosition().x, wo->getPosition().y, wo->getPosition().z));
    if (wo->getLabel() == "Height Field")
    {
        Aftr::Mat4 mat;
        mat = mat.rotate(Vector(1, 0, 0), Aftr::PI / 2);
        mat = mat.rotate(Vector(0, 0, -1), Aftr::PI / 2);
        PxMat44 m;
        m.column0.x = mat.getX().x;
        m.column0.y = mat.getX().y;
        m.column0.z = mat.getX().z;
        m.column1.x = mat.getY().x;
        m.column1.y = mat.getY().y;
        m.column1.z = mat.getY().z;
        m.column2.x = mat.getZ().x;
        m.column2.y = mat.getZ().y;
        m.column2.z = mat.getZ().z;
        m.column3.x = wo->getPosition().x;
        m.column3.y = wo->getPosition().y;
        m.column3.z = wo->getPosition().z;

        // Create a transform for the collision shape
        PxTransform shapeTransform(m);

        // Apply the rotation to the collision shape
        PxShape* shape;
        actor->getShapes(&shape, 1);
        shape->setLocalPose(shapeTransform);
    }
    else
    {
        actor->setGlobalPose(initialTransform);
    }
}

physx::PxShape* WOPhysX::createWOShape(physx::PxPhysics* physics)
{
    if (wo->getLabel() == "Cube")
    {
        return physics->createShape(physx::PxBoxGeometry(2.0f, 2.0f, 2.0f), *material);
    }
    else if (wo->getLabel() == "Height Field")
    {
        WOHeightField* heightField = dynamic_cast<WOHeightField*>(wo);
        if (heightField)
        {
            return createHeightFieldShape(physics, heightField->getHeightData(), heightField->getNumRows(), heightField->getNumCols());
        }
    }
    else
    {
        // Default shape, flat ground
        return physics->createShape(physx::PxBoxGeometry(10.0f, 10.0f, 1.0f), *material);
    }
}

physx::PxShape* WOPhysX::createHeightFieldShape(physx::PxPhysics* physics, const std::vector<float>& heightData, int numRows, int numCols)
{
    // Create the height field samples from heightData
    std::vector<physx::PxHeightFieldSample> samples(numRows * numCols);
    for (int i = 0; i < numRows * numCols; ++i)
    {
        samples[i].height = static_cast<physx::PxI16>(heightData[i]);
        samples[i].materialIndex0 = 0;  // Default material
        samples[i].materialIndex1 = 0;  // Default material
        samples[i].clearTessFlag();
    }

    // Create the height field descriptor
    physx::PxHeightFieldDesc hfDesc;
    hfDesc.nbRows = numRows;
    hfDesc.nbColumns = numCols;
    hfDesc.samples.data = samples.data();
    hfDesc.samples.stride = sizeof(physx::PxHeightFieldSample);

    // Cook the height field
    PxHeightField* heightField = PxCreateHeightField(hfDesc, physics->getPhysicsInsertionCallback());

    // Adjust the scale of the height field
    physx::PxHeightFieldGeometry hfGeom(heightField, physx::PxMeshGeometryFlags(), 1.0f, 1.0f, 1.0f);

    // Create the shape
    return physics->createShape(hfGeom, *material);
}

void WOPhysX::initPhysicsActor()
{
    physx::PxTransform transform(physx::PxVec3(wo->getPosition().x, wo->getPosition().y, wo->getPosition().z));
    if (isStatic)
    {
        actor = physics->createRigidStatic(transform);
    }
    else
    {
        actor = physics->createRigidDynamic(transform);
        physx::PxRigidBodyExt::updateMassAndInertia(*static_cast<physx::PxRigidDynamic*>(actor), 10.0f);
    }

    physx::PxShape* shape = createWOShape(physics);
    actor->attachShape(*shape);
    shape->release();
}

void WOPhysX::updateFromPhysics()
{
    const physx::PxTransform& transform = actor->getGlobalPose();
    wo->setPose(Quat(transform.q.x, transform.q.y, transform.q.z, transform.q.w));
    wo->setPosition(Vector(transform.p.x, transform.p.y, transform.p.z));
}

void WOPhysX::syncWithPhysics()
{
    physx::PxTransform transform(physx::PxVec3(wo->getPosition().x, wo->getPosition().y, wo->getPosition().z));
    actor->setGlobalPose(transform);
}

GLViewFinalProject* GLViewFinalProject::New( const std::vector< std::string >& args )
{
   GLViewFinalProject* glv = new GLViewFinalProject( args );
   glv->init( Aftr::GRAVITY, Vector( 0, 0, -1.0f ), "aftr.conf", PHYSICS_ENGINE_TYPE::petODE );
   glv->onCreate();
   return glv;
}

void GLViewFinalProject::initRaceTrack()
{
    // Clear any existing control points
    raceTrack.clearControlPoints();

    // Generate random control points
    for (int i = 0; i < 10; ++i) // Adjust the number of control points as needed
    {
        raceTrack.addControlPoint(generateRandomControlPoint());
    }

    // Generate the race track points using Catmull-Rom spline
    generateRaceTrack(raceTrack, raceTrackPoints);
}

float GLViewFinalProject::randomFloat(float min, float max)
{
    return min + static_cast<float>(rand()) / static_cast<float>(RAND_MAX / (max - min));
}

GLViewFinalProject::ControlPoint Aftr::GLViewFinalProject::generateRandomControlPoint()
{
    // Generate a random control point within the specified area
    float x = randomFloat(-70.0f, 70.0f);
    float y = randomFloat(-70.0f, 70.0f);
    float tangentX = randomFloat(-1.0f, 1.0f); // Random tangent (normalized)
    float tangentY = randomFloat(-1.0f, 1.0f); // Random tangent (normalized)
    return ControlPoint(x, y, tangentX, tangentY);
}

void GLViewFinalProject::insertAdditionalControlPoints(std::vector<Vector>& sortedPoints)
{
    const float maxAllowedDistance = 15.0f; // Define a threshold for maximum distance between points

    for (size_t i = 0; i < sortedPoints.size() - 1; ++i)
    {
        Vector& current = sortedPoints[i];
        Vector& next = sortedPoints[i + 1];

        float distance = std::sqrt(squaredDistance(current, next));

        if (distance > maxAllowedDistance)
        {
            // Insert a new control point halfway between the current and next point
            Vector midPoint(
                (current.x + next.x) / 2.0f,
                (current.y + next.y) / 2.0f,
                0.0f
            );
            sortedPoints.insert(sortedPoints.begin() + i + 1, midPoint);
            --i; // Recheck the new segment
        }
    }
}

Vector GLViewFinalProject::catmullRomSpline(const Vector& p0, const Vector& p1, const Vector& p2, const Vector& p3, float t)
{
    float t2 = t * t;
    float t3 = t2 * t;

    float b0 = 0.5f * (-t3 + 2.0f * t2 - t);
    float b1 = 0.5f * (3.0f * t3 - 5.0f * t2 + 3.0f);
    float b2 = 0.5f * (-3.0f * t3 + 4.0f * t2 + t);
    float b3 = 0.5f * (t3 - t2);

    return Vector(
        p0.x * b0 + p1.x * b1 + p2.x * b2 + p3.x * b3,
        p0.y * b0 + p1.y * b1 + p2.y * b2 + p3.y * b3,
        0.0f // For a flat track on the XY plane
    );
}

void GLViewFinalProject::createHeightFieldsFromControlPoints()
{
    // Generate height fields based on the race track points
    for (size_t i = 0; i < raceTrackPoints.size(); ++i)
    {
        // Create height data for the height field
        std::vector<float> heightData;
        int numRows = 15; // Adjust as needed
        int numCols = 15; // Adjust as needed

        // Fill heightData with appropriate values
        for (int row = 0; row < numRows; ++row)
        {
            for (int col = 0; col < numCols; ++col)
            {
                float height = 0;
                heightData.push_back(height);
            }
        }

        // Create a new height field
        WOHeightField* heightField = WOHeightField::New(heightData, numRows, numCols);
        heightField->setPosition(raceTrackPoints[i]);

        // Add the height field to the scene
        woStaticPhysX.push_back(heightField);
        worldLst->push_back(heightField);
    }
   
}

float GLViewFinalProject::squaredDistance(const Vector& a, const Vector& b)
{
    return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
}

void GLViewFinalProject::generateRaceTrack(Track& track, std::vector<Vector>& raceTrackPoints)
{
    raceTrackPoints.clear();

    // Copy the control points to a temporary list
    std::vector<Vector> controlPoints;
    for (const auto& cp : track.controlPoints)
    {
        controlPoints.emplace_back(cp.x, cp.y, 0);
    }

    // Sort the control points to form a nearest-neighbor path
    std::vector<Vector> sortedPoints;
    sortedPoints.push_back(controlPoints.front()); // Start with the first point
    controlPoints.erase(controlPoints.begin());    // Remove it from the list

    while (!controlPoints.empty())
    {
        const Vector& current = sortedPoints.back();
        auto nearestIt = std::min_element(controlPoints.begin(), controlPoints.end(),
            [this, &current](const Vector& a, const Vector& b) {
                return squaredDistance(current, a) < squaredDistance(current, b);
            });

        sortedPoints.push_back(*nearestIt);
        controlPoints.erase(nearestIt);
    }

    // Re-add the first point at the end to close the loop
    sortedPoints.push_back(sortedPoints.front());

    // Insert additional control points if gaps are detected
    insertAdditionalControlPoints(sortedPoints);

    // Interpolate the sorted control points using the Catmull-Rom spline
    for (size_t i = 0; i < sortedPoints.size() - 1; ++i)
    {
        const auto& p0 = sortedPoints[(i - 1 + sortedPoints.size()) % sortedPoints.size()];
        const auto& p1 = sortedPoints[i];
        const auto& p2 = sortedPoints[(i + 1) % sortedPoints.size()];
        const auto& p3 = sortedPoints[(i + 2) % sortedPoints.size()];

        for (float t = 0; t < 1.0f; t += 0.1f)
        {
            raceTrackPoints.push_back(catmullRomSpline(p0, p1, p2, p3, t));
        }
    }

    // Create height fields from the generated race track points
    createHeightFieldsFromControlPoints();
}

GLViewFinalProject::GLViewFinalProject( const std::vector< std::string >& args ) : GLView( args )
{
   //Initialize any member variables that need to be used inside of LoadMap() here.
   //Note: At this point, the Managers are not yet initialized. The Engine initialization
   //occurs immediately after this method returns (see GLViewFinalProject::New() for
   //reference). Then the engine invoke's GLView::loadMap() for this module.
   //After loadMap() returns, GLView::onCreate is finally invoked.

   //The order of execution of a module startup:
   //GLView::New() is invoked:
   //    calls GLView::init()
   //       calls GLView::loadMap() (as well as initializing the engine's Managers)
   //    calls GLView::onCreate()

   //GLViewFinalProject::onCreate() is invoked after this module's LoadMap() is completed.
    gPhysics = nullptr;
    gScene = nullptr;
    gDispatcher = nullptr;
    gMaterial = nullptr;
    gFoundation = nullptr;
    gPvd = nullptr;
}

void GLViewFinalProject::initPhysX()
{
    static PxDefaultErrorCallback gErrorCallback;
    static PxDefaultAllocator gAllocator;
    gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
    gPvd = PxCreatePvd(*gFoundation);
    PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);
    gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);
    gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);
    PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
    sceneDesc.gravity = PxVec3(0.0f, 0.0f, -25.0f);
    gDispatcher = PxDefaultCpuDispatcherCreate(2);
    sceneDesc.cpuDispatcher = gDispatcher;
    sceneDesc.filterShader = PxDefaultSimulationFilterShader;
    gScene = gPhysics->createScene(sceneDesc);
    gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);
    PxInitExtensions(*gPhysics, gPvd);

    PxCookingParams cookingParams(gPhysics->getTolerancesScale());
}

void GLViewFinalProject::createPhysXActors()
{
    for (auto& wo : woDynamicPhysX)
    {
        WOPhysX* woPhysX = new WOPhysX(wo, gPhysics, gMaterial, false); // Dynamic actor
        woPhysXObjects.push_back(woPhysX);
        gScene->addActor(*woPhysX->getActor());
    }

    for (auto& wo : woStaticPhysX)
    {
        if (wo->getLabel() != "Grass")  // Grass already defined in PhysX
        {
            WOPhysX* woPhysX = new WOPhysX(wo, gPhysics, gMaterial, true); // Static actor
            woPhysXObjects.push_back(woPhysX);
            gScene->addActor(*woPhysX->getActor());
        }
    }
}

void GLViewFinalProject::onCreate()
{
   //GLViewFinalProject::onCreate() is invoked after this module's LoadMap() is completed.
   //At this point, all the managers are initialized. That is, the engine is fully initialized.

   if( this->pe != NULL )
   {
      //optionally, change gravity direction and magnitude here
      //The user could load these values from the module's aftr.conf
      this->pe->setGravityNormalizedVector( Vector( 0,0,-1.0f ) );
      this->pe->setGravityScalar( Aftr::GRAVITY );
   }
   this->setActorChaseType( STANDARDEZNAV ); //Default is STANDARDEZNAV mode
   //this->setNumPhysicsStepsPerRender( 0 ); //pause physics engine on start up; will remain paused till set to 1
}


GLViewFinalProject::~GLViewFinalProject()
{
   //Implicitly calls GLView::~GLView()
    gScene->release();
    gDispatcher->release();
    gPhysics->release();
    gFoundation->release();
    for (auto woPhysX : woPhysXObjects)
    {
        delete woPhysX;
    }
}


void GLViewFinalProject::updateWorld()
{
   GLView::updateWorld(); //Just call the parent's update world first.
                          //If you want to add additional functionality, do it after
                          //this call.
   updatePhysX();

   for (auto& woPhysX : woPhysXObjects)
   {
       woPhysX->updateFromPhysics();
   }
}

void GLViewFinalProject::updatePhysX()
{
    gScene->simulate(1.0f / 60.0f);
    gScene->fetchResults(true);
}


std::tuple<std::vector<float>, int, int> GLViewFinalProject::createHeightField()
{
    // Define the heightfield dimensions
    const PxU32 numRows = 100;
    const PxU32 numCols = 100;

    // Create the heightfield samples
    std::vector<PxHeightFieldSample> samples(numRows * numCols);
    std::vector<float> heightData(numRows * numCols);
    for (PxU32 row = 0; row < numRows; ++row)
    {
        for (PxU32 col = 0; col < numCols; ++col)
        {
            PxHeightFieldSample& sample = samples[row * numCols + col];
            float multiplier = 4.0f;
            sample.height = static_cast<PxI16>(multiplier * sinf(row * 0.5f) * cosf(col * 0.5f)); // Test function using sine and cosine
            sample.materialIndex0 = 0;
            sample.materialIndex1 = 0;
            heightData[row * numCols + col] = static_cast<float>(sample.height);
        }
    }

    return std::make_tuple(heightData, numRows, numCols);
}

void GLViewFinalProject::addHeightFieldToScene()
{
    // Create height field data
    auto [heightData, numRows, numCols] = createHeightField();

    // Create WOHeightField object
    WOHeightField* woHeightField = WOHeightField::New(heightData, numRows, numCols);
    woHeightField->setPosition(Vector(15, 0, 0));
    woHeightField->setLabel("Height Field");
    woHeightField->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    woHeightField->createPhysXHeightField(gPhysics, gMaterial);

    // Add WOHeightField to the world list
    worldLst->push_back(woHeightField);
    woStaticPhysX.push_back(woHeightField);
}

void GLViewFinalProject::onResizeWindow( GLsizei width, GLsizei height )
{
   GLView::onResizeWindow( width, height ); //call parent's resize method.
}


void GLViewFinalProject::onMouseDown( const SDL_MouseButtonEvent& e )
{
   GLView::onMouseDown( e );
}


void GLViewFinalProject::onMouseUp( const SDL_MouseButtonEvent& e )
{
   GLView::onMouseUp( e );
}


void GLViewFinalProject::onMouseMove( const SDL_MouseMotionEvent& e )
{
   GLView::onMouseMove( e );
}


void GLViewFinalProject::onKeyDown(const SDL_KeyboardEvent& key)
{
    GLView::onKeyDown(key);
    if (key.keysym.sym == SDLK_0)
        this->setNumPhysicsStepsPerRender(1);

    // Define the force to be applied
    physx::PxVec3 force(0.0f, 0.0f, 0.0f);

    // Determine the force direction based on the key pressed
    switch (key.keysym.sym)
    {
    case SDLK_UP: // Forward
        force = physx::PxVec3(-75.0f, 0.0f, 0.0f);
        break;
    case SDLK_DOWN: // Backward
        force = physx::PxVec3(75.0f, 0.0f, 0.0f);
        break;
    case SDLK_RIGHT: // Right
        force = physx::PxVec3(0.0f, 75.0f, 0.0f);
        break;
    case SDLK_LEFT: // Left
        force = physx::PxVec3(0.0f, -75.0f, 0.0f);
        break;
    case SDLK_SPACE: // Up
        force = physx::PxVec3(0.0f, 0.0f, 100.0f);
        break;
    default:
        break;
    }

    // Apply the force to all dynamic physics objects
    for (WOPhysX* woPhysX : woPhysXObjects)
    {
        if (!woPhysX->isStaticActor())
        {
            physx::PxRigidDynamic* dynamicActor = woPhysX->getActor()->is<physx::PxRigidDynamic>();
            if (dynamicActor)
            {
                dynamicActor->addForce(force);
            }
        }
    }
}


void GLViewFinalProject::onKeyUp( const SDL_KeyboardEvent& key )
{
   GLView::onKeyUp( key );
}


void Aftr::GLViewFinalProject::loadMap()
{
    this->worldLst = new WorldList(); //WorldList is a 'smart' vector that is used to store WO*'s
    this->actorLst = new WorldList();
    this->netLst = new WorldList();

    ManagerOpenGLState::GL_CLIPPING_PLANE = 1000.0;
    ManagerOpenGLState::GL_NEAR_PLANE = 0.1f;
    ManagerOpenGLState::enableFrustumCulling = false;
    Axes::isVisible = true;
    this->glRenderer->isUsingShadowMapping(true); //set to TRUE to enable shadow mapping, must be using GL 3.2+

    this->cam->setPosition(15, 15, 10);

    this->initPhysX();

    std::string shinyRedPlasticCube(ManagerEnvironmentConfiguration::getSMM() + "/models/cube4x4x4redShinyPlastic_pp.wrl");
    std::string wheeledCar(ManagerEnvironmentConfiguration::getSMM() + "/models/rcx_treads.wrl");
    std::string grass(ManagerEnvironmentConfiguration::getSMM() + "/models/grassFloor400x400_pp.wrl");
    std::string human(ManagerEnvironmentConfiguration::getSMM() + "/models/human_chest.wrl");

    //SkyBox Textures readily available
    std::vector< std::string > skyBoxImageNames; //vector to store texture paths
    //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_water+6.jpg" );
    //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_dust+6.jpg" );
    //skyBoxImageNames.push_back(ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_mountains+6.jpg");
    //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_winter+6.jpg" );
    //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/early_morning+6.jpg" );
    //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_afternoon+6.jpg" );
    //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_cloudy+6.jpg" );
    //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_cloudy3+6.jpg" );
    //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_day+6.jpg" );
    //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_day2+6.jpg" );
    //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_deepsun+6.jpg" );
    //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_evening+6.jpg" );
    //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_morning+6.jpg" );
    //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_morning2+6.jpg" );
    //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_noon+6.jpg" );
    //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_warp+6.jpg" );
    //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_Hubble_Nebula+6.jpg" );
    //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_gray_matter+6.jpg" );
    //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_easter+6.jpg" );
    //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_hot_nebula+6.jpg" );
    //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_ice_field+6.jpg" );
    //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_lemon_lime+6.jpg" );
    //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_milk_chocolate+6.jpg" );
    //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_solar_bloom+6.jpg" );
    skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_thick_rb+6.jpg" );

    {
        //Create a light
        float ga = 0.1f; //Global Ambient Light level for this module
        ManagerLight::setGlobalAmbientLight(aftrColor4f(ga, ga, ga, 1.0f));
        WOLight* light = WOLight::New();
        light->isDirectionalLight(true);
        light->setPosition(Vector(0, 0, 100));
        //Set the light's display matrix such that it casts light in a direction parallel to the -z axis (ie, downwards as though it was "high noon")
        //for shadow mapping to work, this->glRenderer->isUsingShadowMapping( true ), must be invoked.
        light->getModel()->setDisplayMatrix(Mat4::rotateIdentityMat({ 0, 1, 0 }, 90.0f * Aftr::DEGtoRAD));
        light->setLabel("Light");
        worldLst->push_back(light);
    }

    {
        //Create the SkyBox
        WO* wo = WOSkyBox::New(skyBoxImageNames.at(0), this->getCameraPtrPtr());
        wo->setPosition(Vector(0, 0, 0));
        wo->setLabel("Sky Box");
        wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
        worldLst->push_back(wo);
    }

    initRaceTrack();

    WO* wo = WO::New(shinyRedPlasticCube, Vector(1, 1, 1), MESH_SHADING_TYPE::mstFLAT);
    wo->setPosition(raceTrackPoints[0].x, raceTrackPoints[0].y, raceTrackPoints[0].z + 5.0f);
    wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    wo->setLabel("Cube");
    worldLst->push_back(wo);
    woDynamicPhysX.push_back(wo);

    {
        //////Create the infinite grass plane (the floor)
        //WO* wo = WO::New(grass, Vector(1, 1, 1), MESH_SHADING_TYPE::mstFLAT);
        //wo->setPosition(Vector(0, 0, 0));
        //wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;ww
        //wo->upon_async_model_loaded([wo]()
        //    {
        //        ModelMeshSkin& grassSkin = wo->getModel()->getModelDataShared()->getModelMeshes().at(0)->getSkins().at(0);
        //        grassSkin.getMultiTextureSet().at(0).setTexRepeats(5.0f);
        //        grassSkin.setAmbient(aftrColor4f(0.4f, 0.4f, 0.4f, 1.0f)); //Color of object when it is not in any light
        //        grassSkin.setDiffuse(aftrColor4f(1.0f, 1.0f, 1.0f, 1.0f)); //Diffuse color components (ie, matte shading color of this object)
        //        grassSkin.setSpecular(aftrColor4f(0.4f, 0.4f, 0.4f, 1.0f)); //Specular color component (ie, how "shiney" it is)
        //        grassSkin.setSpecularCoefficient(10); // How "sharp" are the specular highlights (bigger is sharper, 1000 is very sharp, 10 is very dull)
        //    });
        //wo->setLabel("Grass");
        //worldLst->push_back(wo);
    }

    //{
    //   //Create the infinite grass plane that uses the Open Dynamics Engine (ODE)
    //   WO* wo = WOStatic::New( grass, Vector(1,1,1), MESH_SHADING_TYPE::mstFLAT );
    //   ((WOStatic*)wo)->setODEPrimType( ODE_PRIM_TYPE::PLANE );
    //   wo->setPosition( Vector(0,0,0) );
    //   wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    //   wo->getModel()->getModelDataShared()->getModelMeshes().at(0)->getSkins().at(0).getMultiTextureSet().at(0)->setTextureRepeats( 5.0f );
    //   wo->setLabel( "Grass" );
    //   worldLst->push_back( wo );
    //}

    //{
    //   //Create the infinite grass plane that uses NVIDIAPhysX(the floor)
    //   WO* wo = WONVStaticPlane::New( grass, Vector( 1, 1, 1 ), MESH_SHADING_TYPE::mstFLAT );
    //   wo->setPosition( Vector( 0, 0, 0 ) );
    //   wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    //   wo->getModel()->getModelDataShared()->getModelMeshes().at( 0 )->getSkins().at( 0 ).getMultiTextureSet().at( 0 )->setTextureRepeats( 5.0f );
    //   wo->setLabel( "Grass" );
    //   worldLst->push_back( wo );
    //}

    //{
    //   //Create the infinite grass plane (the floor)
    //   WO* wo = WONVPhysX::New( shinyRedPlasticCube, Vector( 1, 1, 1 ), MESH_SHADING_TYPE::mstFLAT );
    //   wo->setPosition( Vector( 0, 0, 50.0f ) );
    //   wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    //   wo->setLabel( "Grass" );
    //   worldLst->push_back( wo );
    //}

    //{
    //   WO* wo = WONVPhysX::New( shinyRedPlasticCube, Vector( 1, 1, 1 ), MESH_SHADING_TYPE::mstFLAT );
    //   wo->setPosition( Vector( 0, 0.5f, 75.0f ) );
    //   wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    //   wo->setLabel( "Grass" );
    //   worldLst->push_back( wo );
    //}

    //{
    //   WO* wo = WONVDynSphere::New( ManagerEnvironmentConfiguration::getVariableValue( "sharedmultimediapath" ) + "/models/sphereRp5.wrl", Vector( 1.0f, 1.0f, 1.0f ), mstSMOOTH );
    //   wo->setPosition( 0, 0, 100.0f );
    //   wo->setLabel( "Sphere" );
    //   this->worldLst->push_back( wo );
    //}

    //{
    //   WO* wo = WOHumanCal3DPaladin::New( Vector( .5, 1, 1 ), 100 );
    //   ((WOHumanCal3DPaladin*)wo)->rayIsDrawn = false; //hide the "leg ray"
    //   ((WOHumanCal3DPaladin*)wo)->isVisible = false; //hide the Bounding Shell
    //   wo->setPosition( Vector( 20, 20, 20 ) );
    //   wo->setLabel( "Paladin" );
    //   worldLst->push_back( wo );
    //   actorLst->push_back( wo );
    //   netLst->push_back( wo );
    //   this->setActor( wo );
    //}
    //
    //{
    //   WO* wo = WOHumanCyborg::New( Vector( .5, 1.25, 1 ), 100 );
    //   wo->setPosition( Vector( 20, 10, 20 ) );
    //   wo->isVisible = false; //hide the WOHuman's bounding box
    //   ((WOHuman*)wo)->rayIsDrawn = false; //show the 'leg' ray
    //   wo->setLabel( "Human Cyborg" );
    //   worldLst->push_back( wo );
    //   actorLst->push_back( wo ); //Push the WOHuman as an actor
    //   netLst->push_back( wo );
    //   this->setActor( wo ); //Start module where human is the actor
    //}

    //{
    //   //Create and insert the WOWheeledVehicle
    //   std::vector< std::string > wheels;
    //   std::string wheelStr( "../../../shared/mm/models/WOCar1970sBeaterTire.wrl" );
    //   wheels.push_back( wheelStr );
    //   wheels.push_back( wheelStr );
    //   wheels.push_back( wheelStr );
    //   wheels.push_back( wheelStr );
    //   WO* wo = WOCar1970sBeater::New( "../../../shared/mm/models/WOCar1970sBeater.wrl", wheels );
    //   wo->setPosition( Vector( 5, -15, 20 ) );
    //   wo->setLabel( "Car 1970s Beater" );
    //   ((WOODE*)wo)->mass = 200;
    //   worldLst->push_back( wo );
    //   actorLst->push_back( wo );
    //   this->setActor( wo );
    //   netLst->push_back( wo );
    //}

 
   
   //// Create the CurrentPosition WOImGui object
   // WOImGui* currentPosition = WOImGui::New();
   // currentPosition->setLabel("Current Position");
   // currentPosition->setPosition(Vector(0, 0, 0));
   // worldLst->push_back(currentPosition);

    //std::vector<ControlPoint> controlPoints; // Vector to store ControlPoints
    //std::vector<WO*> testCubes; // Vector to store testCube objects
    //WO* raceTrackWO = WO::New();
    //raceTrackWO->setPosition(Vector(0, 0, 0));
    //worldLst->push_back(raceTrackWO);

    //for (int i = 0; i < 10; i++) {
    //    ControlPoint controlPoint = generateRandomControlPoint();
    //    controlPoints.push_back(controlPoint);

    //    WO* testCube = WO::New(shinyRedPlasticCube);
    //    testCube->setPosition(Vector(controlPoint.x, controlPoint.y, 0));
    //    testCubes.push_back(testCube);
    //    worldLst->push_back(testCube);
    //}

    createPhysXActors();

    createFinalProjectWayPoints();
}


void GLViewFinalProject::createFinalProjectWayPoints()
{
   // Create a waypoint with a radius of 3, a frequency of 5 seconds, activated by GLView's camera, and is visible.
   WayPointParametersBase params(this);
   params.frequency = 5000;
   params.useCamera = true;
   params.visible = false;
   WOWayPointSpherical* wayPt = WOWayPointSpherical::New( params, 3 );
   wayPt->setPosition( Vector( 50, 0, 3 ) );
   worldLst->push_back( wayPt );
}

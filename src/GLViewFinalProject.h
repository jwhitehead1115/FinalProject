#pragma once

#include "GLView.h"
#include "PxPhysicsAPI.h"
#include "cooking/PxCooking.h"
#include "WO.h"
#include <vector>
#include <cmath>
#include <GL/glew.h>
#include "WOGrid.h"
#include "gdal_priv.h"
#include "errno.h"

namespace Aftr
{
   class Camera;

/**
   \class GLViewFinalProject
   \author Scott Nykl 
   \brief A child of an abstract GLView. This class is the top-most manager of the module.

   Read \see GLView for important constructor and init information.

   \see GLView

    \{
*/

class WOHeightField : public WOGrid
{
public:
    static WOHeightField* New(const std::vector<float>& heightData, int numRows, int numCols);
    void createPhysXHeightField(physx::PxPhysics* physics, physx::PxMaterial* material);


    physx::PxRigidStatic* getActor() { return heightFieldActor; }
    const std::vector<float>& getHeightData() const { return heightData; }
    int getNumRows() const { return numRows; }
    int getNumCols() const { return numCols; }

protected:
    WOHeightField(const std::vector<float>& heightData, int numRows, int numCols);
    void createHeightFieldMesh(const std::vector<float>& heightData, int numRows, int numCols);

    std::vector<float> heightData;
    int numRows;
    int numCols;
    physx::PxRigidStatic* heightFieldActor = nullptr;
    physx::PxHeightField* heightField = nullptr;
};

class WOPhysX : public WO
{
public:
    WOPhysX(WO* wo, physx::PxPhysics* physics, physx::PxMaterial* material, bool isStatic);

    void initPhysicsActor();
    void updateFromPhysics();
    void syncWithPhysics();
    physx::PxShape* createWOShape(physx::PxPhysics* physics);
    physx::PxRigidActor* getActor() { return actor;}
    physx::PxShape* createHeightFieldShape(physx::PxPhysics* physics, const std::vector<float>& heightData, int numRows, int numCols);
    bool isStaticActor() const { return isStatic; }


protected:
    WO* wo;
    physx::PxRigidActor* actor;
    physx::PxPhysics* physics;
    physx::PxMaterial* material;
    bool isStatic = true;
};

class GLViewFinalProject : public GLView
{
public:
   static GLViewFinalProject* New( const std::vector< std::string >& outArgs );
   virtual ~GLViewFinalProject();
   virtual void updateWorld(); ///< Called once per frame
   virtual void loadMap(); ///< Called once at startup to build this module's scene
   virtual void createFinalProjectWayPoints();
   virtual void onResizeWindow( GLsizei width, GLsizei height );
   virtual void onMouseDown( const SDL_MouseButtonEvent& e );
   virtual void onMouseUp( const SDL_MouseButtonEvent& e );
   virtual void onMouseMove( const SDL_MouseMotionEvent& e );
   virtual void onKeyDown( const SDL_KeyboardEvent& key );
   virtual void onKeyUp( const SDL_KeyboardEvent& key );

   class ControlPoint {
   public:
       float x;
       float y;
       float tangentX;
       float tangentY;

       ControlPoint(float x, float y, float tangentX, float tangentY)
           : x(x), y(y), tangentX(tangentX), tangentY(tangentY) {}

       ControlPoint() = default;
   };

   float randomFloat(float min, float max);
   ControlPoint generateRandomControlPoint();
   void createHeightFieldsFromControlPoints();
   float squaredDistance(const Vector& a, const Vector& b);


   class Track {
   public:
       std::vector<ControlPoint> controlPoints;

       void addControlPoint(const ControlPoint& point) {
           controlPoints.push_back(point);
       }

       void clearControlPoints() {
           controlPoints.clear();
       }

       ControlPoint getControlPoint(int index) {
           return controlPoints[index];
       }
   };

   void insertAdditionalControlPoints(std::vector<Vector>& sortedPoints);
   Vector catmullRomSpline(const Vector& p0, const Vector& p1, const Vector& p2, const Vector& p3, float t);
   void generateRaceTrack(Track& track, std::vector<Vector>& raceTrackPoints);
   void initRaceTrack();


protected:
   GLViewFinalProject( const std::vector< std::string >& args );
   virtual void onCreate();
   void initPhysX();
   void createPhysXActors();
   void updatePhysX();
   static physx::PxPhysics* gPhysics;
   physx::PxScene* gScene;
   physx::PxDefaultCpuDispatcher* gDispatcher;
   static physx::PxMaterial* gMaterial;
   static physx::PxFoundation* gFoundation;
   physx::PxPvd* gPvd;
   std::vector<WOPhysX*> woPhysXObjects;
   std::vector<WO*> woStaticPhysX;
   std::vector<WO*> woDynamicPhysX;
   std::tuple<std::vector<float>, int, int>createHeightField();
   void addHeightFieldToScene();
   std::vector<Vector> raceTrackPoints;
   Track raceTrack;
};

/** \} */

} //namespace Aftr

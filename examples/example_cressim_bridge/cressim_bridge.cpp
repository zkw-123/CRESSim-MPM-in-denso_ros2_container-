#include <iostream>
#include <vector>

#include "debug_logger.h"
#include "particle_object.h"
#include "simulation_factory.h"
#include "transform.h"

class CressimBridgeSim
{
public:
    CressimBridgeSim()
    {
        initialize();
    }

    ~CressimBridgeSim()
    {
        release();
    }

    void setCutterPose(float x, float y, float z)
    {
        mCutterTransform.position = crmpm::Vec3f(x, y, z);
    }

    void step(float dt)
    {
        if (!mScene || !mCutterShape)
        {
            return;
        }

        if (dt <= 0.0f)
        {
            return;
        }

        mCutterShape->setKinematicTarget(mCutterTransform, dt);

        mScene->advance(dt);
        mScene->fetchResults();

        updateTargetCentroid();
    }

    void getTargetCentroid(float *outXyz) const
    {
        if (!outXyz)
        {
            return;
        }

        outXyz[0] = mLatestCentroidX;
        outXyz[1] = mLatestCentroidY;
        outXyz[2] = mLatestCentroidZ;
    }

    int getTargetParticleCount() const
    {
        return static_cast<int>(mTargetIndices.size());
    }

    int getParticleCount() const
    {
        if (!mScene)
        {
            return 0;
        }

        const crmpm::ParticleData &particles = mScene->getParticleData();
        return particles.size;
    }

    int copyParticlePositions(float *outXyz, int maxParticles) const
    {
        if (!mScene || !outXyz || maxParticles <= 0)
        {
            return 0;
        }

        const crmpm::ParticleData &particles = mScene->getParticleData();
        const int count = particles.size < maxParticles ? particles.size : maxParticles;

        for (int i = 0; i < count; i++)
        {
            const auto p = particles.positionMass[i];
            outXyz[3 * i + 0] = p.x;
            outXyz[3 * i + 1] = p.y;
            outXyz[3 * i + 2] = p.z;
        }

        return count;
    }

private:
    void initialize()
    {
        const float dt = 0.02f;

        const float blockCenterX = 0.0f;
        const float blockCenterY = 0.5f;
        const float blockCenterZ = 0.0f;

        const float cutterStartX = 0.0f;
        const float cutterStartY = 1.2f;
        const float cutterStartZ = 0.0f;

        const float ringRadius = 0.35f;
        const float edgeRadius = 0.05f;

        const float targetCenterX = blockCenterX;
        const float targetCenterY = blockCenterY;
        const float targetCenterZ = blockCenterZ;
        const float targetRadius = 0.25f;

        mFactory = crmpm::createFactory(10000, 4, 4, 15000, true);

        crmpm::SceneDesc sceneDesc;
        sceneDesc.solverType = crmpm::MpmSolverType::eGpuMlsMpmSolver;
        sceneDesc.numMaxParticles = 10000;

        // No gravity. The block is already placed near the working height.
        sceneDesc.gravity = crmpm::Vec3f(0.0f, 0.0f, 0.0f);

        sceneDesc.gridBounds = crmpm::Bounds3(
            crmpm::Vec3f(-5.0f, -0.5f, -5.0f),
            crmpm::Vec3f(5.0f, 5.0f, 5.0f)
        );
        sceneDesc.gridCellSize = 0.2f;
        sceneDesc.solverIntegrationStepSize = 0.002f;
        sceneDesc.solverIterations = 20;

        mScene = mFactory->createScene(sceneDesc);

        constexpr float E = 1e5f;
        constexpr float nu = 0.4f;
        constexpr float lambda = (E * nu) / ((1.0f + nu) * (1.0f - 2.0f * nu));
        constexpr float mu = E / (2.0f * (1.0f + nu));

        crmpm::Geometry *blockGeometry =
            mFactory->createGeometry(
                crmpm::GeometryType::eBox,
                make_float4(1, 1, 1, 0)
            );

        crmpm::ParticleObjectDesc poDesc;
        poDesc.particleSpacing = 0.1f;
        poDesc.particleMass = 1.0f;
        poDesc.geometry = blockGeometry;
        poDesc.position = crmpm::Vec3f(blockCenterX, blockCenterY, blockCenterZ);
        poDesc.rotation = crmpm::Quat();
        poDesc.invScale = crmpm::Vec3f(2, 2, 2);
        poDesc.materialType = crmpm::ParticleMaterialType::eCoRotational;
        poDesc.materialParams = make_float4(lambda, mu, 0, 0);

        mParticleObject = mFactory->createParticleObject(poDesc);
        blockGeometry->release();

        if (mParticleObject)
        {
            mParticleObject->addToScene(*mScene);
        }

        crmpm::Geometry *ringGeometry =
            mFactory->createGeometry(
                crmpm::GeometryType::eRingSlicer,
                make_float4(ringRadius, edgeRadius, 0, 0)
            );

        mCutterTransform = crmpm::Transform();
        mCutterTransform.position = crmpm::Vec3f(cutterStartX, cutterStartY, cutterStartZ);

        // Same orientation convention as the original slicer example.
        mCutterTransform.rotation = crmpm::Quat(0, 0, 0.7071068f, 0.7071068f);

        crmpm::ShapeDesc shapeDesc;
        shapeDesc.type = crmpm::ShapeType::eKinematic;
        shapeDesc.transform = mCutterTransform;
        shapeDesc.sdfSmoothDistance = 0.0f;
        shapeDesc.sdfFatten = 0.1f;
        shapeDesc.drag = 0.5f;
        shapeDesc.friction = 0.0f;
        shapeDesc.invScale = crmpm::Vec3f(1, 1, 1);

        mCutterShape = mFactory->createShape(*ringGeometry, shapeDesc);
        ringGeometry->release();

        if (mCutterShape)
        {
            mScene->addShape(mCutterShape);
        }

        mScene->advance(dt);
        mScene->fetchResults();

        initializeTargetParticleGroup(
            targetCenterX,
            targetCenterY,
            targetCenterZ,
            targetRadius
        );

        updateTargetCentroid();
    }

    void initializeTargetParticleGroup(
        float centerX,
        float centerY,
        float centerZ,
        float radius)
    {
        mTargetIndices.clear();

        if (!mScene)
        {
            return;
        }

        const crmpm::ParticleData &particles = mScene->getParticleData();

        for (int i = 0; i < particles.size; i++)
        {
            const auto p = particles.positionMass[i];

            const float dx = p.x - centerX;
            const float dy = p.y - centerY;
            const float dz = p.z - centerZ;
            const float dist2 = dx * dx + dy * dy + dz * dz;

            if (dist2 <= radius * radius)
            {
                mTargetIndices.push_back(i);
            }
        }
    }

    void updateTargetCentroid()
    {
        if (!mScene)
        {
            return;
        }

        const crmpm::ParticleData &particles = mScene->getParticleData();

        float centroidX = 0.0f;
        float centroidY = 0.0f;
        float centroidZ = 0.0f;
        int validCount = 0;

        for (int idx : mTargetIndices)
        {
            if (idx < 0 || idx >= particles.size)
            {
                continue;
            }

            const auto p = particles.positionMass[idx];

            centroidX += p.x;
            centroidY += p.y;
            centroidZ += p.z;
            validCount++;
        }

        if (validCount > 0)
        {
            centroidX /= static_cast<float>(validCount);
            centroidY /= static_cast<float>(validCount);
            centroidZ /= static_cast<float>(validCount);
        }

        mLatestCentroidX = centroidX;
        mLatestCentroidY = centroidY;
        mLatestCentroidZ = centroidZ;
    }

    void release()
    {
        if (mCutterShape)
        {
            mCutterShape->release();
            mCutterShape = nullptr;
        }

        if (mParticleObject)
        {
            mParticleObject->release();
            mParticleObject = nullptr;
        }

        if (mScene)
        {
            mScene->release();
            mScene = nullptr;
        }

        if (mFactory)
        {
            crmpm::releaseFactory(mFactory);
            mFactory = nullptr;
        }
    }

private:
    crmpm::SimulationFactory *mFactory = nullptr;
    crmpm::Scene *mScene = nullptr;
    crmpm::ParticleObject *mParticleObject = nullptr;
    crmpm::Shape *mCutterShape = nullptr;

    crmpm::Transform mCutterTransform;

    std::vector<int> mTargetIndices;

    float mLatestCentroidX = 0.0f;
    float mLatestCentroidY = 0.0f;
    float mLatestCentroidZ = 0.0f;
};

extern "C"
{
    void *cressim_create()
    {
        CressimBridgeSim *sim = new CressimBridgeSim();
        return static_cast<void *>(sim);
    }

    void cressim_set_cutter_pose(void *handle, float x, float y, float z)
    {
        if (!handle)
        {
            return;
        }

        CressimBridgeSim *sim = static_cast<CressimBridgeSim *>(handle);
        sim->setCutterPose(x, y, z);
    }

    void cressim_step(void *handle, float dt)
    {
        if (!handle)
        {
            return;
        }

        CressimBridgeSim *sim = static_cast<CressimBridgeSim *>(handle);
        sim->step(dt);
    }

    void cressim_get_target_centroid(void *handle, float *outXyz)
    {
        if (!handle)
        {
            return;
        }

        CressimBridgeSim *sim = static_cast<CressimBridgeSim *>(handle);
        sim->getTargetCentroid(outXyz);
    }

    int cressim_get_target_particle_count(void *handle)
    {
        if (!handle)
        {
            return 0;
        }

        CressimBridgeSim *sim = static_cast<CressimBridgeSim *>(handle);
        return sim->getTargetParticleCount();
    }

    int cressim_get_particle_count(void *handle)
    {
        if (!handle)
        {
            return 0;
        }

        CressimBridgeSim *sim = static_cast<CressimBridgeSim *>(handle);
        return sim->getParticleCount();
    }

    int cressim_get_particle_positions(void *handle, float *outXyz, int maxParticles)
    {
        if (!handle)
        {
            return 0;
        }

        CressimBridgeSim *sim = static_cast<CressimBridgeSim *>(handle);
        return sim->copyParticlePositions(outXyz, maxParticles);
    }

    void cressim_destroy(void *handle)
    {
        if (!handle)
        {
            return;
        }

        CressimBridgeSim *sim = static_cast<CressimBridgeSim *>(handle);
        delete sim;
    }
}

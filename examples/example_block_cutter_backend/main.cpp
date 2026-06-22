#include <iostream>
#include <vector>

#include "debug_logger.h"
#include "particle_object.h"
#include "simulation_factory.h"
#include "transform.h"

int main()
{
    const float dt = 0.02f;
    const int numSteps = 300;

    // Block placement.
    // Original example used blockCenterY = 2.0f and relied on gravity.
    // Here gravity is disabled, so we place the block near the ground from the beginning.
    const float blockCenterX = 0.0f;
    const float blockCenterY = 0.5f;
    const float blockCenterZ = 0.0f;

    // Cutter placement.
    // Ring cutter starts above the block and moves downward after 0.2 s.
    const float cutterStartX = 0.0f;
    const float cutterStartY = 1.2f;
    const float cutterStartZ = 0.0f;

    const float cutterMoveStartTime = 0.2f;
    const float cutterStepY = 0.01f;

    // Ring cutter parameters.
    const float ringRadius = 0.35f;
    const float edgeRadius = 0.05f;

    // Target particle group.
    const float targetCenterX = blockCenterX;
    const float targetCenterY = blockCenterY;
    const float targetCenterZ = blockCenterZ;
    const float targetRadius = 0.25f;

    crmpm::SimulationFactory *simFactory =
        crmpm::createFactory(10000, 4, 4, 15000, true);

    CR_DEBUG_LOG_INFO("%s", "Create Scene");

    crmpm::SceneDesc sceneDesc;
    sceneDesc.solverType = crmpm::MpmSolverType::eGpuMlsMpmSolver;
    sceneDesc.numMaxParticles = 10000;

    // No gravity. We want to isolate the cutter effect.
    sceneDesc.gravity = crmpm::Vec3f(0.0f, 0.0f, 0.0f);

    sceneDesc.gridBounds = crmpm::Bounds3(
        crmpm::Vec3f(-5.0f, -0.5f, -5.0f),
        crmpm::Vec3f(5.0f, 5.0f, 5.0f)
    );
    sceneDesc.gridCellSize = 0.2f;
    sceneDesc.solverIntegrationStepSize = 0.002f;
    sceneDesc.solverIterations = 20;

    crmpm::Scene *scene = simFactory->createScene(sceneDesc);

    CR_DEBUG_LOG_INFO("%s", "Create Scene ok");

    constexpr float E = 1e5f;
    constexpr float nu = 0.4f;
    constexpr float lambda = (E * nu) / ((1.0f + nu) * (1.0f - 2.0f * nu));
    constexpr float mu = E / (2.0f * (1.0f + nu));

    CR_DEBUG_LOG_INFO("%s", "Create particle object");

    crmpm::Geometry *geom1 =
        simFactory->createGeometry(
            crmpm::GeometryType::eBox,
            make_float4(1, 1, 1, 0)
        );

    crmpm::ParticleObjectDesc poDesc;
    poDesc.particleSpacing = 0.1f;
    poDesc.particleMass = 1.0f;
    poDesc.geometry = geom1;
    poDesc.position = crmpm::Vec3f(blockCenterX, blockCenterY, blockCenterZ);
    poDesc.rotation = crmpm::Quat();

    // Keep the same scaling as the original example.
    poDesc.invScale = crmpm::Vec3f(2, 2, 2);

    poDesc.materialType = crmpm::ParticleMaterialType::eCoRotational;
    poDesc.materialParams = make_float4(lambda, mu, 0, 0);

    crmpm::ParticleObject *po1 = simFactory->createParticleObject(poDesc);
    geom1->release();

    if (po1->addToScene(*scene))
    {
        CR_DEBUG_LOG_INFO("%s", "Create particle object ok");
    }
    else
    {
        CR_DEBUG_LOG_INFO("%s", "Create particle object failed");
    }

    CR_DEBUG_LOG_INFO("%s", "Create ring slicer Geometry");

    crmpm::Geometry *geom2 =
        simFactory->createGeometry(
            crmpm::GeometryType::eRingSlicer,
            make_float4(ringRadius, edgeRadius, 0, 0)
        );

    CR_DEBUG_LOG_INFO("%s", "Create ring slicer Geometry ok");

    CR_DEBUG_LOG_INFO("%s", "Create ring slicer Shape");

    crmpm::Transform shapeTransform = crmpm::Transform();
    shapeTransform.position = crmpm::Vec3f(cutterStartX, cutterStartY, cutterStartZ);

    // Same orientation convention as the original slicer example.
    shapeTransform.rotation = crmpm::Quat(0, 0, 0.7071068f, 0.7071068f);

    crmpm::ShapeDesc shapeDesc;
    shapeDesc.type = crmpm::ShapeType::eKinematic;
    shapeDesc.transform = shapeTransform;
    shapeDesc.sdfSmoothDistance = 0.0f;
    shapeDesc.sdfFatten = 0.1f;
    shapeDesc.drag = 0.5f;
    shapeDesc.friction = 0.0f;
    shapeDesc.invScale = crmpm::Vec3f(1, 1, 1);

    crmpm::Shape *shape1 = simFactory->createShape(*geom2, shapeDesc);
    geom2->release();

    CR_DEBUG_LOG_INFO("%s", "Create ring slicer Shape ok");

    scene->advance(dt);
    scene->fetchResults();

    std::vector<int> targetIndices;

    {
        const crmpm::ParticleData &particles = scene->getParticleData();

        for (int i = 0; i < particles.size; i++)
        {
            const auto p = particles.positionMass[i];

            const float dx = p.x - targetCenterX;
            const float dy = p.y - targetCenterY;
            const float dz = p.z - targetCenterZ;
            const float dist2 = dx * dx + dy * dy + dz * dz;

            if (dist2 <= targetRadius * targetRadius)
            {
                targetIndices.push_back(i);
            }
        }

        std::cout << "Target particle count = "
                  << targetIndices.size()
                  << std::endl;
    }

    float timeElapsed = 0.0f;
    bool shouldAddShape = true;

    for (int stepId = 0; stepId < numSteps; stepId++)
    {
        timeElapsed += dt;

        if (timeElapsed > cutterMoveStartTime)
        {
            if (shouldAddShape)
            {
                scene->addShape(shape1);
                shouldAddShape = false;
            }

            shapeTransform.position -= crmpm::Vec3f(0.0f, cutterStepY, 0.0f);
            shape1->setKinematicTarget(shapeTransform, dt);
        }

        scene->advance(dt);
        scene->fetchResults();

        const crmpm::ParticleData &particles = scene->getParticleData();

        float centroidX = 0.0f;
        float centroidY = 0.0f;
        float centroidZ = 0.0f;
        int validCount = 0;

        for (int idx : targetIndices)
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

        if (stepId % 10 == 0)
        {
            std::cout << "step " << stepId
                      << ", cutter y = "
                      << shapeTransform.position.y()
                      << ", target centroid = "
                      << centroidX << ", "
                      << centroidY << ", "
                      << centroidZ
                      << std::endl;
        }
    }

    shape1->release();
    po1->release();
    scene->release();
    crmpm::releaseFactory(simFactory);

    return 0;
}
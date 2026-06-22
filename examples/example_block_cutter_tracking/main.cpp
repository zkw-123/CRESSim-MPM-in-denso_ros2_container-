// BSD 3-Clause License
//
// Copyright (c) 2025, Yafei Ou and Mahdi Tavakoli
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <iostream>
#include <vector>

#include "debug_logger.h"
#include "particle_object.h"
#include "simulation_factory.h"
#include "transform.h"

int main()
{
    const float dt = 0.02f;
    const int numSteps = 500;

    // Create factory
    crmpm::SimulationFactory *simFactory = crmpm::createFactory(10000, 4, 4, 15000, true);

    // Create Scene
    CR_DEBUG_LOG_INFO("%s", "Create Scene");

    crmpm::SceneDesc sceneDesc;
    sceneDesc.solverType = crmpm::MpmSolverType::eGpuMlsMpmSolver;
    sceneDesc.numMaxParticles = 10000;
    sceneDesc.gravity = crmpm::Vec3f(0, -9.81f, 0);
    sceneDesc.gridBounds = crmpm::Bounds3(
        crmpm::Vec3f(-5.0f, -0.5f, -5.0f),
        crmpm::Vec3f(5.0f, 5.0f, 5.0f)
    );
    sceneDesc.gridCellSize = 0.2f;
    sceneDesc.solverIntegrationStepSize = 0.002f;
    sceneDesc.solverIterations = 20;

    crmpm::Scene *scene = simFactory->createScene(sceneDesc);

    CR_DEBUG_LOG_INFO("%s", "Create Scene ok");

    // Except for PB-MPM, compute Lame parameters
    constexpr float E = 1e5f;  // Young's modulus
    constexpr float nu = 0.4f; // Poisson's ratio
    constexpr float lambda = (E * nu) / ((1 + nu) * (1 - 2 * nu));
    constexpr float mu = E / (2 * (1 + nu));

    CR_DEBUG_LOG_INFO("%s", "Create particle object");

    // Geometry for initializing particles
    crmpm::Geometry *geom1 = simFactory->createGeometry(
        crmpm::GeometryType::eBox,
        make_float4(1, 1, 1, 0)
    );

    crmpm::ParticleObjectDesc poDesc;
    poDesc.particleSpacing = 0.1f;
    poDesc.particleMass = 1.0f;
    poDesc.geometry = geom1;
    poDesc.position = crmpm::Vec3f(0.0f, 2.0f, 0.0f);
    poDesc.rotation = crmpm::Quat();
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

    // Slicer geometry and shape
    CR_DEBUG_LOG_INFO("%s", "Create slicer Geometry");

    crmpm::Geometry *geom2 = simFactory->createGeometry(
        crmpm::GeometryType::eQuadSlicer,
        make_float4(1, 1, 0, 0)
    );

    CR_DEBUG_LOG_INFO("%s", "Create slicer Geometry ok");

    CR_DEBUG_LOG_INFO("%s", "Create slicer Shape");

    crmpm::Transform shapeTransform = crmpm::Transform();
    shapeTransform.position = crmpm::Vec3f(0.0f, 2.0f, 0.0f);
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

    CR_DEBUG_LOG_INFO("%s", "Create slicer Shape ok");

    // Initial advance, then fetch initial particle positions.
    scene->advance(dt);
    scene->fetchResults();

    // Initialize target particle group.
    // The block center is around (0, 2, 0) in the original example.
    std::vector<int> targetIndices;

    const float targetCenterX = 0.0f;
    const float targetCenterY = 2.0f;
    const float targetCenterZ = 0.0f;
    const float targetRadius = 0.25f;

    {
        const crmpm::ParticleData &particles = scene->getParticleData();

        for (int i = 0; i < particles.size; i++)
        {
            const auto p = particles.positionMass[i];

            float dx = p.x - targetCenterX;
            float dy = p.y - targetCenterY;
            float dz = p.z - targetCenterZ;
            float dist2 = dx * dx + dy * dy + dz * dz;

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

    // Fixed-step backend simulation loop.
    for (int stepId = 0; stepId < numSteps; stepId++)
    {
        timeElapsed += dt;

        if (timeElapsed > 2.0f)
        {
            if (shouldAddShape)
            {
                scene->addShape(shape1);
                shouldAddShape = false;
            }

            shapeTransform.position -= crmpm::Vec3f(0, 0.01f, 0);
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
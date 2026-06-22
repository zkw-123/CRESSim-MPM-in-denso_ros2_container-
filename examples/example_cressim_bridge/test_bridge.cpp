#include <iostream>

extern "C"
{
    void *cressim_create();
    void cressim_set_cutter_pose(void *handle, float x, float y, float z);
    void cressim_step(void *handle, float dt);
    void cressim_get_target_centroid(void *handle, float *outXyz);
    int cressim_get_target_particle_count(void *handle);
    void cressim_destroy(void *handle);
}

int main()
{
    const float dt = 0.02f;
    const int numSteps = 300;

    void *sim = cressim_create();

    if (!sim)
    {
        std::cerr << "Failed to create CRESSim bridge sim." << std::endl;
        return 1;
    }

    const int targetCount = cressim_get_target_particle_count(sim);
    std::cout << "Target particle count = "
              << targetCount
              << std::endl;

    for (int stepId = 0; stepId < numSteps; stepId++)
    {
        float cutterY = 1.2f;

        if (stepId > 10)
        {
            cutterY -= static_cast<float>(stepId - 10) * 0.01f;
        }

        cressim_set_cutter_pose(sim, 0.0f, cutterY, 0.0f);
        cressim_step(sim, dt);

        if (stepId % 10 == 0)
        {
            float centroid[3] = {0.0f, 0.0f, 0.0f};
            cressim_get_target_centroid(sim, centroid);

            std::cout << "step " << stepId
                      << ", cutter y = "
                      << cutterY
                      << ", target centroid = "
                      << centroid[0] << ", "
                      << centroid[1] << ", "
                      << centroid[2]
                      << std::endl;
        }
    }

    cressim_destroy(sim);

    return 0;
}

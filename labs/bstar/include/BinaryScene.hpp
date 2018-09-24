#pragma once

#include "Body.hpp"

#include <atlas/tools/ModellingScene.hpp>
#include <atlas/utils/FPSCounter.hpp>

namespace bstar
{
    class BinaryScene : public atlas::tools::ModellingScene
    {
    public:
        BinaryScene();

        void updateScene(double time) override;
        void renderScene() override;

    private:
        bool mPlay;
        int mFPSOption;
        float mFPS;

        atlas::core::Time<float> mAnimTime;
        atlas::utils::FPSCounter mCounter;

        Body mBall;
    };
}

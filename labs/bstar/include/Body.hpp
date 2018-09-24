#pragma once

#include <atlas/utils/Geometry.hpp>
#include <atlas/gl/Buffer.hpp>
#include <atlas/gl/VertexArrayObject.hpp>
#include <atlas/gl/Texture.hpp>

namespace bstar
{
    class Body : public atlas::utils::Geometry
    {
    public:
        Body();

        void updateGeometry(atlas::core::Time<> const& t) override;
        void drawGui() override;
        void renderGeometry(atlas::math::Matrix4 const& projection,
            atlas::math::Matrix4 const& view) override;

        void resetGeometry() override;

    private:
        void eulerIntegrator(atlas::core::Time<> const& t);
        void implicitEulerIntegrator(atlas::core::Time<> const& t);
        void verletIntegrator(atlas::core::Time<> const& t);
        void rk4Integrator(atlas::core::Time<> const& t);
        float gravity(atlas::math::Vector m1Position, atlas::math::Vector m2Position, float m1Mass, float m2Mass);

        atlas::gl::Buffer mVertexBuffer;
        atlas::gl::Buffer mIndexBuffer;
        atlas::gl::VertexArrayObject mVao;

        // Mass 1 details
        atlas::math::Vector mPosition;
        atlas::math::Vector mOffset;
        atlas::math::Vector mVelocity;
        atlas::math::Vector mOldPosition;
        atlas::math::Vector mForce;

        float mMass;
        float tForce;

        int mIntegrator;

        GLsizei mIndexCount;
    };
}

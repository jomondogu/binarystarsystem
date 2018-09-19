#include "GolfBall.hpp"
#include "Paths.hpp"
#include "LayoutLocations.glsl"

#include <atlas/utils/Mesh.hpp>
#include <atlas/core/STB.hpp>
#include <atlas/core/Float.hpp>
#include <atlas/utils/GUI.hpp>

namespace lab2
{
    GolfBall::GolfBall() :
        mVertexBuffer(GL_ARRAY_BUFFER),
        mIndexBuffer(GL_ELEMENT_ARRAY_BUFFER),
        mTruePosition(6, 1, 12),
        mApproxPosition(-6, 1, 12),
        mOffset(6, 1, 12),
        mApproxVelocity(0),
        mApproxOldPosition(-6, 1, 12),
        mForce(0, 0, -10),
        mMass(1.0f),
        mIntegrator(0)
    {
        using atlas::utils::Mesh;
        namespace gl = atlas::gl;
        namespace math = atlas::math;

        Mesh sphere;
        std::string path{ DataDirectory };
        path = path + "sphere.obj";
        Mesh::fromFile(path, sphere);

        mIndexCount = static_cast<GLsizei>(sphere.indices().size());

        std::vector<float> data;
        for (std::size_t i = 0; i < sphere.vertices().size(); ++i)
        {
            data.push_back(sphere.vertices()[i].x);
            data.push_back(sphere.vertices()[i].y);
            data.push_back(sphere.vertices()[i].z);

            data.push_back(sphere.normals()[i].x);
            data.push_back(sphere.normals()[i].y);
            data.push_back(sphere.normals()[i].z);

            data.push_back(sphere.texCoords()[i].x);
            data.push_back(sphere.texCoords()[i].y);
        }

        mVao.bindVertexArray();
        mVertexBuffer.bindBuffer();
        mVertexBuffer.bufferData(gl::size<float>(data.size()), data.data(),
            GL_STATIC_DRAW);
        mVertexBuffer.vertexAttribPointer(VERTICES_LAYOUT_LOCATION, 3, GL_FLOAT,
            GL_FALSE, gl::stride<float>(8), gl::bufferOffset<float>(0));
        mVertexBuffer.vertexAttribPointer(NORMALS_LAYOUT_LOCATION, 3, GL_FLOAT,
            GL_FALSE, gl::stride<float>(8), gl::bufferOffset<float>(3));
        mVertexBuffer.vertexAttribPointer(TEXTURES_LAYOUT_LOCATION, 2, GL_FLOAT,
            GL_FALSE, gl::stride<float>(8), gl::bufferOffset<float>(6));

        mVao.enableVertexAttribArray(VERTICES_LAYOUT_LOCATION);
        mVao.enableVertexAttribArray(NORMALS_LAYOUT_LOCATION);
        mVao.enableVertexAttribArray(TEXTURES_LAYOUT_LOCATION);

        mIndexBuffer.bindBuffer();
        mIndexBuffer.bufferData(gl::size<GLuint>(sphere.indices().size()),
            sphere.indices().data(), GL_STATIC_DRAW);

        mIndexBuffer.unBindBuffer();
        mVertexBuffer.unBindBuffer();
        mVao.unBindVertexArray();

        std::vector<gl::ShaderUnit> shaders
        {
            {std::string(ShaderDirectory) + "Golf.vs.glsl", GL_VERTEX_SHADER},
            {std::string(ShaderDirectory) + "Golf.fs.glsl", GL_FRAGMENT_SHADER}
        };

        mShaders.emplace_back(shaders);
        mShaders[0].setShaderIncludeDir(ShaderDirectory);
        mShaders[0].compileShaders();
        mShaders[0].linkShaders();

        auto var = mShaders[0].getUniformVariable("model");
        mUniforms.insert(UniformKey("model", var));
        var = mShaders[0].getUniformVariable("projection");
        mUniforms.insert(UniformKey("projection", var));
        var = mShaders[0].getUniformVariable("view");
        mUniforms.insert(UniformKey("view", var));
        var = mShaders[0].getUniformVariable("materialColour");
        mUniforms.insert(UniformKey("materialColour", var));

        mShaders[0].disableShaders();
    }

    void GolfBall::updateGeometry(atlas::core::Time<> const& t)
    {
        using atlas::core::leq;

        if (leq(mTruePosition.z, -12.0f))
        {
            return;
        }

        switch (mIntegrator)
        {
        case 0:
            eulerIntegrator(t);
            break;

        case 1:
            implicitEulerIntegrator(t);
            break;

        case 2:
            verletIntegrator(t);
            break;
        
        case 3:
            rk4Integrator(t);
            break;

        default:
            break;
        }

        float timeSq = t.currentTime * t.currentTime;
        mTruePosition = mOffset + 0.5f * (mForce / mMass) * timeSq;
    }

    void GolfBall::drawGui()
    {
        ImGui::SetNextWindowSize(ImVec2(300, 100), ImGuiSetCond_FirstUseEver);
        ImGui::Begin("Integration Controls");

        std::vector<const char*> integratorNames = { "Euler Intergrator",
        "Implicit Euler Integrator", "Verlet Integrator", "Runge-Kutta Integrator" };
        ImGui::Combo("Integrator", &mIntegrator, integratorNames.data(),
            ((int)integratorNames.size()));
        ImGui::InputFloat("Set force", &mForce.z, 1.0f, 5.0f, 1);
        ImGui::End();
    }
    
    void GolfBall::renderGeometry(atlas::math::Matrix4 const& projection,
        atlas::math::Matrix4 const& view)
    {
        namespace math = atlas::math;

        mShaders[0].hotReloadShaders();
        if (!mShaders[0].shaderProgramValid())
        {
            return;
        }

        mShaders[0].enableShaders();

        mVao.bindVertexArray();
        mIndexBuffer.bindBuffer();

        glUniformMatrix4fv(mUniforms["projection"], 1, GL_FALSE,
            &projection[0][0]);
        glUniformMatrix4fv(mUniforms["view"], 1, GL_FALSE, &view[0][0]);

        // Render the true sphere first.
        {
            const math::Vector copper{ 0.71f, 0.44f, 0.19f };
            auto model = glm::translate(math::Matrix4(1.0f), mTruePosition);
            glUniformMatrix4fv(mUniforms["model"], 1, GL_FALSE, &model[0][0]);
            glUniform3fv(mUniforms["materialColour"], 1, &copper[0]);
            glDrawElements(GL_TRIANGLES, mIndexCount, GL_UNSIGNED_INT, 0);
        }

        // Now render the integrated sphere.
        {
            const math::Vector metalGray{ 0.0f, 0.46f, 0.69f };
            auto model = glm::translate(math::Matrix4(1.0f), mApproxPosition);
            glUniformMatrix4fv(mUniforms["model"], 1, GL_FALSE, &model[0][0]);
            glUniform3fv(mUniforms["materialColour"], 1, &metalGray[0]);
            glDrawElements(GL_TRIANGLES, mIndexCount, GL_UNSIGNED_INT, 0);
        }

        mIndexBuffer.unBindBuffer();
        mVao.unBindVertexArray();
        mShaders[0].disableShaders();
    }

    void GolfBall::resetGeometry()
    {
        mTruePosition = { 6, 1, 12 };
        mApproxPosition = { -6, 1, 12 };
        mApproxOldPosition = mApproxPosition;
        mApproxVelocity = { 0, 0, 0 };
    }

    void GolfBall::eulerIntegrator(atlas::core::Time<> const& t)
    {
        // TODO: Fill this in.
        mApproxPosition = mApproxOldPosition + t.deltaTime * mApproxVelocity;
        mApproxVelocity = mApproxVelocity + t.deltaTime * (mForce / mMass);
    }

    void GolfBall::implicitEulerIntegrator(atlas::core::Time<> const& t)
    {
        // TODO: Fill this in.
        mApproxVelocity = mApproxVelocity + t.deltaTime * (mForce / mMass);
        mApproxPosition = mApproxOldPosition + t.deltaTime * mApproxVelocity;
    }

    void GolfBall::verletIntegrator(atlas::core::Time<> const& t)
    {
        // TODO: Fill this in.
        atlas::math::Vector tmp = mApproxPosition;
        mApproxPosition += mApproxPosition - mApproxOldPosition + t.deltaTime * t.deltaTime * (mForce / mMass);
        mApproxOldPosition = tmp;
    }
    
    void GolfBall::rk4Integrator(atlas::core::Time<> const& t)
    {
        //Position
        atlas::math::Vector pki; //output of all ki needs to be vector (double check)
        //k1 = F(xn)
        atlas::math::Vector pk1 = mApproxVelocity;
        //k2 = F(xn + 0.5 * h * k1)
        atlas::math::Vector pk2 = mApproxVelocity + 0.5f * t.deltaTime * pk1;
        //k3 = F(xn + 0.5 * h * k2)
        atlas::math::Vector pk3 = mApproxVelocity + 0.5f * t.deltaTime * pk2;
        //k4 = F(xn + h * k3)
        atlas::math::Vector pk4 = mApproxVelocity + t.deltaTime * pk3;
        //y(n+1) = y(n) + (k1 + 2 * k2 + 2 * k3 + k4) / 6
        mApproxPosition = mApproxPosition + t.deltaTime * (pk1 + 2.0f * pk2 + 2.0f * pk3 + pk4) / 6.0f;
        
        //Velocity
        atlas::math::Vector vki; //output of all ki needs to be vector (double check)
        //k1 = F(xn)
        atlas::math::Vector vk1 = (mForce / mMass);
        //k2 = F(xn + 0.5 * h * k1)
        atlas::math::Vector vk2 = (mForce / mMass) + 0.5f * t.deltaTime * vk1;
        //k3 = F(xn + 0.5 * h * k2)
        atlas::math::Vector vk3 = (mForce / mMass) + 0.5f * t.deltaTime * vk2;
        //k4 = F(xn + h * k3)
        atlas::math::Vector vk4 = (mForce / mMass) + t.deltaTime * vk3;
        //y(n+1) = y(n) + (k1 + 2 * k2 + 2 * k3 + k4) / 6
        mApproxVelocity = mApproxVelocity + t.deltaTime * (vk1 + 2.0f * vk2 + 2.0f * vk3 + vk4) / 6.0f;
    }
}

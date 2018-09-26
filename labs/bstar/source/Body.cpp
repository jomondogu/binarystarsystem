#include "Body.hpp"
#include "Paths.hpp"
#include "LayoutLocations.glsl"

#include <atlas/utils/Mesh.hpp>
#include <atlas/core/STB.hpp>
#include <atlas/core/Float.hpp>
#include <atlas/utils/GUI.hpp>

const float g = 6.67e-11;

namespace bstar
{
    Body::Body() :
        mVertexBuffer(GL_ARRAY_BUFFER),
        mIndexBuffer(GL_ELEMENT_ARRAY_BUFFER),
        mPosition(0,0,10),
        mOffset(0,0,10),
        mVelocity(0,0,0),
        mOldPosition(2,0,10),
        mForce(0, 0, 0),
        mMass(1.0e11),

        s1Position(3,0,0),
        s1Offset(3,0,0),
        s1Velocity(0,0,0),
        s1OldPosition(3,0,1),
        s1Force(0, 0, 0),
        s1Mass(1.0e13),

        s2Position(-3,0,0),
        s2Offset(-3,0,0),
        s2Velocity(0,0,0),
        s2OldPosition(-3,0,-1),
        s2Force(0, 0, 0),
        s2Mass(1.0e13),

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
            {std::string(ShaderDirectory) + "Body.vs.glsl", GL_VERTEX_SHADER},
            {std::string(ShaderDirectory) + "Body.fs.glsl", GL_FRAGMENT_SHADER}
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

    void Body::updateGeometry(atlas::core::Time<> const& t)
    {

        //determine forces at current state
        //Gravitational for Mass 1 (accounts for both stars)
        float mgForce1 = gravity(mPosition, s1Position, mMass, s1Mass);
        atlas::math::Vector mgDir1 = normalize(s1Position - mPosition);
        float mgForce2 = gravity(mPosition, -s2Position, mMass, s2Mass);
        atlas::math::Vector mgDir2 = normalize(s2Position - mPosition);

        //Gravitational for Star 1 (accounts only for star 2's mass)
        float s1gForce = gravity(s1Position, s2Position, s1Mass, s2Mass);
        atlas::math::Vector s1gDir = normalize(s2Position - s1Position);

        //Gravitational for Star 2 (accounts only for star 1's mass)
        float s2gForce = gravity(s2Position, s1Position, s2Mass, s1Mass);
        atlas::math::Vector s2gDir = normalize(s1Position - s2Position);

        //Total
        mForce = (mgForce1 * mgDir1) + (mgForce2 * mgDir2);
        s1Force = s1gForce * s1gDir;
        s2Force = s2gForce * s2gDir;

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
    }

    void Body::drawGui()
    {
        ImGui::SetNextWindowSize(ImVec2(300, 200), ImGuiSetCond_FirstUseEver);
        ImGui::Begin("Integration Controls");

        std::vector<const char*> integratorNames = { "Euler Integrator",
        "Implicit Euler Integrator", "Verlet Integrator", "Runge-Kutta Integrator" };
        ImGui::Combo("Integrator", &mIntegrator, integratorNames.data(),
            ((int)integratorNames.size()));
        ImGui::InputFloat("Set planet mass", &mMass, 1.0e11, 5.0f, 1);
        ImGui::InputFloat("Set star 1 mass", &s1Mass, 1.0e11, 5.0f, 1);
        ImGui::InputFloat("Set star 2 mass", &s2Mass, 1.0e11, 5.0f, 1);
        ImGui::End();
    }

    void Body::renderGeometry(atlas::math::Matrix4 const& projection,
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

        // Render star A
        {
            const math::Vector white{ 1.0f, 1.0f, 1.0f };
            auto model = glm::translate(math::Matrix4(1.0f), s1Position) * glm::scale(math::Matrix4(1.0f), math::Vector(0.25f));
            glUniformMatrix4fv(mUniforms["model"], 1, GL_FALSE, &model[0][0]);
            glUniform3fv(mUniforms["materialColour"], 1, &white[0]);
            glDrawElements(GL_TRIANGLES, mIndexCount, GL_UNSIGNED_INT, 0);
        }

        // Render star B
        {
            const math::Vector white{ 1.0f, 1.0f, 1.0f };
            auto model = glm::translate(math::Matrix4(1.0f), s2Position) * glm::scale(math::Matrix4(1.0f), math::Vector(0.25f));
            glUniformMatrix4fv(mUniforms["model"], 1, GL_FALSE, &model[0][0]);
            glUniform3fv(mUniforms["materialColour"], 1, &white[0]);
            glDrawElements(GL_TRIANGLES, mIndexCount, GL_UNSIGNED_INT, 0);
        }

        // Render planet
        {
            const math::Vector black{ 0.0f, 0.0f, 0.0f };
            auto model = glm::translate(math::Matrix4(1.0f), mPosition) * glm::scale(math::Matrix4(1.0f), math::Vector(0.1f));
            glUniformMatrix4fv(mUniforms["model"], 1, GL_FALSE, &model[0][0]);
            glUniform3fv(mUniforms["materialColour"], 1, &black[0]);
            glDrawElements(GL_TRIANGLES, mIndexCount, GL_UNSIGNED_INT, 0);
        }

        mIndexBuffer.unBindBuffer();
        mVao.unBindVertexArray();
        mShaders[0].disableShaders();
    }

    void Body::resetGeometry()
    {
        mPosition = { 0, 0, 10 };
        mOldPosition = {2, 0, 10};
        mVelocity = { 0, 0, 0 };

        s1Position = { 3, 0, 0 };
        s1OldPosition = {3, 0, 1};
        s1Velocity = { 0, 0, 0 };

        s2Position = { -3, 0, 0 };
        s2OldPosition = {-3, 0, -1};
        s2Velocity = { 0, 0, 0 };
    }

    void Body::eulerIntegrator(atlas::core::Time<> const& t)
    {
        //mass 1
        mPosition += mOldPosition + t.deltaTime * mVelocity;
        mVelocity += mVelocity + t.deltaTime * (mForce / mMass);

        //star 1
        s1Position += s1OldPosition + t.deltaTime * s1Velocity;
        s1Velocity += s1Velocity + t.deltaTime * (s1Force / s1Mass);

        //star 2
        s2Position += s2OldPosition + t.deltaTime * s2Velocity;
        s2Velocity += s2Velocity + t.deltaTime * (s2Force / s2Mass);
    }

    void Body::implicitEulerIntegrator(atlas::core::Time<> const& t)
    {
        //mass 1
        mVelocity += mVelocity + t.deltaTime * (mForce / mMass);
        mPosition += mOldPosition + t.deltaTime * mVelocity;

        //star 1
        s1Velocity += s1Velocity + t.deltaTime * (s1Force / s1Mass);
        s1Position += s1OldPosition + t.deltaTime * s1Velocity;

        //star 2
        s2Velocity += s2Velocity + t.deltaTime * (s2Force / s2Mass);
        s2Position += s2OldPosition + t.deltaTime * s2Velocity;
    }

    void Body::verletIntegrator(atlas::core::Time<> const& t)
    {
        //mass 1
        atlas::math::Vector mtmp = mPosition;
        mPosition += mPosition - mOldPosition + t.deltaTime * t.deltaTime * (mForce / mMass);
        mOldPosition = mtmp;

        //star 1
        atlas::math::Vector s1tmp = s1Position;
        s1Position += s1Position - s1OldPosition + t.deltaTime * t.deltaTime * (s1Force / s1Mass);
        s1OldPosition = s1tmp;

        //star 2
        atlas::math::Vector s2tmp = s2Position;
        s2Position += s2Position - s2OldPosition + t.deltaTime * t.deltaTime * (s2Force / s2Mass);
        s2OldPosition = s2tmp;
    }

    void Body::rk4Integrator(atlas::core::Time<> const& t)
    {
        namespace math = atlas::math;

        //Mass 1 Position
        math::Vector pk1 = mVelocity;
        math::Vector pk2 = mVelocity + 0.5f * t.deltaTime * pk1;
        math::Vector pk3 = mVelocity + 0.5f * t.deltaTime * pk2;
        math::Vector pk4 = mVelocity + t.deltaTime * pk3;
        mPosition += mOldPosition + t.deltaTime * (pk1 + 2.0f * pk2 + 2.0f * pk3 + pk4) / 6.0f;

        //Mass 1 Velocity
        math::Vector vk1 = (mForce / mMass);
        math::Vector vk2 = (mForce / mMass) + 0.5f * t.deltaTime * vk1;
        math::Vector vk3 = (mForce / mMass) + 0.5f * t.deltaTime * vk2;
        math::Vector vk4 = (mForce / mMass) + t.deltaTime * vk3;
        mVelocity += mVelocity + t.deltaTime * (vk1 + 2.0f * vk2 + 2.0f * vk3 + vk4) / 6.0f;

        //Star 1 Position
        pk1 = s1Velocity;
        pk2 = s1Velocity + 0.5f * t.deltaTime * pk1;
        pk3 = s1Velocity + 0.5f * t.deltaTime * pk2;
        pk4 = s1Velocity + t.deltaTime * pk3;
        s1Position += s1OldPosition + t.deltaTime * (pk1 + 2.0f * pk2 + 2.0f * pk3 + pk4) / 6.0f;

        //Star 1 Velocity
        vk1 = (s1Force / s1Mass);
        vk2 = (s1Force / s1Mass) + 0.5f * t.deltaTime * vk1;
        vk3 = (s1Force / s1Mass) + 0.5f * t.deltaTime * vk2;
        vk4 = (s1Force / s1Mass) + t.deltaTime * vk3;
        s1Velocity += s1Velocity + t.deltaTime * (vk1 + 2.0f * vk2 + 2.0f * vk3 + vk4) / 6.0f;

        //Star 2 Position
        pk1 = s2Velocity;
        pk2 = s2Velocity + 0.5f * t.deltaTime * pk1;
        pk3 = s2Velocity + 0.5f * t.deltaTime * pk2;
        pk4 = s2Velocity + t.deltaTime * pk3;
        s2Position += s2OldPosition + t.deltaTime * (pk1 + 2.0f * pk2 + 2.0f * pk3 + pk4) / 6.0f;

        //Star 2 Velocity
        vk1 = (s2Force / s2Mass);
        vk2 = (s2Force / s2Mass) + 0.5f * t.deltaTime * vk1;
        vk3 = (s2Force / s2Mass) + 0.5f * t.deltaTime * vk2;
        vk4 = (s2Force / s2Mass) + t.deltaTime * vk3;
        s2Velocity += s2Velocity + t.deltaTime * (vk1 + 2.0f * vk2 + 2.0f * vk3 + vk4) / 6.0f;
    }

    float Body::gravity(atlas::math::Vector m1_pos, atlas::math::Vector m2_pos, float m1_mass, float m2_mass)
    {
        float distance = (m1_pos - m2_pos).length();
        return g * m1_mass * m2_mass / distance * distance;
    }

}

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
        mPosition(10,0,0),
        mOffset(10,0,0),
        mVelocity(0,0,-10),
        mOldPosition(10,0,0),
        mForce(0, 0, 0),
        mMass(1.0e11),
        tForce(5.0e14),

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
        //Gravitational
        float gForce1 = gravity(mPosition, atlas::math::Vector(3,0,0), mMass, 1.0e13);
        atlas::math::Vector unitDirection1 = normalize(atlas::math::Vector(3,0,0) - mPosition);
        float gForce2 = gravity(mPosition, atlas::math::Vector(-3,0,0), mMass, 1.0e13);
        atlas::math::Vector unitDirection2 = normalize(atlas::math::Vector(-3,0,0) - mPosition);

        //Centripetal
        atlas::math::Vector unitTangent = normalize(cross(atlas::math::Vector(0,1,0), mPosition - atlas::math::Vector(0,0,0)));

        //Total
        mForce = (gForce1 * unitDirection1) + (gForce2 * unitDirection2) + (tForce * unitTangent);

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
        ImGui::SetNextWindowSize(ImVec2(300, 100), ImGuiSetCond_FirstUseEver);
        ImGui::Begin("Integration Controls");

        std::vector<const char*> integratorNames = { "Euler Integrator",
        "Implicit Euler Integrator", "Verlet Integrator", "Runge-Kutta Integrator" };
        ImGui::Combo("Integrator", &mIntegrator, integratorNames.data(),
            ((int)integratorNames.size()));
        ImGui::InputFloat("Set tangent force", &tForce, 1.0e14f, 5.0f, 1);
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
            auto model = glm::translate(math::Matrix4(1.0f), atlas::math::Vector(3,0,0));
            glUniformMatrix4fv(mUniforms["model"], 1, GL_FALSE, &model[0][0]);
            glUniform3fv(mUniforms["materialColour"], 1, &white[0]);
            glDrawElements(GL_TRIANGLES, mIndexCount, GL_UNSIGNED_INT, 0);
        }

        // Render star B
        {
            const math::Vector white{ 1.0f, 1.0f, 1.0f };
            auto model = glm::translate(math::Matrix4(1.0f), atlas::math::Vector(-3,0,0));
            glUniformMatrix4fv(mUniforms["model"], 1, GL_FALSE, &model[0][0]);
            glUniform3fv(mUniforms["materialColour"], 1, &white[0]);
            glDrawElements(GL_TRIANGLES, mIndexCount, GL_UNSIGNED_INT, 0);
        }

        // Render planet
        {
            const math::Vector black{ 0.0f, 0.0f, 0.0f };
            auto model = glm::translate(math::Matrix4(1.0f), mPosition) * glm::scale(math::Matrix4(1.0f), math::Vector(0.25f));
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
        mPosition = { 10, 0, 0 };
        mOldPosition = mPosition;
        mVelocity = { 0, 0, -1000 };
    }

    void Body::eulerIntegrator(atlas::core::Time<> const& t)
    {
        mPosition = mOldPosition + t.deltaTime * mVelocity;
        mVelocity = mVelocity + t.deltaTime * (mForce / mMass);
    }

    void Body::implicitEulerIntegrator(atlas::core::Time<> const& t)
    {
        mVelocity = mVelocity + t.deltaTime * (mForce / mMass);
        mPosition = mOldPosition + t.deltaTime * mVelocity;
    }

    void Body::verletIntegrator(atlas::core::Time<> const& t)
    {
        atlas::math::Vector tmp = mPosition;
        mPosition += mPosition - mOldPosition + t.deltaTime * t.deltaTime * (mForce / mMass);
        mOldPosition = tmp;
    }

    void Body::rk4Integrator(atlas::core::Time<> const& t)
    {
        namespace math = atlas::math;

        //Position
        math::Vector pk1 = mVelocity;
        math::Vector pk2 = mVelocity + 0.5f * t.deltaTime * pk1;
        math::Vector pk3 = mVelocity + 0.5f * t.deltaTime * pk2;
        math::Vector pk4 = mVelocity + t.deltaTime * pk3;
        mPosition = mOldPosition + t.deltaTime * (pk1 + 2.0f * pk2 + 2.0f * pk3 + pk4) / 6.0f;

        //Velocity
        math::Vector vk1 = (mForce / mMass);
        math::Vector vk2 = (mForce / mMass) + 0.5f * t.deltaTime * vk1;
        math::Vector vk3 = (mForce / mMass) + 0.5f * t.deltaTime * vk2;
        math::Vector vk4 = (mForce / mMass) + t.deltaTime * vk3;
        mVelocity = mVelocity + t.deltaTime * (vk1 + 2.0f * vk2 + 2.0f * vk3 + vk4) / 6.0f;
    }

    float Body::gravity(atlas::math::Vector m1_pos, atlas::math::Vector m2_pos, float m1_mass, float m2_mass)
    {
        float distance = (m1_pos - m2_pos).length();
        return g * m1_mass * m2_mass / distance * distance;
    }

}

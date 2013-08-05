///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
/// \file:  skeletal_mesh.h
/// \author Ben Crist
///
/// \brief  Class header for the Vertex struct and SkeletalMesh class.

#ifndef SKELETAL_MESH_H_
#define SKELETAL_MESH_H_

#include "demo.h"
#include <vector>

///////////////////////////////////////////////////////////////////////////////
/// \brief  A vertex contains the coordinates of a point in bind-pose model
///         space and any extra data associated with it that the vertex shader
///         will need.
///
/// \details For the purposes of this demo, the position is specified in 2D
///         space, but skinning works the same way in 3D space.  The only
///         other information specified are the indices of up to 3 joints
///         which influence the vertex's final position, and the relative
///         weight of each influencing vertex.
///
///         If there are fewer than 3 joints which influence a vertex, the
///         weight for the extra joints can be set to 0, indicating that that
///         joint doesn't influence the vertex at all.  The sum of the values
///         of joint_weights[0..2] must always be 1.0.
struct Vertex
{
    vec2 position;              ///< The vertex's 2D position in bind-pose model space.
    GLuint joint_indices[3];    ///< The indices of 3 joints which affect the vertex.
    GLfloat joint_weights[3];   ///< The amount that the joints identified above affect the vertex.
};

///////////////////////////////////////////////////////////////////////////////
/// \brief  A skeletal mesh object is a Vertex Array Object (VAO) that has an
///         Index Buffer Object (IBO) and a Vertex Buffer Object (VBO) which
///         is suitable for use with a skinning vertex shader.
class SkeletalMesh
{
public:
    SkeletalMesh();
    ~SkeletalMesh();

    void uploadMesh() const;

    std::vector<Vertex> vertices;
    std::vector<GLushort> indices;

    const GLuint& vao_id;
    const GLuint& vbo_id;
    const GLuint& ibo_id;

private:
    GLuint vao_id_;
    GLuint vbo_id_;
    GLuint ibo_id_;
};

#endif

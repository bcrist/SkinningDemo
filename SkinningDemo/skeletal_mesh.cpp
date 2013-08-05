///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
/// \file:  skeletal_mesh.cpp
/// \author Ben Crist
///
/// \brief  Implementations of SkeletalMesh class functions.

#include "skeletal_mesh.h"

///////////////////////////////////////////////////////////////////////////////
/// \brief  Constructs a new skeletal mesh object, allocating a new VAO,
///         VBO, and IBO in the current OpenGL context.
SkeletalMesh::SkeletalMesh()
    : vao_id(vao_id_),
      vbo_id(vbo_id_),
      ibo_id(ibo_id_)
{
    glGenVertexArrays(1, &vao_id_);     // Create VAO
    glGenBuffers(1, &vbo_id_);          // Create VBO
    glGenBuffers(1, &ibo_id_);          // Create IBO
}

///////////////////////////////////////////////////////////////////////////////
/// \brief  Destroys the skeletal mesh, releasing the graphics buffers
///         created in the constructor.
SkeletalMesh::~SkeletalMesh()
{
    glDeleteBuffers(1, &vao_id_);       // Delete VAO
    glDeleteBuffers(1, &vbo_id_);       // Delete VBO
    glDeleteBuffers(1, &ibo_id_);       // Delete IBO
}

///////////////////////////////////////////////////////////////////////////////
/// \brief  Uploads the vertex and index data in the public indices and
///         vertices fields to the graphics buffers created in the constructor.
///
/// \details In addition to uploading data, it ensures that the VAO vertex
///         attribute pointers are setup and enabled.
void SkeletalMesh::uploadMesh() const
{
    glBindVertexArray(vao_id);  // bind VAO

    glBindBuffer(GL_ARRAY_BUFFER, vbo_id);  // bind VBO
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_id);  // bind IBO

    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLushort), indices.data(), GL_STATIC_DRAW);

    Vertex va[2];

    void* attr_ptr_position      = nullptr;
    void* attr_ptr_bone_index_0  = (void*)((char*)(&va[0].joint_indices[0]) - (char*)(&va[0]));
    void* attr_ptr_bone_index_1  = (void*)((char*)(&va[0].joint_indices[1]) - (char*)(&va[0]));///reinterpret_cast<void*>(sizeof(vec2) + sizeof(GLuint));
    void* attr_ptr_bone_index_2  = (void*)((char*)(&va[0].joint_indices[2]) - (char*)(&va[0]));//reinterpret_cast<void*>(sizeof(vec2) + sizeof(GLuint) * 2);
    void* attr_ptr_bone_weight_0 = (void*)((char*)(&va[0].joint_weights[0]) - (char*)(&va[0]));//reinterpret_cast<void*>(sizeof(vec2) + sizeof(GLuint) * 3);
    void* attr_ptr_bone_weight_1 = (void*)((char*)(&va[0].joint_weights[1]) - (char*)(&va[0]));//reinterpret_cast<void*>(sizeof(vec2) + sizeof(GLuint) * 3 + sizeof(GLfloat));
    void* attr_ptr_bone_weight_2 = (void*)((char*)(&va[0].joint_weights[2]) - (char*)(&va[0]));//reinterpret_cast<void*>(sizeof(vec2) + sizeof(GLuint) * 3 + sizeof(GLfloat) * 2);
    glVertexAttribPointer(0, 2, GL_FLOAT,        GL_FALSE, sizeof(Vertex), attr_ptr_position);
    glVertexAttribIPointer(1, 1, GL_UNSIGNED_INT, sizeof(Vertex), attr_ptr_bone_index_0);
    glVertexAttribIPointer(2, 1, GL_UNSIGNED_INT, sizeof(Vertex), attr_ptr_bone_index_1);
    glVertexAttribIPointer(3, 1, GL_UNSIGNED_INT, sizeof(Vertex), attr_ptr_bone_index_2);
    glVertexAttribPointer(4, 1, GL_FLOAT,        GL_FALSE, sizeof(Vertex), attr_ptr_bone_weight_0);
    glVertexAttribPointer(5, 1, GL_FLOAT,        GL_FALSE, sizeof(Vertex), attr_ptr_bone_weight_1);
    glVertexAttribPointer(6, 1, GL_FLOAT,        GL_FALSE, sizeof(Vertex), attr_ptr_bone_weight_2);

    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glEnableVertexAttribArray(2);
    glEnableVertexAttribArray(3);
    glEnableVertexAttribArray(4);
    glEnableVertexAttribArray(5);
    glEnableVertexAttribArray(6);

    // GL_ARRAY_BUFFER is not part of the VAO state, so we need to make sure we unbind the VBO.
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindVertexArray(0);   // un-bind VAO
}

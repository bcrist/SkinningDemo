
// Make sure we're linking against GLEW and freeGLUT
#ifdef DEBUG
#pragma comment (lib, "glew32sd.lib")
#else
#pragma comment (lib, "glew32s.lib")
#endif
#pragma comment (lib, "freeglut.lib")


// Includes

#include <GL/glew.h>
#include <GL/freeglut.h>

// OpenGL Mathematics library
// Provides C++ analogs of GLSL vector and matrix data types
// as well as equivalents for things like gluPerspective().
#include <glm/glm.hpp>

#include <iostream>
#include <string>
#include <vector>


// GLM vector & matrix typedefs for convenience.
typedef glm::vec2 vec2;     ///< 2-component vector of floats
typedef glm::vec3 vec3;     ///< 3-component vector of floats
typedef glm::vec4 vec4;     ///< 4-component vector of floats
typedef glm::vec4 color4;   ///< 4-component vector of floats representing an RGBA color.
typedef glm::mat4 mat4;     ///< 4x4 matrix of floats


struct Vertex
{
    vec2 position;
    GLuint bone_indices[3];
    GLfloat bone_weights[3];
};

struct SkeletalMesh
{
    SkeletalMesh()
    {
        glGenVertexArrays(1, &vao_id);
        glGenBuffers(1, &vbo_id);
        glGenBuffers(1, &ibo_id);
    }

    ~SkeletalMesh()
    {
        glDeleteBuffers(1, &vao_id);
        glDeleteBuffers(1, &vbo_id);
        glDeleteBuffers(1, &ibo_id);
    }

    void uploadMesh() const
    {
        glBindVertexArray(vao_id);  // bind VAO

        
        glBindBuffer(GL_ARRAY_BUFFER, vbo_id);  // bind VBO
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_id);  // bind IBO

        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLushort), indices.data(), GL_STATIC_DRAW);

        void* attr_ptr_position      = nullptr;
        void* attr_ptr_bone_index_0  = reinterpret_cast<void*>(sizeof(vec2));
        void* attr_ptr_bone_index_1  = reinterpret_cast<void*>(sizeof(vec2) + sizeof(GLuint));
        void* attr_ptr_bone_index_2  = reinterpret_cast<void*>(sizeof(vec2) + sizeof(GLuint) * 2);
        void* attr_ptr_bone_weight_0 = reinterpret_cast<void*>(sizeof(vec2) + sizeof(GLuint) * 3);
        void* attr_ptr_bone_weight_1 = reinterpret_cast<void*>(sizeof(vec2) + sizeof(GLuint) * 3 + sizeof(GLfloat));
        void* attr_ptr_bone_weight_2 = reinterpret_cast<void*>(sizeof(vec2) + sizeof(GLuint) * 3 + sizeof(GLfloat) * 2);
        glVertexAttribPointer(0, 2, GL_FLOAT,        GL_FALSE, sizeof(Vertex), attr_ptr_position);
        glVertexAttribPointer(1, 1, GL_UNSIGNED_INT, GL_FALSE, sizeof(Vertex), attr_ptr_bone_index_0);
        glVertexAttribPointer(2, 1, GL_UNSIGNED_INT, GL_FALSE, sizeof(Vertex), attr_ptr_bone_index_1);
        glVertexAttribPointer(3, 1, GL_UNSIGNED_INT, GL_FALSE, sizeof(Vertex), attr_ptr_bone_index_2);
        glVertexAttribPointer(4, 1, GL_FLOAT,        GL_FALSE, sizeof(Vertex), attr_ptr_bone_weight_0);
        glVertexAttribPointer(5, 1, GL_FLOAT,        GL_FALSE, sizeof(Vertex), attr_ptr_bone_weight_1);
        glVertexAttribPointer(6, 1, GL_FLOAT,        GL_FALSE, sizeof(Vertex), attr_ptr_bone_weight_2);

        // GL_ARRAY_BUFFER is not part of the VAO state, so we need to make sure we unbind the VBO.
        glBindBuffer(GL_ARRAY_BUFFER, 0);


        glBindVertexArray(0);   // un-bind VAO
    }

    std::vector<Vertex> vertices;
    std::vector<GLushort> indices;

    GLuint vao_id;
    GLuint vbo_id;
    GLuint ibo_id;
};


struct JointPose
{
    JointPose* parent;
    color4 color;
    vec2 translation;
    float rotation;
    float scale;
};

struct Pose
{
    JointPose joints[6];
};

const size_t N_POSES = 3;
Pose poses[N_POSES];


GLuint shader_program_id = 0;

const std::string vertex_shader_source =
    "#version 330"                                  "\n"
                                                    "\n"
    "uniform mat4 bind_pose_inv[6]"                 "\n"
    "uniform mat4 current_pose[6]"                  "\n"
    "uniform vec4 current_pose_colors[6]"           "\n"
                                                    "\n"
    "layout(location = 0) in vec2 position;"        "\n"
    "layout(location = 1) in uint bone_0_index;"    "\n"
    "layout(location = 2) in uint bone_1_index;"    "\n"
    "layout(location = 3) in uint bone_2_index;"    "\n"
    "layout(location = 4) in uint bone_0_weight;"   "\n"
    "layout(location = 5) in uint bone_1_weight;"   "\n"
    "layout(location = 6) in uint bone_2_weight;"   "\n"

    "out vec4 color;"

    "void main()"
    "{"
    "   vec4 vertex_coords = vec4(position, 0, 1);"
    "   gl_Position = vec4(0,0,0,0);"
    "   color = vec4(0,0,0,0);"
    ""   
    //  For each bone affecting this vertex, find the vertex's position relative to the bone in bind pose
    //  by using bind_pose_inv, then transform that position using the current pose transform to find
    //  the position where the vertex should be considering only that bone.
    //
    //  Take the weighted average of the positions where each bone thinks the vertex should be, and that is
    //  the final vertex position.


    //  first bone affecting vertex
    "   color += bone_0_weight * current_pose_colors[bone_0_index];"
    "   position += bone_0_weight * (current_pose[bone_0_index] *"
    "                                bind_pose_inv[bone_0_index] *"
    "                                vertex_coords);"

    // second bone affecting vertex
    "   color += bone_1_weight * current_pose_colors[bone_1_index];"
    "   position += bone_1_weight * (current_pose[bone_1_index] *"
    "                                bind_pose_inv[bone_1_index] *"
    "                                vertex_coords);"

    // third bone affecting vertex
    "   color += bone_2_weight * current_pose_colors[bone_2_index];"
    "   position += bone_2_weight * (current_pose[bone_2_index] *"
    "                                bind_pose_inv[bone_2_index] *"
    "                                vertex_coords);"
    "}"
    ;


const std::string fragment_shader_source = 
    "\n"
    ;



void initPoses()
{
    // poses[0] => bind pose.
    poses[0].joints[0].color = color4(1.0f, 1.0f, 1.0f, 1.0f);
    poses[0].joints[0].parent = nullptr;
    poses[0].joints[0].translation = vec2(0, -0.75f);
    poses[0].joints[0].rotation = 90.0f;
    poses[0].joints[0].scale = 1.0f;

    poses[0].joints[1].color = color4(1.0f, 1.0f, 1.0f, 1.0f);
    poses[0].joints[1].parent = &poses[0].joints[0];
    poses[0].joints[1].translation = vec2(0.25f, 0);
    poses[0].joints[1].rotation = 90.0f;
    poses[0].joints[1].scale = 1.0f;
}


int main(int argc, char** argv)
{
    glutInit(&argc, argv);

    glutCreateWindow("Skeletal Mesh Skinning Demo");

    glutMainLoop();
}


void initShaderProgram()
{
    GLuint vert_shader_id = glCreateShader(GL_VERTEX_SHADER);
    GLuint frag_shader_id = glCreateShader(GL_FRAGMENT_SHADER);

    const char* vert_cstr = vertex_shader_source.c_str();
    const char* frag_cstr = fragment_shader_source.c_str();
                                    
    glShaderSource(vert_shader_id, 1, &vert_cstr, NULL);
    glShaderSource(frag_shader_id, 1, &frag_cstr, NULL);

    glCompileShader(vert_shader_id);
    glCompileShader(frag_shader_id);

    // check if there was a problem with compilation.
    GLint result = GL_FALSE;
    glGetShaderiv(vert_shader_id, GL_COMPILE_STATUS, &result);
    if (result != GL_TRUE)
    {
        GLint infolog_len;
        glGetShaderiv(vert_shader_id, GL_INFO_LOG_LENGTH, &infolog_len);
        char *infolog = new char[std::max(1, infolog_len)];
        glGetShaderInfoLog(vert_shader_id, infolog_len, NULL, infolog);

        std::cerr << "Error compiling vertex shader!" << std::endl
                  << "GL Compile Status: " << result << std::endl
                  << "      GL Info Log: " << infolog << std::endl
                  << "    Shader Source: " << vertex_shader_source << std::endl;

        delete[] infolog;

        glDeleteShader(vert_shader_id);
        glDeleteShader(frag_shader_id);
        throw std::runtime_error("Error compiling vertex shader!");
    }

    glGetShaderiv(frag_shader_id, GL_COMPILE_STATUS, &result);
    if (result != GL_TRUE)
    {
        GLint infolog_len;
        glGetShaderiv(frag_shader_id, GL_INFO_LOG_LENGTH, &infolog_len);
        char *infolog = new char[std::max(1, infolog_len)];
        glGetShaderInfoLog(frag_shader_id, infolog_len, NULL, infolog);

        std::cerr << "Error compiling fragment shader!" << std::endl
                  << "GL Compile Status: " << result << std::endl
                  << "      GL Info Log: " << infolog << std::endl
                  << "    Shader Source: " << fragment_shader_source << std::endl;

        delete[] infolog;

        glDeleteShader(vert_shader_id);
        glDeleteShader(frag_shader_id);
        throw std::runtime_error("Error compiling fragment shader!");
    }

    shader_program_id = glCreateProgram();
    glAttachShader(shader_program_id, vert_shader_id);
    glAttachShader(shader_program_id, frag_shader_id);
    glLinkProgram(shader_program_id);

    glGetProgramiv(shader_program_id, GL_LINK_STATUS, &result);

    if (result != GL_TRUE)
    {
        GLint infolog_len;
        glGetProgramiv(shader_program_id, GL_INFO_LOG_LENGTH, &infolog_len);
        char *infolog = new char[std::max(1, infolog_len)];
        glGetProgramInfoLog(shader_program_id, infolog_len, NULL, infolog);

        std::cerr << "Error linking program!" << PBJ_LOG_NL
                  << "GL Link Status: " << result << PBJ_LOG_NL
                  << "   GL Info Log: " << infolog << PBJ_LOG_END;

        delete[] infolog;

        invalidate_();
        throw std::runtime_error("Error linking program!");
    }
}

void display()
{
    
}

void reshape(int width, int height)
{
}





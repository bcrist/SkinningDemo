///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
/// \file:  main.cpp
/// \author Ben Crist
///
/// \brief  Skeletal Mesh Skinning Demo entry point.

///////////////////////////////////////////////////////////////////////////////
// Make sure we're linking against GLEW and freeGLUT
#ifdef DEBUG
#pragma comment (lib, "glew32sd.lib")
#else
#pragma comment (lib, "glew32s.lib")
#endif
#pragma comment (lib, "freeglut.lib")

///////////////////////////////////////////////////////////////////////////////
// Includes
#include "demo.h"
#include "skeletal_mesh.h"

#include <iostream>
#include <string>

///////////////////////////////////////////////////////////////////////////////
// Function Prototypes
void initGL();
void initShaderProgram();
void initMeshes();
void initPoses();

void cleanup();

void reshape(int width, int height);
void display();
void keyboard(unsigned char key, int x, int y);
void mouseMove(int x, int y);

///////////////////////////////////////////////////////////////////////////////
/// \brief  Defines a joint in a particular pose, which is really just a
///         local coordinate system defined by a translation, rotation, and
///         scale relative to the parent joint's coordinate system.
struct JointPose
{
    JointPose* parent;
    color4 color;
    vec2 translation;
    float rotation;
    float scale;
};

///////////////////////////////////////////////////////////////////////////////
/// \brief  Defines the pose of a skeletal system as an array of each of the
///         joints that make it up.
struct Pose
{
    JointPose joints[7];
};

///////////////////////////////////////////////////////////////////////////////
// Global Variables

#pragma region shader source code
const std::string vertex_shader_source =
    "#version 330"                                                          "\n"
                                                                            "\n"
    "uniform mat4 bind_pose_inv[7];"                                        "\n"
    "uniform mat4 current_pose[7];"                                         "\n"
    "uniform vec4 current_pose_colors[7];"                                  "\n"
                                                                            "\n"
    "layout(location = 0) in vec2 position;"                                "\n"
    "layout(location = 1) in uint joint_0_index;"                           "\n"
    "layout(location = 2) in uint joint_1_index;"                           "\n"
    "layout(location = 3) in uint joint_2_index;"                           "\n"
    "layout(location = 4) in float joint_0_weight;"                         "\n"
    "layout(location = 5) in float joint_1_weight;"                         "\n"
    "layout(location = 6) in float joint_2_weight;"                         "\n"
                                                                            "\n"
    "out vec4 color;"                                                       "\n"
                                                                            "\n"
    "void main()"                                                           "\n"
    "{"                                                                     "\n"
    "   vec4 vertex_coords = vec4(position, 0, 1);"                         "\n"
    "   gl_Position = vec4(0,0,0,0);"                                       "\n"
    "   color = vec4(0,0,0,0);"                                             "\n"
                                                                            "\n"
    "   // For each joint affecting this vertex, find the vertex's"         "\n"
    "   // position relative to the joint in bind pose by using"            "\n"
    "   // bind_pose_inv, then transform that position using the"           "\n"
    "   // current pose transform to find the position where the"           "\n"
    "   // vertex should be considering only that joint."                   "\n"
    "   //"                                                                 "\n"
    "   // Take the weighted average of the positions where each"           "\n"
    "   // joint thinks the vertex should be, and that is the"              "\n"
    "   // final vertex position."                                          "\n"
                                                                            "\n"
    "   // first joint affecting vertex"                                    "\n"
    "   color += joint_0_weight * current_pose_colors[joint_0_index];"      "\n"
    "   gl_Position += joint_0_weight * (current_pose[joint_0_index] *"     "\n"
    "                                   bind_pose_inv[joint_0_index] *"     "\n"
    "                                   vertex_coords);"                    "\n"
                                                                            "\n"
    "   // second joint affecting vertex"                                   "\n"
    "   color += joint_1_weight * current_pose_colors[joint_1_index];"      "\n"
    "   gl_Position += joint_1_weight * (current_pose[joint_1_index] *"     "\n"
    "                                   bind_pose_inv[joint_1_index] *"     "\n"
    "                                   vertex_coords);"                    "\n"
                                                                            "\n"
    "   // third joint affecting vertex"                                    "\n"
    "   color += joint_2_weight * current_pose_colors[joint_2_index];"      "\n"
    "   gl_Position += joint_2_weight * (current_pose[joint_2_index] *"     "\n"
    "                                   bind_pose_inv[joint_2_index] *"     "\n"
    "                                   vertex_coords);"                    "\n"
    "}"                                                                     "\n";

const std::string fragment_shader_source = 
    "#version 330"                                                      "\n"
                                                                        "\n"
    "in vec4 color;"                                                    "\n"
                                                                        "\n"
    "layout(location = 0) out vec4 out_fragcolor;"                      "\n"
                                                                        "\n"
    "void main()"                                                       "\n"
    "{"                                                                 "\n"
    "   out_fragcolor = color;"                                         "\n"
    "}"                                                                 "\n";
#pragma endregion

glm::ivec2 viewport;        ///< The current size of the viewport in pixels.

GLuint shader_program_id = 0;
GLint current_pose_uniform_location;
GLint current_pose_colors_uniform_location;

SkeletalMesh* mesh;

bool draw_joints = true;
bool wireframe = false;


// variables relating to poses.
const size_t N_POSES = 3;   ///< The number of different skeleton poses we have available.
Pose poses[N_POSES];        ///< An array of skeleton poses.

size_t left_pose = 2;       ///< The current pose
size_t right_pose = 1;

Pose current_pose;

///////////////////////////////////////////////////////////////////////////////
/// \brief  Generates and returns a matrix which transforms coordinates from
///         a joint's local coordinate space to the joint's parent's coordinate
///         space.
///
/// \param  joint_pose The joint to generate the transform for.
/// \return The joint's local-to-parent transformation matrix.
mat4 getJointLocalTransform(const JointPose& joint_pose)
{
    mat4 transform;

    transform = glm::translate(transform, vec3(joint_pose.translation, 0));

    if (joint_pose.rotation != 0.0f)
        transform = glm::rotate(transform, joint_pose.rotation, vec3(0, 0, 1));

    if (joint_pose.scale != 1.0f)
        transform = glm::scale(transform, vec3(joint_pose.scale,
                                               joint_pose.scale,
                                               joint_pose.scale));

    return transform;
}

///////////////////////////////////////////////////////////////////////////////
/// \brief  Generates and returns a matrix which transforms coordinates from
///         a joint's local coordinate space to model space.
///
/// \param  joint_pose The joint to generate the transform for.
/// \return The joint's local-to-model transformation matrix.
mat4 getJointTransform(const JointPose& joint_pose)
{
    // if this joint has a parent, then lets start in it's
    // coordinate space, otherwise start with an identity matrix.
    if (joint_pose.parent != nullptr)
        return getJointTransform(*joint_pose.parent) * getJointLocalTransform(joint_pose);

    return getJointLocalTransform(joint_pose);
}

///////////////////////////////////////////////////////////////////////////////
/// \brief  Demo entry point.  Initializes the demo, then enters the GLUT
///         main loop.
///
/// \param  argc The number of command line arguments.
/// \param  argv An array of c-strings representing the command line arguments.
/// \return A status code indicating the program completed successfully.
int main(int argc, char** argv)
{
    // freeglut initialization
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
    glutInitWindowPosition(100, 100);
    glutInitWindowSize(800, 800);
    glutCreateWindow("Skeletal Mesh Skinning Demo");

    // GLEW initialization
    GLenum err = glewInit();
    if (err != GLEW_OK)
    {
        std::cerr << "Error intitializing GLEW!" << std::endl;
        return 0;
    }

    initPoses();
    initGL();
   
    glutReshapeFunc(reshape);
    glutDisplayFunc(display);
    glutKeyboardFunc(keyboard);
    glutPassiveMotionFunc(mouseMove);
    glutMotionFunc(mouseMove);
   
    glutMainLoop();

    cleanup();
    return 0;
}

///////////////////////////////////////////////////////////////////////////////
/// \brief  Initializes various OpenGL functionality, like blending, point
///         size, etc.
///
/// \details Additionally, allocates OpenGL objects for the shader program
///         and mesh.
void initGL()
{
    // make projection and modelview matrices the identity matrix
    // for drawing in immediate mode ( glBegin() / glEnd() )
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glClearColor(0, 0, 0, 0);

    glPointSize(10);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);

    initShaderProgram();
    initMeshes();

    GLint bind_pose_inv_uniform_location = glGetUniformLocation(shader_program_id, "bind_pose_inv");
    current_pose_uniform_location        = glGetUniformLocation(shader_program_id, "current_pose");
    current_pose_colors_uniform_location = glGetUniformLocation(shader_program_id, "current_pose_colors");

    mat4 bind_pose_inv_data[7];
    for (size_t joint = 0; joint < 7; ++joint)
    {
        JointPose& jp = poses[0].joints[joint];
        bind_pose_inv_data[joint] = glm::inverse(getJointTransform(jp));
    }

    glUseProgram(shader_program_id);
    glUniformMatrix4fv(bind_pose_inv_uniform_location, 7, GL_FALSE, &bind_pose_inv_data[0][0][0]);
    glUseProgram(0);
}

///////////////////////////////////////////////////////////////////////////////
/// \brief  Compiles and links the vertex and fragment shader into an
///         executable shader program.
void initShaderProgram()
{
    // First, compile vertex/fragment shaders.

    GLuint vert_shader_id = glCreateShader(GL_VERTEX_SHADER);
    GLuint frag_shader_id = glCreateShader(GL_FRAGMENT_SHADER);

    const char* vert_cstr = vertex_shader_source.c_str();
    const char* frag_cstr = fragment_shader_source.c_str();
                                    
    glShaderSource(vert_shader_id, 1, &vert_cstr, NULL);
    glShaderSource(frag_shader_id, 1, &frag_cstr, NULL);

    glCompileShader(vert_shader_id);
    glCompileShader(frag_shader_id);

    // check if there was a problem with vertex shader compilation.
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

    // check if there was a problem with fragment shader compilation.
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

    // Next, link shaders together into a program and delete the individual
    // shaders (we don't need them after the program is linked).

    shader_program_id = glCreateProgram();
    glAttachShader(shader_program_id, vert_shader_id);
    glAttachShader(shader_program_id, frag_shader_id);
    glLinkProgram(shader_program_id);
  //  glDeleteShader(vert_shader_id);
    //glDeleteShader(frag_shader_id);

    // check if there was a problem with linking.
    glGetProgramiv(shader_program_id, GL_LINK_STATUS, &result);
    if (result != GL_TRUE)
    {
        GLint infolog_len;
        glGetProgramiv(shader_program_id, GL_INFO_LOG_LENGTH, &infolog_len);
        char *infolog = new char[std::max(1, infolog_len)];
        glGetProgramInfoLog(shader_program_id, infolog_len, NULL, infolog);

        std::cerr << "Error linking program!" << std::endl
                  << "GL Link Status: " << result << std::endl
                  << "   GL Info Log: " << infolog << std::endl;

        delete[] infolog;

        glDeleteProgram(shader_program_id);
        shader_program_id = 0;
        throw std::runtime_error("Error linking program!");
    }
}

///////////////////////////////////////////////////////////////////////////////
/// \brief  Creates a SkeletalMesh object and loads the skeletal mesh data into
///         graphics memory for use by the vertex shader.
///
/// \details For simplicity's sake, the model was created in Maya and exported
///         as an OBJ file, then manually edited into the source code below.
///
///         A mesh loader would be a much more durable solution for a real
///         game.
void initMeshes()
{
    mesh = new SkeletalMesh();
    Vertex v;
    mesh->vertices.push_back(v);    // unused vertex (the indices used start at 1)

    v.joint_indices[0] = 0;     v.joint_weights[0] = 0.6f;
    v.joint_indices[1] = 1;     v.joint_weights[1] = 0.1f;
    v.joint_indices[2] = 2;     v.joint_weights[2] = 0.1f;
    v.position = vec2( -0.057707,  0.033317);
    mesh->vertices.push_back(v);
    
    v.joint_indices[0] = 0;     v.joint_weights[0] = 0.6f;
    v.joint_indices[1] = 1;     v.joint_weights[1] = 0.1f;
    v.joint_indices[2] = 3;     v.joint_weights[2] = 0.1f;
    v.position = vec2(  0.057707,  0.033317);
    mesh->vertices.push_back(v);



    
    v.joint_indices[0] = 1;     v.joint_weights[0] = 0.2f;
    v.joint_indices[1] = 4;     v.joint_weights[1] = 0.8f;
    v.joint_indices[2] = 0;     v.joint_weights[2] = 0.0f;
    v.position = vec2( -0.042311,  0.784992);
    mesh->vertices.push_back(v);
    v.position = vec2(  0.042311,  0.784992);
    mesh->vertices.push_back(v);

    v.joint_indices[0] = 4;     v.joint_weights[0] = 1.0f;
    v.joint_indices[1] = 1;     v.joint_weights[1] = 0.0f;
    v.position = vec2(  0.000000,  0.935516);
    mesh->vertices.push_back(v);

    v.joint_indices[0] = 1;     v.joint_weights[0] = 0.8f;
    v.joint_indices[1] = 4;     v.joint_weights[1] = 0.2f;
    v.position = vec2( -0.042311,  0.568087);
    mesh->vertices.push_back(v);
    v.position = vec2(  0.042311,  0.568087);
    mesh->vertices.push_back(v);

    v.joint_indices[0] = 1;     v.joint_weights[0] = 0.9f;
    v.joint_indices[1] = 0;     v.joint_weights[1] = 0.1f;
    v.position = vec2( -0.042311,  0.451115);
    mesh->vertices.push_back(v);
    v.position = vec2(  0.042311,  0.451115);
    mesh->vertices.push_back(v);

    v.joint_indices[0] = 0;     v.joint_weights[0] = 0.4f;
    v.joint_indices[1] = 1;     v.joint_weights[1] = 0.6f;
    v.position = vec2( -0.042311,  0.206357);
    mesh->vertices.push_back(v);
    v.position = vec2(  0.042311,  0.206357);
    mesh->vertices.push_back(v);

    v.joint_indices[0] = 1;     v.joint_weights[0] = 0.8f;
    v.joint_indices[1] = 0;     v.joint_weights[1] = 0.2f;
    v.position = vec2( -0.042311,  0.328522);
    mesh->vertices.push_back(v);
    v.position = vec2(  0.042311,  0.328522);
    mesh->vertices.push_back(v);

    v.joint_indices[0] = 1;     v.joint_weights[0] = 0.6f;
    v.joint_indices[1] = 4;     v.joint_weights[1] = 0.4f;
    v.position = vec2( -0.042311,  0.681860);
    mesh->vertices.push_back(v);
    v.position = vec2(  0.042311,  0.681860);
    mesh->vertices.push_back(v);




    v.joint_indices[0] = 0;     v.joint_weights[0] = 0.6f;
    v.joint_indices[1] = 2;     v.joint_weights[1] = 0.1f;
    v.joint_indices[2] = 3;     v.joint_weights[2] = 0.1f;
    v.position = vec2(  0.000000, -0.066635);
    mesh->vertices.push_back(v);





    v.joint_indices[0] = 3;     v.joint_weights[0] = 0.2f;
    v.joint_indices[1] = 6;     v.joint_weights[1] = 0.8f;
    v.joint_indices[2] = 0;     v.joint_weights[2] = 0.0f;
    v.position = vec2(  0.700979, -0.355853);
    mesh->vertices.push_back(v);
    v.position = vec2(  0.658668, -0.429139);
    mesh->vertices.push_back(v);

    v.joint_indices[0] = 6;     v.joint_weights[0] = 1.0f;
    v.joint_indices[1] = 0;     v.joint_weights[1] = 0.0f;
    v.position = vec2(  0.810180, -0.467758);
    mesh->vertices.push_back(v);

    v.joint_indices[0] = 3;     v.joint_weights[0] = 0.8f;
    v.joint_indices[1] = 6;     v.joint_weights[1] = 0.2f;
    v.position = vec2(  0.513133, -0.247401);
    mesh->vertices.push_back(v);
    v.position = vec2(  0.470822, -0.320686);
    mesh->vertices.push_back(v);

    v.joint_indices[0] = 3;     v.joint_weights[0] = 0.9f;
    v.joint_indices[1] = 0;     v.joint_weights[1] = 0.1f;
    v.position = vec2(  0.411833, -0.188915);
    mesh->vertices.push_back(v);
    v.position = vec2(  0.369521, -0.262200);
    mesh->vertices.push_back(v);

    v.joint_indices[0] = 0;     v.joint_weights[0] = 0.4f;
    v.joint_indices[1] = 3;     v.joint_weights[1] = 0.6f;
    v.position = vec2(  0.199866, -0.066536);
    mesh->vertices.push_back(v);
    v.position = vec2(  0.157554, -0.139821);
    mesh->vertices.push_back(v);

    v.joint_indices[0] = 3;     v.joint_weights[0] = 0.8f;
    v.joint_indices[1] = 0;     v.joint_weights[1] = 0.2f;
    v.position = vec2(  0.305664, -0.127618);
    mesh->vertices.push_back(v);
    v.position = vec2(  0.263353, -0.200904);
    mesh->vertices.push_back(v);

    v.joint_indices[0] = 3;     v.joint_weights[0] = 0.6f;
    v.joint_indices[1] = 6;     v.joint_weights[1] = 0.4f;
    v.position = vec2(  0.611663, -0.304287);
    mesh->vertices.push_back(v);
    v.position = vec2(  0.569352, -0.377572);
    mesh->vertices.push_back(v);




    v.joint_indices[0] = 2;     v.joint_weights[0] = 0.2f;
    v.joint_indices[1] = 5;     v.joint_weights[1] = 0.8f;
    v.joint_indices[2] = 0;     v.joint_weights[2] = 0.0f;
    v.position = vec2( -0.658668, -0.429139);
    mesh->vertices.push_back(v);
    v.position = vec2( -0.700979, -0.355853);
    mesh->vertices.push_back(v);

    v.joint_indices[0] = 5;     v.joint_weights[0] = 1.0f;
    v.joint_indices[1] = 0;     v.joint_weights[1] = 0.0f;
    v.position = vec2( -0.810180, -0.467758);
    mesh->vertices.push_back(v);

    v.joint_indices[0] = 2;     v.joint_weights[0] = 0.8f;
    v.joint_indices[1] = 5;     v.joint_weights[1] = 0.2f;
    v.position = vec2( -0.470822, -0.320686);
    mesh->vertices.push_back(v);
    v.position = vec2( -0.513133, -0.247401);
    mesh->vertices.push_back(v);

    v.joint_indices[0] = 2;     v.joint_weights[0] = 0.9f;
    v.joint_indices[1] = 0;     v.joint_weights[1] = 0.1f;
    v.position = vec2( -0.369521, -0.262200);
    mesh->vertices.push_back(v);
    v.position = vec2( -0.411833, -0.188915);
    mesh->vertices.push_back(v);

    v.joint_indices[0] = 0;     v.joint_weights[0] = 0.4f;
    v.joint_indices[1] = 2;     v.joint_weights[1] = 0.6f;
    v.position = vec2( -0.157554, -0.139821);
    mesh->vertices.push_back(v);
    v.position = vec2( -0.199866, -0.066536);
    mesh->vertices.push_back(v);

    v.joint_indices[0] = 2;     v.joint_weights[0] = 0.8f;
    v.joint_indices[1] = 0;     v.joint_weights[1] = 0.2f;
    v.position = vec2( -0.263353, -0.200904);
    mesh->vertices.push_back(v);
    v.position = vec2( -0.305664, -0.127618);
    mesh->vertices.push_back(v);

    v.joint_indices[0] = 2;     v.joint_weights[0] = 0.6f;
    v.joint_indices[1] = 5;     v.joint_weights[1] = 0.4f;
    v.position = vec2( -0.569352, -0.377572);
    mesh->vertices.push_back(v);
    v.position = vec2( -0.611663, -0.304287);
    mesh->vertices.push_back(v);

    mesh->indices.push_back(1 ); mesh->indices.push_back(2 ); mesh->indices.push_back(10);
    mesh->indices.push_back(10); mesh->indices.push_back(2 ); mesh->indices.push_back(11);
    mesh->indices.push_back(3 ); mesh->indices.push_back(4 ); mesh->indices.push_back(5 );
    mesh->indices.push_back(14); mesh->indices.push_back(15); mesh->indices.push_back(3 );
    mesh->indices.push_back(3 ); mesh->indices.push_back(15); mesh->indices.push_back(4 );
    mesh->indices.push_back(2 ); mesh->indices.push_back(16); mesh->indices.push_back(24);
    mesh->indices.push_back(24); mesh->indices.push_back(16); mesh->indices.push_back(25);
    mesh->indices.push_back(17); mesh->indices.push_back(18); mesh->indices.push_back(19);
    mesh->indices.push_back(28); mesh->indices.push_back(29); mesh->indices.push_back(17);
    mesh->indices.push_back(17); mesh->indices.push_back(29); mesh->indices.push_back(18);
    mesh->indices.push_back(16); mesh->indices.push_back(1 ); mesh->indices.push_back(37);
    mesh->indices.push_back(37); mesh->indices.push_back(1 ); mesh->indices.push_back(38);
    mesh->indices.push_back(30); mesh->indices.push_back(31); mesh->indices.push_back(32);
    mesh->indices.push_back(41); mesh->indices.push_back(42); mesh->indices.push_back(30);
    mesh->indices.push_back(30); mesh->indices.push_back(42); mesh->indices.push_back(31);
    mesh->indices.push_back(2 ); mesh->indices.push_back(1 ); mesh->indices.push_back(16);
    mesh->indices.push_back(33); mesh->indices.push_back(34); mesh->indices.push_back(41);
    mesh->indices.push_back(41); mesh->indices.push_back(34); mesh->indices.push_back(42);
    mesh->indices.push_back(37); mesh->indices.push_back(38); mesh->indices.push_back(39);
    mesh->indices.push_back(39); mesh->indices.push_back(38); mesh->indices.push_back(40);
    mesh->indices.push_back(33); mesh->indices.push_back(36); mesh->indices.push_back(34);
    mesh->indices.push_back(40); mesh->indices.push_back(35); mesh->indices.push_back(39);
    mesh->indices.push_back(36); mesh->indices.push_back(33); mesh->indices.push_back(35);
    mesh->indices.push_back(35); mesh->indices.push_back(40); mesh->indices.push_back(36);
    mesh->indices.push_back(10); mesh->indices.push_back(11); mesh->indices.push_back(12);
    mesh->indices.push_back(12); mesh->indices.push_back(11); mesh->indices.push_back(13);
    mesh->indices.push_back(6 ); mesh->indices.push_back(7 ); mesh->indices.push_back(14);
    mesh->indices.push_back(14); mesh->indices.push_back(7 ); mesh->indices.push_back(15);
    mesh->indices.push_back(6 ); mesh->indices.push_back(9 ); mesh->indices.push_back(7 );
    mesh->indices.push_back(8 ); mesh->indices.push_back(13); mesh->indices.push_back(9 );
    mesh->indices.push_back(13); mesh->indices.push_back(8 ); mesh->indices.push_back(12);
    mesh->indices.push_back(9 ); mesh->indices.push_back(6 ); mesh->indices.push_back(8 );
    mesh->indices.push_back(24); mesh->indices.push_back(25); mesh->indices.push_back(26);
    mesh->indices.push_back(26); mesh->indices.push_back(25); mesh->indices.push_back(27);
    mesh->indices.push_back(20); mesh->indices.push_back(21); mesh->indices.push_back(28);
    mesh->indices.push_back(28); mesh->indices.push_back(21); mesh->indices.push_back(29);
    mesh->indices.push_back(20); mesh->indices.push_back(23); mesh->indices.push_back(21);
    mesh->indices.push_back(22); mesh->indices.push_back(27); mesh->indices.push_back(23);
    mesh->indices.push_back(27); mesh->indices.push_back(22); mesh->indices.push_back(26);
    mesh->indices.push_back(23); mesh->indices.push_back(20); mesh->indices.push_back(22);

    mesh->uploadMesh();
}

///////////////////////////////////////////////////////////////////////////////
/// \brief  Initializes the skeleton poses.
///
/// \details The first pose is the "bind pose" which is the pose that the
///         raw vertex positions are in.  The next two poses are deformed
///         versions of the bind pose with rotations changed.  In addition
///         to rotations, translations and scale may also be changed.
void initPoses()
{
    // poses[0] => bind pose.
    poses[0].joints[0].color = color4(1.0f, 1.0f, 1.0f, 1.0f);
    poses[0].joints[0].parent = nullptr;
    poses[0].joints[0].translation = vec2(0, 0);
    poses[0].joints[0].rotation = 0.0f;
    poses[0].joints[0].scale = 1.0f;

    poses[0].joints[1] = poses[0].joints[0];
    poses[0].joints[1].color = color4(1.0f, 0.0f, 0.0f, 1.0f);
    poses[0].joints[1].parent = &poses[0].joints[0];
    poses[0].joints[1].translation = vec2(0, 0.206357);
    poses[0].joints[1].rotation = 90.0f;

    poses[0].joints[2] = poses[0].joints[0];
    poses[0].joints[2].color = color4(0.0f, 1.0f, 0.0f, 1.0f);
    poses[0].joints[2].parent = &poses[0].joints[0];
    poses[0].joints[2].translation = vec2(-0.178710, -0.103178);
    poses[0].joints[2].rotation = 210.0f;

    poses[0].joints[3] = poses[0].joints[0];
    poses[0].joints[3].color = color4(0.0f, 0.0f, 1.0f, 1.0f);
    poses[0].joints[3].parent = &poses[0].joints[0];
    poses[0].joints[3].translation = vec2(0.178710, -0.103178);
    poses[0].joints[3].rotation = -30.0f;

    poses[0].joints[4] = poses[0].joints[1];
    poses[0].joints[4].color = color4(1.0f, 1.0f, 0.0f, 1.0f);
    poses[0].joints[4].parent = &poses[0].joints[1];
    poses[0].joints[4].translation = vec2(0.475503, 0);
    poses[0].joints[4].rotation = 0.0f;

    poses[0].joints[5] = poses[0].joints[4];
    poses[0].joints[5].color = color4(0.0f, 1.0f, 1.0f, 1.0f);
    poses[0].joints[5].parent = &poses[0].joints[2];

    poses[0].joints[6] = poses[0].joints[4];
    poses[0].joints[6].color = color4(1.0f, 0.0f, 1.0f, 1.0f);
    poses[0].joints[6].parent = &poses[0].joints[3];

    poses[1] = poses[0];
    poses[1].joints[1].parent = &poses[1].joints[0];
    poses[1].joints[1].rotation = 45.0f;

    poses[1].joints[2].parent = &poses[1].joints[0];
    poses[1].joints[2].rotation = 165.0f;

    poses[1].joints[3].parent = &poses[1].joints[0];
    poses[1].joints[3].rotation = -75.0f;

    poses[1].joints[4].parent = &poses[1].joints[1];
    poses[1].joints[4].rotation = -45.0f;

    poses[1].joints[5].parent = &poses[1].joints[2];
    poses[1].joints[5].rotation = -45.0f;

    poses[1].joints[6].parent = &poses[1].joints[3];
    poses[1].joints[6].rotation = -45.0f;


    poses[2] = poses[0];
    poses[2].joints[1].parent = &poses[2].joints[0];
    poses[2].joints[1].rotation = 135.0f;

    poses[2].joints[2].parent = &poses[2].joints[0];
    poses[2].joints[2].rotation = 255.0f;

    poses[2].joints[3].parent = &poses[2].joints[0];
    poses[2].joints[3].rotation = 15.0f;

    poses[2].joints[4].parent = &poses[2].joints[1];
    poses[2].joints[4].rotation = 45.0f;

    poses[2].joints[5].parent = &poses[2].joints[2];
    poses[2].joints[5].rotation = 45.0f;

    poses[2].joints[6].parent = &poses[2].joints[3];
    poses[2].joints[6].rotation = 45.0f;

    current_pose = poses[0];
}

///////////////////////////////////////////////////////////////////////////////
/// \brief  Cleans up the demo in preparation for exit.  Releases all OpenGL
///         resources remaining.
void cleanup()
{
    if (shader_program_id != 0)
    {
        glDeleteProgram(shader_program_id);
        shader_program_id = 0;
    }

    delete mesh;
}

///////////////////////////////////////////////////////////////////////////////
/// \brief  GLUT callback handling window resize events.
///
/// \param width The window's new width.
/// \param height The window's new height.
void reshape(int width, int height)
{
    glutReshapeWindow(width, height);

    viewport.x = width;
    viewport.y = height;
    glViewport(0, 0, width, height);

    glutPostRedisplay();
}

///////////////////////////////////////////////////////////////////////////////
/// \brief  GLUT callback to render a frame.
///
/// \details There is remarkably little needed to render a skeletal mesh:
///         just bind the right shader program and vertex array and make sure
///         the shader's uniforms are up-to-date, then call glDrawElements.
void display()
{
    glClear(GL_COLOR_BUFFER_BIT);

    glUseProgram(shader_program_id);
    glBindVertexArray(mesh->vao_id);

    mat4 current_pose_data[7];
    vec4 current_pose_color_data[7];
    for (size_t joint = 0; joint < 7; ++joint)
    {
        JointPose& jp = current_pose.joints[joint];
        current_pose_data[joint] = getJointTransform(jp);
        current_pose_color_data[joint] = jp.color;
    }

    glUniformMatrix4fv(current_pose_uniform_location, 7, GL_FALSE, &current_pose_data[0][0][0]);
    glUniform4fv(current_pose_colors_uniform_location, 7, &current_pose_color_data[0][0]);

    glDrawElements(GL_TRIANGLES, mesh->indices.size(), GL_UNSIGNED_SHORT, 0);

    glBindVertexArray(0);
    glUseProgram(0);

    // draw joints/bones using immediate mode, because it's just for debugging purposes.
    if (draw_joints)
    {
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();

        for (size_t joint = 0; joint < 7; ++joint)
        {
            JointPose& joint_pose = current_pose.joints[joint];

            mat4 transform = getJointTransform(joint_pose);
            glLoadMatrixf(&transform[0][0]);

            if (joint_pose.parent)
            {
                mat4 parent_to_local_transform = glm::inverse(getJointLocalTransform(joint_pose));
                vec4 parent_position = parent_to_local_transform * vec4(0, 0, 0, 1);

                if (parent_position.x != 0 || parent_position.y != 0)
                {
                    vec4 tangent = glm::normalize(vec4(parent_position.y, -parent_position.x, 0, 1)) * 0.05f;
                    vec4 parent_0 = parent_position + tangent;
                    vec4 parent_1 = parent_position - tangent;

                    glBegin(GL_LINES);
                        glColor4fv(&joint_pose.parent->color[0]);
                        glVertex2fv(&parent_0[0]);

                        glColor4fv(&joint_pose.color[0]);
                        glVertex2f(0, 0);
                        glVertex2f(0, 0);

                        glColor4fv(&joint_pose.parent->color[0]);
                        glVertex2fv(&parent_1[0]);
                    glEnd();
                }
            }

            glBegin(GL_LINES);
                glColor4f(1, 0, 0, 1);
                glVertex2f(0.0f, 0.0f);
                glVertex2f(0.1f, 0.0f);

                glColor4f(0, 1, 0, 1);
                glVertex2f(0.0f, 0.0f);
                glVertex2f(0.0f, 0.1f);
            glEnd();

            glColor4fv(&(joint_pose.color[0]));
            glBegin(GL_POINTS);
                glVertex2f(0, 0);
            glEnd();
        }

        glPopMatrix();
    }

    glutSwapBuffers();
}

///////////////////////////////////////////////////////////////////////////////
/// \brief  GLUT callback handing keyboard input keypresses.
///
/// \param  key The ASCII value of the character pressed.
/// \param  x The x-coordinate of the mouse when the event occured.
/// \param  y The y-coordinate of the mouse when the event occured.
void keyboard(unsigned char key, int x, int y)
{
    switch (key)
    {
        case 27:
            glutLeaveMainLoop();
            return;

        case 'w':
            wireframe = !wireframe;
            glPolygonMode(GL_FRONT_AND_BACK, wireframe ? GL_LINE : GL_FILL);
            break;

        case 'j':
            draw_joints = !draw_joints;
            break;

        case 'h':
            std::cerr << "Skeletal Mesh Skinning Demo" << std::endl << std::endl
                      << "    H - Display this message." << std::endl
                      << "    W - Toggle wireframe mode." << std::endl
                      << "    J - Toggle joint/bone debug rendering." << std::endl
                      << "  Esc - Exit" << std::endl << std::endl;
            break;

        default:
            break;
    }

    glutPostRedisplay();
}

///////////////////////////////////////////////////////////////////////////////
/// \brief  GLUT callback handling mouse motion.
///
/// \param  x The x-coordinate of the mouse when the event occured.
/// \param  y The y-coordinate of the mouse when the event occured.
void mouseMove(int x, int y)
{
    float f = float(x) / viewport.x;
    float g = 1 - f;

    current_pose = poses[left_pose];
    for (size_t joint = 0; joint < 7; ++joint)
    {
        JointPose& jp = current_pose.joints[joint];
        if (jp.parent)
        {
            ptrdiff_t offset = reinterpret_cast<char*>(jp.parent) - reinterpret_cast<char*>(&poses[left_pose]);
            jp.parent = reinterpret_cast<JointPose*>(reinterpret_cast<char*>(&current_pose) + offset);
        }

        current_pose.joints[joint].color = poses[left_pose].joints[joint].color * g +
                                           poses[right_pose].joints[joint].color * f;

        current_pose.joints[joint].rotation = poses[left_pose].joints[joint].rotation * g +
                                              poses[right_pose].joints[joint].rotation * f;

        current_pose.joints[joint].scale = poses[left_pose].joints[joint].scale * g +
                                           poses[right_pose].joints[joint].scale * f;

        current_pose.joints[joint].translation = poses[left_pose].joints[joint].translation * g +
                                                 poses[right_pose].joints[joint].translation * f;
    }

    glutPostRedisplay();
}

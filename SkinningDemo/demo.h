///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
/// \file:  demo.h
/// \author Ben Crist
///
/// \brief  Standard header for Skeletal Mesh Skinning Demo.
///
/// \details Includes GLEW and freeglut, and creates global aliases of the
///         GLM data types we'll be needing to work with.

#ifndef DEMO_H_
#define DEMO_H_

///////////////////////////////////////////////////////////////////////////////
// Includes
#include <GL/glew.h>
#include <GL/freeglut.h>

// OpenGL Mathematics library
// Provides C++ analogs of GLSL vector and matrix data types
// as well as equivalents for things like gluPerspective().
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

///////////////////////////////////////////////////////////////////////////////
// GLM vector & matrix typedefs for convenience.
typedef glm::vec2 vec2;     ///< 2-component vector of floats
typedef glm::vec3 vec3;     ///< 3-component vector of floats
typedef glm::vec4 vec4;     ///< 4-component vector of floats
typedef glm::vec4 color4;   ///< 4-component vector of floats representing an RGBA color.
typedef glm::mat4 mat4;     ///< 4x4 matrix of floats

#endif

#ifndef GL_HELPERS_H
#define GL_HELPERS_H

#include <GL/glew.h>
#include <GL/gl.h>

// Include portable printf-style format macros
#define __STDC_FORMAT_MACROS

#ifdef _GCC_
#  define GL_DEPRECATED __attribute__((deprecated))
#elif defined _MSVC_
#  define GL_DEPRECATED __declspec(deprecated)
#else
#  define GL_DEPRECATED
#endif


#define GL_EXPORT

#define GL_UNUSED(x) (void)(x)

#ifdef _APPLE_IOS_
// Not supported on this platform.
#define __thread
#endif // _APPLE_IOS_

// HOST / DEVICE Annotations
#ifdef __CUDACC__
#  include <cuda_runtime.h>
#  define GL_HOST_DEVICE __host__ __device__
#else
#  define GL_HOST_DEVICE
#endif

// Non-standard check that header exists (Clang, GCC 5.X)
// Useful for
#if defined(__has_include)
#  define GL_HEADER_EXISTS(x) __has_include(x)
#else
#  define GL_HEADER_EXISTS(x) 0
#endif

#include <gl/gl_assert.h>
#include <gl/gl_log.h>

#define CheckGlDieOnError() gl::_CheckGlDieOnError( __FILE__, __LINE__ );
namespace gl {
inline void _CheckGlDieOnError( const char *sFile, const int nLine )
{
    GLenum glError = glGetError();
    if( glError != GL_NO_ERROR ) {
//        gl_print_error( "OpenGL Error: %s (%d)\n", glErrorString(glError), glError );
        gl_print_error("In: %s, line %d\n", sFile, nLine);
    }
}
}

#endif // GL_HELPERS_H

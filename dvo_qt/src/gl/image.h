/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2011 Steven Lovegrove
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once

#include <gl/gl_helpers.h>
#include <gl/gl_memcpy.h>

#include <cstddef>
#include <functional>
#include <limits>
#include <cstring>

#ifdef GL_ENABLE_BOUNDS_CHECKS
#   define GL_BOUNDS_ASSERT(...) GL_ENSURE(##__VA_ARGS__)
#else
#   define GL_BOUNDS_ASSERT(...) ((void)0)
#endif

// Allow user defined macro to be inserted into Image class.
#ifndef GL_EXTENSION_IMAGE
#  define GL_EXTENSION_IMAGE
#endif

namespace gl
{

// Simple image wrapper
template<typename T>
struct Image
{
    inline Image()
        : pitch(0), ptr(0), w(0), h(0)
    {
    }

    inline Image(T* ptr, size_t w, size_t h, size_t pitch)
        : pitch(pitch), ptr(ptr), w(w), h(h)
    {
    }


    GL_HOST_DEVICE inline
    size_t SizeBytes() const
    {
        return pitch * h;
    }
    
    GL_HOST_DEVICE inline
    size_t Area() const
    {
        return w * h;
    }

    GL_HOST_DEVICE inline
    bool IsValid() const
    {
        return ptr != 0;
    }

    GL_HOST_DEVICE inline
    bool IsContiguous() const
    {
        return w*sizeof(T) == pitch;
    }

    //////////////////////////////////////////////////////
    // Iterators
    //////////////////////////////////////////////////////

    GL_HOST_DEVICE inline
    T* begin()
    {
        return ptr;
    }

    GL_HOST_DEVICE inline
    T* end()
    {
        return RowPtr(h-1) + w;
    }

    GL_HOST_DEVICE inline
    const T* begin() const
    {
        return ptr;
    }

    GL_HOST_DEVICE inline
    const T* end() const
    {
        return RowPtr(h-1) + w;
    }

    GL_HOST_DEVICE inline
    size_t size() const
    {
        return w*h;
    }

    //////////////////////////////////////////////////////
    // Image transforms
    //////////////////////////////////////////////////////

    template<typename UnaryOperation>
    GL_HOST_DEVICE inline
    void Transform(UnaryOperation unary_op)
    {
        PANGO_ASSERT(IsValid());

        for(size_t y=0; y < h; ++y) {
            T* el = RowPtr(y);
            const T* el_end = el+w;
            for( ; el != el_end; ++el) {
                *el = unary_op(*el);
            }
        }
    }

    GL_HOST_DEVICE inline
    void Fill(const T& val)
    {
        Transform( [&](const T&) {return val;} );
    }

    GL_HOST_DEVICE inline
    void Replace(const T& oldval, const T& newval)
    {
        Transform( [&](const T& val) {
            return (val == oldval) ? newval : val;
        });
    }

    inline
    void Memset(unsigned char v = 0)
    {
        GL_ASSERT(IsValid());
        if(IsContiguous()) {
            ::gl::Memset((char*)ptr, v, pitch*h);
        }else{
            for(size_t y=0; y < h; ++y) {
                ::gl::Memset((char*)RowPtr(y), v, pitch);
            }
        }
    }

    inline
    void CopyFrom(const Image<T>& img)
    {
        if(IsValid() && img.IsValid()) {
            GL_ASSERT(w >= img.w && h >= img.h);
            PitchedCopy((char*)ptr,pitch,(char*)img.ptr,img.pitch, std::min(img.w,w)*sizeof(T), std::min(img.h,h) );
        }else if( img.IsValid() != IsValid() ){
            GL_ASSERT(false && "Cannot copy from / to an unasigned image." );
        }
    }

    //////////////////////////////////////////////////////
    // Reductions
    //////////////////////////////////////////////////////

    template<typename BinaryOperation>
    GL_HOST_DEVICE inline
    T Accumulate(const T init, BinaryOperation binary_op)
    {
        GL_ASSERT(IsValid());

        T val = init;
        for(size_t y=0; y < h; ++y) {
            T* el = RowPtr(y);
            const T* el_end = el+w;
            for(; el != el_end; ++el) {
                val = binary_op(val, *el);
            }
        }
        return val;
    }

    std::pair<T,T> MinMax() const
    {
        GL_ASSERT(IsValid());

        const std::pair<T,T> minmax(std::numeric_limits<T>::max(), std::numeric_limits<T>::lowest());
        for(size_t r=0; r < h; ++r) {
            const T* ptr = RowPtr(r);
            const T* end = ptr + w;
            while( ptr != end) {
                minmax.first  = std::min(*ptr, minmax.first);
                minmax.second = std::max(*ptr, minmax.second);
                ++ptr;
            }
        }
        return minmax;
    }

    template<typename Tout=T>
    Tout Sum() const
    {
        return Accumulate((T)0, [](const T& lhs, const T& rhs){ return lhs + rhs; });
    }

    template<typename Tout=T>
    Tout Mean() const
    {
        return Sum<Tout>() / Area();
    }


    //////////////////////////////////////////////////////
    // Direct Pixel Access
    //////////////////////////////////////////////////////

    GL_HOST_DEVICE inline
    T* RowPtr(size_t y)
    {
        return (T*)((unsigned char*)(ptr) + y*pitch);
    }

    GL_HOST_DEVICE inline
    const T* RowPtr(size_t y) const
    {
        return (T*)((unsigned char*)(ptr) + y*pitch);
    }

    GL_HOST_DEVICE inline
    T& operator()(size_t x, size_t y)
    {
        GL_BOUNDS_ASSERT( InBounds(x,y) );
        return RowPtr(y)[x];
    }

    GL_HOST_DEVICE inline
    const T& operator()(size_t x, size_t y) const
    {
        GL_BOUNDS_ASSERT( InBounds(x,y) );
        return RowPtr(y)[x];
    }

    template<typename TVec>
    GL_HOST_DEVICE inline
    T& operator()(const TVec& p)
    {
        GL_BOUNDS_ASSERT( InBounds(p[0],p[1]) );
        return RowPtr(p[1])[p[0]];
    }

    template<typename TVec>
    GL_HOST_DEVICE inline
    const T& operator()(const TVec& p) const
    {
        GL_BOUNDS_ASSERT( InBounds(p[0],p[1]) );
        return RowPtr(p[1])[p[0]];
    }

    GL_HOST_DEVICE inline
    T& operator[](size_t ix)
    {
        PANGO_BOUNDS_ASSERT( InImage(ptr+ix) );
        return ptr[ix];
    }

    GL_HOST_DEVICE inline
    const T& operator[](size_t ix) const
    {
        PANGO_BOUNDS_ASSERT( InImage(ptr+ix) );
        return ptr[ix];
    }

    //////////////////////////////////////////////////////
    // Bounds Checking
    //////////////////////////////////////////////////////

    GL_HOST_DEVICE
    bool InImage(const T* ptest) const
    {
        return ptr <= ptest && ptest < RowPtr(h);
    }

    GL_HOST_DEVICE inline
    bool InBounds(int x, int y) const
    {
        return 0 <= x && x < (int)w && 0 <= y && y < (int)h;
    }

    GL_HOST_DEVICE inline
    bool InBounds(float x, float y, float border) const
    {
        return border <= x && x < (w-border) && border <= y && y < (h-border);
    }

    template<typename TVec, typename TBorder>
    GL_HOST_DEVICE inline
    bool InBounds( const TVec& p, const TBorder border = (TBorder)0 ) const
    {
        return  border <= p[0] && p[0] < ((int)w - border) &&  border <= p[1] && p[1] < ((int)h - border);
    }

    //////////////////////////////////////////////////////
    // Obtain slices / subimages
    //////////////////////////////////////////////////////

    GL_HOST_DEVICE inline
    const Image<const T> SubImage(size_t x, size_t y, size_t width, size_t height) const
    {
        GL_ASSERT( (x+width) <= w && (y+height) <= h);
        return Image<const T>( RowPtr(y)+x, width, height, pitch);
    }

    GL_HOST_DEVICE inline
    Image<T> SubImage(size_t x, size_t y, size_t width, size_t height)
    {
        GL_ASSERT( (x+width) <= w && (y+height) <= h);
        return Image<T>( RowPtr(y)+x, width, height, pitch);
    }

    GL_HOST_DEVICE inline
    Image<T> Row(int y) const
    {
        return SubImage(0,y,w,1);
    }

    GL_HOST_DEVICE inline
    Image<T> Col(int x) const
    {
        return SubImage(x,0,1,h);
    }

    //////////////////////////////////////////////////////
    // Data mangling
    //////////////////////////////////////////////////////

    template<typename TRecast>
    GL_HOST_DEVICE inline
    Image<TRecast> Reinterpret() const
    {
        GL_ASSERT(sizeof(TRecast) == sizeof(T), "sizeof(TRecast) must match sizeof(T): % != %", sizeof(TRecast), sizeof(T) );
        return UnsafeReinterpret<TRecast>();
    }

    template<typename TRecast>
    GL_HOST_DEVICE inline
    Image<TRecast> UnsafeReinterpret() const
    {
        return Image<TRecast>((TRecast*)ptr,w,h,pitch);
    }

    //////////////////////////////////////////////////////
    // Deprecated methods
    //////////////////////////////////////////////////////

//    PANGOLIN_DEPRECATED inline
    Image(size_t w, size_t h, size_t pitch, T* ptr)
        : pitch(pitch), ptr(ptr), w(w), h(h)
    {
    }

    // Use RAII/move aware pangolin::ManagedImage instead
//    PANGOLIN_DEPRECATED inline
    void Dealloc()
    {
        if(ptr) {
            ::operator delete(ptr);
            ptr = nullptr;
        }
    }

    // Use RAII/move aware pangolin::ManagedImage instead
//    PANGOLIN_DEPRECATED inline
    void Alloc(size_t w, size_t h, size_t pitch)
    {
        Dealloc();
        this->w = w;
        this->h = h;
        this->pitch = pitch;
        this->ptr = (T*)::operator new(h*pitch);
    }

    //////////////////////////////////////////////////////
    // Data members
    //////////////////////////////////////////////////////

    size_t pitch;
    T* ptr;
    size_t w;
    size_t h;

    GL_EXTENSION_IMAGE
};

}

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
#include <gl/view.h>

#include <functional>
#include <memory>

#include <map>
#include <queue>

#ifdef BUILD_PANGOLIN_VIDEO
#  include <pangolin/video/video_output.h>
#endif // BUILD_PANGOLIN_VIDEO


namespace gl
{

typedef std::map<const std::string,View*> ViewMap;
typedef std::map<int,std::function<void(void)> > KeyhookMap;

struct GL_EXPORT GLContext
{
    GLContext();
    ~GLContext();
    
    // Base container for displays
    View base;
    
    // Named views which are managed by pangolin (i.e. created / deleted by pangolin)
    ViewMap named_managed_views;

};

}


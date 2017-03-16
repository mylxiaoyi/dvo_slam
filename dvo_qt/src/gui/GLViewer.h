#ifndef GLVIEWER_H
#define GLVIEWER_H

#include <GL/glew.h>
#include <QGLViewer/qglviewer.h>

class GLViewer : public QGLViewer
{
public:
    GLViewer(QWidget *parent = 0);
    virtual ~GLViewer();

protected:
    void init();
    void draw();
    void postDraw();

    void drawCurrentFrame();
};

#endif // GLVIEWER_H

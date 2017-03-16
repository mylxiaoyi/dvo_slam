#include "GLViewer.h"
#include "utils/UCommand.h"
#include <glog/logging.h>

void glViewerHandle (void* ptr, std::string command, std::string sParams)
{
    GLViewer* thisptr = (GLViewer*)ptr;
    if (command == "DrawText")
    {
        glDisable (GL_LIGHTING);
        glDisable (GL_DEPTH_TEST);
        glColor3f (0.0, 0.0, 0.0);
        if (thisptr)
            thisptr->drawText (10, 20, QString::fromStdString (sParams));
    }
}

GLViewer::GLViewer (QWidget* parent) : QGLViewer (parent)
{
    setMinimumSize(QSize(1280, 960));
    ucommand.RegisterCommand ("DrawText", glViewerHandle, this);
}

GLViewer::~GLViewer ()
{
}

void GLViewer::init ()
{
    glDisable (GL_LIGHTING);
    this->setBackgroundColor (QColor (Qt::white));

    glewInit();
}

void GLViewer::draw ()
{
    glViewport (0, 0, this->width (), this->height ());

    ucommand.Call ("DrawOpenGL", "");

}

void GLViewer::drawCurrentFrame ()
{
    //    int viewport[4];
    //    int scissor[4];

    //    // The viewport and the scissor are changed to fit the lower left
    //    // corner. Original values are saved.
    //    glGetIntegerv(GL_VIEWPORT, viewport);
    //    glGetIntegerv(GL_SCISSOR_BOX, scissor);

    //    // Axis viewport size, in pixels
    //    const int width = 320;
    //    const int height = 240;
    //    glViewport(0,0,width,height);
    //    glScissor(0,0,width,height);

    //    // The Z-buffer is cleared to make the axis appear over the
    //    // original image.
    //    glClear(GL_DEPTH_BUFFER_BIT);

    //    // Tune for best line rendering
    ////        glDisable(GL_LIGHTING);
    ////        glLineWidth(3.0);

    //    glMatrixMode(GL_PROJECTION);
    //    glPushMatrix();
    //    glLoadIdentity();
    //    glOrtho(-1, 1, -1, 1, -1, 1);

    //    glMatrixMode(GL_MODELVIEW);
    //    glPushMatrix();
    //    glLoadIdentity();
    //    glMultMatrixd(camera()->orientation().inverse().matrix());

    ////        glBegin(GL_LINES);
    ////        glColor3f(1.0, 0.0, 0.0);
    ////        glVertex3f(0.0, 0.0, 0.0);
    ////        glVertex3f(1.0, 0.0, 0.0);

    ////        glColor3f(0.0, 1.0, 0.0);
    ////        glVertex3f(0.0, 0.0, 0.0);
    ////        glVertex3f(0.0, 1.0, 0.0);

    ////        glColor3f(0.0, 0.0, 1.0);
    ////        glVertex3f(0.0, 0.0, 0.0);
    ////        glVertex3f(0.0, 0.0, 1.0);
    ////        glEnd();
    //    if (mpMapDrawer != NULL)
    //        mpMapDrawer->DrawCurrentFrame();

    //    if (mpMyMapDrawer != NULL)
    //        mpMyMapDrawer->DrawCurrentFrame();

    //    glMatrixMode(GL_PROJECTION);
    //    glPopMatrix();

    //    glMatrixMode(GL_MODELVIEW);
    //    glPopMatrix();

    ////        glEnable(GL_LIGHTING);

    //    // The viewport and the scissor are restored.
    //    glScissor(scissor[0],scissor[1],scissor[2],scissor[3]);
    //    glViewport(viewport[0],viewport[1],viewport[2],viewport[3]);
}

void GLViewer::postDraw ()
{
    QGLViewer::postDraw ();

    //    drawCurrentFrame ();
}

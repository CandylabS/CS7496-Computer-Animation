#include <string>
#include <iostream>
#include <stdlib.h>

#if defined(__APPLE__) && defined(__MACH__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "MyWorld.h"
#include "Timer.h"
#include <Math.h>

using namespace std;

// opengl setup related variables
unsigned int window_width = 640, window_height = 640;

// ui related variables
bool mouse_down = false;
bool leftClick = false;
bool rightClick = false;
int mouseX;
int mouseY;
bool viewVelocity = false;
bool viewColor = false;

// simulation related variables
MyWorld mySimulator;
bool simulating = false;
int frame_number = 0;
Timer timer;
int numCells = 64; // Number os cells in a row/column

// simulation functions
int getNumCells() {
    return numCells;
}

// opengl functions
void myGlutResize(int w, int h);

void myGlutIdle(void);

void myGlutDisplay(void);

void myGlutKeyboard(unsigned char key, int x, int y);

void myGlutMouse(int button, int state, int x, int y);

void myGlutMotion(int x, int y);

void drawVelocity();

void drawDensity();

// main function
int main(int argc, char *argv[]) {
    mySimulator.initialize(numCells, 0.1, 0.0, 0.0);

    glutInit(&argc, argv);
    glutInitWindowSize(window_width, window_height);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutCreateWindow("Fluid Sim");
    glutIdleFunc(myGlutIdle);
    glutDisplayFunc(myGlutDisplay);
    glutReshapeFunc(myGlutResize);
    glutKeyboardFunc(myGlutKeyboard);
    glutMouseFunc(myGlutMouse);
    glutMotionFunc(myGlutMotion);

    // anti aliasing
    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glutMainLoop();
    return 0;
}

void myGlutResize(int w, int h) {
    window_width = w;
    window_height = h;
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glutPostRedisplay();
}

void myGlutIdle(void) {
    if (simulating) {
        timer.stop();
        double time_diff_in_sec = timer.getLastElapsedTime();

        if (time_diff_in_sec > 0.01) {
            while (time_diff_in_sec > 0.01) {
                mySimulator.simulate();
                frame_number++;
                time_diff_in_sec -= 0.01;
            }

            timer.start();
        }
    }

    glutPostRedisplay();
}

void myGlutDisplay(void) {
    glClearColor(1.f , 1.f, 1.f , 1.0f);
    ::glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_COLOR_MATERIAL);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 1);

    glViewport(0, 0, window_width, window_height);
    glMatrixMode(GL_PROJECTION);    // opengl matrix for camera
    glLoadIdentity();
    gluOrtho2D(0.0, 1.0, 0.0, 1.0);

    // lighting
    glEnable(GL_LIGHTING);
    float ambient[4] = {0.5, 0.5, 0.5, 1};
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);
    float diffuse[4] = {0.5, 0.5, 0.5, 1};
    float position[4] = {10, 10, 10, 0};
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT0, GL_POSITION, position);


    if (viewVelocity)
        drawVelocity();
    else
        drawDensity();

    glutSwapBuffers();
}

void myGlutKeyboard(unsigned char key, int x, int y) {
    switch (key) {
        case 27:    // esc
            exit(0);
            break;

        case ' ':   // toggle simulation
            simulating = !simulating;

            if (simulating) timer.start();

            break;

        case 'v': // toggle between velocity view and density view
            viewVelocity = !viewVelocity;
            break;

        case 'c': // toggle between colorful and gray
            viewColor = !viewColor;
            break;

        default:
            break;
    }

    glutPostRedisplay();
}

void myGlutMouse(int button, int state, int x, int y) {
    mouse_down = (state == GLUT_DOWN);

    if (mouse_down) {
        mouseX = x;
        mouseY = y;
        int i = (int)((mouseX / (double)window_width) * mySimulator.getNumCells() + 1);
        int j = (int)(((window_height - mouseY) / (double)window_height) * mySimulator.getNumCells() + 1);

        if (i < 1 || i > mySimulator.getNumCells() || j < 1 || j > mySimulator.getNumCells())
            return;

        if (button == GLUT_LEFT_BUTTON) {
            leftClick = true;

            for (int chan = 0; chan < 3; chan++) {
                if (!viewColor) {
                    mySimulator.setDensity(i, j, chan, 100.0);
                } else {
                    mySimulator.setDensity(i, j, chan, rand() % 20);
                }
            }

        } else if (button == GLUT_RIGHT_BUTTON || button == GLUT_MIDDLE_BUTTON) {
            rightClick = true;
            mySimulator.setU(i, j, 5.0);
            mySimulator.setV(i, j, 5.0);
        }
    } else {
        leftClick = false;
        rightClick = false;
    }

    glutPostRedisplay();
}

void myGlutMotion(int x, int y) {
    int i = (int)((x / (double)window_width) * mySimulator.getNumCells() + 1);
    int j = (int)(((window_height - y) / (double)window_height) * mySimulator.getNumCells() + 1);

    if (i < 1 || i > mySimulator.getNumCells() || j < 1 || j > mySimulator.getNumCells())
        return;

    if (leftClick) {
        for (int chan = 0; chan < 3; chan++) {
            if (!viewColor) {
                mySimulator.setDensity(i, j, chan, 100.0);
            } else {
                mySimulator.setDensity(i, j, chan, rand() % 20);
            }
        }

    } else if (rightClick) {
        mySimulator.setU(i, j, x - mouseX);
        mySimulator.setV(i, j, mouseY - y);
    }

    mouseX = x;
    mouseY = y;
    glutPostRedisplay();
}

void RenderBitmapString(float x, float y, void *font, char *string) {
    char *c;
    ::glRasterPos2f(x, y);

    for (c = string; *c != '\0'; c++) {
        ::glutBitmapCharacter(font, *c);
    }

    ::glRasterPos2f(x + 1, y);

    for (c = string; *c != '\0'; c++) {
        ::glutBitmapCharacter(font, *c);
    }
}


void drawVelocity() {
    double h = 1.0 / mySimulator.getNumCells();

    glColor3f(1.0f, 1.0f, 1.0f);
    glLineWidth(1.0f);

    glBegin(GL_LINES);

    for (int i = 1 ; i <= mySimulator.getNumCells(); i++) {
        double x = (i - 0.5) * h;

        for (int j = 1; j <= mySimulator.getNumCells(); j++) {
            double y = (j - 0.5) * h;

            glVertex2f(x, y);
            glVertex2f(x + mySimulator.getVelocityU(IX(i, j)), y + mySimulator.getVelocityV(IX(i, j)));
        }
    }

    glEnd();
}

void drawDensity() {
    double h = 1.0 / mySimulator.getNumCells();
    glBegin(GL_QUADS);

    for (int i = 0; i <= mySimulator.getNumCells(); i++) {
        double x = (i - 0.5) * h;

        for (int j = 0; j <= mySimulator.getNumCells(); j++) {
            double y = (j - 0.5) * h;

            double r00 = mySimulator.getDensity(IXC(i, j, 0));
            double r01 = mySimulator.getDensity(IXC(i, j + 1, 0));
            double r10 = mySimulator.getDensity(IXC(i + 1, j, 0));
            double r11 = mySimulator.getDensity(IXC(i + 1, j + 1, 0));
            double g00 = mySimulator.getDensity(IXC(i, j, 1));
            double g01 = mySimulator.getDensity(IXC(i, j + 1, 1));
            double g10 = mySimulator.getDensity(IXC(i + 1, j, 1));
            double g11 = mySimulator.getDensity(IXC(i + 1, j + 1, 1));
            double b00 = mySimulator.getDensity(IXC(i, j, 2));
            double b01 = mySimulator.getDensity(IXC(i, j + 1, 2));
            double b10 = mySimulator.getDensity(IXC(i + 1, j, 2));
            double b11 = mySimulator.getDensity(IXC(i + 1, j + 1, 2));

            glColor3d(r00, g00, b00);
            glVertex3f(x, y, 0);
            glColor3d(r10, g10, b10);
            glVertex3f(x + h, y, 0);
            glColor3d(r11, g11, b11);
            glVertex3f(x + h, y + h, 0);
            glColor3d(r01, g01, b01);
            glVertex3f(x, y + h, 0);

        }
    }

    glEnd();

}

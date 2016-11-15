#include <string>
#include <iostream>
#include <stdlib.h>

#if defined(__APPLE__) && defined(__MACH__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "particle.h"
#include "simulator.h"

#include "Timer.h"
#include <Eigen/Dense>
#include <memory>

using namespace std;

// opengl setup related variables
unsigned int window_width = 800, window_height = 600;

// ui related variables
//Eigen::Vector3d view_translation = Eigen::Vector3d(0, 0, 0);
//Eigen::Vector2d view_rotation = Eigen::Vector2d::Zero();
//bool mouse_down = false;
//Eigen::Vector2d mouse_click_position;

// simulation related variables
//Simulator mySimulator;
std::shared_ptr<Simulator> mySimulator;
//curSys == 0 : galileo; curSys == 1 : tinkertoy
int curSys = 0;
typedef std::vector<std::shared_ptr<Simulator> > SysMapTyp;

SysMapTyp systems;

bool simulating = false;
//int frame_number = 0;
Timer timer;

// opengl functions
void myGlutResize(int w, int h);

void myGlutIdle(void);

void myGlutDisplay(void);

void myGlutKeyboard(unsigned char key, int x, int y);

void myGlutMouse(int button, int state, int x, int y);

void myGlutMotion(int x, int y);

void drawGalileo();

void drawTinkerToy();

void ShowText();

void DrawBackground();

// main function
int main(int argc, char *argv[])
{
	//init simulator holder
	systems = { std::make_shared<Simulator>(0), std::make_shared<Simulator>(1) };
	mySimulator = systems[curSys];

    glutInit(&argc, argv);
    glutInitWindowSize(window_width, window_height);
    glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGBA|GLUT_DEPTH);
    glutCreateWindow("Project 1 - Particle System");
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

void myGlutResize(int w, int h)
{
    window_width = w;
    window_height = h;
    glViewport(0,0,w,h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glutPostRedisplay();
}

void myGlutIdle(void) {
    if (simulating) {
        timer.stop();
        double time_diff_in_sec = timer.getLastElapsedTime();
        //cout << time_diff_in_sec << endl;
        if (time_diff_in_sec > mySimulator->getTimeStep()) {
            while (time_diff_in_sec > mySimulator->getTimeStep()) {
                mySimulator->simulate();
                time_diff_in_sec -= mySimulator->getTimeStep();
            }
            timer.start();
        }
    }
    
    glutPostRedisplay();
}

void myGlutDisplay(void) {
    glClearColor(1.f , 1.f, 1.f ,1.0f);
    ::glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_COLOR_MATERIAL);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 1);
    
    glMatrixMode(GL_PROJECTION);    // opengl matrix for camera
    glLoadIdentity();
    gluPerspective(45, window_width*1.0/window_height, 0.01, 1000);
    glMatrixMode(GL_MODELVIEW);     // opengl matrix for object
    glLoadIdentity();
    
    // lighting
    glEnable(GL_LIGHTING);
    float ambient[4] = {0.5, 0.5, 0.5, 1};
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);
    float diffuse[4] = {0.5, 0.5, 0.5, 1};
    float position[4] = {10, 10, 10, 0};
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT0, GL_POSITION, position);
    
    glTranslatef(0, 0, -1);
    
	if (curSys == 0) {
		drawGalileo();
	} else if (curSys == 1){
		drawTinkerToy();
	}
 
    // render the text
    ShowText();
    
    glutSwapBuffers();
}

void drawGalileo() {
	// Pan the camera based on the average height of all particles
	double average_height = 0.0;
	for (int i = 0; i < mySimulator->getNumParticles(); i++)
		average_height += mySimulator->getParticle(i)->mPosition[1];
	average_height /= mySimulator->getNumParticles();
	mySimulator->view_translation[1] = average_height;

	// transform view centered by average position of particles
	glRotatef(mySimulator->view_rotation[0], 1, 0, 0);
	glRotatef(mySimulator->view_rotation[1], 0, 1, 0);
	glTranslatef(-mySimulator->view_translation[0], -mySimulator->view_translation[1], -mySimulator->view_translation[2]);

	// Draw particles
	for (int i = 0; i < mySimulator->getNumParticles(); i++) {
		mySimulator->getParticle(i)->draw();
	}

	//Draw a checker board background
	DrawBackground();
}

void drawTinkerToy() {
	// Draw a circle
	glColor3d(0, 0, 0);
	glDisable(GL_LIGHTING);
	glBegin(GL_LINE_LOOP);
	double rad = 3.14 / 180.0;
	double radius = 0.2;
	for (int i = 0; i < 360; i++) {
		double angle = i * rad;
		glVertex3d(radius * cos(angle), radius * sin(angle), 0.0);
	}
	glEnd();
	// Draw a line

	glBegin(GL_LINES);
	Eigen::Vector3d p1 = mySimulator->getParticle(0)->mPosition;
	Eigen::Vector3d p2 = mySimulator->getParticle(1)->mPosition;
	glVertex3f(p1[0], p1[1], p1[2]);
	glVertex3f(p2[0], p2[1], p2[2]);
	glEnd();
	glEnable(GL_LIGHTING);

	// Draw particles
	for (int i = 0; i < mySimulator->getNumParticles(); i++) {
		mySimulator->getParticle(i)->draw();
	}
}


void myGlutKeyboard(unsigned char key, int x, int y) {
    switch (key) {
	case 27: {    // esc
		exit(0);
		break; }
	case ' ': {   // toggle simulation
		simulating = !simulating;
		if (simulating) timer.start();
		break; }
	case 'r': {   // reset simulation
		mySimulator->reset();
		simulating = false;
		break; }
	case '1': {			//part 1 of project 1 - galileo experiment
		cout << "Part 1 : Galileo Experiement" << endl;
		curSys = 0;
		mySimulator = systems[curSys];
		mySimulator->reset();
		simulating = false;
		break; }
	case '2': {			//part 2 of project 1 - tinker toy
		cout << "Part 2 : Tinker Toy" << endl;
		curSys = 1;
		mySimulator = systems[curSys];
		mySimulator->reset();
		simulating = false;
		break; }
	default:
            break;
    }
    
    glutPostRedisplay();
}

void myGlutMouse(int button, int state, int x, int y) {
	mySimulator->mouse_down = (state == GLUT_DOWN);
    if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
		mySimulator->mouse_click_position << x, y;
        cout << "Left Mouse Clicked" << endl;
    } else if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN) {
        cout << "Right Mouse Clicked" << endl;
    }
    
    glutPostRedisplay();
}

void myGlutMotion(int x, int y) {    
    glutPostRedisplay();
}

void RenderBitmapString(float x, float y, void *font,char *string)
{
    char *c;
    ::glRasterPos2f(x, y);
    for (c=string; *c != '\0'; c++) {
        ::glutBitmapCharacter(font, *c);
    }
    ::glRasterPos2f(x+1, y);
    for (c=string; *c != '\0'; c++) {
        ::glutBitmapCharacter(font, *c);
    }
}

void ShowText()
{
    int* pFont=(int*)GLUT_BITMAP_TIMES_ROMAN_24;
    
    GLint viewport[4];
    ::glGetIntegerv(GL_VIEWPORT,viewport);
    const int win_w = viewport[2];
    const int win_h = viewport[3];
    
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, window_width, 0, window_height);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glScalef(1, -1, 1);
    glTranslatef(0, -win_h, 0);
    
    // Display the frame count in 2D text
    char s_tmp[256];
    sprintf(s_tmp,"%d", mySimulator->getFrameNum());
    glColor3d(0.0, 0.0, 0.0);
    RenderBitmapString(10, window_height - 20, pFont, s_tmp);
    
    pFont=(int*)GLUT_BITMAP_8_BY_13;
    // Display control instructions
	if (curSys == 0) {
		strcpy(s_tmp, "Part 1 : Galileo Experiment");
	} else {
		strcpy(s_tmp, "Part 2 : Tinker Toy");
	}
	RenderBitmapString(10, 20, pFont, s_tmp);

    strcpy(s_tmp,"\' \': Start/Stop simulation");
    RenderBitmapString(10, 20 + 14, pFont, s_tmp);
    
    strcpy(s_tmp,"\'r\': Reset simulation");
    RenderBitmapString(10, 20 + 28, pFont, s_tmp);
    
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

void DrawBackground() {
	//Draw a checker board background
	glDisable(GL_LIGHTING);
	bool flip = true;
	double wall = -5.0;
	for (int i = -20; i < 20; i++) {
		for (int j = -20; j < 25; j++) {
			if (flip == true) {
				glColor4d(0.7, 0.7, 0.7, 1.0);
				flip = false;
			}
			else {
				glColor4d(0.8, 0.8, 0.8, 1.0);
				flip = true;
			}
			glBegin(GL_QUADS);
			glNormal3d(0, 0, 1);
			glVertex3d(i, j, wall);
			glVertex3d(i, j + 1, wall);
			glVertex3d(i + 1, j + 1, wall);
			glVertex3d(i + 1, j, wall);
			glEnd();
		}
	}
	glEnable(GL_LIGHTING);
}




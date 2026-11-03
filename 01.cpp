// g++ bezier_gl.cpp -o bezier_gl -lGL -lGLU -lglut   (Linux)
// Windows 用 freeglut，链接 freeglut32.lib / freeglut.lib
#include <GL/glut.h>
#include <cmath>

struct Vec2 { float x, y; };

Vec2 P[4] = { {100, 500}, {250, 100}, {550, 100}, {700, 500} };
int  gDrag = -1;

// 三次贝塞尔求值，传入的是数组首地址，不是某个数
Vec2 bezier3(const Vec2* p, float t) {
    float u = 1.0f - t;
    float b0 = u*u*u, b1 = 3*u*u*t, b2 = 3*u*t*t, b3 = t*t*t;
    return { b0*p[0].x + b1*p[1].x + b2*p[2].x + b3*p[3].x,
             b0*p[0].y + b1*p[1].y + b2*p[2].y + b3*p[3].y };
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT);//清楚显示器缓存

    // 控制多边形
    glColor3f(0.7f, 0.7f, 0.7f);//颜色？
    glBegin(GL_LINE_STRIP);//三次线性
    for (int i = 0; i < 4; ++i) glVertex2f(P[i].x, P[i].y);//定点切换到浮点数
    glEnd();

    // 贝塞尔曲线
    glColor3f(0.0f, 0.0f, 0.0f);
    glLineWidth(0.5f);
    glBegin(GL_LINE_STRIP);
    const int N = 200;
    for (int i = 0; i <= N; ++i)
        { Vec2 v = bezier3(P, (float)i / N); glVertex2f(v.x, v.y); }
    glEnd();
    glLineWidth(1.0f);

    // 控制点
    glColor3f(0.9f, 0.1f, 0.1f);
    glPointSize(9.0f);
    glBegin(GL_POINTS);
    for (int i = 0; i < 4; ++i) glVertex2f(P[i].x, P[i].y);
    glEnd();

    glutSwapBuffers();
}

void mouse(int btn, int state, int x, int y) {
    if (btn == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        for (int i = 0; i < 4; ++i)
            if (std::hypot(P[i].x - x, P[i].y - y) < 15.0f) { gDrag = i; break; }
    } else if (state == GLUT_UP) {
        gDrag = -1;
    }
}

void motion(int x, int y) {
    if (gDrag >= 0) {
        P[gDrag] = { (float)x, (float)y };
        glutPostRedisplay();
    }
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(800, 600);
    glutCreateWindow("Cubic Bezier Curve");

    glClearColor(1, 1, 1, 1);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, 800, 600, 0);   // y 轴向下，与鼠标坐标一致

    glutDisplayFunc(display);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutMainLoop();
    return 0;
}

// Linux:   g++ -std=c++11 01.cpp -o bezier -lGL -lGLU -lglut
// Windows: g++ -std=c++11 01.cpp -o bezier.exe -lfreeglut -lopengl32 -lglu32
#include <GL/glut.h>

#include <array>
#include <cmath>
#include <cstddef>

namespace {

struct Vec2 {
    float x;
    float y;
};

constexpr int kWindowWidth = 800;
constexpr int kWindowHeight = 600;
constexpr int kSampleCount = 200;
constexpr float kPickRadius = 15.0f;
constexpr int kNoSelection = -1;

std::array<Vec2, 4> controlPoints{{
    {100.0f, 500.0f}, {250.0f, 100.0f},
    {550.0f, 100.0f}, {700.0f, 500.0f}
}};
int draggedPoint = kNoSelection;

Vec2 cubicBezier(const std::array<Vec2, 4>& points, float t) {
    const float u = 1.0f - t;
    const float b0 = u * u * u;
    const float b1 = 3.0f * u * u * t;
    const float b2 = 3.0f * u * t * t;
    const float b3 = t * t * t;

    return {
        b0 * points[0].x + b1 * points[1].x + b2 * points[2].x + b3 * points[3].x,
        b0 * points[0].y + b1 * points[1].y + b2 * points[2].y + b3 * points[3].y
    };
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT);

    glColor3f(0.7f, 0.7f, 0.7f);
    glBegin(GL_LINE_STRIP);
    for (const auto& point : controlPoints) {
        glVertex2f(point.x, point.y);
    }
    glEnd();

    glColor3f(0.0f, 0.0f, 0.0f);
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i <= kSampleCount; ++i) {
        const float t = static_cast<float>(i) / kSampleCount;
        const Vec2 point = cubicBezier(controlPoints, t);
        glVertex2f(point.x, point.y);
    }
    glEnd();

    glColor3f(0.9f, 0.1f, 0.1f);
    glPointSize(9.0f);
    glBegin(GL_POINTS);
    for (const auto& point : controlPoints) {
        glVertex2f(point.x, point.y);
    }
    glEnd();

    glutSwapBuffers();
}

void mouse(int button, int state, int x, int y) {
    if (button != GLUT_LEFT_BUTTON) {
        return;
    }

    if (state == GLUT_DOWN) {
        for (std::size_t i = 0; i < controlPoints.size(); ++i) {
            const auto& point = controlPoints[i];
            if (std::hypot(point.x - x, point.y - y) < kPickRadius) {
                draggedPoint = static_cast<int>(i);
                break;
            }
        }
    } else if (state == GLUT_UP) {
        draggedPoint = kNoSelection;
    }
}

void motion(int x, int y) {
    if (draggedPoint == kNoSelection) {
        return;
    }

    controlPoints[static_cast<std::size_t>(draggedPoint)] = {
        static_cast<float>(x), static_cast<float>(y)
    };
    glutPostRedisplay();
}

} // namespace

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(kWindowWidth, kWindowHeight);
    glutCreateWindow("Cubic Bezier Curve");

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, kWindowWidth, kWindowHeight, 0);

    glutDisplayFunc(display);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutMainLoop();
    return 0;
}

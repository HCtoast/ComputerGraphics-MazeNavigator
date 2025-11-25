#define _CRT_SECURE_NO_WARNINGS

#include <vgl.h>
#include <InitShader.h>
#include "MyCube.h"
#include "MyUtil.h"

#include <vec.h>
#include <mat.h>

#include <vector>
#include <utility>
#include <queue>
using std::vector;
using std::pair;
using std::priority_queue;
#include <algorithm>
#include <cmath>

// ============================================================================
// Constants
// ============================================================================
#define MAZE_FILE "maze.txt"

const float DEFAULT_WINDOW_WIDTH = 1000.0f;
const float DEFAULT_WINDOW_HEIGHT = 500.0f;
const int MAX_MAZE_SIZE = 255;
const int FRAME_DELAY_MS = 16;  // ~60 FPS
const float FIXED_DELTA_TIME = 0.016f;
const float DEFAULT_CAMERA_SPEED = 6.0f;
const float DEFAULT_CAMERA_SIZE = 0.5f;
const float DEFAULT_ROTATE_SPEED = 3.0f;
const float GOAL_SIZE = 0.7f;
const float PI = 3.141592f;
const float GRID_LINE_COUNT = 40.0f;
const int PATHFINDING_INFINITY = 1000000000;

// ============================================================================
// Structures
// ============================================================================
struct Camera {
	vec3 position;
	vec3 viewDirection;
	float yaw;
	float speed;
	float size;

	Camera()
		: position(0, 0, 0)
		, viewDirection(0, 0, -1)
		, yaw(0.0f)
		, speed(DEFAULT_CAMERA_SPEED)
		, size(DEFAULT_CAMERA_SIZE)
	{}
};

struct MazeData {
	int size;
	char cells[MAX_MAZE_SIZE][MAX_MAZE_SIZE];
	vec3 goalPosition;
	int goalIndexI;
	int goalIndexJ;

	MazeData()
		: size(0)
		, goalPosition(0, 0, 0)
		, goalIndexI(0)
		, goalIndexJ(0)
	{
		memset(cells, 0, sizeof(cells));
	}
};

struct PathfindingState {
	bool isFollowingPath;
	int currentIndex;
	int movementState;  // 0 = rotating, 1 = moving
	vector<pair<int, int>> pathCells;

	PathfindingState()
		: isFollowingPath(false)
		, currentIndex(0)
		, movementState(0)
	{}
};

// ============================================================================
// Global Variables
// ============================================================================
MyCube cube;
GLuint program;
mat4 g_Mat = mat4(1.0f);
GLuint uMat;
GLuint uColor;

float wWidth = DEFAULT_WINDOW_WIDTH;
float wHeight = DEFAULT_WINDOW_HEIGHT;
float rotateSpeed = DEFAULT_ROTATE_SPEED;
float g_time = 0.0f;

Camera g_camera;
MazeData g_maze;
PathfindingState g_pathState;
bool bDrawTrace = false;

// ============================================================================
// Utility Functions
// ============================================================================
inline vec3 GetPositionFromIndex(int i, int j) {
	const float unit = 1.0f;
	vec3 leftTopPosition = vec3(
		-g_maze.size / 2.0f + unit / 2.0f,
		0.0f,
		-g_maze.size / 2.0f + unit / 2.0f
	);
	vec3 xDir = vec3(1, 0, 0);
	vec3 zDir = vec3(0, 0, 1);
	return leftTopPosition + i * xDir + j * zDir;
}

bool WorldPosToIndex(const vec3& pos, int& outI, int& outJ) {
	const float unit = 1.0f;
	const float left = -g_maze.size / 2.0f + unit / 2.0f;

	float fx = (pos.x - left) / unit;
	float fz = (pos.z - left) / unit;

	int i = (int)roundf(fx);
	int j = (int)roundf(fz);

	if (i < 0 || i >= g_maze.size || j < 0 || j >= g_maze.size)
		return false;

	outI = i;
	outJ = j;
	return true;
}

bool IsWallAtWorldPos(const vec3& pos) {
	int i, j;
	if (!WorldPosToIndex(pos, i, j))
		return true;

	return (g_maze.cells[i][j] == '*');
}

inline int CalculateHeuristic(int i, int j) {
	return abs(i - g_maze.goalIndexI) + abs(j - g_maze.goalIndexJ);
}

// ============================================================================
// Maze Loading
// ============================================================================
void LoadMaze() {
	FILE* file = fopen(MAZE_FILE, "r");
	char buf[MAX_MAZE_SIZE];

	fgets(buf, MAX_MAZE_SIZE, file);
	sscanf(buf, "%d", &g_maze.size);

	for (int j = 0; j < g_maze.size; j++) {
		fgets(buf, MAX_MAZE_SIZE, file);
		for (int i = 0; i < g_maze.size; i++) {
			g_maze.cells[i][j] = buf[i];

			if (g_maze.cells[i][j] == 'C') {
				g_camera.position = GetPositionFromIndex(i, j);
			}
			if (g_maze.cells[i][j] == 'G') {
				g_maze.goalPosition = GetPositionFromIndex(i, j);
				g_maze.goalIndexI = i;
				g_maze.goalIndexJ = j;
			}
		}
	}
	fclose(file);
}

// ============================================================================
// Drawing Helper Functions
// ============================================================================
void DrawCubeWithTransform(const mat4& modelMatrix, const vec4& color) {
	glUniformMatrix4fv(uMat, 1, GL_TRUE, g_Mat * modelMatrix);
	glUniform4f(uColor, color.x, color.y, color.z, color.w);
	cube.Draw(program);
}

void DrawMaze() {
	for (int j = 0; j < g_maze.size; j++) {
		for (int i = 0; i < g_maze.size; i++) {
			if (g_maze.cells[i][j] == '*') {
				vec3 color = vec3(
					i / (float)g_maze.size,
					j / (float)g_maze.size,
					1.0f
				);
				mat4 modelMat = Translate(GetPositionFromIndex(i, j));
				DrawCubeWithTransform(modelMat, vec4(color, 1.0f));
			}
		}
	}
}

void DrawGrid() {
	const float w = g_maze.size;
	const float h = g_maze.size;

	// Horizontal lines
	for (int i = 0; i < GRID_LINE_COUNT; i++) {
		mat4 m = Translate(0, -0.5, -h / 2 + h / GRID_LINE_COUNT * i)
			* Scale(w, 0.02, 0.02);
		DrawCubeWithTransform(m, vec4(1, 1, 1, 1));
	}

	// Vertical lines
	for (int i = 0; i < GRID_LINE_COUNT; i++) {
		mat4 m = Translate(-w / 2 + w / GRID_LINE_COUNT * i, -0.5, 0)
			* Scale(0.02, 0.02, h);
		DrawCubeWithTransform(m, vec4(1, 1, 1, 1));
	}
}

void DrawCamera() {
	// Camera body
	mat4 bodyMat = Translate(g_camera.position)
		* RotateY(-g_camera.yaw * 180 / PI)
		* Scale(vec3(g_camera.size));
	DrawCubeWithTransform(bodyMat, vec4(0, 1, 0, 1));

	// Camera head (direction indicator)
	mat4 headMat = Translate(g_camera.position + g_camera.viewDirection * g_camera.size / 2)
		* RotateY(-g_camera.yaw * 180 / PI)
		* Scale(vec3(g_camera.size / 2));
	DrawCubeWithTransform(headMat, vec4(0, 1, 0, 1));
}

void DrawGoal() {
	const vec4 goalColor = vec4(0, 0, 0, 0);

	// First cube
	mat4 model1 = Translate(g_maze.goalPosition)
		* RotateY(g_time * 180)
		* Scale(vec3(GOAL_SIZE));
	DrawCubeWithTransform(model1, goalColor);

	// Second cube (45 degrees rotated)
	mat4 model2 = Translate(g_maze.goalPosition)
		* RotateY(g_time * 180 + 45)
		* Scale(vec3(GOAL_SIZE));
	DrawCubeWithTransform(model2, goalColor);
}

void DrawTrace() {
	const vec4 traceColor = vec4(1, 0, 0, 1);

	for (int k = 0; k + 1 < (int)g_pathState.pathCells.size(); ++k) {
		int i0 = g_pathState.pathCells[k].first;
		int j0 = g_pathState.pathCells[k].second;
		int i1 = g_pathState.pathCells[k + 1].first;
		int j1 = g_pathState.pathCells[k + 1].second;

		vec3 p0 = GetPositionFromIndex(i0, j0);
		vec3 p1 = GetPositionFromIndex(i1, j1);

		vec3 mid = (p0 + p1) * 0.5f;
		float len = length(p1 - p0);

		mat4 m;
		if (i0 == i1) {
			m = Translate(mid.x, -0.49f, mid.z) * Scale(0.1f, 0.02f, len);
		}
		else {
			m = Translate(mid.x, -0.49f, mid.z) * Scale(len, 0.02f, 0.1f);
		}

		DrawCubeWithTransform(m, traceColor);
	}
}

void DrawScene(bool bDrawCamera = true) {
	glUseProgram(program);
	uMat = glGetUniformLocation(program, "uMat");
	uColor = glGetUniformLocation(program, "uColor");

	DrawGrid();
	DrawMaze();
	DrawGoal();

	if (bDrawCamera)
		DrawCamera();
	if (bDrawTrace)
		DrawTrace();
}

// ============================================================================
// Pathfinding (A* Algorithm)
// ============================================================================
bool FindPathAStar(int startI, int startJ) {
	static int gCost[MAX_MAZE_SIZE][MAX_MAZE_SIZE];
	static bool closed[MAX_MAZE_SIZE][MAX_MAZE_SIZE];
	static pair<int, int> parentCell[MAX_MAZE_SIZE][MAX_MAZE_SIZE];

	// Initialize
	for (int i = 0; i < g_maze.size; i++) {
		for (int j = 0; j < g_maze.size; j++) {
			gCost[i][j] = PATHFINDING_INFINITY;
			closed[i][j] = false;
		}
	}

	struct Node {
		int i, j;
		int g, f;
	};

	struct Compare {
		bool operator()(const Node& a, const Node& b) const {
			return a.f > b.f;
		}
	};

	priority_queue<Node, vector<Node>, Compare> openSet;

	gCost[startI][startJ] = 0;
	openSet.push({ startI, startJ, 0, CalculateHeuristic(startI, startJ) });

	const int di[4] = { 1, -1, 0, 0 };
	const int dj[4] = { 0, 0, 1, -1 };

	while (!openSet.empty()) {
		Node current = openSet.top();
		openSet.pop();

		int i = current.i, j = current.j;
		if (closed[i][j]) continue;
		closed[i][j] = true;

		// Goal reached
		if (i == g_maze.goalIndexI && j == g_maze.goalIndexJ) {
			g_pathState.pathCells.clear();
			int ci = i, cj = j;

			while (!(ci == startI && cj == startJ)) {
				g_pathState.pathCells.push_back({ ci, cj });
				auto p = parentCell[ci][cj];
				ci = p.first;
				cj = p.second;
			}
			g_pathState.pathCells.push_back({ startI, startJ });
			reverse(g_pathState.pathCells.begin(), g_pathState.pathCells.end());

			return true;
		}

		// Explore neighbors
		for (int k = 0; k < 4; k++) {
			int ni = i + di[k];
			int nj = j + dj[k];

			// Boundary and wall check
			if (ni < 0 || ni >= g_maze.size || nj < 0 || nj >= g_maze.size)
				continue;
			if (g_maze.cells[ni][nj] == '*')
				continue;

			int newG = gCost[i][j] + 1;
			if (newG < gCost[ni][nj]) {
				gCost[ni][nj] = newG;
				parentCell[ni][nj] = { i, j };

				int newF = newG + CalculateHeuristic(ni, nj);
				openSet.push({ ni, nj, newG, newF });
			}
		}
	}

	return false;
}

// ============================================================================
// Camera Movement
// ============================================================================
bool CanMoveCameraTo(const vec3& tryPos, const vec3& viewDirection, bool isForward) {
	vec3 forward = vec3(viewDirection.x, 0.0f, viewDirection.z);
	if (length(forward) < 1e-5f)
		forward = vec3(0, 0, -1);

	forward = normalize(forward);
	vec3 right = normalize(vec3(forward.z, 0.0f, -forward.x));

	const float bodyHalf = g_camera.size * 0.5f;
	const float headHalf = g_camera.size * 0.25f;

	vec3 pts[6];
	pts[0] = tryPos - right * bodyHalf - forward * bodyHalf;
	pts[1] = tryPos - right * bodyHalf + forward * bodyHalf;
	pts[2] = tryPos + right * bodyHalf + forward * bodyHalf;
	pts[3] = tryPos + right * bodyHalf - forward * bodyHalf;

	if (isForward) {
		const int idxs[4] = { 1, 2, 4, 5 };
		for (int n = 0; n < 4; ++n) {
			int k = idxs[n];
			if (IsWallAtWorldPos(pts[k])) {
				return false;
			}
		}
	}
	else {
		const int idxs[2] = { 0, 3 };
		for (int n = 0; n < 2; ++n) {
			int k = idxs[n];
			if (IsWallAtWorldPos(pts[k])) {
				return false;
			}
		}
	}

	return true;
}

void UpdateCameraViewDirection() {
	g_camera.viewDirection = normalize(vec3(
		sinf(g_camera.yaw),
		0.0f,
		-cosf(g_camera.yaw)
	));
}

void HandleManualMovement(float dt) {
	// Rotation with A/D keys
	if ((GetAsyncKeyState('A') & 0x8000) == 0x8000)
		g_camera.yaw -= rotateSpeed * dt;
	if ((GetAsyncKeyState('D') & 0x8000) == 0x8000)
		g_camera.yaw += rotateSpeed * dt;

	UpdateCameraViewDirection();

	// Forward movement with W key
	if ((GetAsyncKeyState('W') & 0x8000) == 0x8000) {
		vec3 tryPos = g_camera.position + g_camera.speed * dt * g_camera.viewDirection;
		if (CanMoveCameraTo(tryPos, g_camera.viewDirection, true))
			g_camera.position = tryPos;
	}

	// Backward movement with S key
	if ((GetAsyncKeyState('S') & 0x8000) == 0x8000) {
		vec3 tryPos = g_camera.position - g_camera.speed * dt * g_camera.viewDirection;
		if (CanMoveCameraTo(tryPos, g_camera.viewDirection, false))
			g_camera.position = tryPos;
	}
}

void AutoMoveCameraAlongPath(float dt) {
	const float FOLLOW_POS_EPS = 0.01f;
	const float FOLLOW_ANG_EPS = 0.03f;

	// Check if reached the end
	if (g_pathState.currentIndex >= (int)g_pathState.pathCells.size() - 1) {
		g_pathState.isFollowingPath = false;
		return;
	}

	// Get current and next cell
	auto currentCell = g_pathState.pathCells[g_pathState.currentIndex];
	auto nextCell = g_pathState.pathCells[g_pathState.currentIndex + 1];
	vec3 pCurrent = GetPositionFromIndex(currentCell.first, currentCell.second);
	vec3 pNext = GetPositionFromIndex(nextCell.first, nextCell.second);

	vec3 desiredDir = pNext - pCurrent;
	desiredDir.y = 0.0f;
	float segmentLength = length(desiredDir);

	if (segmentLength < 1e-5f) {
		g_pathState.currentIndex++;
		return;
	}
	desiredDir /= segmentLength;

	float targetYaw = atan2f(desiredDir.x, -desiredDir.z);

	// State 0: Rotating
	if (g_pathState.movementState == 0) {
		float delta = targetYaw - g_camera.yaw;

		// Normalize angle difference
		while (delta > PI) delta -= 2.0f * PI;
		while (delta < -PI) delta += 2.0f * PI;

		if (fabs(delta) < FOLLOW_ANG_EPS) {
			g_camera.yaw = targetYaw;
			UpdateCameraViewDirection();
			g_camera.position = pCurrent;
			g_pathState.movementState = 1;
		}
		else {
			float step = rotateSpeed * dt;
			g_camera.yaw += (delta > 0 ? step : -step);
			UpdateCameraViewDirection();
		}
		return;
	}

	// State 1: Moving
	if (g_pathState.movementState == 1) {
		vec3 moveDir = normalize(vec3(g_camera.viewDirection.x, 0.0f, g_camera.viewDirection.z));
		vec3 tryPos = g_camera.position + moveDir * dt * g_camera.speed;
		vec3 fromCurrent = tryPos - pCurrent;
		float progress = dot(fromCurrent, desiredDir);

		if (progress >= segmentLength) {
			g_camera.position = pNext;
			g_pathState.currentIndex++;
			g_pathState.movementState = 0;
		}
		else {
			g_camera.position = tryPos;
		}
	}
}

// ============================================================================
// Input Handling
// ============================================================================
void HandlePathfindingToggle() {
	static bool prevQDown = false;
	bool qDown = (GetAsyncKeyState('Q') & 0x8000) != 0;

	if (qDown && !prevQDown) {
		int si, sj;
		if (WorldPosToIndex(g_camera.position, si, sj)) {
			bDrawTrace = !bDrawTrace;
			FindPathAStar(si, sj);
		}
	}
	prevQDown = qDown;
}

void HandleAutoMovementToggle() {
	static bool prevSpaceDown = false;
	bool spaceDown = (GetAsyncKeyState(VK_SPACE) & 0x8000) != 0;

	if (spaceDown && !prevSpaceDown) {
		int si, sj;
		if (WorldPosToIndex(g_camera.position, si, sj)) {
			if (FindPathAStar(si, sj)) {
				g_pathState.isFollowingPath = !g_pathState.isFollowingPath;
				g_pathState.currentIndex = 0;
				g_pathState.movementState = 0;
			}
			else {
				g_pathState.isFollowingPath = false;
			}
		}
	}
	prevSpaceDown = spaceDown;
}

// ============================================================================
// Main Loop Functions
// ============================================================================
void myInit() {
	LoadMaze();
	cube.Init();
	program = InitShader("vshader.glsl", "fshader.glsl");
}

void display() {
	glEnable(GL_DEPTH_TEST);

	const float vWidth = wWidth / 2;
	const float vHeight = wHeight;

	// LEFT SCREEN: First-person view (Perspective)
	glViewport(0, 0, vWidth, vHeight);
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

	float aspectRatio = vWidth / vHeight;
	mat4 viewMat = myLookAt(
		g_camera.position,
		g_camera.position + g_camera.viewDirection,
		vec3(0, 1, 0)
	);
	mat4 projMat = myPerspective(45, aspectRatio, 0.01, 20);

	g_Mat = projMat * viewMat;
	DrawScene(false);

	// RIGHT SCREEN: Top-down view (Orthographic)
	glViewport(vWidth, 0, vWidth, vHeight);

	float h = g_maze.size;
	float w = aspectRatio * h;
	viewMat = myLookAt(vec3(0, 5, 0), vec3(0, 0, 0), vec3(0, 0, -1));
	projMat = myOrtho(-w / 2, w / 2, -h / 2, h / 2, 0, 20);

	g_Mat = projMat * viewMat;
	DrawScene(true);

	glutSwapBuffers();
}

void idle() {
	g_time += FIXED_DELTA_TIME;

	if (g_pathState.isFollowingPath) {
		AutoMoveCameraAlongPath(FIXED_DELTA_TIME);
	}
	else {
		HandleManualMovement(FIXED_DELTA_TIME);
	}

	HandlePathfindingToggle();
	HandleAutoMovementToggle();

	Sleep(FRAME_DELAY_MS);
	glutPostRedisplay();
}

void reshape(int wx, int wy) {
	printf("%d %d \n", wx, wy);
	wWidth = wx;
	wHeight = wy;
	glutPostRedisplay();
}

// ============================================================================
// Main Entry Point
// ============================================================================
int main(int argc, char** argv) {
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(wWidth, wHeight);
	glutCreateWindow("Homework3 (Maze Navigator)");

	glewExperimental = true;
	glewInit();

	printf("OpenGL %s, GLSL %s\n",
		glGetString(GL_VERSION),
		glGetString(GL_SHADING_LANGUAGE_VERSION));

	myInit();
	glutDisplayFunc(display);
	glutIdleFunc(idle);
	glutReshapeFunc(reshape);
	glutMainLoop();

	return 0;
}
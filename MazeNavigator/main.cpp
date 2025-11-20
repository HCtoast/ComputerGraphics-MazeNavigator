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

#define MAZE_FILE	"maze.txt"

MyCube cube;
GLuint program;

mat4 g_Mat = mat4(1.0f);
GLuint uMat;
GLuint uColor;

float wWidth = 1000;
float wHeight = 500;

vec3 cameraPos = vec3(0, 0, 0);
vec3 viewDirection = vec3(0, 0, -1);
vec3 goalPos = vec3(0, 0, 0);

float cameraYaw = 0.0f;		
float rotateSpeed = 3.0f;	

int MazeSize;
char maze[255][255] = { 0 };

float cameraSpeed = 6.0f;
float cameraSize = 0.5;

float g_time = 0;

int goalindexI = 0;
int goalindexJ = 0;

vector<pair<int, int>> g_pathCells;

inline vec3 getPositionFromIndex(int i, int j)
{
	float unit = 1;
	vec3 leftTopPosition = vec3(-MazeSize / 2.0 + unit / 2, 0, -MazeSize / 2.0 + unit / 2);
	vec3 xDir = vec3(1, 0, 0);
	vec3 zDir = vec3(0, 0, 1);
	return leftTopPosition + i * xDir + j * zDir;
}

void LoadMaze()
{
	FILE* file = fopen(MAZE_FILE, "r");
	char buf[255];
	fgets(buf, 255, file);
	sscanf(buf, "%d", &MazeSize);
	for (int j = 0; j < MazeSize; j++)
	{
		fgets(buf, 255, file);
		for (int i = 0; i < MazeSize; i++)
		{
			maze[i][j] = buf[i];
			if (maze[i][j] == 'C')				// Setup Camera Position
				cameraPos = getPositionFromIndex(i, j);
			if (maze[i][j] == 'G') {			// Setup Goal Position
				goalPos = getPositionFromIndex(i, j);
				goalindexI = i;
				goalindexJ = j;
			}
		}
	}
	fclose(file);
}

void DrawMaze()
{
	for (int j = 0; j < MazeSize; j++)
		for (int i = 0; i < MazeSize; i++)
			if (maze[i][j] == '*')
			{
				vec3 color = vec3(i / (float)MazeSize, j / (float)MazeSize, 1);
				mat4 ModelMat = Translate(getPositionFromIndex(i, j));
				glUniformMatrix4fv(uMat, 1, GL_TRUE, g_Mat * ModelMat);
				glUniform4f(uColor, color.x, color.y, color.z, 1);
				cube.Draw(program);
			}
}

void myInit()
{
	LoadMaze();
	cube.Init();
	program = InitShader("vshader.glsl", "fshader.glsl");

}

void DrawGrid()
{
	float n = 40;
	float w = MazeSize;
	float h = MazeSize;

	for (int i = 0; i < n; i++)
	{
		mat4 m = Translate(0, -0.5, -h / 2 + h / n * i) * Scale(w, 0.02, 0.02);
		glUniformMatrix4fv(uMat, 1, GL_TRUE, g_Mat * m);
		glUniform4f(uColor, 1, 1, 1, 1);
		cube.Draw(program);
	}
	for (int i = 0; i < n; i++)
	{
		mat4 m = Translate(-w / 2 + w / n * i, -0.5, 0) * Scale(0.02, 0.02, h);
		glUniformMatrix4fv(uMat, 1, GL_TRUE, g_Mat * m);
		glUniform4f(uColor, 1, 1, 1, 1);
		cube.Draw(program);
	}
}


void drawCamera()
{
	mat4 ModelMat = Translate(cameraPos) * RotateY(-cameraYaw * 180 / 3.141592) * Scale(vec3(cameraSize));
	glUseProgram(program);
	glUniformMatrix4fv(uMat, 1, GL_TRUE, g_Mat * ModelMat);
	glUniform4f(uColor, 0, 1, 0, 1);
	cube.Draw(program);

	ModelMat = Translate(cameraPos + viewDirection * cameraSize / 2) * RotateY(-cameraYaw * 180 / 3.141592) * Scale(vec3(cameraSize / 2));
	glUseProgram(program);
	glUniformMatrix4fv(uMat, 1, GL_TRUE, g_Mat * ModelMat);
	glUniform4f(uColor, 0, 1, 0, 1);
	cube.Draw(program);
}

void drawGoal()
{
	glUseProgram(program);
	float GoalSize = 0.7;

	mat4 ModelMat = Translate(goalPos) * RotateY(g_time * 180) * Scale(vec3(GoalSize));
	glUniformMatrix4fv(uMat, 1, GL_TRUE, g_Mat * ModelMat);
	glUniform4f(uColor, 0, 0, 0, 0);
	cube.Draw(program);

	ModelMat = Translate(goalPos) * RotateY(g_time * 180 + 45) * Scale(vec3(GoalSize));
	glUniformMatrix4fv(uMat, 1, GL_TRUE, g_Mat * ModelMat);
	glUniform4f(uColor, 0, 0, 0, 0);
	cube.Draw(program);
}

void drawTrace(bool bDrawTrace = true) {
	glUseProgram(program);

	for (int k = 0; k + 1 < (int)g_pathCells.size(); ++k)
	{
		int i0 = g_pathCells[k].first;
		int j0 = g_pathCells[k].second;
		int i1 = g_pathCells[k + 1].first;
		int j1 = g_pathCells[k + 1].second;

		vec3 p0 = getPositionFromIndex(i0, j0);
		vec3 p1 = getPositionFromIndex(i1, j1);

		vec3 mid = (p0 + p1) * 0.5f;
		float len = length(p1 - p0);

		mat4 m;
		if (i0 == i1) {
			m = Translate(mid.x, -0.49f, mid.z) * Scale(0.1f, 0.02f, len);
		}
		else {
			m = Translate(mid.x, -0.49f, mid.z) * Scale(len, 0.02f, 0.1f);
		}

		glUniformMatrix4fv(uMat, 1, GL_TRUE, g_Mat * m);
		glUniform4f(uColor, 1, 0, 0, 1); // 빨간색
		cube.Draw(program);
	}
}

bool bDrawTrace = false;

void drawScene(bool bDrawCamera = true)
{
	glUseProgram(program);
	uMat = glGetUniformLocation(program, "uMat");
	uColor = glGetUniformLocation(program, "uColor");

	DrawGrid();
	DrawMaze();
	drawGoal();

	if (bDrawCamera)
		drawCamera();
	if (bDrawTrace)
		drawTrace();
}

void display()
{
	glEnable(GL_DEPTH_TEST);

	float vWidth = wWidth / 2;
	float vHeight = wHeight;

	// LEFT SCREEN : View From Camera (Perspective Projection)
	glViewport(0, 0, vWidth, vHeight);
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

	float h = 4;
	float aspectRatio = vWidth / vHeight;
	float w = aspectRatio * h;
	mat4 ViewMat = myLookAt(cameraPos, cameraPos + viewDirection, vec3(0, 1, 0));
	mat4 ProjMat = myPerspective(45, aspectRatio, 0.01, 20);

	g_Mat = ProjMat * ViewMat;
	drawScene(false);							// drawing scene except the camera


	// RIGHT SCREEN : View from above (Orthographic parallel projection)
	glViewport(vWidth, 0, vWidth, vHeight);
	h = MazeSize;
	w = aspectRatio * h;
	ViewMat = myLookAt(vec3(0, 5, 0), vec3(0, 0, 0), vec3(0, 0, -1));
	ProjMat = myOrtho(-w / 2, w / 2, -h / 2, h / 2, 0, 20);

	g_Mat = ProjMat * ViewMat;
	drawScene(true);


	glutSwapBuffers();
}

inline int Heuristic(int i, int j)
{
	return abs(i - goalindexI) + abs(j - goalindexJ);
}

bool FindPathAStar(int si, int sj)
{
	const int INF = 1e9;
	static int gCost[255][255];
	static bool closed[255][255];
	static pair<int, int> parentCell[255][255];

	// 초기화
	for (int i = 0; i < MazeSize; i++) {
		for (int j = 0; j < MazeSize; j++) {
			gCost[i][j] = INF;
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

	priority_queue<Node, vector<Node>, Compare> Navigator;

	gCost[si][sj] = 0;
	Navigator.push({ si, sj, 0, Heuristic(si, sj) });

	int di[4] = { 1, -1, 0, 0 };
	int dj[4] = { 0, 0, 1, -1 };

	while (!Navigator.empty()) {
		Node cur = Navigator.top();
		Navigator.pop();

		int i = cur.i, j = cur.j;
		if (closed[i][j]) continue;
		closed[i][j] = true;

		// 목표 도착
		if (i == goalindexI && j == goalindexJ) {
			g_pathCells.clear();
			int ci = i, cj = j;

			while (!(ci == si && cj == sj)) {
				g_pathCells.push_back({ ci, cj });
				auto p = parentCell[ci][cj];
				ci = p.first;
				cj = p.second;
			}
			g_pathCells.push_back({ si, sj });
			reverse(g_pathCells.begin(), g_pathCells.end());

			return true;
		}

		// 이웃 탐색
		for (int k = 0; k < 4; k++) {
			int ni = i + di[k];
			int nj = j + dj[k];

			// 범위 및 벽 체크
			if (ni < 0 || ni >= MazeSize || nj < 0 || nj >= MazeSize) continue;
			if (maze[ni][nj] == '*') continue;

			int newG = gCost[i][j] + 1;
			if (newG < gCost[ni][nj]) {
				gCost[ni][nj] = newG;
				parentCell[ni][nj] = { i, j };

				int newF = newG + Heuristic(ni, nj);
				Navigator.push({ ni, nj, newG, newF });
			}
		}
	}

	return false;
}


bool WorldPosToIndex(const vec3& pos, int& outI, int& outJ)
{
	float unit = 1.0f;

	float left = -MazeSize / 2.0f + unit / 2.0f;

	float fx = (pos.x - left) / unit;
	float fz = (pos.z - left) / unit;

	int i = (int)roundf(fx);
	int j = (int)roundf(fz);

	if (i < 0 || i >= MazeSize || j < 0 || j >= MazeSize)
		return false; // 미로 밖

	outI = i;
	outJ = j;
	return true;
}

bool IsWallAtWorldPos(const vec3& pos)
{
	int i, j;
	if (!WorldPosToIndex(pos, i, j))
		return true;        // 밖은 막힌 걸로 취급

	return (maze[i][j] == '*');
}


bool CanMoveCameraTo(const vec3& tryPos, const vec3& viewDirection, bool isForward)
{
	// forward / right 벡터 만들기
	vec3 forward = vec3(viewDirection.x, 0.0f, viewDirection.z);
	if (length(forward) < 1e-5f)
		forward = vec3(0, 0, -1);

	forward = normalize(forward);

	vec3 right = normalize(vec3(forward.z, 0.0f, -forward.x));

	float bodyHalf = cameraSize * 0.5f;
	float headHalf = cameraSize * 0.25f;

	vec3 headCenter = tryPos + forward * (bodyHalf + headHalf);
	vec3 pts[6];

	// 몸통 4점 
	pts[0] = tryPos - right * bodyHalf - forward * bodyHalf;
	pts[1] = tryPos - right * bodyHalf + forward * bodyHalf;
	pts[2] = tryPos + right * bodyHalf + forward * bodyHalf;
	pts[3] = tryPos + right * bodyHalf - forward * bodyHalf;

	if (isForward) {
		// 앞으로 갈 때: 앞쪽 4점만 체크
		int idxs[4] = { 1, 2, 4, 5 };
		for (int n = 0; n < 4; ++n) {
			int k = idxs[n];
			if (IsWallAtWorldPos(pts[k])) {
				return false;
			}
		}
	}
	else {
		// 뒤로 갈 때: 뒤쪽 2점만 체크
		int idxs[2] = { 0, 3 };
		for (int n = 0; n < 2; ++n) {
			int k = idxs[n];
			if (IsWallAtWorldPos(pts[k])) {
				return false;
			}
		}
	}

	return true;
}

bool g_followPath = false;	// 경로 따라가는 중인지
int  g_followIndex = 0;		// g_pathCells에서 현재 몇 번째까지 왔는지
int  g_followState = 0;		// 0 = 회전 중, 1 = 직선 이동 중

void AutoMoveCameraAlongPath(float dt)
{
	const float FOLLOW_POS_EPS = 0.01f; // 위치 오차 허용
	const float FOLLOW_ANG_EPS = 0.03f; // 각도 오차 허용

	// 마지막 지점까지 도달했으면 종료
	if (g_followIndex >= (int)g_pathCells.size() - 1) {
		g_followPath = false;
		return;
	}

	// 다음 목표 셀
	auto curCell = g_pathCells[g_followIndex];
	auto nextCell = g_pathCells[g_followIndex + 1];
	vec3 pCur = getPositionFromIndex(curCell.first, curCell.second);
	vec3 pNext = getPositionFromIndex(nextCell.first, nextCell.second);

	vec3 desiredDir = pNext - pCur;
	desiredDir.y = 0.0f;
	float segLen = length(desiredDir);
	if (segLen < 1e-5f) {
		// 셀 좌표가 이상하면 그냥 다음으로
		g_followIndex++;
		return;
	}
	desiredDir /= segLen;

	float targetYaw = atan2f(desiredDir.x, -desiredDir.z);

	// 상태 0: 회전 중
	if (g_followState == 0) {

		// 최단 회전 방향으로 보정
		float delta = targetYaw - cameraYaw;
		const float PI = 3.141592f;
		while (delta > PI) delta -= 2.0f * 2.0f * PI * 0.5f;
		while (delta < -PI) delta += 2.0f * 2.0f * PI * 0.5f;

		if (fabs(delta) < FOLLOW_ANG_EPS) {
			cameraYaw = targetYaw;
			viewDirection = normalize(vec3(sinf(cameraYaw), 0.0f, -cosf(cameraYaw)));
			cameraPos = pCur;
			g_followState = 1; // 직선 이동으로 전환
		}
		else {
			float step = rotateSpeed * dt;
			//if (fabs(delta) < step) step = fabs(delta); // overshoot 방지
			cameraYaw += (delta > 0 ? step : -step);
			viewDirection = normalize(vec3(sinf(cameraYaw), 0.0f, -cosf(cameraYaw)));
		}

		return;
	}

	// 상태 1: 직선 이동 중
	if (g_followState == 1) {

		vec3 moveDir = normalize(vec3(viewDirection.x, 0.0f, viewDirection.z));
		vec3 tryPos = cameraPos + moveDir * dt * cameraSpeed;
		vec3 fromCur = tryPos - pCur;
		float prog = dot(fromCur, desiredDir);

		if (prog >= segLen) {
			cameraPos = pNext;
			g_followIndex++;
			g_followState = 0; // 회전 상태로 전환
		}
		else {
			cameraPos = tryPos;
		}
	}
}

bool g_prevQDown = false;
bool g_prevSpaceDown = false;

void idle()
{
	float dt = 0.016f;
	g_time += dt;
	if (g_followPath) {										// 자동 이동 중
		AutoMoveCameraAlongPath(dt);
	}
	else {
		if ((GetAsyncKeyState('A') & 0x8000) == 0x8000)		// 왼쪽으로 회전
			cameraYaw -= rotateSpeed * dt;
		if ((GetAsyncKeyState('D') & 0x8000) == 0x8000)		// 오른쪽으로 회전
			cameraYaw += rotateSpeed * dt;
		viewDirection = normalize(vec3(sinf(cameraYaw), 0.0f, -cosf(cameraYaw)));

		if ((GetAsyncKeyState('W') & 0x8000) == 0x8000) {	// 카메라 보는 방향으로 이동
			vec3 tryPos = cameraPos + cameraSpeed * dt * viewDirection;
			if (CanMoveCameraTo(tryPos, viewDirection, true))
				cameraPos = tryPos;
		}
		if ((GetAsyncKeyState('S') & 0x8000) == 0x8000) {	// 카메라 보는 방향의 반대 방향으로 이동
			vec3 tryPos = cameraPos - cameraSpeed * dt * viewDirection;
			if (CanMoveCameraTo(tryPos, viewDirection, false))
				cameraPos = tryPos;
		}
	}
	bool qDown = (GetAsyncKeyState('Q') & 0x8000) != 0; // Q 키를 눌렀을 때 경로 탐색 시작/종료
	if (qDown && !g_prevQDown) {						// Q 키가 새로 눌린 경우
		int si, sj;
		if (WorldPosToIndex(cameraPos, si, sj)) {
			bDrawTrace = !bDrawTrace;
			FindPathAStar(si, sj);
			/*if (bDrawTrace) {
				if (FindPathAStar(si, sj))
					printf("Path found.\n");
				else
					printf("Not found.\n");
			}*/
		}
	}
	g_prevQDown = qDown;								// 이전 프레임의 Q 키 상태 저장
	bool spaceDown = (GetAsyncKeyState(VK_SPACE) & 0x8000) != 0;
	if (spaceDown && !g_prevSpaceDown) {

		int si, sj;
		if (WorldPosToIndex(cameraPos, si, sj)) {
			if (FindPathAStar(si, sj)) {
				g_followPath = !g_followPath;	// 자동 이동 시작
				g_followIndex = 0;
				g_followState = 0;
			}
			else {
				g_followPath = false;
			}
		}
	}
	g_prevSpaceDown = spaceDown;


	Sleep(16);											// for vSync
	glutPostRedisplay();
}

void reshape(int wx, int wy)
{
	printf("%d %d \n", wx, wy);
	wWidth = wx;
	wHeight = wy;
	glutPostRedisplay();
}


int main(int argc, char** argv)
{
	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(wWidth, wHeight);

	glutCreateWindow("Homework3 (Maze Navigator)");

	glewExperimental = true;
	glewInit();

	printf("OpenGL %s, GLSL %s\n", glGetString(GL_VERSION),
		glGetString(GL_SHADING_LANGUAGE_VERSION));

	myInit();
	glutDisplayFunc(display);
	glutIdleFunc(idle);
	glutReshapeFunc(reshape);
	glutMainLoop();

	return 0;
}
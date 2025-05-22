#include <iostream>
#include <queue>
#include <stack>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <cstring>
#include <limits>
#include <string>
#include <algorithm>
#include <vector>
#include <SFML/Graphics.hpp>
#include <iomanip>

using namespace std;

const int ROWS = 25;
const int COLS = 50;
const char WALL = '#';
const char PATH = '.';
const char START = 'S';
const char END = 'E';
const char VISITED = '*';

struct Point {
	int x, y;
	Point(int x = 0, int y = 0) : x(x), y(y) {}
	bool operator==(const Point& other) const { return x == other.x && y == other.y; }
	bool operator<(const Point& other) const { return x != other.x ? x < other.x : y < other.y; }
};

char grid[ROWS][COLS];
bool visited[ROWS][COLS];
Point parent[ROWS][COLS];
int dist[ROWS][COLS];      // Dijkstra
int gScore[ROWS][COLS];    // A*

Point start(0, 0);
Point endpoint(ROWS - 1, COLS - 1);

const int DIRS = 8;
int dx[DIRS] = { -1, -1, 0, 1, 1, 1, 0, -1 };
int dy[DIRS] = { 0, 1, 1, 1, 0, -1, -1, -1 };

bool isValid(int x, int y) {
	return x >= 0 && y >= 0 && x < ROWS && y < COLS && grid[x][y] != WALL;
}

void initializeGrid() {
	for (int i = 0; i < ROWS; ++i)
		for (int j = 0; j < COLS; ++j)
			grid[i][j] = WALL;
	memset(visited, false, sizeof(visited));
}

void carvePathDFS(Point curr) {
	visited[curr.x][curr.y] = true;
	grid[curr.x][curr.y] = PATH;
	int dirs[] = { 0, 1, 2, 3 };
	random_shuffle(dirs, dirs + 4);
	for (int i = 0; i < 4; ++i) {
		int dir = dirs[i];
		int nx = curr.x + dx[dir] * 2;
		int ny = curr.y + dy[dir] * 2;
		if (nx >= 0 && nx < ROWS && ny >= 0 && ny < COLS && !visited[nx][ny]) {
			grid[curr.x + dx[dir]][curr.y + dy[dir]] = PATH;
			carvePathDFS(Point(nx, ny));
		}
	}
}

void guaranteeSolvableMaze() {
	initializeGrid();
	carvePathDFS(start);
	if (grid[endpoint.x][endpoint.y] == WALL) grid[endpoint.x][endpoint.y] = PATH;
	if (endpoint.x > 0 && grid[endpoint.x - 1][endpoint.y] == WALL) grid[endpoint.x - 1][endpoint.y] = PATH;
	if (endpoint.y > 0 && grid[endpoint.x][endpoint.y - 1] == WALL) grid[endpoint.x][endpoint.y - 1] = PATH;
	for (int i = 1; i < ROWS - 1; ++i)
		for (int j = 1; j < COLS - 1; ++j)
			if (grid[i][j] == WALL && rand() % 100 < 40)
				grid[i][j] = PATH;
	grid[start.x][start.y] = START;
	grid[endpoint.x][endpoint.y] = END;
}

void printGrid() {
	for (int i = 0; i < ROWS; ++i) {
		for (int j = 0; j < COLS; ++j)
			cout << grid[i][j];
		cout << endl;
	}
}

void tracePath(Point p, int& pathLength) {
	pathLength = 0;
	while (!(p == start)) {
		if (grid[p.x][p.y] != START && grid[p.x][p.y] != END) {
			grid[p.x][p.y] = VISITED;
			pathLength++;
		}
		p = parent[p.x][p.y];
	}
}

bool bfs(int& nodesExplored) {
	memset(visited, false, sizeof(visited));
	queue<Point> q;
	q.push(start);
	visited[start.x][start.y] = true;
	nodesExplored = 0;

	while (!q.empty()) {
		Point curr = q.front(); q.pop();
		nodesExplored++;
		if (curr == endpoint) return true;

		for (int i = 0; i < 4; ++i) {
			int nx = curr.x + dx[i];
			int ny = curr.y + dy[i];
			if (isValid(nx, ny) && !visited[nx][ny]) {
				visited[nx][ny] = true;
				parent[nx][ny] = curr;
				q.push(Point(nx, ny));
			}
		}
	}
	return false;
}

bool dfs(Point curr, int& nodesExplored) {
	if (!isValid(curr.x, curr.y) || visited[curr.x][curr.y])
		return false;

	visited[curr.x][curr.y] = true;
	nodesExplored++;

	if (curr == endpoint)
		return true;

	for (int i = 0; i < 4; ++i) {
		int nx = curr.x + dx[i];
		int ny = curr.y + dy[i];
		Point next(nx, ny);

		if (isValid(nx, ny) && !visited[nx][ny]) {
			parent[nx][ny] = curr;
			if (dfs(next, nodesExplored))
				return true;
		}
	}
	return false;
}


bool dijkstra(int& nodesExplored) {
	memset(visited, false, sizeof(visited));
	for (int i = 0; i < ROWS; ++i)
		for (int j = 0; j < COLS; ++j)
			dist[i][j] = INT_MAX;
	dist[start.x][start.y] = 0;
	typedef pair<int, Point> PIP;
	priority_queue<PIP, deque<PIP>, greater<PIP>> pq;
	pq.push({ 0, start });
	nodesExplored = 0;

	while (!pq.empty()) {
		PIP top = pq.top(); pq.pop();
		Point curr = top.second;
		if (visited[curr.x][curr.y]) continue;
		visited[curr.x][curr.y] = true;
		nodesExplored++;
		if (curr == endpoint) return true;

		for (int i = 0; i < 4; ++i) {
			int nx = curr.x + dx[i];
			int ny = curr.y + dy[i];
			if (isValid(nx, ny) && dist[nx][ny] > dist[curr.x][curr.y] + 1) {
				dist[nx][ny] = dist[curr.x][curr.y] + 1;
				parent[nx][ny] = curr;
				pq.push({ dist[nx][ny], Point(nx, ny) });
			}
		}
	}
	return false;
}

int heuristic(Point a, Point b) {
	return max(abs(a.x - b.x), abs(a.y - b.y)); // Chebyshev distance for 8-dir
}

bool astar(int& nodesExplored) {
	memset(visited, false, sizeof(visited));
	for (int i = 0; i < ROWS; ++i)
		for (int j = 0; j < COLS; ++j)
			gScore[i][j] = INT_MAX;

	gScore[start.x][start.y] = 0;
	typedef pair<int, Point> PIP;
	priority_queue<PIP, deque<PIP>, greater<PIP>> openSet;
	openSet.push({ heuristic(start, endpoint), start });
	nodesExplored = 0;

	while (!openSet.empty()) {
		PIP top = openSet.top(); openSet.pop();
		Point curr = top.second;
		if (visited[curr.x][curr.y]) continue;
		visited[curr.x][curr.y] = true;
		nodesExplored++;
		if (curr == endpoint) return true;

		for (int i = 0; i < 4; ++i) {
			int nx = curr.x + dx[i];
			int ny = curr.y + dy[i];
			if (!isValid(nx, ny)) continue;
			int tentative_g = gScore[curr.x][curr.y] + 1;
			if (tentative_g < gScore[nx][ny]) {
				gScore[nx][ny] = tentative_g;
				parent[nx][ny] = curr;
				openSet.push({ tentative_g + heuristic(Point(nx, ny), endpoint), Point(nx, ny) });
			}
		}
	}
	return false;
}
bool hasForcedNeighbor(int x, int y, int dx, int dy) {
	if (dx != 0 && dy != 0) {
		if ((isValid(x - dx, y + dy) && !isValid(x - dx, y)) ||
			(isValid(x + dx, y - dy) && !isValid(x, y - dy)))
			return true;
	}
	else if (dx != 0) {
		if ((isValid(x + dx, y + 1) && !isValid(x, y + 1)) ||
			(isValid(x + dx, y - 1) && !isValid(x, y - 1)))
			return true;
	}
	else if (dy != 0) {
		if ((isValid(x + 1, y + dy) && !isValid(x + 1, y)) ||
			(isValid(x - 1, y + dy) && !isValid(x - 1, y)))
			return true;
	}
	return false;
}

bool jump(int x, int y, int dx, int dy, Point& end, Point& out) {
	if (!isValid(x, y) || grid[x][y] == '#') return false;
	if (Point(x, y) == end) {
		out = Point(x, y);
		return true;
	}
	if (hasForcedNeighbor(x, y, dx, dy)) {
		out = Point(x, y);
		return true;
	}
	if (dx != 0 && dy != 0) {
		Point temp;
		if (jump(x + dx, y, dx, 0, end, temp) || jump(x, y + dy, 0, dy, end, temp)) {
			out = Point(x, y);
			return true;
		}
	}
	return jump(x + dx, y + dy, dx, dy, end, out);
}

bool jps(int& nodesExplored) {
	memset(visited, false, sizeof(visited));
	for (int i = 0; i < ROWS; ++i)
		for (int j = 0; j < COLS; ++j)
			gScore[i][j] = INT_MAX;

	gScore[start.x][start.y] = 0;
	typedef pair<int, Point> PIP;
	priority_queue<PIP, deque<PIP>, greater<PIP>> openSet;
	openSet.push({ heuristic(start, endpoint), start });
	nodesExplored = 0;

	while (!openSet.empty()) {
		PIP top = openSet.top(); openSet.pop();
		Point curr = top.second;
		if (visited[curr.x][curr.y]) continue;
		visited[curr.x][curr.y] = true;
		nodesExplored++;
		if (curr == endpoint) return true;

		for (int i = 0; i < DIRS; ++i) {
			int dx_ = dx[i], dy_ = dy[i];
			Point jumpPoint;
			if (jump(curr.x + dx_, curr.y + dy_, dx_, dy_, endpoint, jumpPoint)) {
				int tentative_g = gScore[curr.x][curr.y] + heuristic(curr, jumpPoint);
				if (tentative_g < gScore[jumpPoint.x][jumpPoint.y]) {
					gScore[jumpPoint.x][jumpPoint.y] = tentative_g;
					parent[jumpPoint.x][jumpPoint.y] = curr;
					openSet.push({ tentative_g + heuristic(jumpPoint, endpoint), jumpPoint });
				}
			}
		}
	}
	return false;
}

void resetGridPath() {
	for (int i = 0; i < ROWS; ++i)
		for (int j = 0; j < COLS; ++j)
			if (grid[i][j] == VISITED)
				grid[i][j] = PATH;
	grid[start.x][start.y] = START;
	grid[endpoint.x][endpoint.y] = END;
}

void runAlgorithm(int choice) {
	resetGridPath();
	bool found = false;
	int pathLength = 0;
	int nodesExplored = 0;
	string name;
	auto start_time = chrono::high_resolution_clock::now();

	if (choice == 1) {
		name = "BFS";
		found = bfs(nodesExplored);
	}
	else if (choice == 2) {
		name = "DFS";
		parent[start.x][start.y] = start;
		found = dfs(start, nodesExplored);
	}
	else if (choice == 3) {
		name = "Dijkstra";
		found = dijkstra(nodesExplored);
	}
	else if (choice == 4) {
		name = "A*";
		found = astar(nodesExplored);
	}
	else if (choice == 5) {
		name = "JumpPoint";
		found = jps(nodesExplored);
	}

	auto end_time = chrono::high_resolution_clock::now();
	auto duration = chrono::duration_cast<chrono::microseconds>(end_time - start_time);

	if (found) {
		tracePath(endpoint, pathLength);
		grid[start.x][start.y] = START;
		grid[endpoint.x][endpoint.y] = END;
		printGrid();
		cout << "\nAlgorithm: " << name;
		cout << "\nPath Length: " << pathLength;
		cout << "\nNodes Explored: " << nodesExplored;
		cout << "\nTime Taken: " << duration.count() << " microseconds.";
		cout << "\nOperations: " << (pathLength * nodesExplored) << endl;
	}
	else {
		cout << "\nNo path found.\n";
	}
}


void compareAlgorithms() {
	string names[] = { "BFS", "DFS", "Dijkstra", "A*", "JumpPoint" };
	cout << left << setw(12) << "Algorithm" << setw(15) << "Path Length" << setw(18) << "Nodes Explored" << setw(18) << "Time (us)" << "Operations" << endl;
	cout << string(65, '-') << endl;
	for (int i = 1; i <= 5; ++i) {
		resetGridPath();
		bool found = false;
		int pathLength = 0, nodesExplored = 0;
		auto start_time = chrono::high_resolution_clock::now();
		if (i == 1) found = bfs(nodesExplored);
		else if (i == 2) found = dfs(start, nodesExplored);
		else if (i == 3) found = dijkstra(nodesExplored);
		else if (i == 4) found = astar(nodesExplored);
		else if (i == 5) found = jps(nodesExplored);
		auto end_time = chrono::high_resolution_clock::now();
		auto duration = chrono::duration_cast<chrono::microseconds>(end_time - start_time);
		if (found) tracePath(endpoint, pathLength);
		cout << left << setw(12) << names[i - 1] << setw(15) << pathLength << setw(18) << nodesExplored << setw(18) << duration.count() << pathLength * nodesExplored << endl;
	}
	cout << endl;
}

// sfml

void runMenu(sf::RenderWindow& window, int& selectedAlgo) {
	sf::Font font;
	if (!font.loadFromFile("Bemirs-reg.ttf")) {
		cout << "Font loading failed!" << endl;
		return;
	}

	vector<string> options = {
		"Run BFS",
		"Run DFS",
		"Run Dijkstra",
		"Run A*",
		"Run Jump Point",
		"Compare Algorithms"
	};

	vector<sf::Text> menuTexts;
	for (int i = 0; i < options.size(); i++) {
		sf::Text text(options[i], font, 36);
		text.setFillColor(sf::Color::Black);
		sf::FloatRect textRect = text.getLocalBounds();
		text.setOrigin(textRect.left + textRect.width / 2.0f, textRect.top + textRect.height / 2.0f);
		text.setPosition(window.getSize().x / 2.0f, 150.f + i * 60.f);
		menuTexts.push_back(text);
	}

	while (window.isOpen()) {
		sf::Event event;

		// Get correct mouse position (relative to window)
		sf::Vector2i pixelPos = sf::Mouse::getPosition(window);
		sf::Vector2f mousePos = window.mapPixelToCoords(pixelPos);

		while (window.pollEvent(event)) {
			if (event.type == sf::Event::Closed)
				window.close();

			if (event.type == sf::Event::MouseButtonPressed &&
				event.mouseButton.button == sf::Mouse::Left) {

				// Update mouse position again in case event timing matters
				sf::Vector2i clickPixelPos = sf::Mouse::getPosition(window);
				sf::Vector2f clickMousePos = window.mapPixelToCoords(clickPixelPos);

				for (int i = 0; i < menuTexts.size(); i++) {
					if (menuTexts[i].getGlobalBounds().contains(clickMousePos)) {
						selectedAlgo = i + 1;
						return;
					}
				}
			}
		}

		window.clear(sf::Color::White);

		// Hover effect
		for (int i = 0; i < menuTexts.size(); i++) {
			if (menuTexts[i].getGlobalBounds().contains(mousePos)) {
				menuTexts[i].setScale(1.2f, 1.2f);
			}
			else {
				menuTexts[i].setScale(1.0f, 1.0f);
			}
			window.draw(menuTexts[i]);
		}

		window.display();
	}
}

int main() {
	guaranteeSolvableMaze();

	sf::RenderWindow window(sf::VideoMode(800, 600), "Maze Visualizer");

	int selectedAlgo = 0;
	runMenu(window, selectedAlgo); // show menu

	window.clear(sf::Color::Black); // clear for clean screen
	window.display();
	sf::sleep(sf::seconds(1));
	if (selectedAlgo >= 1 && selectedAlgo <= 5) {
		runAlgorithm(selectedAlgo);
	}
	else if (selectedAlgo == 6) {
		compareAlgorithms();
	}

	return 0;
}
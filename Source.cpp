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

// Constants for grid dimensions and tile types
const int ROWS = 25;
const int COLS = 50;
const char WALL = '#';
const char PATH = '.';
const char START = 'S';
const char END = 'E';
const char VISITED = '*'; // For path visualization
const char FINAL_PATH = 'P';  // Cyan path that is the final path

// Constants for SFML window and tile size
const int TILE_SIZE = 20;
const int WINDOW_WIDTH = COLS * TILE_SIZE;
const int WINDOW_HEIGHT = ROWS * TILE_SIZE;

// Structure to represent a point (x, y) on the grid
struct Point {
    int x, y;
    Point(int x = 0, int y = 0) : x(x), y(y) {}
    // Overload operators for comparison
    bool operator==(const Point& other) const { return x == other.x && y == other.y; }
    bool operator<(const Point& other) const { return x != other.x ? x < other.x : y < other.y; }
};
void delay(int milliseconds) {
    sf::sleep(sf::milliseconds(milliseconds));
}
// Global grid, visited status, parent pointers, and distance arrays
char grid[ROWS][COLS];
bool visited[ROWS][COLS];
Point parent[ROWS][COLS]; // To reconstruct the path
int dist[ROWS][COLS];     // For Dijkstra's algorithm
int gScore[ROWS][COLS];   // For A* and JPS (cost from start to current)

// Start and end points of the maze
Point start(0, 0);
Point endpoint(ROWS - 1, COLS - 1);

// Directional arrays for 8-directional movement (used by JPS)
const int DIRS_8 = 8;
int dx_8[DIRS_8] = { -1, -1, 0, 1, 1, 1, 0, -1 }; // All 8 directions (N, NE, E, SE, S, SW, W, NW)
int dy_8[DIRS_8] = { 0, 1, 1, 1, 0, -1, -1, -1 };

// Directional arrays for 4-directional (cardinal) movement (used by maze generation, BFS, DFS, Dijkstra, A*)
const int DIRS_4 = 4;
int dx_4[DIRS_4] = { -1, 1, 0, 0 }; // Up, Down, Left, Right
int dy_4[DIRS_4] = { 0, 0, -1, 1 };

// Checks if a given coordinate is within grid bounds and not a wall
bool isValid(int x, int y) {
    return x >= 0 && y >= 0 && x < ROWS && y < COLS && grid[x][y] != WALL;
}

// Initializes the grid with walls and resets visited status
void initializeGrid() {
    for (int i = 0; i < ROWS; ++i)
        for (int j = 0; j < COLS; ++j)
            grid[i][j] = WALL;
    memset(visited, false, sizeof(visited)); // Reset visited array
}

// Carves paths in the maze using Depth-First Search (DFS)
void carvePathDFS(Point curr) {
    visited[curr.x][curr.y] = true;
    grid[curr.x][curr.y] = PATH;

    // Randomize directions for a more organic maze
    int dirs[] = { 0, 1, 2, 3 }; // Using indices for dx_4, dy_4
    random_shuffle(dirs, dirs + 4);

    for (int i = 0; i < 4; ++i) {
        int dir_idx = dirs[i];
        // Calculate new position by moving two steps in a direction
        int nx = curr.x + dx_4[dir_idx] * 2;
        int ny = curr.y + dy_4[dir_idx] * 2;

        // Check if the new position is valid and not visited
        if (nx >= 0 && nx < ROWS && ny >= 0 && ny < COLS && !visited[nx][ny]) {
            // Carve the path between current and new position
            grid[curr.x + dx_4[dir_idx]][curr.y + dy_4[dir_idx]] = PATH;
            carvePathDFS(Point(nx, ny)); // Recurse
        }
    }
}

// Generates a maze ensuring it's solvable and adds some randomness
void guaranteeSolvableMaze() {
    initializeGrid();
    carvePathDFS(start); // Start carving from the start point

    // Ensure start and end points are paths
    grid[start.x][start.y] = START;
    grid[endpoint.x][endpoint.y] = END;

    // Ensure the endpoint is reachable if it was accidentally a wall
    if (grid[endpoint.x][endpoint.y] == WALL) grid[endpoint.x][endpoint.y] = PATH;
    // Also ensure neighbors of endpoint are path if they were walls, to aid reachability
    if (endpoint.x > 0 && grid[endpoint.x - 1][endpoint.y] == WALL) grid[endpoint.x - 1][endpoint.y] = PATH;
    if (endpoint.y > 0 && grid[endpoint.x][endpoint.y - 1] == WALL) grid[endpoint.x][endpoint.y - 1] = PATH;

    // Add some random paths to make the maze less sparse and more interesting
    for (int i = 1; i < ROWS - 1; ++i) {
        for (int j = 1; j < COLS - 1; ++j) {
            if (grid[i][j] == WALL && rand() % 100 < 40) { // 40% chance to turn a wall into a path
                grid[i][j] = PATH;
            }
        }
    }
    // Re-set start and end points after random path additions
    grid[start.x][start.y] = START;
    grid[endpoint.x][endpoint.y] = END;
}

// Draws the grid on the SFML window
void drawGrid(sf::RenderWindow& window) {
    for (int i = 0; i < ROWS; ++i) {
        for (int j = 0; j < COLS; ++j) {
            sf::RectangleShape cell(sf::Vector2f(TILE_SIZE - 1, TILE_SIZE - 1)); // -1 for border effect
            cell.setPosition(j * static_cast<float>(TILE_SIZE), i * static_cast<float>(TILE_SIZE));

            // Set color based on cell type
            switch (grid[i][j]) {
            case WALL:      cell.setFillColor(sf::Color::White); break;
            case PATH:      cell.setFillColor(sf::Color::Black); break;
            case START:     cell.setFillColor(sf::Color(255, 0, 255)); break;
            case END:       cell.setFillColor(sf::Color(158, 135, 204)); break;
            case VISITED:   cell.setFillColor(sf::Color(242, 216, 253)); break;
            case FINAL_PATH:cell.setFillColor(sf::Color(151, 29, 136)); break;
            default:        cell.setFillColor(sf::Color::Magenta); break;
            }

            window.draw(cell);
        }
    }
}

// Traces the path from endpoint back to start using parent pointers
void tracePath(Point p, int& pathLength, sf::RenderWindow& window) {
    pathLength = 0;
    vector<Point> path;

    // Backtrack from end to start and store in a vector
    while (!(p == start)) {
        if (grid[p.x][p.y] != START && grid[p.x][p.y] != END) {
            path.push_back(p);
            pathLength++;
        }
        p = parent[p.x][p.y];
    }

    // Reverse path so animation plays from start to end
    reverse(path.begin(), path.end());

    for (Point pt : path) {
        grid[pt.x][pt.y] = FINAL_PATH;
        window.clear();
        drawGrid(window);
        window.display();
        delay(10);
    }
}


// Breadth-First Search (BFS) algorithm
bool bfs(int& nodesExplored, sf::RenderWindow& window) {
    memset(visited, false, sizeof(visited));
    memset(parent, 0, sizeof(parent));
    queue<Point> q;
    q.push(start);
    visited[start.x][start.y] = true;
    nodesExplored = 0;
    parent[start.x][start.y] = start;

    while (!q.empty()) {
        Point curr = q.front(); q.pop();
        nodesExplored++;

        if (curr == endpoint) return true;

        for (int i = 0; i < DIRS_4; ++i) {
            int nx = curr.x + dx_4[i];
            int ny = curr.y + dy_4[i];
            if (isValid(nx, ny) && !visited[nx][ny]) {
                visited[nx][ny] = true;
                parent[nx][ny] = curr;
                q.push(Point(nx, ny));

                if (grid[nx][ny] != END) grid[nx][ny] = VISITED;
                window.clear(); drawGrid(window); window.display();
                delay(5);
            }
        }
    }
    return false;
}


// Depth-First Search (DFS) algorithm (recursive)
bool dfs(Point curr, int& nodesExplored, sf::RenderWindow& window) {
    if (!isValid(curr.x, curr.y) || visited[curr.x][curr.y]) return false;

    visited[curr.x][curr.y] = true;
    nodesExplored++;

    if (curr == endpoint) return true;

    for (int i = 0; i < DIRS_4; ++i) {
        int nx = curr.x + dx_4[i];
        int ny = curr.y + dy_4[i];
        Point next(nx, ny);

        if (isValid(nx, ny) && !visited[nx][ny]) {
            parent[nx][ny] = curr;

            if (grid[nx][ny] != END) grid[nx][ny] = VISITED;
            window.clear(); drawGrid(window); window.display();
            delay(5);

            if (dfs(next, nodesExplored, window)) return true;
        }
    }
    return false;
}

// Dijkstra's algorithm
bool dijkstra(int& nodesExplored, sf::RenderWindow& window) {
    memset(visited, false, sizeof(visited));
    memset(parent, 0, sizeof(parent));
    for (int i = 0; i < ROWS; ++i)
        for (int j = 0; j < COLS; ++j)
            dist[i][j] = numeric_limits<int>::max();

    dist[start.x][start.y] = 0;
    typedef pair<int, Point> PIP;
    priority_queue<PIP, vector<PIP>, greater<PIP>> pq;
    pq.push({ 0, start });
    nodesExplored = 0;
    parent[start.x][start.y] = start;

    while (!pq.empty()) {
        PIP top = pq.top(); pq.pop();
        Point curr = top.second;
        if (visited[curr.x][curr.y]) continue;

        visited[curr.x][curr.y] = true;
        nodesExplored++;

        if (curr == endpoint) return true;

        for (int i = 0; i < DIRS_4; ++i) {
            int nx = curr.x + dx_4[i];
            int ny = curr.y + dy_4[i];
            if (isValid(nx, ny) && dist[nx][ny] > dist[curr.x][curr.y] + 1) {
                dist[nx][ny] = dist[curr.x][curr.y] + 1;
                parent[nx][ny] = curr;
                pq.push({ dist[nx][ny], Point(nx, ny) });

                if (grid[nx][ny] != END) grid[nx][ny] = VISITED;
                window.clear(); drawGrid(window); window.display();
                delay(5);
            }
        }
    }
    return false;
}

// Manhattan distance heuristic for 4-directional movement (used by A*)
int heuristic_manhattan(Point a, Point b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}

// Chebyshev distance heuristic for 8-directional movement (used by JPS)
int heuristic_chebyshev(Point a, Point b) {
    return max(abs(a.x - b.x), abs(a.y - b.y));
}

// A* search algorithm
bool astar(int& nodesExplored, sf::RenderWindow& window) {
    memset(visited, false, sizeof(visited));
    memset(parent, 0, sizeof(parent));
    for (int i = 0; i < ROWS; ++i)
        for (int j = 0; j < COLS; ++j)
            gScore[i][j] = numeric_limits<int>::max();

    gScore[start.x][start.y] = 0;
    typedef pair<int, Point> PIP;
    priority_queue<PIP, vector<PIP>, greater<PIP>> openSet;
    openSet.push({ heuristic_manhattan(start, endpoint), start });
    nodesExplored = 0;
    parent[start.x][start.y] = start;

    while (!openSet.empty()) {
        PIP top = openSet.top(); openSet.pop();
        Point curr = top.second;
        if (visited[curr.x][curr.y]) continue;

        visited[curr.x][curr.y] = true;
        nodesExplored++;

        if (curr == endpoint) return true;

        for (int i = 0; i < DIRS_4; ++i) {
            int nx = curr.x + dx_4[i];
            int ny = curr.y + dy_4[i];
            if (!isValid(nx, ny)) continue;

            int tentative_g = gScore[curr.x][curr.y] + 1;
            if (tentative_g < gScore[nx][ny]) {
                gScore[nx][ny] = tentative_g;
                parent[nx][ny] = curr;
                openSet.push({ tentative_g + heuristic_manhattan(Point(nx, ny), endpoint), Point(nx, ny) });

                if (grid[nx][ny] != END) grid[nx][ny] = VISITED;
                window.clear(); drawGrid(window); window.display();
                delay(5);
            }
        }
    }
    return false;
}

// Checks for forced neighbors in JPS
bool hasForcedNeighbor(int x, int y, int dx, int dy) {
    // Diagonal move
    if (dx != 0 && dy != 0) {
        // Check for obstacles that force a turn
        if ((isValid(x - dx, y + dy) && !isValid(x - dx, y)) ||
            (isValid(x + dx, y - dy) && !isValid(x, y - dy)))
            return true;
    }
    // Cardinal horizontal move
    else if (dx != 0) { // Moving horizontally (dx != 0, dy == 0)
        // Check for obstacles that force a turn
        if ((isValid(x + dx, y + 1) && !isValid(x, y + 1)) ||
            (isValid(x + dx, y - 1) && !isValid(x, y - 1)))
            return true;
    }
    // Cardinal vertical move
    else if (dy != 0) { // Moving vertically (dx == 0, dy != 0)
        // Check for obstacles that force a turn
        if ((isValid(x + 1, y + dy) && !isValid(x + 1, y)) ||
            (isValid(x - 1, y + dy) && !isValid(x - 1, y)))
            return true;
    }
    return false;
}

// Jump function for JPS
bool jump(int x, int y, int dx, int dy, Point& end, Point& out) {
    // Base cases for recursion
    if (!isValid(x, y) || grid[x][y] == WALL) return false; // Invalid or wall
    if (Point(x, y) == end) { // Reached endpoint
        out = Point(x, y);
        return true;
    }
    if (hasForcedNeighbor(x, y, dx, dy)) { // Found a forced neighbor
        out = Point(x, y);
        return true;
    }

    // If moving diagonally, check cardinal directions for forced neighbors
    if (dx != 0 && dy != 0) {
        Point temp;
        // Check if a jump point exists in cardinal directions
        if (jump(x + dx, y, dx, 0, end, temp) || jump(x, y + dy, 0, dy, end, temp)) {
            out = Point(x, y);
            return true;
        }
    }
    // Continue jumping in the current direction
    return jump(x + dx, y + dy, dx, dy, end, out);
}

// Jump Point Search (JPS) algorithm
bool jps(int& nodesExplored, sf::RenderWindow& window) {
    memset(visited, false, sizeof(visited));
    memset(parent, 0, sizeof(parent));
    for (int i = 0; i < ROWS; ++i)
        for (int j = 0; j < COLS; ++j)
            gScore[i][j] = numeric_limits<int>::max();

    gScore[start.x][start.y] = 0;
    typedef pair<int, Point> PIP;
    priority_queue<PIP, vector<PIP>, greater<PIP>> openSet;
    openSet.push({ heuristic_chebyshev(start, endpoint), start });
    nodesExplored = 0;
    parent[start.x][start.y] = start;

    while (!openSet.empty()) {
        PIP top = openSet.top(); openSet.pop();
        Point curr = top.second;
        if (visited[curr.x][curr.y]) continue;

        visited[curr.x][curr.y] = true;
        nodesExplored++;

        if (curr == endpoint) return true;

        for (int i = 0; i < DIRS_8; ++i) {
            int dx_ = dx_8[i], dy_ = dy_8[i];
            Point jumpPoint;
            if (jump(curr.x + dx_, curr.y + dy_, dx_, dy_, endpoint, jumpPoint)) {
                int tentative_g = gScore[curr.x][curr.y] + heuristic_chebyshev(curr, jumpPoint);
                if (tentative_g < gScore[jumpPoint.x][jumpPoint.y]) {
                    gScore[jumpPoint.x][jumpPoint.y] = tentative_g;
                    parent[jumpPoint.x][jumpPoint.y] = curr;
                    openSet.push({ tentative_g + heuristic_chebyshev(jumpPoint, endpoint), jumpPoint });

                    if (grid[jumpPoint.x][jumpPoint.y] != END)
                        grid[jumpPoint.x][jumpPoint.y] = VISITED;

                    window.clear(); drawGrid(window); window.display();
                    delay(5);
                }
            }
        }
    }
    return false;
}

// Resets the grid path visualization (turns VISITED cells back to PATH)
void resetGridPath() {
    for (int i = 0; i < ROWS; ++i)
        for (int j = 0; j < COLS; ++j)
            if (grid[i][j] == VISITED || grid[i][j] == FINAL_PATH)
                grid[i][j] = PATH;
    grid[start.x][start.y] = START;
    grid[endpoint.x][endpoint.y] = END;
}

// Runs a selected pathfinding algorithm and displays results
void runAlgorithm(int choice, sf::RenderWindow& window) {
    resetGridPath();
    memset(parent, 0, sizeof(parent));
    bool found = false;
    int pathLength = 0;
    int nodesExplored = 0;
    string name;
    auto start_time = chrono::high_resolution_clock::now();

    if (choice == 1) {
        name = "BFS";
        found = bfs(nodesExplored, window);
    }
    else if (choice == 2) {
        name = "DFS";
        parent[start.x][start.y] = start;
        found = dfs(start, nodesExplored, window);
    }
    else if (choice == 3) {
        name = "Dijkstra";
        found = dijkstra(nodesExplored, window);
    }
    else if (choice == 4) {
        name = "A*";
        found = astar(nodesExplored, window);
    }
    else if (choice == 5) {
        name = "JumpPoint";
        found = jps(nodesExplored, window);
    }

    auto end_time = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(end_time - start_time);

    if (found) {
        tracePath(endpoint, pathLength, window);
        grid[start.x][start.y] = START;
        grid[endpoint.x][endpoint.y] = END;
        window.clear(); drawGrid(window); window.display();

        cout << "\nAlgorithm: " << name
            << "\nPath Length: " << pathLength
            << "\nNodes Explored: " << nodesExplored
            << "\nTime Taken: " << duration.count() << " microseconds."
            << "\nOperations: " << (pathLength * nodesExplored) << endl;
    }
    else {
        cout << "\nPath not found by " << name << ".\n";
    }
}
// Compares all algorithms and prints their statistics
void compareAlgorithms(sf::RenderWindow& window) {
    string names[] = { "BFS", "DFS", "Dijkstra", "A*", "JumpPoint" };
    cout << left << setw(12) << "Algorithm" << setw(15) << "Path Length" << setw(18) << "Nodes Explored" << setw(18) << "Time (us)" << "Operations" << endl;
    cout << string(80, '-') << endl;

    for (int i = 1; i <= 5; ++i) {
        resetGridPath();
        memset(parent, 0, sizeof(parent));

        bool found = false;
        int pathLength = 0, nodesExplored = 0;
        auto start_time = chrono::high_resolution_clock::now();

        if (i == 1) found = bfs(nodesExplored, window);
        else if (i == 2) {
            parent[start.x][start.y] = start;
            found = dfs(start, nodesExplored, window);
        }
        else if (i == 3) found = dijkstra(nodesExplored, window);
        else if (i == 4) found = astar(nodesExplored, window);
        else if (i == 5) found = jps(nodesExplored, window);

        auto end_time = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::microseconds>(end_time - start_time);

        if (found) tracePath(endpoint, pathLength, window);
        else pathLength = 0;

        cout << left << setw(12) << names[i - 1]
            << setw(15) << (found ? to_string(pathLength) : "N/A")
            << setw(18) << nodesExplored
            << setw(18) << duration.count()
            << (found ? to_string(pathLength * nodesExplored) : "N/A") << endl;
    }
    cout << endl;
}

// SFML based functions for menu and interaction

// Runs the main menu for algorithm selection
void runMenu(sf::RenderWindow& window, int& selectedAlgo) {
    sf::Font font;
    // Attempt to load the .ttf
    if (!font.loadFromFile("Bemirs-reg.ttf")) {
        cout << "Error: Font file 'Bemirs-reg.ttf' not found. Please ensure it's in the same directory as the executable." << endl;
        // Fallback to default font or handle error
        // For now, just return, which might cause text not to render
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
        // Center the text
        sf::FloatRect textRect = text.getLocalBounds();
        text.setOrigin(textRect.left + textRect.width / 2.0f, textRect.top + textRect.height / 2.0f);
        text.setPosition(window.getSize().x / 2.0f, 150.f + i * 60.f); // Position vertically
        menuTexts.push_back(text);
    }

    while (window.isOpen()) {
        sf::Event event;

        // Get correct mouse position relative to window
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
                        selectedAlgo = i + 1; // Set selected algorithm choice
                        return; // Exit menu loop
                    }
                }
            }
        }

        window.clear(sf::Color::White); // Clear window for drawing

        // Hover effect for menu items
        for (int i = 0; i < menuTexts.size(); i++) {
            if (menuTexts[i].getGlobalBounds().contains(mousePos)) {
                menuTexts[i].setScale(1.1f, 1.1f); // Scale up on hover
                menuTexts[i].setFillColor(sf::Color::Magenta); // Change color on hover
            }
            else {
                menuTexts[i].setScale(1.0f, 1.0f); // Reset scale
                menuTexts[i].setFillColor(sf::Color::Black); // Reset color
            }
            window.draw(menuTexts[i]); // Draw menu item
        }

        window.display(); // Display drawn elements
    }
}


int main() {
    // Seed random number generator for maze generation
    srand(static_cast<unsigned int>(time(0)));

    // Generate the maze before anything else
    guaranteeSolvableMaze();

    // Create the SFML window sized according to grid dimensions
    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "HK's Maze Visualizer");

    int selectedAlgo = 0;
    runMenu(window, selectedAlgo); // Show the menu and get algorithm choice

    // Clear screen before algorithm runs to show a fresh grid
    window.clear(sf::Color::White);
    drawGrid(window); // Draw the initial maze
    window.display();
    sf::sleep(sf::seconds(1)); // Optional pause to see initial maze

    // Run selected algorithm or comparison based on menu choice
    if (selectedAlgo >= 1 && selectedAlgo <= 5) {
        runAlgorithm(selectedAlgo, window); // Run a single algorithm
    }
    else if (selectedAlgo == 6) {
        compareAlgorithms(window); // Run comparison of all algorithms
    }

    // Keep the maze visualization window open until closed by user
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear();
        drawGrid(window); // Continuously draw the maze and path
        window.display();
    }

    return 0;
}
#include <SFML/Graphics.hpp>
#include <queue>
#include <vector>
#include <iostream>
#include <cmath>
#include <set>
#include <ctime>

const int GRID_SIZE = 50;
const int CELL_SIZE = 20;

struct Node {
    int x, y;
    float gCost, hCost, fCost;
    bool operator>(const Node& other) const {
        return fCost > other.fCost;  // Priority queue needs the smallest fCost at the top
    }
};

float manhattanHeuristic(const sf::Vector2i& a, const sf::Vector2i& b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);  // Manhattan distance
}

void drawGrid(sf::RenderWindow& window, const std::vector<std::vector<int>>& grid, const sf::Vector2i& start, const sf::Vector2i& end) {
    for (int i = 0; i < GRID_SIZE; ++i) {
        for (int j = 0; j < GRID_SIZE; ++j) {
            sf::RectangleShape cell(sf::Vector2f(CELL_SIZE - 1, CELL_SIZE - 1));
            cell.setPosition(j * CELL_SIZE, i * CELL_SIZE);
            if (grid[i][j] == 1) {
                cell.setFillColor(sf::Color::Black);  // Wall
            }
            else if (sf::Vector2i(j, i) == start) {
                cell.setFillColor(sf::Color::Green);  // Start
            }
            else if (sf::Vector2i(j, i) == end) {
                cell.setFillColor(sf::Color::Red);    // End
            }
            else if (grid[i][j] == 2) {
                cell.setFillColor(sf::Color(200, 200, 0));  // Visited
            }
            else if (grid[i][j] == 3) {
                cell.setFillColor(sf::Color::Blue);  // Path
            }
            else {
                cell.setFillColor(sf::Color::White);  // Empty
            }
            window.draw(cell);
        }
    }
}

std::vector<sf::Vector2i> getNeighbors(const sf::Vector2i& node, const std::vector<std::vector<int>>& grid) {
    std::vector<sf::Vector2i> neighbors;
    std::vector<sf::Vector2i> directions = { {0, 1}, {1, 0}, {0, -1}, {-1, 0} };
    for (auto& dir : directions) {
        sf::Vector2i neighbor = node + dir;
        if (neighbor.x >= 0 && neighbor.y >= 0 && neighbor.x < GRID_SIZE && neighbor.y < GRID_SIZE && grid[neighbor.y][neighbor.x] == 0) {
            neighbors.push_back(neighbor);
        }
    }
    return neighbors;
}

void visualizeAStar(sf::RenderWindow& window, std::vector<std::vector<int>>& grid, const sf::Vector2i& start, const sf::Vector2i& end) {
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
    std::vector<std::vector<float>> gCost(GRID_SIZE, std::vector<float>(GRID_SIZE, INFINITY));
    std::vector<std::vector<sf::Vector2i>> prev(GRID_SIZE, std::vector<sf::Vector2i>(GRID_SIZE, { -1, -1 }));
    gCost[start.y][start.x] = 0;

    pq.push({ start.x, start.y, 0, manhattanHeuristic(start, end), manhattanHeuristic(start, end) });

    while (!pq.empty()) {
        Node current = pq.top();
        pq.pop();

        sf::Vector2i currentPos(current.x, current.y);
        if (currentPos == end) break;

        for (auto& neighbor : getNeighbors(currentPos, grid)) {
            float tentativeGCost = gCost[current.y][current.x] + 1;  // Each move has a cost of 1
            if (tentativeGCost < gCost[neighbor.y][neighbor.x]) {
                gCost[neighbor.y][neighbor.x] = tentativeGCost;
                float hCost = manhattanHeuristic(neighbor, end);
                pq.push({ neighbor.x, neighbor.y, tentativeGCost, hCost, tentativeGCost + hCost });
                prev[neighbor.y][neighbor.x] = currentPos;
            }
        }

        // Mark the current cell as visited
        grid[current.y][current.x] = 2;  // Visited cell
        drawGrid(window, grid, start, end);  // Draw grid with updated cells
        window.display();  // Display the grid
        sf::sleep(sf::milliseconds(20));  // Pause for visualization effect
    }

    // Trace back the path
    sf::Vector2i trace = end;
    while (trace != start && trace != sf::Vector2i(-1, -1)) {
        grid[trace.y][trace.x] = 3;  // Mark as part of the path
        trace = prev[trace.y][trace.x];
    }

    // Final visualization with path marked
    drawGrid(window, grid, start, end);
    window.display();
}

int main() {
    sf::RenderWindow window(sf::VideoMode(GRID_SIZE * CELL_SIZE, GRID_SIZE * CELL_SIZE), "A* Visualization");

    // Seed the random number generator
    std::srand(static_cast<unsigned>(std::time(0)));

    // Initialize the grid
    std::vector<std::vector<int>> grid(GRID_SIZE, std::vector<int>(GRID_SIZE, 0));
    for (int i = 0; i < GRID_SIZE; ++i) {
        for (int j = 0; j < GRID_SIZE; ++j) {
            if (rand() % 4 == 0) grid[i][j] = 1;  // Random walls
        }
    }

    sf::Vector2i start(-1, -1), end(-1, -1);

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) window.close();

            if (event.type == sf::Event::MouseButtonPressed) {
                int x = event.mouseButton.x / CELL_SIZE;
                int y = event.mouseButton.y / CELL_SIZE;

                if (start == sf::Vector2i(-1, -1)) {
                    start = { x, y };
                }
                else if (end == sf::Vector2i(-1, -1)) {
                    end = { x, y };
                    visualizeAStar(window, grid, start, end);  // Start visualization when both start and end are selected
                }
                else {
                    // Reset the grid for a new run
                    grid.assign(GRID_SIZE, std::vector<int>(GRID_SIZE, 0));
                    for (int i = 0; i < GRID_SIZE; ++i) {
                        for (int j = 0; j < GRID_SIZE; ++j) {
                            if (rand() % 4 == 0) grid[i][j] = 1;
                        }
                    }
                    start = { x, y };
                    end = { -1, -1 };
                }
            }
        }

        window.clear();
        drawGrid(window, grid, start, end);
        window.display();
    }

    return 0;
}

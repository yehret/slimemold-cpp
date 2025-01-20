#include <SFML/Graphics.hpp>
#include <queue>
#include <vector>
#include <iostream>
#include <cmath>
#include <ctime>
#include <thread>
#include <mutex>

const int GRID_SIZE = 50;
const int CELL_SIZE = 20;
const int WALL_CHANCE = 4;  // Chance of a wall appearing (1 in 4)

struct Node {
    int x, y;
    float gCost, hCost, fCost;
    bool operator>(const Node& other) const {
        return fCost > other.fCost;
    }
};

float manhattanHeuristic(const sf::Vector2i& a, const sf::Vector2i& b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

void drawGrid(sf::RenderWindow& window, const std::vector<std::vector<int>>& grid, const sf::Vector2i& start, const std::vector<sf::Vector2i>& foodSources, const std::vector<std::vector<int>>& pathGrid) {
    for (int i = 0; i < GRID_SIZE; ++i) {
        for (int j = 0; j < GRID_SIZE; ++j) {
            sf::RectangleShape cell(sf::Vector2f(CELL_SIZE - 1, CELL_SIZE - 1));
            cell.setPosition(j * CELL_SIZE, i * CELL_SIZE);

            // Coloring cells based on state
            if (grid[i][j] == 1) {
                cell.setFillColor(sf::Color::Black);  // Wall
            }
            else if (sf::Vector2i(j, i) == start) {
                cell.setFillColor(sf::Color::Green);  // Start
            }
            else {
                bool isFood = false;
                for (const auto& food : foodSources) {
                    if (sf::Vector2i(j, i) == food) {
                        isFood = true;
                        break;
                    }
                }
                if (isFood) {
                    cell.setFillColor(sf::Color::Yellow);  // Food
                }
                else if (pathGrid[i][j] == 2) {
                    cell.setFillColor(sf::Color(200, 200, 0));  // Visited
                }
                else if (pathGrid[i][j] == 3) {
                    cell.setFillColor(sf::Color::Blue);  // Path
                }
                else {
                    cell.setFillColor(sf::Color::White);  // Empty
                }
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

void visualizeAStarAsync(const std::vector<std::vector<int>>& grid, const sf::Vector2i& start, const sf::Vector2i& end, std::vector<std::vector<int>>& pathGrid, std::mutex& gridMutex) {
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
            float tentativeGCost = gCost[current.y][current.x] + 1;
            if (tentativeGCost < gCost[neighbor.y][neighbor.x]) {
                gCost[neighbor.y][neighbor.x] = tentativeGCost;
                float hCost = manhattanHeuristic(neighbor, end);
                pq.push({ neighbor.x, neighbor.y, tentativeGCost, hCost, tentativeGCost + hCost });
                prev[neighbor.y][neighbor.x] = currentPos;
            }
        }

        // Mark the current cell as visited
        {
            std::lock_guard<std::mutex> lock(gridMutex);
            pathGrid[current.y][current.x] = 2;  // Visited cell
        }

        // Add a delay to create animation effect
        sf::sleep(sf::milliseconds(50));  // You can adjust the sleep duration for slower/faster animation

        // Refresh the window during the animation
        {
            std::lock_guard<std::mutex> lock(gridMutex);
        }
    }

    // Trace back the path
    sf::Vector2i trace = end;
    while (trace != start && trace != sf::Vector2i(-1, -1)) {
        pathGrid[trace.y][trace.x] = 3;  // Mark as part of the path
        trace = prev[trace.y][trace.x];
    }

    // Lock the grid to safely update shared data
    std::lock_guard<std::mutex> lock(gridMutex);
}

int main() {
    sf::RenderWindow window(sf::VideoMode(GRID_SIZE * CELL_SIZE, GRID_SIZE * CELL_SIZE + 60), "A* Visualization");

    std::srand(static_cast<unsigned>(std::time(0)));
    std::vector<std::vector<int>> grid(GRID_SIZE, std::vector<int>(GRID_SIZE, 0));
    for (int i = 0; i < GRID_SIZE; ++i) {
        for (int j = 0; j < GRID_SIZE; ++j) {
            if (rand() % WALL_CHANCE == 0) grid[i][j] = 1;
        }
    }

    sf::Vector2i start(-1, -1);
    std::vector<sf::Vector2i> foodSources;
    sf::Font font;
    if (!font.loadFromFile("Roboto.ttf")) {
        std::cerr << "Error loading font\n";
        return 1;
    }

    sf::RectangleShape startButton(sf::Vector2f(100, 40));
    startButton.setPosition(10, GRID_SIZE * CELL_SIZE + 10);
    startButton.setFillColor(sf::Color::Blue);

    sf::RectangleShape resetButton(sf::Vector2f(100, 40));
    resetButton.setPosition(120, GRID_SIZE * CELL_SIZE + 10);
    resetButton.setFillColor(sf::Color::White);

    sf::Text startText("Start", font, 20);
    startText.setPosition(35, GRID_SIZE * CELL_SIZE + 15);
    startText.setFillColor(sf::Color::Black);

    sf::Text resetText("Reset", font, 20);
    resetText.setPosition(145, GRID_SIZE * CELL_SIZE + 15);
    resetText.setFillColor(sf::Color::Black);

    bool isVisualizing = false;
    std::mutex gridMutex;
    std::vector<std::vector<int>> pathGrid(GRID_SIZE, std::vector<int>(GRID_SIZE, 0));

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
            if (event.type == sf::Event::MouseButtonPressed && !isVisualizing) {
                int x = event.mouseButton.x / CELL_SIZE;
                int y = event.mouseButton.y / CELL_SIZE;

                if (startButton.getGlobalBounds().contains(event.mouseButton.x, event.mouseButton.y)) {
                    isVisualizing = true;
                    std::vector<std::thread> threads;

                    // For each food source, initiate A* search asynchronously
                    for (const auto& food : foodSources) {
                        threads.push_back(std::thread(visualizeAStarAsync, std::ref(grid), std::ref(start), food, std::ref(pathGrid), std::ref(gridMutex)));
                    }

                    for (auto& t : threads) {
                        t.join();
                    }

                    isVisualizing = false;
                }
                else if (resetButton.getGlobalBounds().contains(event.mouseButton.x, event.mouseButton.y)) {
                    start = sf::Vector2i(-1, -1);
                    foodSources.clear();
                    grid = std::vector<std::vector<int>>(GRID_SIZE, std::vector<int>(GRID_SIZE, 0));
                    for (int i = 0; i < GRID_SIZE; ++i) {
                        for (int j = 0; j < GRID_SIZE; ++j) {
                            if (rand() % WALL_CHANCE == 0) grid[i][j] = 1;
                        }
                    }
                }
                else if (start == sf::Vector2i(-1, -1)) {
                    start = sf::Vector2i(x, y);  // Set start point
                }
                else {
                    foodSources.push_back(sf::Vector2i(x, y));  // Add food source
                }
            }
        }

        window.clear();
        drawGrid(window, grid, start, foodSources, pathGrid);

        window.draw(startButton);
        window.draw(resetButton);
        window.draw(startText);
        window.draw(resetText);

        window.display();
    }

    return 0;
}

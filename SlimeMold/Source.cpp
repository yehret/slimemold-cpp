#include <SFML/Graphics.hpp>
#include <queue>
#include <vector>
#include <iostream>
#include <cmath>
#include <set>
#include <thread>
#include <atomic>
#include <chrono>

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

void drawGrid(sf::RenderWindow& window, const std::vector<std::vector<int>>& grid, const sf::Vector2i& start, const std::vector<sf::Vector2i>& foodSources) {
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
            else if (std::find(foodSources.begin(), foodSources.end(), sf::Vector2i(j, i)) != foodSources.end()) {
                cell.setFillColor(sf::Color::Red);    // Food sources
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

void drawButtons(sf::RenderWindow& window, sf::Font& font, sf::RectangleShape& startButton, sf::RectangleShape& resetButton) {
    sf::Text startText("Start", font, 20);
    startText.setFillColor(sf::Color::White);
    startText.setPosition(startButton.getPosition().x + 10, startButton.getPosition().y + 10);
    window.draw(startButton);
    window.draw(startText);

    sf::Text resetText("Reset", font, 20);
    resetText.setFillColor(sf::Color::Black);
    resetText.setPosition(resetButton.getPosition().x + 10, resetButton.getPosition().y + 10);
    window.draw(resetButton);
    window.draw(resetText);
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

// Pathfinding algorithm
void visualizeAStar(std::vector<std::vector<int>>& grid, const sf::Vector2i& start, const sf::Vector2i& foodSource, std::vector<sf::Vector2i>& finalPath) {
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
    std::vector<std::vector<float>> gCost(GRID_SIZE, std::vector<float>(GRID_SIZE, INFINITY));
    std::vector<std::vector<sf::Vector2i>> prev(GRID_SIZE, std::vector<sf::Vector2i>(GRID_SIZE, { -1, -1 }));
    gCost[start.y][start.x] = 0;

    pq.push({ start.x, start.y, 0, manhattanHeuristic(start, foodSource), manhattanHeuristic(start, foodSource) });

    bool pathFound = false;

    while (!pq.empty()) {
        Node current = pq.top();
        pq.pop();

        sf::Vector2i currentPos(current.x, current.y);
        if (currentPos == foodSource) {
            pathFound = true;
            break;
        }

        for (auto& neighbor : getNeighbors(currentPos, grid)) {
            float tentativeGCost = gCost[current.y][current.x] + 1;  // Each move has a cost of 1
            if (tentativeGCost < gCost[neighbor.y][neighbor.x]) {
                gCost[neighbor.y][neighbor.x] = tentativeGCost;
                float hCost = manhattanHeuristic(neighbor, foodSource);
                pq.push({ neighbor.x, neighbor.y, tentativeGCost, hCost, tentativeGCost + hCost });
                prev[neighbor.y][neighbor.x] = currentPos;
            }
        }

        // Mark the current cell as visited
        grid[current.y][current.x] = 2;  // Visited cell
        std::this_thread::sleep_for(std::chrono::milliseconds(20));  // Sleep for animation effect
    }

    if (!pathFound) {
        std::cerr << "No valid path found to food source at: (" << foodSource.x << ", " << foodSource.y << ")\n";
        return;
    }

    // Trace back the path with animation
    sf::Vector2i trace = foodSource;
    while (trace != start) {
        if (trace.x < 0 || trace.y < 0 || trace.x >= GRID_SIZE || trace.y >= GRID_SIZE) {
            std::cerr << "Error: trace out of bounds at (" << trace.x << ", " << trace.y << ")\n";
            break;
        }
        finalPath.push_back(trace);
        trace = prev[trace.y][trace.x];
    }
}


int main() {
    sf::RenderWindow window(sf::VideoMode(GRID_SIZE * CELL_SIZE, GRID_SIZE * CELL_SIZE + 60), "Slime Mold by A* Visualization");

    std::srand(static_cast<unsigned>(std::time(0)));

    // Creating walls on the grid
    std::vector<std::vector<int>> grid(GRID_SIZE, std::vector<int>(GRID_SIZE, 0));
    for (int i = 0; i < GRID_SIZE; ++i) {
        for (int j = 0; j < GRID_SIZE; ++j) {
            if (rand() % 4 == 0) grid[i][j] = 1;
        }
    }

    sf::Vector2i start(-1, -1);
    std::vector<sf::Vector2i> foodSources;

    // Loading font for the text components
    sf::Font font;
    if (!font.loadFromFile("Roboto.ttf")) {
        std::cerr << "Error loading font\n";
        return 1;
    }

    // Drawing start and reset buttons in application
    sf::RectangleShape startButton(sf::Vector2f(100, 40));
    startButton.setPosition(10, GRID_SIZE * CELL_SIZE + 10);
    startButton.setFillColor(sf::Color::Blue);

    sf::RectangleShape resetButton(sf::Vector2f(100, 40));
    resetButton.setPosition(120, GRID_SIZE * CELL_SIZE + 10);
    resetButton.setFillColor(sf::Color::White);

    bool isVisualizing = false;
    std::atomic<bool> pathfindingComplete(false);

    std::vector<std::vector<sf::Vector2i>> allPaths;

    // Main loop
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) window.close();

            if (event.type == sf::Event::MouseButtonPressed) {
                int x = event.mouseButton.x / CELL_SIZE;
                int y = event.mouseButton.y / CELL_SIZE;

                // Add start point and food sources
                if (y < GRID_SIZE && !isVisualizing) {
                    if (start == sf::Vector2i(-1, -1)) {
                        start = { x, y };
                    }
                    else {
                        foodSources.push_back({ x, y });
                    }
                }

                if (startButton.getGlobalBounds().contains(event.mouseButton.x, event.mouseButton.y)) {
                    if (start != sf::Vector2i(-1, -1) && !foodSources.empty()) {
                        isVisualizing = true;

                        std::thread([&]() {
                            for (const auto& foodSource : foodSources) {
                                std::vector<sf::Vector2i> path;
                                visualizeAStar(grid, start, foodSource, path);
                                allPaths.push_back(path);

                                // Reset visited cells for the next pathfinding process
                                for (int i = 0; i < GRID_SIZE; ++i) {
                                    for (int j = 0; j < GRID_SIZE; ++j) {
                                        if (grid[i][j] == 2) {
                                            grid[i][j] = 0;
                                        }
                                    }
                                }
                            }
                            pathfindingComplete = true;
                            }).detach();

                    }
                }
                else if (resetButton.getGlobalBounds().contains(event.mouseButton.x, event.mouseButton.y)) {
                    grid = std::vector<std::vector<int>>(GRID_SIZE, std::vector<int>(GRID_SIZE, 0));
                    for (int i = 0; i < GRID_SIZE; ++i) {
                        for (int j = 0; j < GRID_SIZE; ++j) {
                            if (rand() % 4 == 0) grid[i][j] = 1;
                        }
                    }
                    start = { -1, -1 };
                    foodSources.clear();
                    allPaths.clear();
                    pathfindingComplete = false;
                }
            }
        }

        window.clear();
        drawGrid(window, grid, start, foodSources);
        drawButtons(window, font, startButton, resetButton);

        if (pathfindingComplete) {
            for (int step = 0; step < GRID_SIZE * GRID_SIZE; ++step) {
                bool anyUpdate = false;

                for (const auto& path : allPaths) {
                    if (step < path.size()) {
                        sf::Vector2i pos = path[path.size() - 1 - step];
                        grid[pos.y][pos.x] = 3;  // Mark as part of the path
                        anyUpdate = true;
                    }
                }

                if (!anyUpdate) break;

                window.clear();
                drawGrid(window, grid, start, foodSources);
                drawButtons(window, font, startButton, resetButton);
                window.display();
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            isVisualizing = false;
        }

        window.display();
    }

    return 0;
}

#include <SFML/Graphics.hpp>
#include <queue>
#include <vector>
#include <iostream>
#include <cmath>
#include <set>
#include <thread>
#include <atomic>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <random>

const int GRID_SIZE = 70;
const int CELL_SIZE = 15;
const int RADIUS = 20;  // Radius of the circle for the border

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

std::vector<std::vector<sf::Color>> cellColors(GRID_SIZE, std::vector<sf::Color>(GRID_SIZE));

void initializeGrid(std::vector<std::vector<int>>& grid) {
    // Get the center of the grid
    int centerX = GRID_SIZE / 2;
    int centerY = GRID_SIZE / 2;

    // Set the radius of the circle
    int radius = GRID_SIZE * 0.555;

    // Initialize grid with walls and empty spaces
    for (int i = 0; i < GRID_SIZE; ++i) {
        for (int j = 0; j < GRID_SIZE; ++j) {
            // Check if the current cell is inside the circle using the equation
            int dx = j - centerX;
            int dy = i - centerY;

            if (dx * dx + dy * dy <= radius * radius) {
                // Inside the circle: random chance to place walls
                if (rand() % 100 < 20) {  // 20% chance for walls inside the circle
                    grid[i][j] = 1;  // Wall
                }
                else {
                    grid[i][j] = 0;  // Open space
                }
            }
            else {
                // Outside the circle: make walls
                grid[i][j] = 1;  // Wall outside the circle
            }
        }
    }

    // Initialize colors for empty cells (those not walls)
    for (int i = 0; i < GRID_SIZE; ++i) {
        for (int j = 0; j < GRID_SIZE; ++j) {
            if (grid[i][j] == 0) {
                // Random grayish color for empty spaces
                int grayValue = rand() % 20 + 220;
                cellColors[i][j] = sf::Color(grayValue, grayValue, grayValue - 20);
            }
        }
    }
}

void drawGrid(sf::RenderWindow& window, const std::vector<std::vector<int>>& grid, const sf::Vector2i& start, const std::vector<sf::Vector2i>& foodSources) {
    for (int i = 0; i < GRID_SIZE; ++i) {
        for (int j = 0; j < GRID_SIZE; ++j) {
            sf::RectangleShape cell(sf::Vector2f(CELL_SIZE - 0.5, CELL_SIZE - 1));
            cell.setPosition(j * CELL_SIZE, i * CELL_SIZE);

            // Draw based on the type of cell
            if (grid[i][j] == 1) {
                cell.setFillColor(sf::Color::Black);  // Wall
            }
            else if (sf::Vector2i(j, i) == start) {
                cell.setFillColor(sf::Color(255, 162, 0));  // Start point
            }
            else if (std::find(foodSources.begin(), foodSources.end(), sf::Vector2i(j, i)) != foodSources.end()) {
                cell.setFillColor(sf::Color(250, 70, 20));  // Food sources
            }
            else if (grid[i][j] == 2) {
                int randValue = rand() % 10 + 200;
                cell.setFillColor(sf::Color(randValue, randValue, 0)); // Visited cells
            }
            else if (grid[i][j] == 3) {
                int randValue = rand() % 10 + 245;
                cell.setFillColor(sf::Color(randValue, randValue, 0));  // Path
            }
            else {
                // Precomputed colors for empty spaces
                cell.setFillColor(cellColors[i][j]);
            }

            window.draw(cell);
        }
    }
}


void drawButtons(sf::RenderWindow& window, sf::Font& font, sf::RectangleShape& startButton, sf::RectangleShape& resetButton) {
    sf::Text startText("START", font, 20);
    startText.setFillColor(sf::Color::White);
    startText.setPosition(startButton.getPosition().x + 20, startButton.getPosition().y + 8);
    window.draw(startButton);
    window.draw(startText);

    sf::Text resetText("RESET", font, 20);
    resetText.setFillColor(sf::Color::Black);
    resetText.setPosition(resetButton.getPosition().x + 20, resetButton.getPosition().y + 8);
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

// Mutex and condition variable for synchronization
std::mutex gridMutex;
std::condition_variable cv;
std::atomic<int> activeThreads(0);
std::atomic<int> currentStep(0);
int totalSteps = 0;

// Global variables to track state of the pathfinding process
std::atomic<bool> pathfindingComplete(false);
std::vector<std::vector<sf::Vector2i>> allPaths;

// Synchronized A* visualization
void visualizeAStar(std::vector<std::vector<int>>& grid, const sf::Vector2i& start, const sf::Vector2i& foodSource, std::vector<sf::Vector2i>& finalPath) {
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
    std::vector<std::vector<float>> gCost(GRID_SIZE, std::vector<float>(GRID_SIZE, INFINITY));
    std::vector<std::vector<sf::Vector2i>> prev(GRID_SIZE, std::vector<sf::Vector2i>(GRID_SIZE, { -1, -1 }));
    gCost[start.y][start.x] = 0;

    pq.push({ start.x, start.y, 0, manhattanHeuristic(start, foodSource), manhattanHeuristic(start, foodSource) });

    bool pathFound = false;

    // Create a random number generator and a uniform distribution
    std::random_device rd;
    std::mt19937 gen(rd());

    while (!pq.empty()) {
        Node current = pq.top();
        pq.pop();

        sf::Vector2i currentPos(current.x, current.y);
        if (currentPos == foodSource) {
            pathFound = true;
            break;
        }

        // Get neighbors
        std::vector<sf::Vector2i> neighbors = getNeighbors(currentPos, grid);

        // Apply randomness: occasionally pick a random neighbor instead of the best one
        if (rand() % 100 < 20) { // 20% chance to pick a random neighbor
            // Shuffle the neighbors randomly
            std::shuffle(neighbors.begin(), neighbors.end(), gen);
        }

        // Add the neighbors to the priority queue with some randomness in heuristic
        for (auto& neighbor : neighbors) {
            float baseHeuristic = manhattanHeuristic(neighbor, foodSource);

            // Randomly bias the heuristic to make the path less straight
            float randomHeuristic = baseHeuristic + (rand() % 20 - 10);  // Randomly modify the heuristic by -10 to +10

            float tentativeGCost = gCost[current.y][current.x] + 1;  // Each move has a cost of 1
            if (tentativeGCost < gCost[neighbor.y][neighbor.x]) {
                gCost[neighbor.y][neighbor.x] = tentativeGCost;
                pq.push({ neighbor.x, neighbor.y, tentativeGCost, randomHeuristic, tentativeGCost + randomHeuristic });
                prev[neighbor.y][neighbor.x] = currentPos;
            }
        }

        // Synchronize visited cell marking
        {
            std::unique_lock<std::mutex> lock(gridMutex);
            grid[current.y][current.x] = 2;  // Mark as visited
            ++totalSteps;
        }

        // Notify all threads and wait for synchronization
        cv.notify_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(40));  // Sleep for animation effect
    }

    if (!pathFound) {
        std::cerr << "No valid path found to food source at: (" << foodSource.x << ", " << foodSource.y << ")\n";
        return;
    }

    // Trace back the path
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

void generateCircularWalls(std::vector<std::vector<int>>& grid, const sf::Vector2i& center, int radius) {
    for (int i = 0; i < GRID_SIZE; ++i) {
        for (int j = 0; j < GRID_SIZE; ++j) {
            // Calculate the distance from the center
            int dx = i - center.y;
            int dy = j - center.x;
            if (std::sqrt(dx * dx + dy * dy) <= radius) {
                grid[i][j] = 1;  // Wall inside the circle
            }
        }
    }
}

// This will be called once when the Start button is clicked
void startPathfinding(std::vector<std::vector<int>>& grid, const sf::Vector2i& start, const std::vector<sf::Vector2i>& foodSources) {
    std::vector<std::thread> threads;
    allPaths.clear();  // Clear previous paths

    // Start a thread for each food source
    for (const auto& foodSource : foodSources) {
        threads.push_back(std::thread([&, foodSource]() {
            std::vector<sf::Vector2i> path;
            visualizeAStar(grid, start, foodSource, path);
            allPaths.push_back(path);
            }));
    }

    // Wait for all threads to complete
    for (auto& t : threads) {
        t.join();
    }

    pathfindingComplete = true;
    std::cout << "Paths have been found\n";
}

int main() {
    sf::RenderWindow window(sf::VideoMode(GRID_SIZE * CELL_SIZE, GRID_SIZE * CELL_SIZE + 60), "Slime Mold by A* Visualization");

    std::srand(static_cast<unsigned>(std::time(0)));

    // Create the grid and set walls and empty spaces
    std::vector<std::vector<int>> grid(GRID_SIZE, std::vector<int>(GRID_SIZE, 0));
    initializeGrid(grid);  // Call the new function to initialize grid and walls

    sf::Vector2i start(-1, -1);
    std::vector<sf::Vector2i> foodSources;

    // Loading font for the text components
    sf::Font font;
    if (!font.loadFromFile("Roboto.ttf")) {
        std::cerr << "Error loading font\n";
        return 1;
    }

    // Drawing start and reset buttons in the application
    sf::RectangleShape startButton(sf::Vector2f(100, 40));
    startButton.setPosition(10, GRID_SIZE * CELL_SIZE + 10);
    startButton.setFillColor(sf::Color(76, 158, 0));

    sf::RectangleShape resetButton(sf::Vector2f(100, 40));
    resetButton.setPosition(120, GRID_SIZE * CELL_SIZE + 10);
    resetButton.setFillColor(sf::Color::White);

    bool isVisualizing = false;

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
                        std::cout << "Start has been placed at: (" << x << ", " << y << ")\n";
                    }
                    else {
                        foodSources.push_back({ x, y });
                        std::cout << "Food source has been placed at: (" << x << ", " << y << ")\n";
                    }
                }

                if (startButton.getGlobalBounds().contains(event.mouseButton.x, event.mouseButton.y)) {
                    std::cout << "Start button has been pressed. Starting!\n";
                    if (start != sf::Vector2i(-1, -1) && !foodSources.empty()) {
                        isVisualizing = true;

                        // Start the pathfinding process in a separate thread
                        std::thread(&startPathfinding, std::ref(grid), start, std::ref(foodSources)).detach();
                    }
                }
                else if (resetButton.getGlobalBounds().contains(event.mouseButton.x, event.mouseButton.y) && !isVisualizing) {
                    std::cout << "Resetting the grid!\n";

                    grid = std::vector<std::vector<int>>(GRID_SIZE, std::vector<int>(GRID_SIZE, 0));
                    initializeGrid(grid);  // Reinitialize the grid and walls
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
            // Animate the paths for all food sources simultaneously
            std::cout << "Starting to draw paths\n";
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
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }

            isVisualizing = false;
            pathfindingComplete = false;
            std::cout << "Finish!\n";
        }

        window.display();
    }

    return 0;
}

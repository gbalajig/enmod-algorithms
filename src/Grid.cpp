#include "enmod/Grid.h"
#include <sstream>
#include <algorithm>

Grid::Grid(const json& config) : original_config(config) {
    name = config.value("name", "Unnamed Grid");
    rows = config.value("rows", 0);
    cols = config.value("cols", 0);

    // Resize map
    if (rows > 0 && cols > 0) {
        grid_map.resize(rows, std::vector<CellType>(cols, CellType::EMPTY));
    }

    // Start Position
    if (config.contains("start")) {
        start_pos = {config["start"]["row"], config["start"]["col"]};
    }

    // Exit Positions
    if (config.contains("exits")) {
        for (const auto& ex : config["exits"]) {
            Position p = {ex["row"], ex["col"]};
            exit_pos.push_back(p);
            if(isValid(p.row, p.col)) grid_map[p.row][p.col] = CellType::EXIT;
        }
    }

    // Walls
    if (config.contains("walls")) {
        for (const auto& w : config["walls"]) {
            int r = w["row"];
            int c = w["col"];
            if(isValid(r, c)) grid_map[r][c] = CellType::WALL;
        }
    }

    // Dynamic Events
    if (config.contains("dynamic_events")) {
        for (const auto& event : config["dynamic_events"]) {
            if (event.value("time_step", 0) == 0) {
                addHazard(event);
            }
        }
    }
}

// Accessors
int Grid::getRows() const { return rows; }
int Grid::getCols() const { return cols; }
Position Grid::getStartPosition() const { return start_pos; }
const std::vector<Position>& Grid::getExitPositions() const { return exit_pos; }
const json& Grid::getConfig() const { return original_config; }
std::string Grid::getName() const { return name; }

// Logic
bool Grid::isValid(int r, int c) const {
    return r >= 0 && r < rows && c >= 0 && c < cols;
}

bool Grid::isWalkable(int r, int c) const {
    if (!isValid(r, c)) return false;
    return grid_map[r][c] != CellType::WALL && grid_map[r][c] != CellType::FIRE;
}

bool Grid::isExit(int r, int c) const {
    if (!isValid(r, c)) return false;
    return grid_map[r][c] == CellType::EXIT;
}

CellType Grid::getCellType(const Position& pos) const {
    if (!isValid(pos.row, pos.col)) return CellType::WALL;
    return grid_map[pos.row][pos.col];
}

std::string Grid::getSmokeIntensity(const Position& pos) const {
    auto it = smoke_intensities.find(pos);
    if (it != smoke_intensities.end()) return it->second;
    return "none";
}

Cost Grid::getMoveCost(const Position& pos) const {
    int time_cost = 1;
    int smoke_cost = 0;
    
    std::string smoke = getSmokeIntensity(pos);
    if (smoke == "low") smoke_cost = 10;
    else if (smoke == "medium") smoke_cost = 50;
    else if (smoke == "heavy") smoke_cost = 100;

    return {1, time_cost, smoke_cost};
}

Position Grid::getNextPosition(const Position& current, Direction dir) const {
    int dr = 0, dc = 0;
    if (dir == Direction::UP) dr = -1;
    else if (dir == Direction::DOWN) dr = 1;
    else if (dir == Direction::LEFT) dc = -1;
    else if (dir == Direction::RIGHT) dc = 1;
    return {current.row + dr, current.col + dc};
}

void Grid::addHazard(const json& event) {
    std::string type = event.value("type", "");
    if (!event.contains("position")) return;
    
    Position p = {event["position"]["row"], event["position"]["col"]};
    if (!isValid(p.row, p.col)) return;

    if (type == "fire") {
        grid_map[p.row][p.col] = CellType::FIRE;
    } else if (type == "smoke") {
        smoke_intensities[p] = event.value("intensity", "low");
        if (grid_map[p.row][p.col] == CellType::EMPTY) grid_map[p.row][p.col] = CellType::SMOKE;
    }
}

void Grid::setCellUnwalkable(const Position& pos) {
    if (isValid(pos.row, pos.col)) grid_map[pos.row][pos.col] = CellType::WALL;
}

void Grid::updateFromSensors(const std::vector<std::pair<Position, std::string>>& sensor_data) {
    for (const auto& reading : sensor_data) {
        Position p = reading.first;
        std::string val = reading.second;
        if (!isValid(p.row, p.col)) continue;

        if (val == "fire") {
            grid_map[p.row][p.col] = CellType::FIRE;
        } else if (val == "smoke") {
            smoke_intensities[p] = "medium"; // Default for generic sensor
            if (grid_map[p.row][p.col] == CellType::EMPTY) grid_map[p.row][p.col] = CellType::SMOKE;
        }
    }
}

// Visualization
std::string Grid::toHtmlString() const {
    return toHtmlStringWithAgent({-1, -1});
}

std::string Grid::toHtmlStringWithAgent(const Position& agent_pos) const {
    std::stringstream ss;
    ss << "<table class='grid-table'>\n";
    for (int r = 0; r < rows; ++r) {
        ss << "<tr>\n";
        for (int c = 0; c < cols; ++c) {
            std::string cell_class = "grid-cell";
            std::string content = "";
            
            if (r == agent_pos.row && c == agent_pos.col) {
                cell_class += " start"; 
                content = "A"; 
            } else if (grid_map[r][c] == CellType::WALL) {
                cell_class += " wall";
            } else if (grid_map[r][c] == CellType::EXIT) {
                cell_class += " exit";
                content = "E";
            } else if (grid_map[r][c] == CellType::FIRE) {
                cell_class += " fire";
                content = "F";
            } else if (grid_map[r][c] == CellType::SMOKE) {
                cell_class += " smoke";
                content = "S";
            }
            ss << "<td class='" << cell_class << "'>" << content << "</td>";
        }
        ss << "</tr>\n";
    }
    ss << "</table>\n";
    return ss.str();
}

std::string Grid::toHtmlStringWithPath(const std::vector<Position>& path) const {
    return toHtmlStringWithAgent(path.empty() ? Position{-1,-1} : path[0]);
}

std::string Grid::toHtmlStringWithCost(const std::vector<std::vector<Cost>>& cost_map) const {
    std::stringstream ss;
    ss << "<table class='grid-table'>\n";
    for (int r = 0; r < rows; ++r) {
        ss << "<tr>\n";
        for (int c = 0; c < cols; ++c) {
            std::string cell_class = "grid-cell";
            std::string content = "";
            std::string style = "";

            if (grid_map[r][c] == CellType::WALL) {
                cell_class += " wall";
            } else if (grid_map[r][c] == CellType::EXIT) {
                cell_class += " exit";
                content = "E";
            } else {
                int cost = cost_map[r][c].distance;
                if (cost < MAX_COST) {
                    int intensity = std::min(255, cost * 5); 
                    style = "background-color: rgb(255, " + std::to_string(255 - intensity) + ", " + std::to_string(255 - intensity) + ");";
                    content = std::to_string(cost);
                } else {
                    content = "∞";
                    style = "background-color: #eee; color: #aaa;";
                }
            }
            ss << "<td class='" << cell_class << "' style='" << style << "'>" << content << "</td>";
        }
        ss << "</tr>\n";
    }
    ss << "</table>\n";
    return ss.str();
}

std::string Grid::toHtmlStringWithPolicy(const std::vector<std::vector<Direction>>& policy) const {
    std::stringstream ss;
    ss << "<table class='grid-table'>\n";
    for (int r = 0; r < rows; ++r) {
        ss << "<tr>\n";
        for (int c = 0; c < cols; ++c) {
            std::string cell_class = "grid-cell";
            std::string content = "";
            
            if (grid_map[r][c] == CellType::WALL) {
                cell_class += " wall";
            } else if (grid_map[r][c] == CellType::EXIT) {
                cell_class += " exit";
                content = "E";
            } else {
                Direction d = policy[r][c];
                if (d == Direction::UP) content = "&uarr;";
                else if (d == Direction::DOWN) content = "&darr;";
                else if (d == Direction::LEFT) content = "&larr;";
                else if (d == Direction::RIGHT) content = "&rarr;";
                else content = ".";
            }
            ss << "<td class='" << cell_class << "'>" << content << "</td>";
        }
        ss << "</tr>\n";
    }
    ss << "</table>\n";
    return ss.str();
}
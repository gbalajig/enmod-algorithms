#ifndef ENMOD_GRID_H
#define ENMOD_GRID_H

#include "Types.h"
#include "json.hpp"
#include <vector>
#include <string>
#include <map>
#include <iostream>

using json = nlohmann::json;

class Grid {
public:
    Grid() = default;
    Grid(const json& config);

    int getRows() const;
    int getCols() const;
    Position getStartPosition() const;
    const std::vector<Position>& getExitPositions() const;
    const json& getConfig() const;
    std::string getName() const;

    bool isValid(int r, int c) const;
    bool isWalkable(int r, int c) const;
    bool isExit(int r, int c) const;
    CellType getCellType(const Position& pos) const;
    std::string getSmokeIntensity(const Position& pos) const;
    
    Cost getMoveCost(const Position& pos) const;
    Position getNextPosition(const Position& current, Direction dir) const;

    void addHazard(const json& hazard_config);
    void setCellUnwalkable(const Position& pos);
    
    // [RESTORED] Sensor Update Method
    void updateFromSensors(const std::vector<std::pair<Position, std::string>>& sensor_data);

    // Reporting / Visualization
    std::string toHtmlString() const;
    std::string toHtmlStringWithAgent(const Position& agent_pos) const;
    std::string toHtmlStringWithPath(const std::vector<Position>& path) const;
    std::string toHtmlStringWithCost(const std::vector<std::vector<Cost>>& cost_map) const;
    
    // [RESTORED] Policy Visualization
    std::string toHtmlStringWithPolicy(const std::vector<std::vector<Direction>>& policy) const;

private:
    std::string name;
    int rows = 0;
    int cols = 0;
    std::vector<std::vector<CellType>> grid_map;
    Position start_pos;
    std::vector<Position> exit_pos;
    std::map<Position, std::string> smoke_intensities;
    json original_config;
};

#endif // ENMOD_GRID_H
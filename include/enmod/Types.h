#ifndef ENMOD_TYPES_H
#define ENMOD_TYPES_H

#include <limits>
#include <string>

// --- ENUMS ---
enum class EvacuationMode {
    NORMAL,
    ALERT,
    PANIC
};

enum class CellType {
    EMPTY,
    WALL,
    EXIT,
    FIRE,
    SMOKE
};

enum class Direction {
    UP,
    DOWN,
    LEFT,
    RIGHT,
    STAY
};

// --- BASIC STRUCTS ---
struct Position {
    int row;
    int col;

    bool operator==(const Position& other) const {
        return row == other.row && col == other.col;
    }
    bool operator!=(const Position& other) const {
        return !(*this == other);
    }
    bool operator<(const Position& other) const {
        if (row != other.row) return row < other.row;
        return col < other.col;
    }
};

struct Cost {
    int distance;
    int time;
    int smoke;

    // [RESTORED] Required for Dynamic Solvers to switch logic globally
    static EvacuationMode current_mode;

    Cost() : distance(0), time(0), smoke(0) {}
    Cost(int d, int t, int s) : distance(d), time(t), smoke(s) {}

    // Comparison based on current_mode
    bool operator<(const Cost& other) const {
        if (current_mode == EvacuationMode::PANIC) {
            return distance < other.distance;
        } 
        else if (current_mode == EvacuationMode::ALERT) {
            // Avoid smoke first, then distance
            if (smoke != other.smoke) return smoke < other.smoke;
            return distance < other.distance;
        }
        // NORMAL: Distance first, then smoke
        if (distance != other.distance) return distance < other.distance;
        return smoke < other.smoke;
    }

    bool operator>(const Cost& other) const { return other < *this; }
    bool operator==(const Cost& other) const {
        return distance == other.distance && smoke == other.smoke && time == other.time;
    }
    bool operator!=(const Cost& other) const { return !(*this == other); }

    Cost operator+(const Cost& other) const {
        int d = (distance >= std::numeric_limits<int>::max() || other.distance >= std::numeric_limits<int>::max()) 
                 ? std::numeric_limits<int>::max() 
                 : distance + other.distance;
        return {d, time + other.time, smoke + other.smoke};
    }
};

// Global Constants
const int MAX_COST = std::numeric_limits<int>::max();

#endif // ENMOD_TYPES_H
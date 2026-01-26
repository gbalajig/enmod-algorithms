#include "enmod/ParallelSolverBase.h"
#include "enmod/Logger.h"
#include <iostream>
#include <cuda_runtime.h>

// Error checking macro
#define checkCuda(call, msg) \
    do { \
        cudaError_t err = call; \
        if (err != cudaSuccess) { \
            std::cerr << "CUDA Error [" << msg << "]: " << cudaGetErrorString(err) << std::endl; \
            exit(1); \
        } \
    } while (0)

// --- DEVICE FUNCTIONS (GPU) ---

__device__ const DeviceCost D_MAX_COST = { DEVICE_MAX_INT, DEVICE_MAX_INT, DEVICE_MAX_INT };

__device__ __forceinline__ bool d_cost_less(const DeviceCost& a, const DeviceCost& b, DeviceEvacuationMode mode) {
    if (a.distance == DEVICE_MAX_INT && b.distance == DEVICE_MAX_INT) return false;
    if (a.distance == DEVICE_MAX_INT) return false;
    if (b.distance == DEVICE_MAX_INT) return true;

    // Prioritize based on mode
    switch (mode) {
        case DeviceEvacuationMode::PANIC: // Only Distance
            return a.distance < b.distance;
        case DeviceEvacuationMode::ALERT: // Smoke > Distance
            if (a.smoke != b.smoke) return a.smoke < b.smoke;
            return a.distance < b.distance;
        case DeviceEvacuationMode::NORMAL: // Distance > Smoke
        default:
            if (a.distance != b.distance) return a.distance < b.distance;
            return a.smoke < b.smoke;
    }
}

__device__ __forceinline__ DeviceCost d_cost_add(const DeviceCost& a, const DeviceCost& b) {
    if (a.distance == DEVICE_MAX_INT || b.distance == DEVICE_MAX_INT) return D_MAX_COST;
    return {a.smoke + b.smoke, a.time + b.time, a.distance + b.distance};
}

// --- RELAXATION KERNEL ---

__global__ void relaxation_kernel(const DeviceGridCell* d_grid, const DeviceCost* d_in, DeviceCost* d_out, int* d_changed, int rows, int cols, DeviceEvacuationMode mode) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= rows * cols) return;

    int r = idx / cols;
    int c = idx % cols;

    // Load current cost
    DeviceCost my_cost = d_in[idx];
    DeviceCost best_new_cost = my_cost;

    // If wall, stay MAX
    if (d_grid[idx].type == 1) { 
        d_out[idx] = D_MAX_COST;
        return;
    }

    // Neighbors: Up, Down, Left, Right
    int dr[] = {-1, 1, 0, 0};
    int dc[] = {0, 0, -1, 1};

    for (int i = 0; i < 4; ++i) {
        int nr = r + dr[i];
        int nc = c + dc[i];

        if (nr >= 0 && nr < rows && nc >= 0 && nc < cols) {
            int n_idx = nr * cols + nc;
            
            // Check neighbor's cost
            DeviceCost n_cost = d_in[n_idx];
            
            if (n_cost.distance != DEVICE_MAX_INT) {
                // Calculate path via neighbor
                DeviceCost move_cost = d_grid[idx].move_cost; 
                DeviceCost potential_cost = d_cost_add(n_cost, move_cost);
                
                if (d_cost_less(potential_cost, best_new_cost, mode)) {
                    best_new_cost = potential_cost;
                }
            }
        }
    }

    d_out[idx] = best_new_cost;

    // Check convergence
    if (best_new_cost.distance != my_cost.distance || 
        best_new_cost.smoke != my_cost.smoke) {
        *d_changed = 1;
    }
}

// --- HOST IMPLEMENTATION ---

ParallelSolverBase::ParallelSolverBase(const Grid& grid_ref, const std::string& name)
    : Solver(grid_ref, name) {
    // Initialize cost map on CPU
    cost_map.assign(grid.getRows(), std::vector<Cost>(grid.getCols(), {MAX_COST, MAX_COST, MAX_COST}));
    initialize_gpu_memory();
    upload_grid_data();
}

ParallelSolverBase::~ParallelSolverBase() {
    free_gpu_memory();
}

void ParallelSolverBase::initialize_gpu_memory() {
    size_t n_cells = grid.getRows() * grid.getCols();
    
    checkCuda(cudaMalloc(&d_grid, n_cells * sizeof(DeviceGridCell)), "Malloc d_grid");
    checkCuda(cudaMalloc(&d_cost_in, n_cells * sizeof(DeviceCost)), "Malloc d_cost_in");
    checkCuda(cudaMalloc(&d_cost_out, n_cells * sizeof(DeviceCost)), "Malloc d_cost_out");
    checkCuda(cudaMalloc(&d_changed_flag, sizeof(int)), "Malloc d_changed");
}

void ParallelSolverBase::free_gpu_memory() {
    if (d_grid) cudaFree(d_grid);
    if (d_cost_in) cudaFree(d_cost_in);
    if (d_cost_out) cudaFree(d_cost_out);
    if (d_changed_flag) cudaFree(d_changed_flag);
    d_grid = nullptr;
}

void ParallelSolverBase::upload_grid_data() {
    int rows = grid.getRows();
    int cols = grid.getCols();
    std::vector<DeviceGridCell> h_grid(rows * cols);

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            int idx = r * cols + c;
            Position pos = {r, c};
            
            // Map Cell Type
            int type = 0;
            if (grid.getCellType(pos) == CellType::WALL) type = 1;
            else if (grid.getCellType(pos) == CellType::EXIT) type = 2;
            else if (grid.getCellType(pos) == CellType::FIRE) type = 3;
            else if (grid.getCellType(pos) == CellType::SMOKE) type = 4;

            // Map Move Cost
            Cost mc = grid.getMoveCost(pos);
            h_grid[idx] = { type, {mc.smoke, mc.time, mc.distance} };
        }
    }
    checkCuda(cudaMemcpy(d_grid, h_grid.data(), h_grid.size() * sizeof(DeviceGridCell), cudaMemcpyHostToDevice), "Copy Grid");
}

void ParallelSolverBase::reset_cost_map(const std::vector<Position>& source_nodes) {
    int rows = grid.getRows();
    int cols = grid.getCols();
    
    // Fill with MAX
    std::vector<DeviceCost> h_costs(rows * cols, {DEVICE_MAX_INT, DEVICE_MAX_INT, DEVICE_MAX_INT});

    // Set Sources to 0
    for (const auto& src : source_nodes) {
        int idx = src.row * cols + src.col;
        if (idx < rows * cols) {
            h_costs[idx] = {0, 0, 0};
        }
    }

    checkCuda(cudaMemcpy(d_cost_in, h_costs.data(), h_costs.size() * sizeof(DeviceCost), cudaMemcpyHostToDevice), "Reset Costs");
}

void ParallelSolverBase::run_cuda_algo(const std::vector<Position>& source_nodes) {
    reset_cost_map(source_nodes);

    int rows = grid.getRows();
    int cols = grid.getCols();
    int n_cells = rows * cols;
    
    int blockSize = 256;
    int numBlocks = (n_cells + blockSize - 1) / blockSize;

    // Convert mode
    DeviceEvacuationMode mode = DeviceEvacuationMode::NORMAL;
    // (If you have a global mode variable, map it here. Defaulting to NORMAL for now)
    
    int changed_h = 1;
    int iterations = 0;

    // Bellman-Ford Iterations
    while (changed_h && iterations < n_cells) {
        changed_h = 0;
        checkCuda(cudaMemcpy(d_changed_flag, &changed_h, sizeof(int), cudaMemcpyHostToDevice), "Reset Flag");

        relaxation_kernel<<<numBlocks, blockSize>>>(d_grid, d_cost_in, d_cost_out, d_changed_flag, rows, cols, mode);
        cudaDeviceSynchronize();

        // Swap pointers (Ping-Pong)
        std::swap(d_cost_in, d_cost_out);

        checkCuda(cudaMemcpy(&changed_h, d_changed_flag, sizeof(int), cudaMemcpyDeviceToHost), "Read Flag");
        iterations++;
    }
    
    // Ensure final result is in d_cost_in (because we swapped)
    download_results();
}

void ParallelSolverBase::download_results() {
    int rows = grid.getRows();
    int cols = grid.getCols();
    std::vector<DeviceCost> h_costs(rows * cols);

    checkCuda(cudaMemcpy(h_costs.data(), d_cost_in, h_costs.size() * sizeof(DeviceCost), cudaMemcpyDeviceToHost), "Download Results");

    // Convert back to Host Cost format
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            DeviceCost dc = h_costs[r * cols + c];
            if (dc.distance >= DEVICE_MAX_INT) {
                cost_map[r][c] = {MAX_COST, MAX_COST, MAX_COST};
            } else {
                cost_map[r][c] = {dc.distance, dc.time, dc.smoke};
            }
        }
    }
}

const std::vector<std::vector<Cost>>& ParallelSolverBase::getCostMap() const {
    return cost_map;
}

Cost ParallelSolverBase::getEvacuationCost() const {
    // Return cost of start position
    Position start = grid.getStartPosition();
    if (grid.isValid(start.row, start.col)) {
        return cost_map[start.row][start.col];
    }
    return {MAX_COST, MAX_COST, MAX_COST};
}

void ParallelSolverBase::generateReport(std::ofstream& report_file) const {
    report_file << "<h3>GPU Cost Map Visualization</h3>";
    report_file << grid.toHtmlStringWithCost(cost_map);
}
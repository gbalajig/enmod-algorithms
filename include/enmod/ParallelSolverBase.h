#ifndef ENMOD_PARALLEL_SOLVER_BASE_H
#define ENMOD_PARALLEL_SOLVER_BASE_H

#include "Solver.h"
#include <vector>
#include <cuda_runtime.h>

// --- CUDA CONSTANTS & STRUCTS ---
#define DEVICE_MAX_INT 2147483647

// POD (Plain Old Data) struct for GPU Cost
struct DeviceCost {
    int smoke;
    int time;
    int distance;
};

// POD struct for GPU Grid Cells
struct DeviceGridCell {
    int type; // 0:Empty, 1:Wall, 2:Exit, 3:Fire, 4:Smoke
    DeviceCost move_cost;
};

// CUDA-compatible Enum
enum class DeviceEvacuationMode {
    NORMAL,
    ALERT,
    PANIC
};

class ParallelSolverBase : public Solver {
public:
    ParallelSolverBase(const Grid& grid_ref, const std::string& name);
    virtual ~ParallelSolverBase();

    // Pure virtual run
    virtual void run() override = 0; 

    // Accessors
    const std::vector<std::vector<Cost>>& getCostMap() const;
    Cost getEvacuationCost() const override;
    void generateReport(std::ofstream& report_file) const override;

protected:
    std::vector<std::vector<Cost>> cost_map;

    // --- GPU Memory Pointers ---
    DeviceGridCell* d_grid = nullptr;
    DeviceCost* d_cost_in = nullptr;
    DeviceCost* d_cost_out = nullptr;
    int* d_changed_flag = nullptr;

    // --- GPU Management Methods ---
    void initialize_gpu_memory();
    void upload_grid_data();
    void reset_cost_map(const std::vector<Position>& source_nodes);
    void run_cuda_algo(const std::vector<Position>& source_nodes);
    void download_results();
    void free_gpu_memory();
};

#endif // ENMOD_PARALLEL_SOLVER_BASE_H
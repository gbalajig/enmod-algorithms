cmake_minimum_required(VERSION 3.10)

# 1. Force x64 architecture to prevent the 32-bit (win32) mismatch crash
if(MSVC)
    set(CMAKE_GENERATOR_PLATFORM x64 CACHE STRING "Explicitly set to x64" FORCE)
endif()

# 2. Allow newer MSVC versions that CUDA 13.0 might not officially recognize yet
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -allow-unsupported-compiler")

project(enmod_app LANGUAGES CXX CUDA)

find_package(CUDAToolkit REQUIRED)
set(CMAKE_CUDA_ARCHITECTURES native)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

set(SOURCES
    src/main.cpp
    src/Grid.cpp
    src/Logger.cpp
    src/ScenarioGenerator.cpp
    src/Solver.cpp
    src/Policy.cpp
    src/HtmlReportGenerator.cpp
    src/Cost.cpp
    src/BIDP.cpp
    src/FIDP.cpp
    src/AVI.cpp
    src/API.cpp
    src/AStarSolver.cpp
    src/PolicyGenerator.cpp
    src/PolicyVerifier.cpp
    src/DynamicBIDPSolver.cpp
    src/DynamicAPISolver.cpp
    src/DynamicFIDPSolver.cpp
    src/DynamicAVISolver.cpp
    src/DynamicAStarSolver.cpp
    src/DynamicHPASolver.cpp
    src/ADASolver.cpp
    src/DStarLiteSolver.cpp
    src/RLSolver.cpp
    src/QLearningSolver.cpp
    src/DynamicQLearningSolver.cpp
    src/SARSASolver.cpp
    src/DynamicSARSASolver.cpp
    src/ActorCriticSolver.cpp
    src/DynamicActorCriticSolver.cpp
    src/DQNSolver.cpp
    src/RLEnhancedAStarSolver.cpp
    src/HybridDPRLSolver.cpp
    src/AdaptiveCostSolver.cpp
    src/InterlacedSolver.cpp
    src/HierarchicalSolver.cpp
    src/PolicyBlendingSolver.cpp
    src/MultiAgentCPSController.cpp
    src/MultiAgentReportGenerator.cpp
    src/SimulatedSensorNetwork.cpp
    src/RealTimeSensorNetwork.cpp
    src/DynamicAdaptiveCostSolver.cpp
    src/DynamicInterlacedSolver.cpp
    src/DynamicHybridDPRLSolver.cpp
    src/DynamicPolicyBlendingSolver.cpp
    src/DynamicHierarchicalSolver.cpp
    src/EnhancedHybridDPRLSolver.cpp
    src/DynamicEnhancedHybridDPRLSolver.cpp
    # Parallel Solvers
    src/ParallelSolverBase.cu
    src/ParallelBIDP.cpp
    src/ParallelStaticSolvers.cpp
    src/ParallelDynamicBIDPSolver.cpp
    src/ParallelDynamicSolvers.cpp
    src/ParallelHybridSolvers.cpp
)

add_executable(enmod_app ${SOURCES})

target_include_directories(enmod_app PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(enmod_app PRIVATE CUDA::cudart)

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR})
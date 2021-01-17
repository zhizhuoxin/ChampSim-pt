cmake_minimum_required(VERSION 3.17)
project(ChampSim_pt)

set(CMAKE_CXX_STANDARD 14)

include_directories(inc)
include_directories(/usr/local/include)
link_directories(/usr/local/lib)

add_executable(ChampSim_pt
        branch/branch_predictor.cc
        inc/block.h
        inc/cache.h
        inc/champsim.h
        inc/dram_controller.h
        inc/instruction.h
        inc/kpcp.h
        inc/memory_class.h
        inc/ooo_cpu.h
        inc/set.h
        inc/spp_dev.h
        inc/trace_reader.h
        inc/trace_reader_pt.h
        inc/uncore.h
        prefetcher/kpcp_util.cc
        prefetcher/l1d_prefetcher.cc
        prefetcher/l1i_prefetcher.cc
        prefetcher/l2c_prefetcher.cc
        prefetcher/llc_prefetcher.cc
        replacement/base_replacement.cc
        replacement/llc_replacement.cc
        src/block.cc
        src/cache.cc
        src/dram_controller.cc
        src/main.cc
        src/ooo_cpu.cc
        src/trace_reader.cc
        src/uncore.cc)

target_link_libraries(ChampSim_pt
        xed
        z)

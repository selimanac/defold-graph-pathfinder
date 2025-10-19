
# A* Graph Pathfinding  Defold native extension

A high-performance A* pathfinding library written in C++11, designed for real-time games and simulations with hundreds to thousands of moving objects. Built with a focus on performance, using flat array data structures and advanced caching mechanisms.



## Features

- **High Performance**: Optimized for real-time pathfinding with 100s-1000s of moving objects
- **A* Algorithm**: Classic A* with heuristics for optimal pathfinding
- **Flat Array Architecture**: Cache-friendly memory layout using `dmArray<T>` (included from Defold SDK)
- **Path Caching**: LRU cache with version-tracked invalidation for frequently used paths
- **Projected Pathfinding**: Find paths from arbitrary positions (not just nodes)
- **Dynamic Graph Updates**: Add/remove nodes and edges at runtime with automatic cache invalidation
- **Min-Heap Priority Queue**: Custom implementation with true zero-copy memory pooling and bulk operations
- **Distance Caching**: Spatial hashing for fast distance lookups


### Using Pre-Built Libraries

**Available Platforms:**
- **Desktop**: Linux (x86_64, arm64), macOS (x86_64, arm64), Windows (x86_64)
- **Mobile**: iOS (arm64, x86_64), Android (armv7, arm64)
- **Web**: JavaScript, WebAssembly (via Emscripten)


### Basic Usage

```cpp
#include "pathfinder_path.h"
#include "pathfinder_types.h"

// Initialize the pathfinding system
pathfinder::path::init(
    120,    // max_nodes
    6,      // max_edges_per_node
    20,     // heap_pool_block_size
    8       // max_cache_path_length
);

// Add nodes
PathStatus status;
Vec2 pos1 = {100.0f, 100.0f};
Vec2 pos2 = {200.0f, 200.0f};

uint32_t node1 = pathfinder::path::add_node(pos1, &status);
uint32_t node2 = pathfinder::path::add_node(pos2, &status);

// Add edges (bidirectional)
float cost = 141.42f; // Euclidean distance
pathfinder::path::add_edge(node1, node2, cost, true, &status);

// Find path
dmArray<uint32_t> path;
uint32_t path_length = pathfinder::path::find_path(
    node1,      // start_id
    node2,      // goal_id
    &path,      // output path
    64,         // max_path_length
    &status     // output status
);

if (status == SUCCESS && path_length > 0) {
    // Use the path
    for (uint32_t i = 0; i < path_length; i++) {
        Vec2 pos = pathfinder::path::get_node_position(path[i]);
        // Move to position...
    }
}

// Cleanup
pathfinder::path::shutdown();
```

### Projected Pathfinding

Find paths from arbitrary positions (not on the graph):

```cpp
Vec2 arbitrary_position = {150.0f, 150.0f};
Vec2 entry_point;
dmArray<uint32_t> path;
PathStatus status;

uint32_t path_length = pathfinder::path::find_path_projected(
    arbitrary_position,  // start position (not a node)
    goal_node_id,        // destination node
    &path,               // output path
    64,                  // max_path_length
    &entry_point,        // where we enter the graph
    &status              // output status
);
```

### Path Smoothing

Apply smoothing to create natural-looking curved paths. The library provides built-in smoothing functions with automatic capacity management:

```cpp
#include "pathfinder_smooth.h"

// After finding a path with A*
dmArray<uint32_t> path;
uint32_t path_length = pathfinder::path::find_path(start_id, goal_id, &path, 64, &status);

// Recommended: Corner-only smoothing (quadratic Bézier)
// Calculate optimal capacity
uint32_t capacity = pathfinder::smooth::calculate_smoothed_path_capacity(path, 8);

dmArray<Vec2> smoothed_path;
smoothed_path.SetCapacity(capacity);

// Smooth with intuitive curve_radius parameter (0.0 = tight, 1.0 = smooth)
pathfinder::smooth::bezier_quadratic(path, smoothed_path, 8, 0.5f);

// Use smoothed_path for character movement
for (uint32_t i = 0; i < smoothed_path.Size(); i++) {
    move_character_to(smoothed_path[i]);
}
```

**Smoothing Methods:**

| Method | Best For | Characteristics |
|--------|----------|-----------------|
| `circular_arc()` | ⭐ Railroads, tower defense, tile-based games | **Perfect circular arcs** for corners - ideal for tile matching |
| `bezier_quadratic()` | Character movement, vehicles | Straight segments stay straight, only corners smoothed |
| `bezier_adaptive()` | Dynamic paths, varying turn angles | Adaptive control point placement based on corner sharpness |
| `catmull_rom()` | Precise waypoint following | Passes through all waypoints |
| `bezier_cubic()` | Cinematic cameras, UI | Maximum smoothness with two control points |

**Circular Arc Smoothing (New! Perfect for Tile-Based Games):**

Use circular arc smoothing for railroad tracks, tower defense roads, and any tile-based system requiring perfect circular corners:

```cpp
// Railroad 90-degree turn with perfect circular arc
dmArray<Vec2> railroad_path;
float arc_radius = 30.0f;  // Radius in world units (matches tile size)

pathfinder::smooth::circular_arc(path, railroad_path, 16, arc_radius);

// Result: Perfect circular arc that matches circular tile graphics exactly
// No approximation - true mathematical circle
```

**Benefits:**
- ✅ Perfect circular geometry (not approximate)
- ✅ Matches circular tile graphics exactly
- ✅ Minimal nodes required (3 nodes for any angle)
- ✅ Supports 90°, 180°, 45°, and custom angles



**curve_radius Parameter Guide (for Bezier smoothing):**

```cpp
// Racing game - tight corners
smooth::bezier_quadratic(path, smoothed, 8, 0.0f);

// Character movement - balanced (default)
smooth::bezier_quadratic(path, smoothed, 8, 0.5f);

// Cinematic camera - very smooth
smooth::bezier_quadratic(path, smoothed, 8, 1.0f);
```

**Auto-Growth Feature:**

Don't know the path length in advance? The array will auto-grow safely:

```cpp
dmArray<Vec2> smoothed;
smoothed.SetCapacity(20);  // Small initial capacity - will auto-grow

smooth::bezier_quadratic(path, smoothed, 8, 0.5f);  // Safe - no crashes!
```

## Architecture

### Core Components

#### Pathfinding (`pathfinder::path` namespace)
- **Files**: `src/pathfinder_path.cpp`, `include/pathfinder_path.h`
- **Purpose**: A* pathfinding algorithm, graph management
- **Key Functions**:
  - `init()`, `shutdown()`: System lifecycle
  - `add_node()`, `move_node()`, `remove_node()`: Node management
  - `add_edge()`, `remove_edge()`: Edge management
  - `find_path()`: Standard A* pathfinding
  - `find_path_projected()`: Path from arbitrary positions

#### Path Cache (`pathfinder::cache` namespace)
- **Files**: `src/pathfinder_cache.cpp`, `include/pathfinder_cache.h`
- **Purpose**: LRU cache for frequently used paths
- **Features**:
  - Version-tracked invalidation
  - Separate caches for standard and projected paths
  - Automatic invalidation on graph changes

#### Priority Queue (`pathfinder::heap` namespace)
- **Files**: `src/pathfinder_heap.cpp`, `include/pathfinder_heap.h`
- **Purpose**: Min-heap for A* open set
- **Features**:
  - True zero-copy memory pooling using `dmArray`
  - Bulk operations (`build()`, `push_many()`) for efficiency
  - Fast insert/extract-min operations

#### Distance Cache (`pathfinder::distance_cache` namespace)
- **Files**: `src/pathfinder_distance_cache.cpp`, `include/pathfinder_distance_cache.h`
- **Purpose**: Spatial hashing for fast distance lookups
- **Features**:
  - Grid-based spatial partitioning
  - Fast nearest neighbor queries

### Memory Management

- **No STL containers**: Uses `dmArray<T>` instead of STL containers
- **Pre-allocation**: All arrays pre-allocated at initialization
- **No runtime allocation**: Pathfinding operations use pre-allocated memory
- **Object pooling**: Heap nodes are pooled and reused

## Performance Characteristics

- **Graph Size**: Tested with 120 nodes, 1000+ paths
- **Cache Hit Rate**: ~90%+ for common paths with appropriate cache size
- **Memory**: Pre-allocated at initialization, zero allocation during pathfinding
- **Pathfinding**: O((V + E) log V) typical A* complexity
- **Heap Operations**: O(log n) push/pop, O(n) bulk build
- **Build Time**: 2-3 seconds (clean build with cached dependencies)

## Code Style & Standards

This project follows strict C++11 guidelines:

- **No `auto` keyword**: All types are explicit
- **No STL containers**: Uses `dmArray<T>` from Defold SDK
- **No modern C++ features**: Strictly C++11 compliant
- **Explicit error handling**: Via `PathStatus` enum and output parameters
- **No exceptions**: Uses error codes
- **Flat arrays**: For cache-friendly memory access
- **No dynamic allocation**: In hot paths


### Defold Native Extension Compatibility

When using this library as a Defold native extension:

- **Apple platforms (macOS, iOS)**: LTO is disabled to ensure compatibility with Defold's extender server (LLVM 15). This avoids bitcode version mismatch errors.
- **Other platforms**: LTO remains enabled for optimal performance.

## Error Handling

All operations return status through output parameters:

```cpp
PathStatus status;
uint32_t result = pathfinder::path::add_node(position, &status);

if (result == ERROR || status != SUCCESS) {
    // Handle error based on status
    switch (status) {
        case ERROR_NODE_FULL:
            // Graph is full - increase max_nodes in init()
            break;
        case ERROR_START_NODE_INVALID:
            // Invalid start node
            break;
        case ERROR_EDGE_FULL:
            // Too many edges per node - increase max_edges_per_node
            break;
        case ERROR_HEAP_FULL:
            // Heap pool exhausted - increase max_nodes (heap pool capacity = max_nodes)
            // Note: pool_block_size is automatically clamped to max_nodes if larger
            break;
        // ... handle other cases
    }
}
```

## Contributing

Contributions are welcome! Please:

1. Follow the existing code style (see `.clang-format`)
2. Adhere to C++11 standards (no modern C++ features)
3. Use `dmArray<T>`, never STL containers
4. No `auto` keyword
5. Maintain explicit types and error handling
6. Test your changes with the test suite
7. Run `clang-format` before committing

## Use Cases

This engine is ideal for:

- **Real-time strategy games**: Multiple unit pathfinding
- **Tower defense games**: Enemy wave navigation
- **Simulation games**: NPC navigation in cities/worlds
- **Robotics**: Path planning in known environments
- **Any application**: Requiring fast pathfinding for many agents

## License

TODO

---



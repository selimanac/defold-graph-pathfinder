
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


### Available Platforms

- **Desktop**: Linux (x86_64, arm64), macOS (x86_64, arm64), Windows (x86_64)
- **Mobile**: iOS (arm64, x86_64), Android (armv7, arm64)
- **Web**: JavaScript, WebAssembly (via Emscripten)


### Smoothing Methods:

| Method | Best For | Characteristics |
|--------|----------|-----------------|
| `circular_arc()` | ⭐ Railroads, tower defense, tile-based games | **Perfect circular arcs** for corners - ideal for tile matching |
| `bezier_quadratic()` | Character movement, vehicles | Straight segments stay straight, only corners smoothed |
| `bezier_adaptive()` | Dynamic paths, varying turn angles | Adaptive control point placement based on corner sharpness |
| `catmull_rom()` | Precise waypoint following | Passes through all waypoints |
| `bezier_cubic()` | Cinematic cameras, UI | Maximum smoothness with two control points |


### Memory Management

- **No STL containers**: Uses `dmArray<T>` or `dmHashtable<T>` instead of STL containers
- **Pre-allocation**: All arrays pre-allocated at initialization
- **No runtime allocation**: Pathfinding operations use pre-allocated memory
- **Object pooling**: Heap nodes are pooled and reused
- **No `auto` keyword**: All types are explicit
- **No modern C++ features**: Strictly C++11 compliant
- **No exceptions**: Uses error codes
- **Flat arrays**: For cache-friendly memory access
- **No dynamic allocation**: In hot paths

## Performance Characteristics

- **Graph Size**: Tested with 120 nodes, 1000+ paths
- **Cache Hit Rate**: ~90%+ for common paths with appropriate cache size
- **Memory**: Pre-allocated at initialization, zero allocation during pathfinding
- **Pathfinding**: O((V + E) log V) typical A* complexity
- **Heap Operations**: O(log n) push/pop, O(n) bulk build



## License

- ✅ Free for personal, educational, and **completely free** games or apps.  
- 💲 Commercial use (any monetized game/app) requires a **one-time $10 payment via GitHub Sponsors**.  
- 📩 Free releases are welcome to share project info (name, genre, date, links).  
- 🔒 The Software is **closed-source**; source access may be granted **upon request**.



---



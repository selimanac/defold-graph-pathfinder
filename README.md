
# A* Graph Pathfinding  Defold native extension

A high-performance A* pathfinding library written in C++11, designed for real-time games and simulations with hundreds to thousands of moving objects. Built with a focus on performance, using flat array data structures and advanced caching mechanisms.

## Technical Features

- **High Performance**: Optimized for real-time pathfinding with hundreds to thousands of moving objects  
- **A* Algorithm**: Classic A* with heuristics for optimal and efficient pathfinding  
- **Flat Array Architecture**: Cache-friendly memory layout using `dmArray<T>` from the Defold SDK  
- **Path Caching**: LRU cache with version-based invalidation for frequently reused paths  
- **Projected Pathfinding**: Supports pathfinding from arbitrary positions, not limited to graph nodes  
- **Dynamic Graph Updates**: Add or remove nodes and edges at runtime with automatic cache invalidation  
- **Min-Heap Priority Queue**: Custom implementation with zero-copy memory pooling and bulk operations  
- **Distance Caching**: Spatial hashing for fast approximate distance lookups  
- **No STL Containers**: Uses `dmArray<T>` and `dmHashTable<T>` for all data structures  
- **Memory Pre-allocation**: All arrays are pre-allocated at initialization; pathfinding uses only pre-allocated memory  
- **No Exceptions**: Uses explicit error codes instead of C++ exceptions

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



---

## Toss a Coin to Your Witcher

If you find my [Defold extensions](https://github.com/selimanac) useful in your projects, please consider [supporting me](https://github.com/sponsors/selimanac) on GitHub Sponsors.  
I’d also love to hear about your games or apps that use these extensions, sharing your released projects really motivates me to keep building more tools for the community!

---

## License

- ✅ Free for personal, educational, and **completely free** games or apps.  
- 💲 Commercial use (any monetized game/app) requires a **one-time $10 payment via GitHub Sponsors**.  
- 📩 Free releases are welcome to share project info (name, genre, date, links).  
- 🔒 The Software is **closed-source**; source access may be granted **upon request**.

---



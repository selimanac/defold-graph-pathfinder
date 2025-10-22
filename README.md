
# A* Graph Pathfinding Defold Native Extension

A high-performance A* pathfinding library written in C++11, designed for real-time games and simulations with hundreds to thousands of moving objects. Built with a focus on performance, using flat array data structures and advanced caching mechanisms.


## Discussions & Release Notes

https://github.com/selimanac/defold-graph-pathfinder/discussions  

## Documentation

For detailed function descriptions and usage examples, see the [API Reference](./API.md).

## Installation

You can use this extension in your own project by adding it as a [Defold library dependency](https://defold.com/manuals/libraries/#setting-up-library-dependencies).  
Open your `game.project` file, select `Project`, and add a stable version from the [releases](./releases) page to the `Dependencies` field.

## Examples

WIP

## Status

This library consists of three parts:

### Pathfinder

The core library. Responsible for managing nodes, edges, and performing pathfinding.

✅ Projected Pathfinding  
✅ Min-Heap Priority Queue  
✅ Path Caching  
✅ Distance Caching  
🚧 Smoothed Path Caching — not planned, maybe in the future  

### Path Smoothing

Responsible for existing path smoothing.  
Currently available smoothing options:  

✅ Quadratic Bézier  
✅ Adaptive Bézier  
✅ Catmull–Rom  
✅ Cubic Bézier  
✅ Circular Arc

### Navigation

Responsible for enabling agents to navigate along the path.

🚧 **Path Following** - WIP  
🚧 **Dynamic Nodes**  
🚧 **Path Invalidation**  
🚧 **Group Assignment**  
🚧 **Group Formation Patterns**  
🚧 **Collision Avoidance** — Not planned, maybe in the future



## Supported Platforms

- **Desktop**: Linux (x86_64, arm64), macOS (x86_64, arm64), Windows (x86_64)
- **Mobile**: iOS (arm64, x86_64), Android (armv7, arm64)
- **Web**: JavaScript, WebAssembly (via Emscripten)

## Technical Features

- **High Performance**: Optimized for real-time pathfinding with hundreds to thousands of moving objects  
- **A* Algorithm **: Classic A* with heuristics for optimal and efficient pathfinding  
- **Flat Array Architecture**: Cache-friendly memory layout using `dmArray<T>` from the Defold SDK  
- **Path Caching**: LRU cache with version-based invalidation for frequently reused paths  
- **Projected Pathfinding**: Supports pathfinding from arbitrary positions, not limited to graph nodes  
- **Dynamic Graph Updates**: Add or remove nodes and edges at runtime with automatic cache invalidation  
- **Min-Heap Priority Queue**: Custom implementation with zero-copy memory pooling and bulk operations  
- **Distance Caching**: Spatial hashing for fast approximate distance lookups  
- **No STL Containers**: Uses `dmArray<T>` and `dmHashTable<T>` for all data structures  
- **Memory Pre-allocation**: All arrays are pre-allocated at initialization; pathfinding uses only pre-allocated memory  
- **No Exceptions**: Uses explicit error codes instead of C++ exceptions


---

## Toss a Coin to Your Witcher

If you find my [Defold extensions](https://github.com/selimanac) useful in your projects, please consider [supporting me](https://github.com/sponsors/selimanac) on GitHub Sponsors.  
I’d also love to hear about your games or apps that use these extensions, sharing your released projects really motivates me to keep building more tools for the community!

---

### License Summary (not legally binding)

- ✅ Free for personal, educational, and **completely free** games or apps.  
- 💲 Commercial use (any monetized game/app) requires a **one-time payment of at least $10 via GitHub Sponsors**.  
- 📩 Free releases are welcome to share project info (name, genre, date, links).  
- 🔒 The Software is **closed-source**; source access may be granted **upon request**.

See  [License](./LICENSE.md)

---



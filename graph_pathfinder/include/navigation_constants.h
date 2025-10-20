#ifndef NAVIGATION_CONST_H
#define NAVIGATION_CONST_H

namespace navigation
{
    // Agent state
    enum AgentState
    {
        INACTIVE = 0,   // Not in navigation system
        ACTIVE = 1,     // Following path
        PAUSED = 2,     // Paused by application
        REPLANNING = 3, // Detected invalidation, finding new path
        ARRIVED = 4     // Reached goal
    };

    // Formation pattern type -  -> Phase 2
    /*  enum FormationType
      {
          NONE = 0,
          COLUMN = 1, // Single file
          LINE = 2,   // Horizontal line
          WEDGE = 3,  // V-shape
          BOX = 4     // Grid formation
      };*/
} // namespace navigation

#endif
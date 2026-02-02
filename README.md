# AtlasSDK

AtlasSDK is a modular **C++20** library designed as a low-level foundation for game engines and real-time simulation systems.  
It focuses on **math, transforms, geometry, and basic physics primitives**, with an explicit emphasis on correctness, clarity, and engine-oriented design.

The SDK is built as a collection of small, mostly independent modules that can be used selectively or as a coherent base for a custom engine.

---

## Current Version

**v0.1.0** — Initial public release  
⚠️ API is considered **unstable** (pre-1.0)

---

## Implemented Modules

### Core Module

- **math**
    - Scalar math utilities and constants
    - Engine-oriented helpers (no STL overengineering)

- **vectors**
    - 2D and 3D vector types
    - Common vector operations (dot, cross, normalization, etc.)

- **matrix**
    - 4×4 matrix implementation
    - Transformation utilities (TRS-oriented)

- **quaternion**
    - Quaternion representation
    - Rotation utilities and conversions

- **transform**
    - Transform class for spatial manipulation
    - Position, rotation, and scale handling
    - Engine-style transform composition

---

### Physics Module

- **shapes**
    - Primitive 2D and 3D shapes
    - Overlap tests between basic primitives

- **raycast**
    - Raycasting in 2D and 3D
    - Ray vs primitive shape intersection tests

- **broadphase**
    - Broadphase data structures for 2D and 3D shapes
    - Spatial partitioning for collision and raycast filtering

---

## Design Principles

- Modular and dependency-light architecture
- Explicit and predictable math behavior
- Engine-first design (not a general-purpose math library)
- No hidden allocations or implicit state
- Clear separation between math, geometry, and physics layers

---

## Requirements

- **C++20**
- A C++20-compliant compiler (Clang, GCC, MSVC)

---

## Project Scope

AtlasSDK is developed as a **portfolio project** aimed at engine-level programming.  
The goal is to progressively evolve this SDK into the core of a custom physics and game engine, maintaining high standards of code quality, API clarity, and correctness.

Future versions will expand on:
- Rigid body dynamics
- Constraint solving
- Narrowphase collision resolution
- Engine-facing abstractions

---

## Versioning

This project follows **Semantic Versioning**.  
Breaking API changes may occur until version **1.0.0**.

---

## License

MIT License

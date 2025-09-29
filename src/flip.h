#ifndef flip_h
#define flip_h
#include <cmath>
#include <cstdint>

#endif

// Cell types
enum CellType {
    FLUID_CELL = 0,
    AIR_CELL = 1,
    SOLID_CELL = 2
};

class FlipFluid {
private:
    // Grid parameters
    float h;                    // Grid cell size
    float invH;                 // 1/h for fast division
    float hHalf;                // half of h, to avoid unesssary divisions
    float particleRadius;
    int gridWidth, gridHeight;  // Number of cells
    float flipRatio;
    float restDensity;
    float driftConstant;
    bool compensateDrift;
    float overRelaxation;
    int projectionIters;

    // U-velocities: stored on vertical faces (left/right edges of cells)
    float** u;          // [gridWidth+1][gridHeight] - extra column for right boundary
    float** prevU;      // Previous u velocities
    float** du;         // Weights for u transfer
    // V-velocities: stored on horizontal faces (top/bottom edges of cells)  
    float** v;          // [gridWidth][gridHeight+1] - extra row for top boundary
    float** prevV;      // Previous v velocities
    float** dv;         // Weights for v transfer
    
    // Cell-centered values
    // float** pressure;           // [gridWidth][gridHeight]
    // float** solid;              // [gridWidth][gridHeight] - 0=solid, 1=fluid
    CellType** cellType;        // [gridWidth][gridHeight]
    float** particleDensity;    // [gridWidth][gridHeight]
    
    // Particle arrays - SEPARATE for better cache performance
    float* pPosX;        // X positions [MAX_PARTICLES]
    float* pPosY;        // Y positions [MAX_PARTICLES]
    float* pVelX;        // X velocities [MAX_PARTICLES]
    float* pVelY;        // Y velocities [MAX_PARTICLES]
    
    // Particle collision grid (for particle-particle collisions)
    int pGridWidth, pGridHeight, totalPCells;
    float pInvSpacing;
    int* numCellParticles;      // [pGridWidth * pGridHeight]
    int* firstCellParticle;     // [pGridWidth * pGridHeight + 1]
    int* cellParticleIds;       // [numParticles]
    int numParticles;

    float fastInvSqrt(float x) {
        float xhalf = 0.5f * x;
        int i = *(int*)&x;
        i = 0x5f3759df - (i >> 1);
        x = *(float*)&i;
        return x * (1.5f - (xhalf * x * x));
    }
    inline int clamp(int value, int minVal, int maxVal) {
        if (value < minVal) return minVal;
        if (value > maxVal) return maxVal;
        return value;
    }
    inline float clamp(float value, float minVal, float maxVal) {
        if (value < minVal) return minVal;
        if (value > maxVal) return maxVal;
        return value;
    }
    inline int min(int a, int b) { return (a < b) ? a : b; }
    inline int max(int a, int b) { return (a > b) ? a : b; }
    
public:
    // Constructor - allocate all arrays
    FlipFluid() {}
    void init(uint16_t screenWidth, uint16_t screenHeight, uint16_t cellSize, float pRadius, float flip, float driftConst, int projectionIterations, float overrelaxation) {
        h = cellSize;
        invH = 1.0f / h;
        hHalf = h * 0.5f;
        gridWidth = screenWidth / h;
        gridHeight = screenHeight / h;
        particleRadius = pRadius;
        flipRatio = flip;
        restDensity = 0.0f;
        driftConstant = driftConst;
        projectionIters = projectionIterations;
        overRelaxation = overrelaxation;
        // particle array allocation and initialization done in another function
        
        // Allocate 2D arrays for U-velocities [gridWidth+1][gridHeight]
        u = new (std::nothrow) float*[gridWidth + 1];
        prevU = new (std::nothrow) float*[gridWidth + 1];
        du = new (std::nothrow) float*[gridWidth + 1];
        for (int i = 0; i <= gridWidth; i++) {
            u[i] = new (std::nothrow) float[gridHeight]();      // () initializes to zero
            prevU[i] = new (std::nothrow) float[gridHeight]();
            du[i] = new (std::nothrow) float[gridHeight]();
        }
        
        // Allocate 2D arrays for V-velocities [gridWidth][gridHeight+1]
        v = new (std::nothrow) float*[gridWidth];
        prevV = new (std::nothrow) float*[gridWidth];
        dv = new (std::nothrow) float*[gridWidth];
        for (int i = 0; i < gridWidth; i++) {
            v[i] = new (std::nothrow) float[gridHeight + 1]();
            prevV[i] = new (std::nothrow) float[gridHeight + 1]();
            dv[i] = new (std::nothrow) float[gridHeight + 1]();
        }
        
        // Allocate 2D arrays for cell-centered values [gridWidth][gridHeight]
        // pressure = new float*[gridWidth];
        // solid = new float*[gridWidth];
        cellType = new CellType*[gridWidth];
        particleDensity = new float*[gridWidth];
        for (int i = 0; i < gridWidth; i++) {
            // pressure[i] = new float[gridHeight]();
            // solid[i] = new float[gridHeight]();
            cellType[i] = new (std::nothrow) CellType[gridHeight]();
            particleDensity[i] = new float[gridHeight]();
        }

        // Set up particle collision grid (finer grid for particle separation)
        pInvSpacing = 1.0f / (2.2f * particleRadius);
        pGridWidth = (int)(screenWidth * pInvSpacing) + 1;
        pGridHeight = (int)(screenHeight * pInvSpacing) + 1;

        // Allocate particle collision grid
        totalPCells = pGridWidth * pGridHeight;
        numCellParticles = new (std::nothrow) int[totalPCells]();
        firstCellParticle = new (std::nothrow) int[totalPCells + 1]();
        // cellParticleIds = new (std::nothrow) int[numParticles]();
    }
    
    // Destructor - clean up all arrays
    ~FlipFluid() {
        // Clean up U arrays
        for (int i = 0; i <= gridWidth; i++) {
            delete[] u[i];
            delete[] prevU[i];
            delete[] du[i];
        }
        delete[] u; delete[] prevU; delete[] du;
        
        // Clean up V arrays
        for (int i = 0; i < gridWidth; i++) {
            delete[] v[i];
            delete[] prevV[i];
            delete[] dv[i];
        }
        delete[] v; delete[] prevV; delete[] dv;
        
        // Clean up cell-centered arrays
        for (int i = 0; i < gridWidth; i++) {
            // delete[] pressure[i];
            // delete[] solid[i];
            delete[] cellType[i];
            delete[] particleDensity[i];
        }
        // delete[] pressure;
        // delete[] solid;
        delete[] cellType; delete[] particleDensity;
        
        // Clean up particle arrays
        delete[] pPosX; delete[] pPosY;
        delete[] pVelX; delete[] pVelY;
        
        // Clean up particle collision grid
        delete[] numCellParticles; delete[] firstCellParticle;
        delete[] cellParticleIds;
    }

    // Initialize boundary conditions
    void initializeBoundaries() {
        for (int i = 0; i < gridWidth; ++i) {
            for (int j = 0; j < gridHeight; ++j) {
                cellType[i][j] = AIR_CELL;
                particleDensity[i][j] = 0.0f;
            }
        }
        for (int i = 0; i <= gridWidth; ++i) {
            for (int j = 0; j < gridHeight; ++j) {
                u[i][j] = prevU[i][j] = 0.0f;
                du[i][j] = 0.0f;
            }
        }
        for (int i = 0; i < gridWidth; ++i) {
            for (int j = 0; j <= gridHeight; ++j) {
                v[i][j] = prevV[i][j] = 0.0f;
                dv[i][j] = 0.0f;
            }
        }
    }

    void initializeParticles() {
        // this function will place a particle at every cell center
        // Allocate particle arrays - SEPARATE for cache efficiency
        numParticles = gridWidth * gridHeight;
        pPosX = new float[numParticles]();
        pPosY = new float[numParticles]();
        pVelX = new float[numParticles]();
        pVelY = new float[numParticles]();
        float x = hHalf;
        float y = hHalf;
        for (int i = 0; i < numParticles; i++){
            pPosX[i] = x;
            pPosY[i] = y;
            pVelX[i] = 0.0f;
            pVelY[i] = 0.0f;
            x += h;
            if (x / h > gridWidth) {x = hHalf; y += h;}
        }
        cellParticleIds = new (std::nothrow) int[numParticles]();
    }
    void initializeParticlesInRadius(float radius, float centerX, float centerY, int multiples) {
        // Calculate screen center
        float radiusSquared = radius * radius;
        
        // First pass: count how many particles will fit in the radius
        int particleCount = 0;
        float x = hHalf;
        float y = hHalf;
        
        for (int col = 0; col < gridWidth; col++) {
            for (int row = 0; row < gridHeight; row++) {
                // Calculate distance from center
                float dx = x - centerX;
                float dy = y - centerY;
                float distSquared = dx * dx + dy * dy;
                
                if (distSquared <= radiusSquared) {
                    particleCount++;
                }
                
                x += h;
            }
            x = hHalf;
            y += h;
        }
        
        // Allocate arrays based on actual particle count
        numParticles = particleCount * multiples;
        pPosX = new float[numParticles]();
        pPosY = new float[numParticles]();
        pVelX = new float[numParticles]();
        pVelY = new float[numParticles]();
        
        // Second pass: actually place the particles
        int particleIndex = 0;
        x = hHalf;
        y = hHalf;
        for (int i = 0; i < multiples; i ++) {
            for (int col = 0; col < gridWidth; col++) {
                for (int row = 0; row < gridHeight; row++) {
                    // Calculate distance from center
                    float dx = x - centerX;
                    float dy = y - centerY;
                    float distSquared = dx * dx + dy * dy;
                    
                    if (distSquared <= radiusSquared) {
                        pPosX[particleIndex] = x - i;
                        pPosY[particleIndex] = y - i;
                        pVelX[particleIndex] = 0.0f;
                        pVelY[particleIndex] = 0.0f;
                        particleIndex++;
                    }
                    
                    x += h;
                }
                x = hHalf;
                y += h;
            }
        }
        cellParticleIds = new (std::nothrow) int[numParticles]();
    }
    void initializeCellTypeCircle(float radius, float centerX, float centerY) {
        float radiusSquared = radius * radius;
        // Loop through all grid cells
        for (int i = 0; i < gridWidth; i++) {
            for (int j = 0; j < gridHeight; j++) {
                // Calculate cell center position in world coordinates
                float cellCenterX = (i + 0.5f) * h;
                float cellCenterY = (j + 0.5f) * h;
                
                // Calculate distance from cell center to circle center
                float dx = cellCenterX - centerX;
                float dy = cellCenterY - centerY;
                float distSquared = dx * dx + dy * dy;
                
                // Set cell type based on distance
                if (distSquared > radiusSquared) {
                    // Outside the circle - solid boundary
                    cellType[i][j] = SOLID_CELL;
                } else {
                    cellType[i][j] = AIR_CELL;
                }
            }
        }
    }
    
    // Main simulation loop
    void simulate(float dt, float* acceleration) {
        integrateParticles(dt, acceleration); // Move particles
        separateParticles(2); // 2 iterations usually enough
        // handleParticleCircularWallCollisions(dt, 120.0f - particleRadius, 120.0f, 120.0f);
        handleParticleRectangularWallCollisions(dt);
        transferVelocity(true); // bool true:ParticlestoGrid false:GridtoParticles
        if (driftConstant > 0.0f) {
            updateParticleDensity();
            projection(projectionIters, overRelaxation, true); // int numIters, float overRelaxation, bool compensateDrift = true
        } else {projection(projectionIters, overRelaxation, false);}
        transferVelocity(false);
    }

    int getNumParticles() const { return numParticles; }
    const float* getParticlePosX() const { return pPosX; }
    const float* getParticlePosY() const { return pPosY; }
    const float* getParticleVelX() const { return pVelX; }
    const float* getParticleVelY() const { return pVelY; }
    const float getParticleRadius() const { return particleRadius; }

    // Grid data
    int getGridWidth() const { return gridWidth; }
    int getGridHeight() const { return gridHeight; }
    // float** getPressure() const { return pressure; }
    CellType** getCellType() const { return cellType; }
    // float** getSolid() const { return solid; }
    float** getParticleDensity() const { return particleDensity; }
    float getParticleDensity(int r, int c) const { return particleDensity[r][c]; }
    float** getU() const { return u; }
    float** getV() const { return v; }

private:
    void integrateParticles(float dt, float* acceleration){
        // Apply gravity and move particles by velocity * dt
        for (int i = 0; i < numParticles; i++) {
            pVelX[i] += -acceleration[1] * dt;
            pVelY[i] += acceleration[0] * dt;
            pPosX[i] += pVelX[i] * dt;
            pPosY[i] += pVelY[i] * dt;
        }
    }
    
    void handleParticleRectangularWallCollisions(float dt){
        // Check particles against walls and obstacles
        // Clamp positions and zero velocities if collision
        float nextX, nextY;
        for (int i = 0; i < numParticles; i++) {
            nextX = pPosX[i] + pVelX[i] * dt;
            nextY = pPosY[i] + pVelY[i] * dt;
            float maxX = float(gridWidth) * h - particleRadius;
            float maxY = float(gridHeight) * h - particleRadius;
            if (nextX > maxX) {pPosX[i] = maxX; pVelX[i] = -0.05f * pVelX[i];}
            if (nextX < particleRadius) {pPosX[i] = particleRadius; pVelX[i] = -0.05f * pVelX[i];}
            if (nextY > maxY) {pPosY[i] = maxY; pVelY[i] = -0.05f * pVelY[i];}
            if (nextY < particleRadius) {pPosY[i] = particleRadius; pVelY[i] = -0.05f * pVelY[i];}
        }
    }

    void handleParticleCircularWallCollisions(float dt, float radius, float centerX, float centerY){
        // Clamp positions and zero velocities if collision
        for (int i = 0; i < numParticles; i++) {
            float nextX = pPosX[i] + pVelX[i] * dt;
            float nextY = pPosY[i] + pVelY[i] * dt;
            // Calculate squared distance to center
            float nextDx = nextX - centerX;
            float nextDy = nextY - centerY;
            float nextDistSq = nextDx*nextDx + nextDy*nextDy;
            
            if (nextDistSq >= radius * radius) {
                float invDist = fastInvSqrt(nextDistSq);
                float nx = nextDx * invDist;
                float ny = nextDy * invDist;
                
                // Calculate both radial and tangential components
                float vDotN = pVelX[i]*nx + pVelY[i]*ny;
                float vRadialX = vDotN * nx;
                float vRadialY = vDotN * ny;
                float vTangentialX = pVelX[i] - vRadialX;
                float vTangentialY = pVelY[i] - vRadialY;
                // Apply damping only to tangential component
                pVelX[i] = vTangentialX * 0.05f;
                pVelY[i] = vTangentialY * 0.05f;
                // Reposition particle to be exactly at boundary (with small epsilon)
                pPosX[i] = centerX + nx * (radius - particleRadius);
                pPosY[i] = centerY + ny * (radius - particleRadius);
            }
        }
    }

    void separateParticles(int numIters) {
        float minDist = 2.0f * particleRadius;
        float minDist2 = minDist * minDist;

        int totalPCells = pGridWidth * pGridHeight;
        for (int i = 0; i < totalPCells; i++) {
            numCellParticles[i] = 0;
        }
        for (int i = 0; i < numParticles; i++) {
            float x = pPosX[i];
            float y = pPosY[i];
            int xi = clamp((int)(x * pInvSpacing), 0, pGridWidth - 1);
            int yi = clamp((int)(y * pInvSpacing), 0, pGridHeight - 1);
            int cellNr = xi * pGridHeight + yi;
            numCellParticles[cellNr]++;
        }
        
        // Build prefix sums (firstCellParticle array)
        int first = 0;
        for (int i = 0; i < totalPCells; i++) {
            first += numCellParticles[i];
            firstCellParticle[i] = first;
        }
        firstCellParticle[totalPCells] = first; // Guard element

        // Fill particles into cells (sort particle IDs by cell)
        for (int i = 0; i < numParticles; i++) {
            float x = pPosX[i];
            float y = pPosY[i];
            int xi = clamp((int)(x * pInvSpacing), 0, pGridWidth - 1);
            int yi = clamp((int)(y * pInvSpacing), 0, pGridHeight - 1);
            int cellNr = xi * pGridHeight + yi;

            // Decrement and place particle ID
            firstCellParticle[cellNr]--;
            cellParticleIds[firstCellParticle[cellNr]] = i;
        }
        
        // Push particles apart (multiple iterations for stability)
        for (int iter = 0; iter < numIters; iter++) {
            for (int i = 0; i < numParticles; i++) {
                float px = pPosX[i];
                float py = pPosY[i];
                int pxi = (int)(px * pInvSpacing);
                int pyi = (int)(py * pInvSpacing);
                int x0 = max(pxi - 1, 0);
                int y0 = max(pyi - 1, 0);
                int x1 = min(pxi + 1, pGridWidth - 1);
                int y1 = min(pyi + 1, pGridHeight - 1);
                for (int xi = x0; xi <= x1; xi++) {
                    for (int yi = y0; yi <= y1; yi++) {
                        int cellNr = xi * pGridHeight + yi;
                        int firstIdx = firstCellParticle[cellNr];
                        int lastIdx = firstCellParticle[cellNr + 1];
                        for (int j = firstIdx; j < lastIdx; j++) {
                            int id = cellParticleIds[j];
                            // Skip self
                            if (id == i) continue;
                            float qx = pPosX[id];
                            float qy = pPosY[id];
                            float dx = qx - px;
                            float dy = qy - py;
                            float d2 = dx * dx + dy * dy;
                            // Skip if too far apart or exactly on top of each other
                            if (d2 > minDist2 || d2 == 0.0f) continue;
                            // Push particles apart
                            float d = sqrt(d2);
                            float s = 0.5f * (minDist - d) / d;
                            dx *= s;
                            dy *= s;
                            // Move both particles away from each other
                            pPosX[i] -= dx;//  * 1.2f;
                            pPosY[i] -= dy;//  * 1.2f;
                            pPosX[id] += dx;//  * 1.2f;
                            pPosY[id] += dy;//  * 1.2f;
                        }
                    }
                }
            }
        }
    }

    void transferVelocity(bool toGrid) {
        // Clear and snapshot when scattering to grid
        if (toGrid) {
            // Reset cellType (air vs fluid marking) and zero accumulators
            for (int c = 0; c < gridWidth; ++c) {
                for (int r = 0; r < gridHeight; ++r) {
                    if (cellType[c][r] != SOLID_CELL) {
                        cellType[c][r] = AIR_CELL;
                    }
                }
            }
            // u: [gridWidth+1][gridHeight]
            for (int i = 0; i <= gridWidth; ++i) {
                for (int j = 0; j < gridHeight; ++j) {
                    prevU[i][j] = u[i][j];
                    u[i][j] = 0.0f;
                    du[i][j] = 0.0f;
                }
            }
            // v: [gridWidth][gridHeight+1]
            for (int i = 0; i < gridWidth; ++i) {
                for (int j = 0; j <= gridHeight; ++j) {
                    prevV[i][j] = v[i][j];
                    v[i][j] = 0.0f;
                    dv[i][j] = 0.0f;
                }
            }
        }

        const float eps_edge = 1e-6f;

        for (int p = 0; p < numParticles; ++p) {
            // Keep inside domain so floor doesn't create out-of-bounds on +1 access
            float px = clamp(pPosX[p], 0.0f, gridWidth * h - eps_edge);
            float py = clamp(pPosY[p], 0.0f, gridHeight * h - eps_edge);

            int cellX = clamp((int)std::floor(px * invH), 0, gridWidth - 1);
            int cellY = clamp((int)std::floor(py * invH), 0, gridHeight - 1);

            if (toGrid && cellType[cellX][cellY] == AIR_CELL) {
                cellType[cellX][cellY] = FLUID_CELL;
            }

            // Local coords in [0,1)
            float cx = px * invH - (float)cellX;
            float cy = py * invH - (float)cellY;

            float wx0 = 1.0f - cx, wx1 = cx;
            float wy0 = 1.0f - cy, wy1 = cy;

            float w00 = wx0 * wy0;
            float w10 = wx1 * wy0;
            float w01 = wx0 * wy1;
            float w11 = wx1 * wy1;

            if (toGrid) {
                // Scatter to u: [0..gridWidth][0..gridHeight-1]
                // Contribute to (cellX,cellY), (cellX+1,cellY), (cellX,cellY+1), (cellX+1,cellY+1) if inside
                int ux0 = cellX, ux1 = cellX + 1;
                int uy0 = cellY, uy1 = cellY + 1;

                if (ux0 >= 0 && ux0 <= gridWidth && uy0 >= 0 && uy0 < gridHeight) { u[ux0][uy0] += pVelX[p] * w00; du[ux0][uy0] += w00; }
                if (ux1 >= 0 && ux1 <= gridWidth && uy0 >= 0 && uy0 < gridHeight) { u[ux1][uy0] += pVelX[p] * w10; du[ux1][uy0] += w10; }
                if (ux0 >= 0 && ux0 <= gridWidth && uy1 >= 0 && uy1 < gridHeight) { u[ux0][uy1] += pVelX[p] * w01; du[ux0][uy1] += w01; }
                if (ux1 >= 0 && ux1 <= gridWidth && uy1 >= 0 && uy1 < gridHeight) { u[ux1][uy1] += pVelX[p] * w11; du[ux1][uy1] += w11; }

                // Scatter to v: [0..gridWidth-1][0..gridHeight]
                int vx0 = cellX, vx1 = cellX + 1;
                int vy0 = cellY, vy1 = cellY + 1;

                if (vx0 >= 0 && vx0 < gridWidth && vy0 >= 0 && vy0 <= gridHeight) { v[vx0][vy0] += pVelY[p] * w00; dv[vx0][vy0] += w00; }
                if (vx1 >= 0 && vx1 < gridWidth && vy0 >= 0 && vy0 <= gridHeight) { v[vx1][vy0] += pVelY[p] * w10; dv[vx1][vy0] += w10; }
                if (vx0 >= 0 && vx0 < gridWidth && vy1 >= 0 && vy1 <= gridHeight) { v[vx0][vy1] += pVelY[p] * w01; dv[vx0][vy1] += w01; }
                if (vx1 >= 0 && vx1 < gridWidth && vy1 >= 0 && vy1 <= gridHeight) { v[vx1][vy1] += pVelY[p] * w11; dv[vx1][vy1] += w11; }
            } else {
                // Gather PIC and FLIP
                auto bilerpU = [&](int i0, int j0, float ww00, float ww10, float ww01, float ww11) {
                    float pic = 0.0f, flip = 0.0f;
                    int i1 = i0 + 1, j1 = j0 + 1;
                    // bounds: i in [0..gridWidth], j in [0..gridHeight-1]
                    if (i0 >= 0 && i0 <= gridWidth && j0 >= 0 && j0 < gridHeight) { pic += u[i0][j0] * ww00; flip += (u[i0][j0] - prevU[i0][j0]) * ww00; }
                    if (i1 >= 0 && i1 <= gridWidth && j0 >= 0 && j0 < gridHeight) { pic += u[i1][j0] * ww10; flip += (u[i1][j0] - prevU[i1][j0]) * ww10; }
                    if (i0 >= 0 && i0 <= gridWidth && j1 >= 0 && j1 < gridHeight) { pic += u[i0][j1] * ww01; flip += (u[i0][j1] - prevU[i0][j1]) * ww01; }
                    if (i1 >= 0 && i1 <= gridWidth && j1 >= 0 && j1 < gridHeight) { pic += u[i1][j1] * ww11; flip += (u[i1][j1] - prevU[i1][j1]) * ww11; }
                    return std::pair<float,float>(pic, flip);
                };
                auto bilerpV = [&](int i0, int j0, float ww00, float ww10, float ww01, float ww11) {
                    float pic = 0.0f, flip = 0.0f;
                    int i1 = i0 + 1, j1 = j0 + 1;
                    // bounds: i in [0..gridWidth-1], j in [0..gridHeight]
                    if (i0 >= 0 && i0 < gridWidth && j0 >= 0 && j0 <= gridHeight) { pic += v[i0][j0] * ww00; flip += (v[i0][j0] - prevV[i0][j0]) * ww00; }
                    if (i1 >= 0 && i1 < gridWidth && j0 >= 0 && j0 <= gridHeight) { pic += v[i1][j0] * ww10; flip += (v[i1][j0] - prevV[i1][j0]) * ww10; }
                    if (i0 >= 0 && i0 < gridWidth && j1 >= 0 && j1 <= gridHeight) { pic += v[i0][j1] * ww01; flip += (v[i0][j1] - prevV[i0][j1]) * ww01; }
                    if (i1 >= 0 && i1 < gridWidth && j1 >= 0 && j1 <= gridHeight) { pic += v[i1][j1] * ww11; flip += (v[i1][j1] - prevV[i1][j1]) * ww11; }
                    return std::pair<float,float>(pic, flip);
                };

                int i0 = cellX, j0 = cellY;
                auto ux = bilerpU(i0, j0, w00, w10, w01, w11);
                auto vy = bilerpV(i0, j0, w00, w10, w01, w11);

                float picVelX  = ux.first;
                float flipVelX = ux.second;
                float picVelY  = vy.first;
                float flipVelY = vy.second;

                pVelX[p] = (flipVelX + pVelX[p]) * flipRatio + picVelX * (1.0f - flipRatio);
                pVelY[p] = (flipVelY + pVelY[p]) * flipRatio + picVelY * (1.0f - flipRatio);
            }
        }

        if (toGrid) {
            // Normalize u: [0..gridWidth][0..gridHeight-1]
            for (int i = 0; i <= gridWidth; ++i) {
                for (int j = 0; j < gridHeight; ++j) {
                    if (du[i][j] > 0.001f) {
                        u[i][j] /= du[i][j];
                    } else {u[i][j] = 0.0f;}
                }
            }
            // Normalize v: [0..gridWidth-1][0..gridHeight]
            for (int i = 0; i < gridWidth; ++i) {
                for (int j = 0; j <= gridHeight; ++j) {
                    if (dv[i][j] > 0.001f) {
                        v[i][j] /= dv[i][j];
                    } else {v[i][j] = 0.0f;}
                }
            }
        }
    }
    
    void updateParticleDensity() {
        for (int c = 0; c < gridWidth; c++) {
            for (int r = 0; r < gridHeight; r++) {
                particleDensity[c][r] = 0.0f;
            }
        }

        const float eps = 1e-7f;
        for (int p = 0; p < numParticles; ++p) {
            const float x = pPosX[p];
            const float y = pPosY[p];
            // Convert to "centered" grid coordinates (u, v) measured in cells
            // u = 0 aligns with center of cell i=0 (at x = 0.5*h)
            // So u = x/h - 0.5
            const float u = x * invH - 0.5f;
            const float v = y * invH - 0.5f;
            // Base cell index = floor of u, v -> left/bottom neighbor center
            int i0 = (int)floor(u);
            int j0 = (int)floor(v);
            int i1 = i0 + 1;
            int j1 = j0 + 1;
            const float fu = u - (float)i0; // in [0,1) typically
            const float fv = v - (float)j0;
            // 1D hat weights in x and y
            // w0 corresponds to the "lower" index (i0/j0), w1 to (i1/j1)
            float wx0 = 1.0f - fu; if (wx0 < 0.0f) wx0 = 0.0f; // numeric guard
            float wx1 = fu;        if (wx1 < 0.0f) wx1 = 0.0f;
            float wy0 = 1.0f - fv; if (wy0 < 0.0f) wy0 = 0.0f;
            float wy1 = fv;        if (wy1 < 0.0f) wy1 = 0.0f;

            float w00 = wx0 * wy0;
            float w10 = wx1 * wy0;
            float w01 = wx0 * wy1;
            float w11 = wx1 * wy1;

            int ci0 = clamp(i0, 0, gridWidth  - 1);
            int ci1 = clamp(i1, 0, gridWidth  - 1);
            int cj0 = clamp(j0, 0, gridHeight - 1);
            int cj1 = clamp(j1, 0, gridHeight - 1);
            if (w00 > eps) particleDensity[ci0][cj0] += w00;
            if (w10 > eps) particleDensity[ci1][cj0] += w10;
            if (w01 > eps) particleDensity[ci0][cj1] += w01;
            if (w11 > eps) particleDensity[ci1][cj1] += w11;
        }
        


        if (restDensity == 0.0f) {
            float sum = 0.0f;
            int numFluidCells = 0;
            for (int c = 0; c < gridWidth; c++) {
                for (int r = 0; r < gridHeight; r++) {
                    if (cellType[c][r] == FLUID_CELL) {
                        sum += particleDensity[c][r];
                        numFluidCells++;
                    }
                }
            }
            if (numFluidCells > 0) {restDensity = sum / float(numFluidCells);}
        }
    }

    void projection(int numIters, float overRelaxation, bool compensateDrift = true) {
        // Main Gauss-Seidel iteration loop
        for (int iter = 0; iter < numIters; iter++) {
            for (int c = 0; c < gridWidth; c++) {
                for (int r = 0; r < gridHeight; r++) {
                    if (cellType[c][r] != FLUID_CELL)
                        continue;

                    float sx0 = (c > 0 && cellType[c-1][r] != SOLID_CELL) ? 1.0f : 0.0f;  // left
                    float sx1 = (c < gridWidth-1 && cellType[c+1][r] != SOLID_CELL) ? 1.0f : 0.0f;  // right
                    float sy0 = (r > 0 && cellType[c][r-1] != SOLID_CELL) ? 1.0f : 0.0f;  // bottom
                    float sy1 = (r < gridHeight-1 && cellType[c][r+1] != SOLID_CELL) ? 1.0f : 0.0f;  // top

                    float s = sx0 + sx1 + sy0 + sy1;
                    if (s == 0.0f)
                        continue;

                    float div = - u[c][r] + u[c + 1][r] - v[c][r] + v[c][r + 1];

                    div *= overRelaxation;

                    if (restDensity > 0.0f && compensateDrift && particleDensity[c][r] > restDensity) {
                        float compression = particleDensity[c][r] - restDensity;
                        div -= driftConstant * compression;
                    }
                    float normalizedDiv = div / s;

                    u[c][r] += sx0 * normalizedDiv;
                    u[c + 1][r] -= sx1 * normalizedDiv;
                    v[c][r] += sy0 * normalizedDiv;
                    v[c][r + 1] -= sy1 * normalizedDiv;
                }
            }
        }
    }
};
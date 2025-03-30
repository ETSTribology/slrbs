FROM ubuntu:22.04 AS builder

# Set non-interactive mode for apt
ENV DEBIAN_FRONTEND=noninteractive

# Install build dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    ccache \
    ninja-build \
    python3-dev \
    python3-pip \
    libgl1-mesa-dev \
    libglew-dev \
    libglfw3-dev \
    libeigen3-dev \
    libomp-dev \
    wget \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Setup working directory
WORKDIR /app

# Copy project files
COPY . .

# Create build directory and prepare for build
RUN mkdir -p build/linux

# Configure and build the project with CMake using the linux-release preset
WORKDIR /app/build/linux
RUN cmake ../.. -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_PYTHON_BINDINGS=ON \
    -DCMAKE_C_COMPILER=gcc \
    -DCMAKE_CXX_COMPILER=g++ \
    && cmake --build . --config Release -j$(nproc)

# Create a smaller runtime image
FROM ubuntu:22.04 AS runtime

# Install runtime dependencies
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libglew2.2 \
    libglfw3 \
    libomp5 \
    python3 \
    python3-pip \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Create app directory
WORKDIR /app

# Copy resources from builder
COPY --from=builder /app/build/linux/Release/slrbs /app/slrbs
COPY --from=builder /app/build/linux/pyslrbs*.so /app/
COPY --from=builder /app/resources /app/resources

# Set up environment
ENV LD_LIBRARY_PATH=/app:$LD_LIBRARY_PATH
ENV PYTHONPATH=/app:$PYTHONPATH

# Run the executable by default
ENTRYPOINT ["/app/slrbs"]
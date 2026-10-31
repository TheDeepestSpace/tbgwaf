# tbgwaf authoritative game server (no SDL/GL). Build from the repo root:
#   docker build -f server/Dockerfile .
FROM docker.io/ubuntu:22.04 AS build
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      build-essential cmake ninja-build git ca-certificates \
      python3 python3-jinja2 python3-yaml && \
    rm -rf /var/lib/apt/lists/*
WORKDIR /src
COPY . .
RUN cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
      -DTBGWAF_BUILD_CLIENT=OFF -DTBGWAF_BUILD_SERVER=ON && \
    cmake --build build --target tbgwaf_server

# Runtime: Node (for the Jev adapter sidecar) on a glibc at least as new as the build's.
FROM docker.io/node:22-bookworm-slim
RUN useradd --system --no-create-home tbgwaf
COPY --from=build /src/build/tbgwaf_server /usr/local/bin/tbgwaf_server
COPY server/jev-adapter/jev-client.mjs server/jev-adapter/server.mjs /opt/jev-adapter/
COPY server/entrypoint.sh /usr/local/bin/tbgwaf-entrypoint
USER tbgwaf
ENV PORT=8080 NODE_ENV=production
EXPOSE 8080
CMD ["tbgwaf-entrypoint"]

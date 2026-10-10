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

FROM docker.io/ubuntu:22.04
RUN apt-get update && apt-get install -y --no-install-recommends libstdc++6 && \
    rm -rf /var/lib/apt/lists/* && useradd --system --no-create-home tbgwaf
COPY --from=build /src/build/tbgwaf_server /usr/local/bin/tbgwaf_server
USER tbgwaf
ENV PORT=8080
EXPOSE 8080
CMD ["tbgwaf_server"]

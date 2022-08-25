#!/bin/bash
cd "$(dirname "$0")"/..
docker build --no-cache -t usdotfhwastol/carma-carla-integration -f Dockerfile . "$@"

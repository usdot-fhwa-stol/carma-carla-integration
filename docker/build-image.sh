#!/bin/sh
cd ..
docker build -t usdotfhwastol/carma-carla-integration -f Dockerfile . "$@"

#!/bin/sh
cd ..
docker build -t usdotfhwastol/carma-carla-integration:carma-carla-1.0.0 -f Dockerfile . "$@"

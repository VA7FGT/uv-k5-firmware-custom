#!/bin/sh
# first clean images older than 24h, you will run out of disk space one day
docker image prune -a --force --filter "until=24h"
docker build -t uvk5 .
docker run --rm -v ${PWD}/compiled-firmware:/app/compiled-firmware uvk5 /bin/bash -c "cd /app && make clean && make && cp firmware* compiled-firmware/"

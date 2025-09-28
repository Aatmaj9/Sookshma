#!/bin/bash
set -e

COMPOSE_FILE=".devcontainer/docker-compose.yml"

docker compose -f "$COMPOSE_FILE" run --rm sookshma bash
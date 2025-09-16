docker buildx build \
  --platform linux/amd64,linux/arm64 \
  --pull \
  --no-cache \
  -f .devcontainer/Dockerfile \
  -t aatmaj9/sookshma:3.0 \
  .


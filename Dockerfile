FROM ghcr.io/astral-sh/uv:0.8.22-python3.12-bookworm-slim
RUN uv pip install --system esphome==2026.8.0
WORKDIR /config
CMD ["uv", "run", "--no-project", "python", "-m", "esphome", "compile", "mr60bha2-extended.yaml"]

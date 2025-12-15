FROM ubuntu:22.04

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# BuildKit automatically provides these at build time, but declare them so they can be used.
ARG TARGETARCH

ARG BOUFFALO_SDK_REPO="https://github.com/nand2mario/bouffalo_sdk.git"
ARG BOUFFALO_SDK_DIR="/opt/bouffalo_sdk"

# Bouffalo's prebuilt (T-Head) toolchain is typically x86_64-only. On non-amd64 images we fall back to the
# distro-provided riscv64-unknown-elf toolchain (works well for many setups, and runs native on Apple Silicon).
ARG BFLB_THEAD_TOOLCHAIN_REPO="https://github.com/bouffalolab/toolchain_gcc_t-head_linux.git"
ARG BFLB_THEAD_TOOLCHAIN_DIR="/opt/toolchain_gcc_t-head_linux"

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    file \
    git \
    ninja-build \
    openssl \
    python-is-python3 \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN if [[ "${TARGETARCH:-}" == "amd64" ]]; then \
      git clone --depth 1 "${BFLB_THEAD_TOOLCHAIN_REPO}" "${BFLB_THEAD_TOOLCHAIN_DIR}"; \
    else \
      apt-get update && apt-get install -y --no-install-recommends \
        binutils-riscv64-unknown-elf \
        gcc-riscv64-unknown-elf \
        && rm -rf /var/lib/apt/lists/*; \
    fi

RUN git clone --depth 1 --recurse-submodules --shallow-submodules \
    "${BOUFFALO_SDK_REPO}" "${BOUFFALO_SDK_DIR}"

# Common Python deps used by Bouffalo tooling (build still works if the SDK doesn't need them).
RUN python3 -m pip install --no-cache-dir --upgrade pip \
    && python3 -m pip install --no-cache-dir \
      ecdsa \
      pycryptodome \
      pyserial \
      pyyaml

ENV BL_SDK_BASE="${BOUFFALO_SDK_DIR}"
ENV PATH="${BFLB_THEAD_TOOLCHAIN_DIR}/bin:${PATH}"

WORKDIR /work
CMD ["make"]

FROM ubuntu:jammy

RUN apt update
RUN apt install curl wget gnupg git git-lfs clang python3 zstd xz-utils file rsync libhdf5-dev micro gedit xauth -y

RUN wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
RUN apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB

RUN echo "deb https://apt.repos.intel.com/openvino/2023 ubuntu22 main" | tee /etc/apt/sources.list.d/intel-openvino-2023.list

RUN apt update
RUN apt install openvino -y

RUN curl https://sh.rustup.rs -sSf > /tmp/rustup-init.sh \
    && chmod +x /tmp/rustup-init.sh \
    && sh /tmp/rustup-init.sh -y \
    && rm -rf /tmp/rustup-init.sh

ENV PATH "$PATH:~/.cargo/bin"
RUN ~/.cargo/bin/rustup install stable
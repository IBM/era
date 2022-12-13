FROM centos:7

WORKDIR /home

# Add repo for cmake3
RUN yum install -y epel-release centos-release-scl
# install python, perl,  CAD tools and software toolchain dependencies, miscellaneous
RUN yum install -y python python-pip python-devel python3 python3-pip python3-devel python3-tkinter \
    perl perl-Env perl-YAML perl-XML-Simple perl-ExtUtils-MakeMaker \
    perl-Thread-Queue perl-Capture-Tiny \
    xterm csh ksh zsh tcl glibc-devel glibc-devel.i686 \
    glibc-static glibc-static.i686 mesa-libGL.i686 mesa-libGLU.i686 mesa-libGL \
    mesa-libGLU mesa-dri-drivers mesa-dri-drivers.i686 readline-devel \
    readline-devel.i686 libXp libXp.i686 openmotif ncurses gdbm-devel \
    gdbm-devel.i686 libSM libSM.i686 libXcursor libXcursor.i686 libXft \
    libXft.i686 libXrandr libXrandr.i686 libXScrnSaver libXScrnSaver.i686 \
    libmpc-devel libmpc-devel.i686 nspr nspr.i686 nspr-devel nspr-devel.i686 \
    tk tk-devel Xvfb dtc bison flex socat minicom tmux \
    git wget bzip2 which unzip file patch bc \
    texinfo texinfo-tex help2man emacs vim xorg-x11-apps java \
    libopencl-devel boost-devel ninja-build gcc-c++ libmpc-devel mpfr-devel \
    gmp-devel pygame autoconf automake openssl-devel libtool --nogpgcheck

RUN wget http://ftp.gnu.org/gnu/make/make-4.2.1.tar.gz && \
    tar xvzf make-4.2.1.tar.gz && \
    cd make-4.2.1 && ./configure && bash build.sh && ./make install
RUN ln -s /lib64/libtiff.so.5 /lib64/libtiff.so.3 && \
    ln -s /usr/lib64/libmpc.so.3 /usr/lib64/libmpc.so.2  && \
    ln -sf /usr/local/bin/make /usr/bin/gmake && \ 
    # enable X11 forwarding
    sed -i 's/#   ForwardX11 no/ForwardX11 yes/g' /etc/ssh/ssh_config

RUN wget https://cmake.org/files/v3.18/cmake-3.18.0-rc1.tar.gz && tar xvzf cmake-3.18.0-rc1.tar.gz && cd cmake-3.18.0-rc1 && ./bootstrap && make && make install 

# Install Z3 Library needed by LLVM
RUN wget http://ftp.mirrorservice.org/sites/sourceware.org/pub/gcc/releases/gcc-7.3.0/gcc-7.3.0.tar.gz && \
    tar zxf gcc-7.3.0.tar.gz && cd gcc-7.3.0 && ./contrib/download_prerequisites && \
    ./configure --disable-multilib --enable-languages=c,c++ && make -j8 && \
    make install && cd .. && git clone https://github.com/Z3Prover/z3.git && cd z3 && \
    git checkout tags/z3-4.7.1 && python scripts/mk_make.py && cd build \
    && make -j4 && make install 

#Need jpeg for pillow which is needed for torchvision
RUN yum install -y libjpeg-devel

# create non-root user
RUN useradd -ms /bin/bash espuser
# switch to non-root user
USER espuser
WORKDIR /home/espuser

# Install Anaconda
RUN  wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh && \
      (echo y; echo yes; echo /home/espuser/miniconda3 ; echo yes) | bash Miniconda3-latest-Linux-x86_64.sh

RUN echo "export LD_LIBRARY_PATH=/usr/local/lib64/:/usr/local/lib/" >> /home/espuser/.bashrc
ENV LD_LIBRARY_PATH="/usr/local/lib64/:/usr/local/lib/"
ENV PATH /home/espuser/miniconda3/bin:$PATH

RUN git config --global url."https://github.com/".insteadOf git://github.com/ && git config --global url."https://gitlab.com/qemu-project/".insteadOf https://git.qemu.org/git/

# clone ESP repository and install software toolchains
RUN git clone https://github.com/sld-columbia/esp.git && cd esp && \
    git submodule update --init --recursive \
    accelerators/stratus_hls/common/inc \
    accelerators/chisel/hw rtl/caches/esp-caches rtl/cores/ariane/ariane \
    soft/ariane/riscv-pk soft/ariane/riscv-tests \
    accelerators/third-party/NV_NVDLA/ip \
    accelerators/third-party/NV_NVDLA/sw && \
    cd /home/espuser/esp/accelerators/third-party/NV_NVDLA && \
    rm -rf ip/verif sw/prebuilt sw/regression && \
    cd /home/espuser/esp && \
    (echo y; echo /home/espuser/riscv; echo 4; echo y; echo n; echo n) | bash utils/toolchain/build_riscv_toolchain.sh && rm -rf /tmp/_riscv_build 
#(echo y; echo /home/espuser/riscv32imc; echo 4; echo n) | bash utils/toolchain/build_riscv32imc_toolchain.sh && rm -rf /tmp/_riscv32imc_build

# HPVM repo setup, install and compile
RUN conda config --set channel_priority strict
RUN git clone -b hpvm-release-epochs --single-branch https://gitlab.engr.illinois.edu/llvm/hpvm-release.git
RUN cd hpvm-release/hpvm && conda init bash && . ~/.bashrc && conda create --name hpvm python=3.7 
RUN conda init bash && . ~/.bashrc && conda activate hpvm && conda install -c conda-forge \
      Cython=0.29.30
RUN conda init bash && . ~/.bashrc && conda activate hpvm && conda install -c conda-forge \
      jinja2=2.11 \
      matplotlib=3.3
RUN conda init bash && . ~/.bashrc && conda activate hpvm && conda install -c conda-forge \
      networkx=2.5 \
      numpy==1.19.4  \
      onnx==1.8.0
RUN conda init bash && . ~/.bashrc && conda activate hpvm && conda install -c conda-forge \
      pandas=1.1 \
      pip \
      pydot=1.4 \
      python=3.7
RUN conda init bash && . ~/.bashrc && conda activate hpvm && conda install -c conda-forge \
      PyYAML=5.4.1 \
      scikit-learn=0.21 \
      scipy=1.7.3
RUN conda init bash && . ~/.bashrc && conda activate hpvm && conda install -c conda-forge \
      tabulate=0.8 \
      tqdm=4.33 \
      xlsxwriter=1.2
RUN conda init bash && . ~/.bashrc && conda activate hpvm && conda install -c pytorch -c conda-forge \
      pytorch=1.5.0 \
      tensorflow==1.14.0 \
      torchvision==0.6 
RUN conda init bash && . ~/.bashrc && conda activate hpvm && pip install \
      argparse==1.4 \
      graphviz==0.10 \
      onnx-simplifier==0.3 \
      torchnet==0.0.4 \
      pexpect==4.8.0 \
      mean_average_precision \
      pytorch-lightning==1.3 \
      imgaug==0.4.0
RUN conda init bash && . ~/.bashrc && conda activate hpvm && pip install lightnet==2.0.1 torchmetrics==0.6.0
RUN rm -rf hpvm-release
RUN git clone -b hpvm-release-epochs --single-branch https://gitlab.engr.illinois.edu/llvm/hpvm-release.git hpvm-release
RUN cd hpvm-release/hpvm && git checkout c71f67fa64b4265573560495d46ad9bd2fa16a21

USER root
# Install gcc > 8.0.0
RUN yum install -y centos-release-scl
RUN yum install -y devtoolset-8-gcc devtoolset-8-gcc-c++
RUN yum install -y man-pages

USER espuser
WORKDIR /home/espuser

#Enable to use gcc and shell from devtoolset-8
RUN echo "source scl_source enable devtoolset-8" >> /home/espuser/.bashrc

RUN cd hpvm-release/hpvm && conda init bash && . ~/.bashrc && conda activate hpvm \
      && sed -i 's/--recursive\"])/--recursive\"],cwd=\"\/home\/espuser\/hpvm-release\")/g' scripts/hpvm_installer.py \
      && sed -i 's/=g++\",/=g++\", \"-DCMAKE_CXX_STANDARD=17\",/g' scripts/hpvm_installer.py \
      && ./install.sh -t "X86;RISCV" -d 
RUN cd hpvm-release/hpvm/build \
      && cmake -DCMAKE_CXX_STANDARD=17 -DLLVM_TARGETS_TO_BUILD="X86;RISCV" -DCMAKE_BUILD_TYPE=Debug ../llvm \
      && make -j4

# build riscv boost to path in soc_lib Makefiles
RUN mkdir -p /home/espuser/riscv_boost_cc
RUN wget -c http://sourceforge.net/projects/boost/files/boost/1.66.0/boost_1_66_0.tar.bz2 --no-check-certificate \
    && tar xvjf boost_1_66_0.tar.bz2 \
    && rm boost_1_66_0.tar.bz2
COPY small/boost.patch boost_1_66_0
ENV PATH=/home/espuser/riscv/bin:${PATH}
# -l in patch is to handle white spaces on ends of lines
# exit 0; is bc boost reports an error in the build, but it is fine.
RUN cd boost_1_66_0 \
    && ./bootstrap.sh --prefix=/home/espuser/riscv_boost_cc \
    && patch -p0 -l < boost.patch \
    && ./b2 -j20 link=static toolset=gcc-riscv install; exit 0
RUN mkdir -p /home/espuser/boost

# soc-library repo setup and compilation
RUN git clone -b meta https://github.com/IBM/scheduler-library.git scheduler-library-hpvm \
    && cd scheduler-library-hpvm \
    && git checkout 9c1d02a
#     && git checkout ce19bef806eee14c58799e4960d98b01a7d02107

ENV HPVM_DIR=/home/espuser/hpvm-release/hpvm
ENV RISCV_BIN_DIR=/home/espuser/riscv/bin
ENV RISCV_BOOST_DIR=/home/espuser/riscv_boost_cc
ENV ESP_ROOT=/home/espuser/esp
ENV SOC_LIB_DIR=/home/espuser/scheduler-library-hpvm
ENV SCHED_CONFIG=soc_utils/config_files/base_me_p3.config 
ENV RISCV_DIR=/home/espuser/riscv

ENV SCHED_LIB_DIR=/home/espuser/scheduler-library-hpvm/sched_library
ENV TASK_LIB_DIR=/home/espuser/scheduler-library-hpvm/task_library
ENV HPVM_BUILD_DIR=${HPVM_DIR}/build
ENV LD_LIBRARY_PATH=${HPVM_DIR}/build/lib:${LD_LIBRARY_PATH}

ENV PATH=${PATH}:${RISCV_BIN_DIR}

RUN cd scheduler-library-hpvm && ln -sf hw_config_files/build_epochs0.config ./hardware.config \
    && make hpvm-epochs

#ERA repo setup and compile for FPGA
RUN git clone -b hetero-integration https://github.com/IBM/era.git
# RUN cd era && git checkout f875d16784a7bcdbf7be499af6c24b8a37f688e7
RUN cd era && git checkout 3ebe6013f314bb037443c17fe90406262bb67ae3

COPY small/setup_paths.patch /home/espuser/era/

RUN cd era && patch -p0 -l < setup_paths.patch && make clean clobber && bash -c "./compile-era.sh riscv"

USER root

# enable X11 forwarding
RUN sed -i 's/#   ForwardX11 no/ForwardX11 yes/g' /etc/ssh/ssh_config
# copy over environment setup script (without CAD tools)
COPY small/esp_env.sh /home/espuser
# copy over environment setup script (with CAD tools)
COPY small/esp_env_cad.sh /home/espuser
# copy over bashrc script
COPY small/bashrc /root/.bashrc
# copy over tmux.conf script
COPY small/tmux.conf /root/.tmux.conf
# copy over emacs script
COPY small/emacs /root/.emacs
